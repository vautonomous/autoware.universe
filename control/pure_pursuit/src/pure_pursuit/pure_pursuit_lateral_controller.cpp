// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pure_pursuit/pure_pursuit_lateral_controller.hpp"

#include "pure_pursuit/pure_pursuit_viz.hpp"
#include "pure_pursuit/util/planning_utils.hpp"
#include "pure_pursuit/util/tf_utils.hpp"

#include <motion_utils/resample/resample.hpp>
#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <algorithm>
#include <memory>
#include <utility>

namespace
{
// alias to use template instance with default allocator
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;

geometry_msgs::msg::Quaternion createOrientationMsgFromYaw(double yaw_angle)
{
  geometry_msgs::msg::Quaternion orientation_msg;
  double roll_angle = 0.0;
  double pitch_angle = 0.0;

  tf2::Quaternion quaternion;
  quaternion.setRPY(roll_angle, pitch_angle, yaw_angle);

  orientation_msg.w = quaternion.getW();
  orientation_msg.x = quaternion.getX();
  orientation_msg.y = quaternion.getY();
  orientation_msg.z = quaternion.getZ();

  return orientation_msg;
}

double calcLateralError(
  const geometry_msgs::msg::Pose & ego_pose,
  const autoware_auto_planning_msgs::msg::TrajectoryPoint & ref_pose)
{
  const double err_x = ego_pose.position.x - ref_pose.pose.position.x;
  const double err_y = ego_pose.position.y - ref_pose.pose.position.y;
  const double ref_yaw = tf2::getYaw(ref_pose.pose.orientation);
  const double lat_err = -std::sin(ref_yaw) * err_x + std::cos(ref_yaw) * err_y;
  return lat_err;
}

double calcLookaheadDistance(
  const double velocity, const double lookahead_distance_ratio, const double min_lookahead_distance,
  const double lateral_error, const double lateral_error_ratio, double curvature,
  const double curvature_ratio)
{
  constexpr double curvature_threshold = 0.001;
  if (curvature <= curvature_threshold) {
    curvature = 0.0;
  }
  const double lookahead_distance = lateral_error_ratio * std::abs(lateral_error) +
                                    lookahead_distance_ratio * std::abs(velocity) -
                                    curvature_ratio * std::abs(curvature);
  return std::max(lookahead_distance, min_lookahead_distance);
}

double calcCurvature(
  const TrajectoryPoint & p1, const TrajectoryPoint & p2, const TrajectoryPoint & p3)
{
  // Calculation details are described in the following page
  // https://en.wikipedia.org/wiki/Menger_curvature
  const double denominator = tier4_autoware_utils::calcDistance2d(p1, p2) *
                             tier4_autoware_utils::calcDistance2d(p2, p3) *
                             tier4_autoware_utils::calcDistance2d(p3, p1);
  if (std::fabs(denominator) < 1e-10) {
    throw std::runtime_error("points are too close for curvature calculation.");
  }
  return 2.0 *
         ((p2.pose.position.x - p1.pose.position.x) * (p3.pose.position.y - p1.pose.position.y) -
          (p2.pose.position.y - p1.pose.position.y) * (p3.pose.position.x - p1.pose.position.x)) /
         denominator;
}

}  // namespace

namespace pure_pursuit
{
PurePursuitLateralController::PurePursuitLateralController(rclcpp::Node & node)
: node_{&node}, self_pose_listener_(&node), tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_)
{
  pure_pursuit_ = std::make_unique<PurePursuit>();

  // Vehicle Parameters
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*node_).getVehicleInfo();
  param_.wheel_base = vehicle_info.wheel_base_m;
  param_.max_steering_angle = vehicle_info.max_steer_angle_rad;

  // Algorithm Parameters
  param_.lookahead_distance_ratio =
    node_->declare_parameter<double>("lookahead_distance_ratio", 2.2);
  param_.min_lookahead_distance = node_->declare_parameter<double>("min_lookahead_distance", 0.5);
  param_.reverse_min_lookahead_distance =
    node_->declare_parameter<double>("reverse_min_lookahead_distance", 7.0);
  param_.converged_steer_rad_ = node_->declare_parameter<double>("converged_steer_rad", 0.1);
  param_.lateral_error_ratio = node_->declare_parameter<double>("lateral_error_ratio", 2.5);
  param_.sampling_ds = node_->declare_parameter<double>("sampling_ds", 0.1);
  param_.curvature_ratio = node_->declare_parameter<double>("curvature_ratio", 2.5);
  param_.prediction_time_period = node_->declare_parameter<double>("prediction_time_period", 4.0);
  param_.prediction_time_length = node_->declare_parameter<double>("prediction_time_length", 0.2);
  param_.curvature_calculation_distance =
    node_->declare_parameter<double>("curvature_calculation_distance", 2.0);
  param_.max_ld =
    node_->declare_parameter<double>("max_ld", 25.0);

  // Debug Publishers
  pub_debug_marker_ =
    node_->create_publisher<visualization_msgs::msg::MarkerArray>("~/debug/markers", 0);
  pub_predicted_trajectory_ =
    node_->create_publisher<Trajectory>("~/output/predicted_trajectory", 1);
  //  Wait for first current pose
  tf_utils::waitForTransform(tf_buffer_, "map", "base_link");
}

bool PurePursuitLateralController::isDataReady()
{
  if (!current_odometry_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "waiting for current_odometry...");
    return false;
  }

  if (!trajectory_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "waiting for trajectory...");
    return false;
  }

  if (!current_pose_) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  return true;
}

void PurePursuitLateralController::setInputData(InputData const & input_data)
{
  trajectory_ = input_data.current_trajectory_ptr;
  current_odometry_ = input_data.current_odometry_ptr;
  current_steering_ = input_data.current_steering_ptr;
}

TrajectoryPoint PurePursuitLateralController::calcNextPose(
  const double dt, TrajectoryPoint & point, AckermannLateralCommand cmd)
{
  geometry_msgs::msg::Transform transform;
  transform.translation = tier4_autoware_utils::createTranslation(dt, 0.0, 0.0);
  transform.rotation =
    createOrientationMsgFromYaw(((tan(cmd.steering_tire_angle) * dt) / param_.wheel_base));
  TrajectoryPoint output_p;

  tf2::Transform tf_pose;
  tf2::Transform tf_offset;
  tf2::fromMsg(transform, tf_offset);
  tf2::fromMsg(point.pose, tf_pose);
  tf2::toMsg(tf_pose * tf_offset, output_p.pose);
  return output_p;
}

boost::optional<LateralOutput> PurePursuitLateralController::run()
{
  current_pose_ = self_pose_listener_.getCurrentPose();
  if (!isDataReady()) {
    return boost::none;
  }
  const auto closest_idx_result = planning_utils::findClosestIdxWithDistAngThr(
    planning_utils::extractPoses(*trajectory_), current_pose_->pose, 3.0, M_PI_4);
  if(!closest_idx_result.first){
    return boost::none;
  }
  const double remaining_distance = planning_utils::calcArcLengthFromWayPoint(
    *trajectory_, closest_idx_result.second, trajectory_->points.size() - 1);

//  const auto num_of_iteration = std::max(
//    static_cast<int>(std::ceil(param_.prediction_time_length / param_.prediction_time_period)), 1);

  const auto num_of_iteration = std::max(
    static_cast<int>(std::ceil(
      std::min(remaining_distance, param_.prediction_time_length) / param_.prediction_time_period)),
    1);
  Trajectory predicted_trajectory;
  AckermannLateralCommand cmd_msg;

  for (int i = 0; i < num_of_iteration; i++) {
    if (i == 0) {
      TrajectoryPoint p;
      p.pose = current_pose_->pose;
      p.longitudinal_velocity_mps = current_odometry_->twist.twist.linear.x;
      predicted_trajectory.points.push_back(p);

      const auto pp_output = calcTargetCurvature(true, predicted_trajectory.points.at(i).pose);

      if (pp_output) {
        cmd_msg = generateCtrlCmdMsg(pp_output->curvature);
        prev_cmd = boost::optional<AckermannLateralCommand>(cmd_msg);
        publishDebugMarker();
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "failed to solve pure_pursuit for control command calculation");
        if (prev_cmd) {
          cmd_msg = *prev_cmd;
        } else {
          cmd_msg = generateCtrlCmdMsg({0.0});
        }
      }
      TrajectoryPoint p2;
      p2 = calcNextPose(param_.prediction_time_period, predicted_trajectory.points.at(i), cmd_msg);
      predicted_trajectory.points.push_back(p2);

    } else {
      const auto pp_output = calcTargetCurvature(false, predicted_trajectory.points.at(i).pose);
      AckermannLateralCommand tmp_msg;

      if (pp_output) {
        tmp_msg = generateCtrlCmdMsg(pp_output->curvature);
        predicted_trajectory.points.at(i).longitudinal_velocity_mps = pp_output->velocity;
      } else {
        RCLCPP_WARN_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 5000,
          "failed to solve pure_pursuit for prediction");
        tmp_msg = generateCtrlCmdMsg({0.0});
      }
      predicted_trajectory.points.push_back(
        calcNextPose(param_.prediction_time_period, predicted_trajectory.points.at(i), tmp_msg));
    }
  }

  // for last point
  predicted_trajectory.points.back().longitudinal_velocity_mps = 0.0;

  predicted_trajectory.header.frame_id = trajectory_->header.frame_id;
  predicted_trajectory.header.stamp = node_->now();
  pub_predicted_trajectory_->publish(predicted_trajectory);

  LateralOutput output;
  output.control_cmd = cmd_msg;
  output.sync_data.is_steer_converged =
    std::abs(cmd_msg.steering_tire_angle - current_steering_->steering_tire_angle) <
    static_cast<float>(param_.converged_steer_rad_);
  return output;
}

AckermannLateralCommand PurePursuitLateralController::generateCtrlCmdMsg(
  const double target_curvature)
{
  const double tmp_steering =
    planning_utils::convertCurvatureToSteeringAngle(param_.wheel_base, target_curvature);
  AckermannLateralCommand cmd;
  cmd.stamp = node_->get_clock()->now();
  cmd.steering_tire_angle = static_cast<float>(
    std::min(std::max(tmp_steering, -param_.max_steering_angle), param_.max_steering_angle));

  // pub_ctrl_cmd_->publish(cmd);
  return cmd;
}

void PurePursuitLateralController::publishDebugMarker() const
{
  visualization_msgs::msg::MarkerArray marker_array;

  marker_array.markers.push_back(createNextTargetMarker(debug_data_.next_target));
  marker_array.markers.push_back(
    createTrajectoryCircleMarker(debug_data_.next_target, current_pose_->pose));

  pub_debug_marker_->publish(marker_array);
}

boost::optional<pp_out> PurePursuitLateralController::calcTargetCurvature(
  bool is_control_output, geometry_msgs::msg::Pose pose)
{
  // Ignore invalid trajectory
  if (trajectory_->points.size() < 3) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 5000, "received path size is < 3, ignored");
    return {};
  }

  // Interpolate with constant interval distance.
  std::vector<double> out_arclength;
  auto trajectory = motion_utils::convertToTrajectoryPointArray(*trajectory_);
  const auto traj_length = motion_utils::calcArcLength(trajectory);
  for (double s = 0; s < traj_length; s += param_.sampling_ds) {
    out_arclength.push_back(s);
  }
  auto output_traj =
    motion_utils::resampleTrajectory(motion_utils::convertToTrajectory(trajectory), out_arclength);

  // Calculate target point for velocity/acceleration
  const auto closest_idx_result = planning_utils::findClosestIdxWithDistAngThr(
    planning_utils::extractPoses(output_traj), pose, 3.0, M_PI_4);

  if (!closest_idx_result.first) {
    RCLCPP_ERROR(node_->get_logger(), "cannot find closest waypoint");
    return {};
  }
  pp_out output_cmd;
  const TrajectoryPoint & target_point = output_traj.points.at(closest_idx_result.second);

  output_cmd.velocity = target_point.longitudinal_velocity_mps;

  const size_t idx_dist = static_cast<size_t>(
    std::max(static_cast<int>((param_.curvature_calculation_distance) / param_.sampling_ds), 1));

  // Find the points in trajectory to calculate curvature
  size_t next_idx = output_traj.points.size() - 1;
  size_t prev_idx = 0;

  if (static_cast<size_t>(closest_idx_result.second) >= idx_dist) {
    prev_idx = closest_idx_result.second - idx_dist;
  }
  if (output_traj.points.size() - 1 >= closest_idx_result.second + idx_dist) {
    next_idx = closest_idx_result.second + idx_dist;
  }

  // Calculate curvature assuming the trajectory points interval is constant
  double current_curvature = 0.0;

  try {
    current_curvature = calcCurvature(
      output_traj.points.at(prev_idx), output_traj.points.at(closest_idx_result.second),
      output_traj.points.at(next_idx));
  } catch (std::exception const & e) {
    // ...code that handles the error...
    RCLCPP_WARN(rclcpp::get_logger("pure_pursuit"), "%s", e.what());
  }

  const double lateral_error =
    calcLateralError(pose, output_traj.points.at(closest_idx_result.second));

  // Calculate lookahead distance
  const bool is_reverse = (output_cmd.velocity < 0);
  const double min_lookahead_distance =
    is_reverse ? param_.reverse_min_lookahead_distance : param_.min_lookahead_distance;
//  RCLCPP_INFO(
//    node_->get_logger(), "lateral error %f, curvature %f, curvature_ratio %f", lateral_error,
//    current_curvature, param_.curvature_ratio);

  double lookahead_distance = min_lookahead_distance;

  if (is_control_output) {
    lookahead_distance = calcLookaheadDistance(
      current_odometry_->twist.twist.linear.x, param_.lookahead_distance_ratio,
      min_lookahead_distance, lateral_error, param_.lateral_error_ratio, current_curvature,
      param_.curvature_ratio);
  } else {
    lookahead_distance = calcLookaheadDistance(
      output_cmd.velocity, param_.lookahead_distance_ratio, min_lookahead_distance, lateral_error,
      param_.lateral_error_ratio, current_curvature, param_.curvature_ratio);
  }
  lookahead_distance = std::min(param_.max_ld, lookahead_distance);
  // Set PurePursuit data
  pure_pursuit_->setCurrentPose(pose);
  pure_pursuit_->setWaypoints(planning_utils::extractPoses(output_traj));
  pure_pursuit_->setLookaheadDistance(lookahead_distance);

  // Run PurePursuit
  const auto pure_pursuit_result = pure_pursuit_->run();
  if (!pure_pursuit_result.first) {
    return {};
  }

  output_cmd.curvature = pure_pursuit_result.second;

  // Set debug data
  if (is_control_output) {
    debug_data_.next_target = pure_pursuit_->getLocationOfNextTarget();
  }

  return output_cmd;
}
}  // namespace pure_pursuit
