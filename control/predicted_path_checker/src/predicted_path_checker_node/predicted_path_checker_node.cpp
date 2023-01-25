// Copyright 2022 LeoDrive A.Ş. All rights reserved.
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

#include "predicted_path_checker/predicted_path_checker_node.hpp"

#include <motion_utils/marker/marker_helper.hpp>
#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/math/unit_conversion.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace predicted_path_checker
{
PredictedPathCheckerNode::PredictedPathCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("predicted_path_checker_node", node_options), updater_(this)
{
  using std::placeholders::_1;

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);
  node_param_.ego_nearest_dist_threshold = declare_parameter("ego_nearest_dist_threshold", 3.0);
  node_param_.ego_nearest_yaw_threshold = declare_parameter("ego_nearest_yaw_threshold", 1.046);
  node_param_.max_deceleration = declare_parameter("max_deceleration", 1.5);
  node_param_.delay_time = declare_parameter("delay_time", 0.5);

  // Collision Checker Parameter
  collision_checker_param_.stop_search_radius =
    declare_parameter("collision_checker_params.stop_search_radius", 5.0);
  collision_checker_param_.longitudinal_margin =
    declare_parameter("collision_checker_params.longitudinal_margin", 0.15);
  collision_checker_param_.width_margin =
    declare_parameter("collision_checker_params.width_margin", 0.2);
  collision_checker_param_.stop_margin =
    declare_parameter("collision_checker_params.stop_margin", 1.0);
  collision_checker_param_.search_radius =
    declare_parameter("collision_checker_params.search_radius", 6.0);
  collision_checker_param_.resample_interval =
    declare_parameter("collision_checker_params.resample_interval", 0.5);

  vehicle_info_ = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();

  // Subscriber
  self_pose_listener_ = std::make_shared<tier4_autoware_utils::SelfPoseListener>(this);
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  sub_obstacle_pointcloud_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    "input/obstacle_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&PredictedPathCheckerNode::onObstaclePointcloud, this, _1));
  sub_reference_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1,
    std::bind(&PredictedPathCheckerNode::onReferenceTrajectory, this, _1));
  sub_predicted_trajectory_ = create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/predicted_trajectory", 1,
    std::bind(&PredictedPathCheckerNode::onPredictedTrajectory, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", 1, std::bind(&PredictedPathCheckerNode::onOdom, this, _1));

  // Publisher
  virtual_wall_publisher_ =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("~/virtual_wall", 1);
  time_publisher_ = std::make_shared<tier4_autoware_utils::ProcessingTimePublisher>(this);

  // Client
  cli_external_stop_ = this->create_client<tier4_external_api_msgs::srv::Engage>(
    "/api/autoware/set/external_stop", rmw_qos_profile_services_default);

  // Core
  collision_checker_ = std::make_unique<collision_checker::CollisionChecker>(*this);
  collision_checker_->setParam(collision_checker_param_);

  // Diagnostic Updater
  updater_.setHardwareID("predicted_path_checker");

  updater_.add("predicted_path_checker", this, &PredictedPathCheckerNode::checkVehicleState);

  // Wait for first self pose
  self_pose_listener_->waitForFirstPose();

  // Timer
  initTimer(1.0 / node_param_.update_rate);
}

void PredictedPathCheckerNode::onObstaclePointcloud(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  obstacle_pointcloud_ = msg;
}

void PredictedPathCheckerNode::onReferenceTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  reference_trajectory_ = msg;
}

void PredictedPathCheckerNode::onPredictedTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  predicted_trajectory_ = msg;
}

void PredictedPathCheckerNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ = std::make_shared<geometry_msgs::msg::Twist>(msg->twist.twist);
}

void PredictedPathCheckerNode::initTimer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&PredictedPathCheckerNode::onTimer, this));
}

bool PredictedPathCheckerNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_pose...");
    return false;
  }

  if (!obstacle_pointcloud_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for obstacle_pointcloud msg...");
    return false;
  }

  if (!reference_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for predicted_trajectory msg...");
    return false;
  }

  if (!current_twist_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_twist msg...");
    return false;
  }

  if (!cli_external_stop_->service_is_ready()) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for external_stop service...");
    return false;
  }

  return true;
}

bool PredictedPathCheckerNode::isDataTimeout()
{
  const auto now = this->now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp).seconds() - now.seconds();
  if (pose_time_diff > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "pose is timeout...");
    return true;
  }

  return false;
}

void PredictedPathCheckerNode::onTimer()
{
  current_pose_ = self_pose_listener_->getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }

  // transform pointcloud
  geometry_msgs::msg::TransformStamped::ConstSharedPtr obstacle_transform_;

  try {
    obstacle_transform_ = transform_listener_->getTransform(
      predicted_trajectory_->header.frame_id, obstacle_pointcloud_->header.frame_id,
      obstacle_pointcloud_->header.stamp, rclcpp::Duration::from_seconds(0.01));
  } catch (tf2::TransformException & ex) {
    RCLCPP_INFO(
      rclcpp::get_logger("predicted_path_checker"), "Could not transform map to %s: %s",
      predicted_trajectory_->header.frame_id.c_str(), ex.what());
    return;
  }

  collision_checker::Input collision_checker_input;
  collision_checker_input.current_pose = current_pose_;
  collision_checker_input.point_cloud = obstacle_pointcloud_;
  collision_checker_input.predicted_trajectory = predicted_trajectory_;
  collision_checker_input.transform_stamped = obstacle_transform_;

  auto collision_checker_output = collision_checker_->run(collision_checker_input);

  updateState(collision_checker_output);

  // send stop request
  if (current_state_ == State::DRIVE) {
    sendRequest(false);
  } else {
    sendRequest(true);
  }

  publishVirtualWall(collision_checker_output);
}

TrajectoryPoints PredictedPathCheckerNode::trimTrajectoryFromSelfPose(
  const TrajectoryPoints & input, const Pose & self_pose)
{
  TrajectoryPoints output{};

  const size_t min_distance_index = motion_utils::findFirstNearestIndexWithSoftConstraints(
    input, self_pose, node_param_.ego_nearest_dist_threshold,
    node_param_.ego_nearest_yaw_threshold);

  for (size_t i = min_distance_index; i < input.size(); ++i) {
    output.push_back(input.at(i));
  }

  return output;
}

bool PredictedPathCheckerNode::isThereStopPointOnReferenceTrajectory(
  const geometry_msgs::msg::Pose & pose)
{
  const auto trimmed_reference_trajectory_array = trimTrajectoryFromSelfPose(
    motion_utils::convertToTrajectoryPointArray(*reference_trajectory_), current_pose_.get()->pose);

  const auto nearest_stop_point_on_ref_trajectory =
    motion_utils::findNearestIndex(trimmed_reference_trajectory_array, pose);

  const auto stop_point_on_trajectory = motion_utils::searchZeroVelocityIndex(
    trimmed_reference_trajectory_array, 0, *nearest_stop_point_on_ref_trajectory);

  if (!stop_point_on_trajectory) {
    return false;
  }
  return true;
}

void PredictedPathCheckerNode::checkVehicleState(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (current_state_ == State::EMERGENCY) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "vehicle will collide with obstacles";
  }
  if (current_state_ == State::PAUSE) {
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    msg = "vehicle will stop due to obstacle";
  }

  stat.summary(level, msg);
}

void PredictedPathCheckerNode::updateState(
  const boost::optional<collision_checker::Output> & output)
{
  if (!output) {
    current_state_ = State::DRIVE;
    updater_.force_update();
    return;
  } else {
    current_state_ = State::PAUSE;
  }

  const auto stop_index = output.get().stop_index;
  const auto & stop_pose = output.get().output_trajectory.at(stop_index).pose;

  if (utils::isInBrakeDistance(
        output.get().output_trajectory, stop_index, current_twist_.get()->linear.x,
        node_param_.max_deceleration, node_param_.delay_time)) {
    current_state_ = State::EMERGENCY;
    updater_.force_update();
    return;
  }

  if (isThereStopPointOnReferenceTrajectory(stop_pose)) {
    current_state_ = State::DRIVE;
  }
  updater_.force_update();
}

void PredictedPathCheckerNode::publishVirtualWall(
  boost::optional<collision_checker::Output> & output)
{
  visualization_msgs::msg::MarkerArray msg;
  rclcpp::Time current_time = this->now();

  if (current_state_ == State::DRIVE) {
    const auto markers = motion_utils::createDeletedStopVirtualWallMarker(current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &msg);
  } else {
    const auto p = tier4_autoware_utils::calcOffsetPose(
      output.get().output_trajectory.at(output.get().stop_index).pose,
      vehicle_info_.max_longitudinal_offset_m, 0.0, 0.0);
    const auto markers =
      motion_utils::createStopVirtualWallMarker(p, "obstacle on the path", current_time, 0);
    tier4_autoware_utils::appendMarkerArray(markers, &msg);
  }
  virtual_wall_publisher_->publish(msg);
}

void PredictedPathCheckerNode::sendRequest(bool make_stop_vehicle)
{
  // external stop request

  auto req = std::make_shared<tier4_external_api_msgs::srv::Engage::Request>();
  req->engage = make_stop_vehicle;
  cli_external_stop_->async_send_request(
    req,
    []([[maybe_unused]] rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedFuture result) {
    });
}
}  // namespace predicted_path_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(predicted_path_checker::PredictedPathCheckerNode)
