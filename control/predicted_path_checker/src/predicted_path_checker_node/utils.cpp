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

#include "predicted_path_checker/utils.hpp"

#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>

#include <diagnostic_msgs/msg/key_value.hpp>

#include <boost/format.hpp>

#include <pcl_conversions/pcl_conversions.h>

namespace utils
{

using motion_utils::findFirstNearestIndexWithSoftConstraints;
using motion_utils::findFirstNearestSegmentIndexWithSoftConstraints;
using tier4_autoware_utils::calcDistance2d;
using tier4_autoware_utils::getRPY;

// Utils Functions
Pose getVehicleCenterFromBase(
  const Pose & base_pose, const vehicle_info_util::VehicleInfo & vehicle_info)
{
  const auto & i = vehicle_info;
  const auto yaw = tier4_autoware_utils::getRPY(base_pose).z;

  Pose center_pose;
  center_pose.position.x =
    base_pose.position.x + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::cos(yaw);
  center_pose.position.y =
    base_pose.position.y + (i.vehicle_length_m / 2.0 - i.rear_overhang_m) * std::sin(yaw);
  center_pose.position.z = base_pose.position.z;
  center_pose.orientation = base_pose.orientation;
  return center_pose;
}

bool convexHull(
  const std::vector<cv::Point2d> & pointcloud, std::vector<cv::Point2d> & polygon_points)
{
  cv::Point2d centroid;
  centroid.x = 0;
  centroid.y = 0;
  for (const auto & point : pointcloud) {
    centroid.x += point.x;
    centroid.y += point.y;
  }
  centroid.x = centroid.x / static_cast<double>(pointcloud.size());
  centroid.y = centroid.y / static_cast<double>(pointcloud.size());

  std::vector<cv::Point> normalized_pointcloud;
  std::vector<cv::Point> normalized_polygon_points;
  for (const auto & p : pointcloud) {
    normalized_pointcloud.emplace_back(
      cv::Point((p.x - centroid.x) * 1000.0, (p.y - centroid.y) * 1000.0));
  }
  cv::convexHull(normalized_pointcloud, normalized_polygon_points);

  for (const auto & p : normalized_polygon_points) {
    cv::Point2d polygon_point;
    polygon_point.x = (p.x / 1000.0 + centroid.x);
    polygon_point.y = (p.y / 1000.0 + centroid.y);
    polygon_points.push_back(polygon_point);
  }
  return true;
}

void createOneStepPolygon(
  const Pose & base_step_pose, const Pose & next_step_pose, std::vector<cv::Point2d> & polygon,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double expand_width,
  const double expand_longitudinal)
{
  std::vector<cv::Point2d> one_step_move_vehicle_corner_points;

  const auto & i = vehicle_info;
  const auto & front_m = i.max_longitudinal_offset_m + expand_longitudinal;
  const auto & width_m = i.vehicle_width_m / 2.0 + expand_width;
  const auto & back_m = i.rear_overhang_m + expand_longitudinal;
  // start step
  {
    const auto yaw = tier4_autoware_utils::getRPY(base_step_pose).z;
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
      base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
      base_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
      base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      base_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
      base_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  // next step
  {
    const auto yaw = tier4_autoware_utils::getRPY(next_step_pose).z;
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * width_m,
      next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * front_m - std::sin(yaw) * -width_m,
      next_step_pose.position.y + std::sin(yaw) * front_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * -width_m,
      next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * -width_m));
    one_step_move_vehicle_corner_points.emplace_back(cv::Point2d(
      next_step_pose.position.x + std::cos(yaw) * -back_m - std::sin(yaw) * width_m,
      next_step_pose.position.y + std::sin(yaw) * -back_m + std::cos(yaw) * width_m));
  }
  convexHull(one_step_move_vehicle_corner_points, polygon);
}

bool withinPolygon(
  const std::vector<cv::Point2d> & cv_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, const PointCloud::Ptr candidate_points,
  PointCloud::Ptr within_points_ptr)
{
  Polygon2d boost_polygon;
  bool find_within_points = false;
  for (const auto & point : cv_polygon) {
    boost_polygon.outer().push_back(boost::geometry::make<Point2d>(point.x, point.y));
  }
  boost_polygon.outer().push_back(
    boost::geometry::make<Point2d>(cv_polygon.front().x, cv_polygon.front().y));

  for (size_t j = 0; j < candidate_points->size(); ++j) {
    Point2d point(candidate_points->at(j).x, candidate_points->at(j).y);
    if (
      boost::geometry::distance(prev_point, point) < radius ||
      boost::geometry::distance(next_point, point) < radius) {
      if (boost::geometry::within(point, boost_polygon)) {
        within_points_ptr->push_back(candidate_points->at(j));
        find_within_points = true;
      }
    }
  }
  return find_within_points;
}

void getNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * nearest_collision_point)
{
  double min_norm = 0.0;
  bool is_init = false;
  const auto yaw = tier4_autoware_utils::getRPY(base_pose).z;
  const Eigen::Vector2d base_pose_vec(std::cos(yaw), std::sin(yaw));

  for (const auto & p : pointcloud) {
    const Eigen::Vector2d pointcloud_vec(p.x - base_pose.position.x, p.y - base_pose.position.y);
    double norm = base_pose_vec.dot(pointcloud_vec);
    if (norm < min_norm || !is_init) {
      min_norm = norm;
      *nearest_collision_point = p;
      is_init = true;
    }
  }
}

TrajectoryPoint calcInterpolatedPoint(
  const TrajectoryPoints & trajectory, const size_t segment_idx,
  const geometry_msgs::msg::Point & target_point)
{
  const auto & base_point = trajectory.at(segment_idx);
  const auto & next_point = trajectory.at(segment_idx + 1);

  Eigen::Vector2d base_to_target(
    target_point.x - base_point.pose.position.x, target_point.y - base_point.pose.position.y);
  Eigen::Vector2d base_to_next(
    next_point.pose.position.x - base_point.pose.position.x,
    next_point.pose.position.y - base_point.pose.position.y);

  const double projected_length =
    (base_to_target.dot(base_to_next)) / base_to_next.norm();  // Not sure check again!
  const double ratio = projected_length / base_to_next.norm();

  geometry_msgs::msg::Pose interpolated_pose =
    tier4_autoware_utils::calcInterpolatedPose(base_point.pose, next_point.pose, ratio, true);

  TrajectoryPoint output;
  output.set__pose(interpolated_pose);
  return output;
}

std::pair<size_t, TrajectoryPoint> findStopPoint(
  TrajectoryPoints & trajectory_array, const size_t collision_idx, const double stop_margin,
  vehicle_info_util::VehicleInfo & vehicle_info)
{
  // It returns the stop point and segment of the point on trajectory.
  // TODO(brkay54): I am not sure if there is any bug on algorithm. Check it again!

  double desired_distance_base_link_to_collision =
    vehicle_info.max_longitudinal_offset_m + stop_margin;
  size_t stop_segment_idx = 0;
  bool found_stop_point = false;
  double distance_point_to_collision = 0.0;

  for (size_t i = collision_idx; i > 0; i--) {
    distance_point_to_collision =
      motion_utils::calcSignedArcLength(trajectory_array, i - 1, collision_idx);
    if (distance_point_to_collision >= desired_distance_base_link_to_collision) {
      stop_segment_idx = i - 1;
      found_stop_point = true;
      break;
    }
  }
  if (found_stop_point) {
    const auto & base_point = trajectory_array.at(stop_segment_idx);
    const auto & next_point = trajectory_array.at(stop_segment_idx + 1);

    double ratio = (distance_point_to_collision - desired_distance_base_link_to_collision) /
                   (std::hypot(
                     base_point.pose.position.x - next_point.pose.position.x,
                     base_point.pose.position.y - next_point.pose.position.y));

    geometry_msgs::msg::Pose interpolated_pose =
      tier4_autoware_utils::calcInterpolatedPose(base_point.pose, next_point.pose, ratio, true);
    TrajectoryPoint output;
    output.set__pose(interpolated_pose);
    return std::make_pair(stop_segment_idx, output);
  } else {
    // It means that there is no enough distance between vehicle and collision point.
    // So, we insert a stop at the first point of the trajectory.
    return std::make_pair(0, trajectory_array.front());
  }
}

bool isInBrakeDistance(
  const TrajectoryPoints & trajectory, const size_t stop_idx, const double current_velocity,
  const double max_deceleration, const double delay_time_sec)
{
  const auto distance_to_obstacle = motion_utils::calcSignedArcLength(
    trajectory, trajectory.front().pose.position, trajectory.at(stop_idx).pose.position);

  const double distance_in_delay = current_velocity * delay_time_sec;
  const double distance_after_delay =
    (current_velocity * current_velocity) / (2.0 * max_deceleration);
  const double brake_distance = distance_in_delay + distance_after_delay;

  return brake_distance > distance_to_obstacle;
}

geometry_msgs::msg::Point pclPointToRosPoint(const pcl::PointXYZ & pcl_point)
{
  geometry_msgs::msg::Point point;
  point.x = pcl_point.x;
  point.y = pcl_point.y;
  point.z = pcl_point.z;
  return point;
}

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point)
{
  tf2::Transform map2goal;
  tf2::fromMsg(goal_point.pose, map2goal);
  tf2::Transform local_extend_point;
  local_extend_point.setOrigin(tf2::Vector3(extend_distance, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  local_extend_point.setRotation(q);
  const auto map2extend_point = map2goal * local_extend_point;
  Pose extend_pose;
  tf2::toMsg(map2extend_point, extend_pose);
  TrajectoryPoint extend_trajectory_point;
  extend_trajectory_point.pose = extend_pose;
  extend_trajectory_point.longitudinal_velocity_mps = goal_point.longitudinal_velocity_mps;
  extend_trajectory_point.lateral_velocity_mps = goal_point.lateral_velocity_mps;
  extend_trajectory_point.acceleration_mps2 = goal_point.acceleration_mps2;
  return extend_trajectory_point;
}
}  // namespace utils
