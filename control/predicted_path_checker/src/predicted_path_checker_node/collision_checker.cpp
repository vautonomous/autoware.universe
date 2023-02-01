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

#include "predicted_path_checker/collision_checker.hpp"

#include "motion_utils/resample/resample.hpp"

#include <pcl_ros/transforms.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/marker_helper.hpp>

#include <memory>
#include <utility>
#include <vector>

namespace collision_checker
{
CollisionChecker::CollisionChecker(rclcpp::Node * node)
: vehicle_info_(vehicle_info_util::VehicleInfoUtil(*node).getVehicleInfo())
{
  debug_ptr_ = std::make_shared<motion_planning::ObstacleStopPlannerDebugNode>(
    node, vehicle_info_.max_longitudinal_offset_m);
}

void CollisionChecker::setParam(const collisionCheckerParam & param) { param_ = param; }

boost::optional<Output> CollisionChecker::run(const Input & input)
{
  TrajectoryPoints predicted_trajectory_array = motion_utils::convertToTrajectoryPointArray(
    motion_utils::resampleTrajectory(*input.predicted_trajectory, param_.resample_interval));

  PointCloud::Ptr obstacle_candidate_pointcloud_ptr(new PointCloud);
  filterPointCloud(*input.point_cloud);

  if (!searchPointCloudNearPredictedTrajectory(
        predicted_trajectory_array, filtered_point_cloud_, obstacle_candidate_pointcloud_ptr,
        *input.transform_stamped)) {
    return boost::none;
  }
  const auto collision_and_stop_idx =
    checkTrajectoryForCollision(predicted_trajectory_array, obstacle_candidate_pointcloud_ptr);
  if (!collision_and_stop_idx) {
    return boost::none;
  }

  Output output;
  output.output_trajectory = predicted_trajectory_array;
  output.collision_index = collision_and_stop_idx.get().first;
  output.stop_index = collision_and_stop_idx.get().second;
  return boost::make_optional(output);
}

boost::optional<std::pair<size_t, size_t>> CollisionChecker::checkTrajectoryForCollision(
  TrajectoryPoints & predicted_trajectory_array,
  const PointCloud::Ptr obstacle_candidate_pointcloud)
{
  // It checks the collision, if there is a collision, it updates the predicted_trajectory_array and
  // returns the index of the stop point.
  // If there is no collision, it returns boost::none.

  for (size_t i = 0; i < predicted_trajectory_array.size() - 1; i++) {
    // create one step circle center for vehicle
    const auto & p_front = predicted_trajectory_array.at(i).pose;
    const auto & p_back = predicted_trajectory_array.at(i + 1).pose;
    const auto prev_center_pose = utils::getVehicleCenterFromBase(p_front, vehicle_info_);
    const Point2d prev_center_point(prev_center_pose.position.x, prev_center_pose.position.y);
    const auto next_center_pose = utils::getVehicleCenterFromBase(p_back, vehicle_info_);
    const Point2d next_center_point(next_center_pose.position.x, next_center_pose.position.y);

    std::vector<cv::Point2d> one_step_move_vehicle_polygon;
    // create one step polygon for vehicle
    utils::createOneStepPolygon(
      p_front, p_back, one_step_move_vehicle_polygon, vehicle_info_, param_.width_margin,
      param_.longitudinal_margin);
    debug_ptr_->pushPolygon(
      one_step_move_vehicle_polygon, p_front.position.z, motion_planning::PolygonType::Vehicle);

    PointCloud::Ptr collision_pointcloud_ptr(new PointCloud);
    collision_pointcloud_ptr->header = obstacle_candidate_pointcloud.get()->header;

    const auto found_collision = utils::withinPolygon(
      one_step_move_vehicle_polygon, param_.stop_search_radius, prev_center_point,
      next_center_point, obstacle_candidate_pointcloud, collision_pointcloud_ptr);

    if (found_collision) {
      pcl::PointXYZ nearest_collision_point;

      utils::getNearestPoint(*collision_pointcloud_ptr, p_front, &nearest_collision_point);

      extendTrajectoryPointsArray(predicted_trajectory_array);

      const auto nearest_segment = motion_utils::findNearestSegmentIndex(
        predicted_trajectory_array, utils::pclPointToRosPoint(nearest_collision_point));

      const auto collision_trajectory_point = utils::calcInterpolatedPoint(
        predicted_trajectory_array, nearest_segment,
        utils::pclPointToRosPoint(nearest_collision_point));

      const size_t collision_idx = nearest_segment + 1;

      predicted_trajectory_array.insert(
        predicted_trajectory_array.begin() + collision_idx, collision_trajectory_point);

      const auto stop_point = utils::findStopPoint(
        predicted_trajectory_array, collision_idx, param_.stop_margin, vehicle_info_);

      const size_t stop_idx = stop_point.first + 1;
      predicted_trajectory_array.insert(
        predicted_trajectory_array.begin() + stop_idx, stop_point.second);

      return boost::make_optional(std::make_pair(collision_idx, stop_idx));
    }
  }
  debug_ptr_->publish();

  return boost::none;
}

bool CollisionChecker::searchPointCloudNearPredictedTrajectory(
  const TrajectoryPoints & trajectory, const PointCloud2::ConstSharedPtr & input_points_ptr,
  PointCloud::Ptr output_points_ptr,
  const geometry_msgs::msg::TransformStamped & obstacle_transform)
{
  PointCloud2 transformed_points{};
  const Eigen::Matrix4f affine_matrix =
    tf2::transformToEigen(obstacle_transform.transform).matrix().cast<float>();
  pcl_ros::transformPointCloud(affine_matrix, *input_points_ptr, transformed_points);
  PointCloud::Ptr transformed_points_ptr(new PointCloud);
  pcl::fromROSMsg(transformed_points, *transformed_points_ptr);

  // Debug here to check the transformed pointcloud!

  output_points_ptr->header = transformed_points_ptr->header;

  // search obstacle candidate pointcloud to reduce calculation cost
  const double search_radius = param_.search_radius;

  const double squared_radius = search_radius * search_radius;
  std::vector<geometry_msgs::msg::Point> center_points;
  center_points.reserve(trajectory.size());
  for (const auto & trajectory_point : trajectory)
    center_points.push_back(
      utils::getVehicleCenterFromBase(trajectory_point.pose, vehicle_info_).position);
  for (const auto & point : transformed_points_ptr->points) {
    for (const auto & center_point : center_points) {
      const double x = center_point.x - point.x;
      const double y = center_point.y - point.y;
      const double squared_distance = x * x + y * y;
      if (squared_distance < squared_radius) {
        output_points_ptr->points.push_back(point);
        break;
      }
    }
  }
  if (output_points_ptr->points.empty()) {
    return false;
  }
  return true;
}

void CollisionChecker::filterPointCloud(const sensor_msgs::msg::PointCloud2 & input)
{
  filtered_point_cloud_ = std::make_shared<PointCloud2>();
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  PointCloud::Ptr pointcloud_ptr(new PointCloud);
  PointCloud::Ptr no_height_pointcloud_ptr(new PointCloud);
  PointCloud::Ptr no_height_filtered_pointcloud_ptr(new PointCloud);

  pcl::fromROSMsg(input, *pointcloud_ptr);

  for (const auto & point : pointcloud_ptr->points) {
    no_height_pointcloud_ptr->push_back(pcl::PointXYZ(point.x, point.y, 0.0));
  }
  filter.setInputCloud(no_height_pointcloud_ptr);
  filter.setLeafSize(0.05f, 0.05f, 100000.0f);
  filter.filter(*no_height_filtered_pointcloud_ptr);
  pcl::toROSMsg(*no_height_filtered_pointcloud_ptr, *filtered_point_cloud_);
  filtered_point_cloud_->header = input.header;
}

void CollisionChecker::extendTrajectoryPointsArray(TrajectoryPoints & trajectory)
{
  // It extends the trajectory to the end of the footprint of the vehicle to get better distance to
  // collision_point.
  const double extend_distance =
    vehicle_info_.max_longitudinal_offset_m + param_.longitudinal_margin;
  const auto & goal_point = trajectory.back();
  const auto trajectory_point_extend = utils::getExtendTrajectoryPoint(extend_distance, goal_point);
  trajectory.push_back(trajectory_point_extend);
}

}  // namespace collision_checker
