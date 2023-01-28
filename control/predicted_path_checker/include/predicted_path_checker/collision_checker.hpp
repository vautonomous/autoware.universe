// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#ifndef PREDICTED_PATH_CHECKER__COLLISION_CHECKER_HPP_
#define PREDICTED_PATH_CHECKER__COLLISION_CHECKER_HPP_

#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <predicted_path_checker/debug_marker.hpp>
#include <predicted_path_checker/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/ros/processing_time_publisher.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>
#include <vehicle_info_util/vehicle_info.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>

#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

namespace collision_checker
{
using autoware_auto_planning_msgs::msg::Trajectory;
using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using TrajectoryPoints = std::vector<TrajectoryPoint>;
using sensor_msgs::msg::PointCloud2;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;

struct collisionCheckerParam
{
  double resample_interval;
  double search_radius;
  double stop_margin;
  double width_margin;
  double longitudinal_margin;
  double stop_search_radius;
};

struct Output
{
  TrajectoryPoints output_trajectory;
  size_t collision_index;
  size_t stop_index;
};

struct Input
{
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose;
  sensor_msgs::msg::PointCloud2::SharedPtr point_cloud;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory;
  geometry_msgs::msg::TransformStamped::ConstSharedPtr transform_stamped;
};

class CollisionChecker
{
public:
  explicit CollisionChecker(rclcpp::Node & node);
  boost::optional<Output> run(const Input & input);
  void setParam(const collisionCheckerParam & param);

private:
  // Functions
  std::shared_ptr<motion_planning::ObstacleStopPlannerDebugNode> debug_ptr_;
  void filterPointCloud(const sensor_msgs::msg::PointCloud2 & input);

  bool searchPointCloudNearPredictedTrajectory(
    const TrajectoryPoints & trajectory, const PointCloud2::ConstSharedPtr & input_points_ptr,
    PointCloud::Ptr output_points_ptr,
    const geometry_msgs::msg::TransformStamped & obstacle_transform);

  boost::optional<std::pair<size_t, size_t>> checkTrajectoryForCollision(
    TrajectoryPoints & predicted_trajectory_array,
    const PointCloud::Ptr obstacle_candidate_pointcloud);

  void extendTrajectoryPointsArray(TrajectoryPoints & trajectory);

  // Parameter
  collisionCheckerParam param_;

  // Variables
  sensor_msgs::msg::PointCloud2::SharedPtr filtered_point_cloud_;
  vehicle_info_util::VehicleInfo vehicle_info_;
};
}  // namespace collision_checker

#endif  // PREDICTED_PATH_CHECKER__COLLISION_CHECKER_HPP_
