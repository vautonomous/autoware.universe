
// Copyright 2022 TIER IV, Inc.
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

#ifndef PREDICTED_PATH_CHECKER__UTILS_HPP_
#define PREDICTED_PATH_CHECKER__UTILS_HPP_

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <autoware_auto_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <boost/optional.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace utils
{

using diagnostic_msgs::msg::DiagnosticStatus;
using diagnostic_msgs::msg::KeyValue;
using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::TransformStamped;
using std_msgs::msg::Header;

using autoware_auto_planning_msgs::msg::TrajectoryPoint;
using tier4_autoware_utils::Point2d;
using tier4_autoware_utils::Polygon2d;
using vehicle_info_util::VehicleInfo;

using TrajectoryPoints = std::vector<TrajectoryPoint>;
using PointCloud = pcl::PointCloud<pcl::PointXYZ>;

Pose getVehicleCenterFromBase(
  const Pose & base_pose, const vehicle_info_util::VehicleInfo & vehicle_info);

void createOneStepPolygon(
  const Pose & base_step_pose, const Pose & next_step_pose, std::vector<cv::Point2d> & polygon,
  const vehicle_info_util::VehicleInfo & vehicle_info, const double expand_width = 0.0,
  const double expand_longitudinal = 0.0);

bool withinPolygon(
  const std::vector<cv::Point2d> & cv_polygon, const double radius, const Point2d & prev_point,
  const Point2d & next_point, const PointCloud::Ptr candidate_points,
  PointCloud::Ptr within_points_ptr);

void getNearestPoint(
  const PointCloud & pointcloud, const Pose & base_pose, pcl::PointXYZ * nearest_collision_point);

TrajectoryPoint calcInterpolatedPoint(
  const TrajectoryPoints & trajectory, const size_t segment_idx,
  const geometry_msgs::msg::Point & target_point);

std::pair<size_t, TrajectoryPoint> findStopPoint(
  TrajectoryPoints & predicted_trajectory_array, const size_t collision_idx,
  const double stop_margin, vehicle_info_util::VehicleInfo & vehicle_info);

bool isInBrakeDistance(
  const TrajectoryPoints & trajectory, const size_t stop_idx, const double current_velocity,
  const double max_deceleration, const double delay_time_sec);

geometry_msgs::msg::Point pclPointToRosPoint(const pcl::PointXYZ & pcl_point);

TrajectoryPoint getExtendTrajectoryPoint(
  const double extend_distance, const TrajectoryPoint & goal_point);

// boost::optional<size_t> searchLastZeroVelocityIndex(
//   const TrajectoryPoints & points_with_twist, const size_t src_idx, const size_t dst_idx);

}  // namespace utils

#endif  // PREDICTED_PATH_CHECKER__UTILS_HPP_
