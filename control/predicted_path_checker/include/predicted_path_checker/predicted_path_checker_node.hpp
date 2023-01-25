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

#ifndef PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_NODE_HPP_
#define PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_NODE_HPP_

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <motion_utils/trajectory/tmp_conversion.hpp>
#include <motion_utils/trajectory/trajectory.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <predicted_path_checker/collision_checker.hpp>
#include <predicted_path_checker/utils.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tier4_autoware_utils/geometry/geometry.hpp>
#include <tier4_autoware_utils/ros/debug_publisher.hpp>
#include <tier4_autoware_utils/ros/processing_time_publisher.hpp>
#include <tier4_autoware_utils/ros/self_pose_listener.hpp>
#include <tier4_autoware_utils/ros/transform_listener.hpp>
#include <tier4_external_api_msgs/srv/engage.hpp>

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

namespace predicted_path_checker
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

struct nodeParam
{
  double update_rate;
  double delay_time;
  double max_deceleration;
  double ego_nearest_dist_threshold;
  double ego_nearest_yaw_threshold;
};

enum class State {
  DRIVE = 0,
  EMERGENCY = 1,
  PAUSE = 2,
};

class PredictedPathCheckerNode : public rclcpp::Node
{
public:
  explicit PredictedPathCheckerNode(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::CallbackGroup::SharedPtr group_cli_;

  // Subscriber
  std::shared_ptr<tier4_autoware_utils::SelfPoseListener> self_pose_listener_;
  std::shared_ptr<tier4_autoware_utils::TransformListener> transform_listener_;
  tf2_ros::Buffer tf_buffer_{get_clock()};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_obstacle_pointcloud_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_reference_trajectory_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::Trajectory>::SharedPtr
    sub_predicted_trajectory_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  // Service
  rclcpp::Client<tier4_external_api_msgs::srv::Engage>::SharedPtr cli_external_stop_;

  // Data Buffer
  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::Twist::ConstSharedPtr current_twist_;
  sensor_msgs::msg::PointCloud2::SharedPtr obstacle_pointcloud_{nullptr};
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory_;
  autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory_;

  // Core
  std::unique_ptr<collision_checker::CollisionChecker> collision_checker_;

  // Variables
  State current_state_{State::DRIVE};
  vehicle_info_util::VehicleInfo vehicle_info_;
  State current_pause_state_{State::DRIVE};
  std::optional<bool> is_start_requested_;
  bool is_calling_set_pause_{false};

  // Callback
  void onObstaclePointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void onReferenceTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onPredictedTrajectory(const autoware_auto_planning_msgs::msg::Trajectory::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  // Publisher
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr virtual_wall_publisher_;
  std::shared_ptr<tier4_autoware_utils::ProcessingTimePublisher> time_publisher_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
  void initTimer(double period_s);

  // Functions
  bool isDataReady();
  bool isDataTimeout();
  bool isThereStopPointOnReferenceTrajectory(const geometry_msgs::msg::Pose & pose);
  void onTimer();
  void checkVehicleState(diagnostic_updater::DiagnosticStatusWrapper & stat);
  void updateState(const boost::optional<collision_checker::Output> & output);
  void publishVirtualWall(boost::optional<collision_checker::Output> & output);
  TrajectoryPoints trimTrajectoryFromSelfPose(
    const TrajectoryPoints & input, const Pose & self_pose);
  void sendRequest(bool make_stop_vehicle);

  // Parameter
  collision_checker::collisionCheckerParam collision_checker_param_;
  nodeParam node_param_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_;
};
}  // namespace predicted_path_checker

#endif  // PREDICTED_PATH_CHECKER__PREDICTED_PATH_CHECKER_NODE_HPP_
