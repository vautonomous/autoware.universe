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
 * Copyright 2019 Autoware Foundation. All rights reserved.
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
 *
 * Authors: Yukihiro Saito
 *
 */

#ifndef TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_
#define TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_

#include "rcl_interfaces/msg/set_parameters_result.hpp"

#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.hpp>
#include <lanelet2_extension/utility/query.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_auto_mapping_msgs/msg/had_map_bin.hpp>
#include <autoware_auto_perception_msgs/msg/traffic_light_roi_array.hpp>
#include <autoware_auto_planning_msgs/msg/had_map_route.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <image_geometry/pinhole_camera_model.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <set>
#include <vector>

namespace traffic_light
{

class TrafficLight
{
private:
  float x;
  float y;
  float z;
  float height;

  std::mutex mtx;

public:
  TrafficLight(float x = 0, float y = 0, float z = 0, float height = 0)
  {
    this->x = x;
    this->y = y;
    this->z = z;
    this->height = height;
  }

  void setX(float x)
  {
    mtx.lock();
    this->x = x;
    mtx.unlock();
  }

  void setY(float y)
  {
    mtx.lock();
    this->y = y;
    mtx.unlock();
  }

  void setZ(float z)
  {
    mtx.lock();
    this->z = z;
    mtx.unlock();
  }
  void setHeight(float height)
  {
    mtx.lock();
    this->height = height;
    mtx.unlock();
  }
  double getHeight() { return this->height; }

  lanelet::LineString3d getTrafficLight3d()
  {
    const double x_offset = 0.10;
    const double y_offset = 0.20;

    lanelet::LineString3d traffic_light_3d;
    traffic_light_3d.push_back(
      lanelet::Point3d(lanelet::utils::getId(), x - x_offset / 2, y + y_offset / 2, z));
    traffic_light_3d.push_back(
      lanelet::Point3d(lanelet::utils::getId(), x + x_offset / 2, y - y_offset / 2, z));
    traffic_light_3d.setAttribute("height", this->getHeight());
    return traffic_light_3d;
  }
};

class MapBasedDetector : public rclcpp::Node
{
public:
  explicit MapBasedDetector(const rclcpp::NodeOptions & node_options);

private:
  struct Config
  {
    double max_vibration_pitch;
    double max_vibration_yaw;
    double max_vibration_height;
    double max_vibration_width;
    double max_vibration_depth;
  };

  struct IdLessThan
  {
    bool operator()(
      const lanelet::ConstLineString3d & left, const lanelet::ConstLineString3d & right) const
    {
      return left.id() < right.id();
    }
  };

private:
  rclcpp::Subscription<autoware_auto_mapping_msgs::msg::HADMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<autoware_auto_planning_msgs::msg::HADMapRoute>::SharedPtr route_sub_;

  rclcpp::Publisher<autoware_auto_perception_msgs::msg::TrafficLightRoiArray>::SharedPtr roi_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr viz_pub_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  using TrafficLightSet = std::set<lanelet::ConstLineString3d, IdLessThan>;

  std::shared_ptr<TrafficLightSet> all_traffic_lights_ptr_;
  std::shared_ptr<TrafficLightSet> route_traffic_lights_ptr_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  Config config_;

  double scale_start_distance_;
  double scale_factor_;

  bool overwrite_tl_;
  double tl_x_;
  double tl_y_;
  double tl_z_;
  double tl_height_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_fake_tl_;

  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  rclcpp::TimerBase::SharedPtr timer_;
  lanelet::LineString3d traffic_light_3d_;

  std::shared_ptr<traffic_light::TrafficLight> tl_ptr_;

  void mapCallback(const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr input_msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr input_msg);
  void routeCallback(const autoware_auto_planning_msgs::msg::HADMapRoute::ConstSharedPtr input_msg);
  void getVisibleTrafficLights(
    const TrafficLightSet & all_traffic_lights, const geometry_msgs::msg::Pose & camera_pose,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    std::vector<lanelet::ConstLineString3d> & visible_traffic_lights);
  bool isInDistanceRange(
    const geometry_msgs::msg::Point & tl_point, const geometry_msgs::msg::Point & camera_point,
    const double max_distance_range) const;
  bool isInAngleRange(
    const double & tl_yaw, const double & camera_yaw, const double max_angle_range) const;
  bool isInImageFrame(
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    const geometry_msgs::msg::Point & point) const;
  bool getTrafficLightRoi(
    const geometry_msgs::msg::Pose & camera_pose,
    const image_geometry::PinholeCameraModel & pinhole_camera_model,
    const lanelet::ConstLineString3d traffic_light, const Config & config,
    autoware_auto_perception_msgs::msg::TrafficLightRoi & tl_roi);
  void publishVisibleTrafficLights(
    const geometry_msgs::msg::PoseStamped camera_pose_stamped,
    const std::vector<lanelet::ConstLineString3d> & visible_traffic_lights,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub);

  void tlFakePosePublisher(const geometry_msgs::msg::PoseStamped camera_pose_stamped);
};
}  // namespace traffic_light
#endif  // TRAFFIC_LIGHT_MAP_BASED_DETECTOR__NODE_HPP_
