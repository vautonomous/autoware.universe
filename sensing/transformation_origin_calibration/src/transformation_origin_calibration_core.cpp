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

#include "transformation_origin_calibration/transformation_origin_calibration_core.hpp"
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <Eigen/Geometry>
#include <tf2_eigen/tf2_eigen.h>

namespace transformation_origin_calibration
{
    TransformationOriginCalibration::TransformationOriginCalibration(const rclcpp::NodeOptions & node_options)
: rclcpp::Node("transformation_origin_calibration", node_options),
  x_(declare_parameter("x", 0.0)),
  y_(declare_parameter("y", 0.0)),
  z_(declare_parameter("z", 0.0))
{
  pub_transformation_origin_calibration_ = this->create_publisher<geometry_msgs::msg::TransformStamped>(
    "transformation_origin_calibration", rclcpp::QoS{1});
  Eigen::Affine3d affine_origin_calibration;
  affine_origin_calibration.translation() << x_, y_, z_;

  geometry_msgs::msg::TransformStamped transform_stamped_origin_calibration = tf2::eigenToTransform(affine_origin_calibration);

  pub_transformation_origin_calibration_->publish(transform_stamped_origin_calibration);
}

}  // namespace transformation_origin_calibration

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(transformation_origin_calibration::TransformationOriginCalibration)
