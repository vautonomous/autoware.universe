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

#ifndef GNSS_POSER__GNSS_POSER_CORE_HPP_
#define GNSS_POSER__GNSS_POSER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace transformation_origin_calibration
{
class TransformationOriginCalibration : public rclcpp::Node
{
public:
  explicit TransformationOriginCalibration(const rclcpp::NodeOptions & node_options);

private:
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr pub_transformation_origin_calibration_;
  geometry_msgs::msg::TransformStamped transformation_origin_calibration_;

    double x_;
    double y_;
    double z_;


};
}  // namespace gnss_poser

#endif  // GNSS_POSER__GNSS_POSER_CORE_HPP_
