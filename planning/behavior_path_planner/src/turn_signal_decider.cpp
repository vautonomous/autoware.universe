// Copyright 2021 Tier IV, Inc.
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

#include "behavior_path_planner/turn_signal_decider.hpp"

#include "behavior_path_planner/utilities.hpp"

#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <limits>
#include <string>
#include <utility>

namespace behavior_path_planner
{
TurnIndicatorsCommand TurnSignalDecider::getTurnSignal(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const RouteHandler & route_handler, const TurnIndicatorsCommand & turn_signal_plan,
  const double plan_distance) const
{
  auto turn_signal = turn_signal_plan;

  // If the distance to intersection is nearer than path change point,
  // use turn signal for turning at the intersection
  const auto intersection_result =
    getIntersectionTurnSignal(path, current_pose, current_seg_idx, route_handler);
  const auto intersection_turn_signal = intersection_result.first;
  const auto intersection_distance = intersection_result.second;

  if (
    intersection_distance < plan_distance ||
    turn_signal_plan.command == TurnIndicatorsCommand::NO_COMMAND ||
    turn_signal_plan.command == TurnIndicatorsCommand::DISABLE) {
    turn_signal.command = intersection_turn_signal.command;
  }

  // If the distance to goal point is nearer than path change point,
  // use turn signal for stopping at the goal point
  const auto approaching_goal_point_result = getGoalPoseTurnSignal(path, current_pose, route_handler);
  const auto approaching_goal_point_turn_signal = approaching_goal_point_result.first;
  const auto approaching_goal_point_distance = approaching_goal_point_result.second;

  if (
      approaching_goal_point_distance < plan_distance &&
      (turn_signal.command == TurnIndicatorsCommand::NO_COMMAND ||
      turn_signal.command == TurnIndicatorsCommand::DISABLE)) {
      turn_signal.command = approaching_goal_point_turn_signal.command;
  }

    // If the distance to goal point is nearer than path change point,
    // use turn signal for stopping at the goal point
    const auto departure_maneuver_result = getDepartureTurnSignal(path, current_pose, current_seg_idx);
    const auto departure_maneuver_turn_signal = departure_maneuver_result.first;
    const auto departure_maneuver_distance = departure_maneuver_result.second;

    if (
       departure_maneuver_distance < plan_distance &&
       (turn_signal.command == TurnIndicatorsCommand::NO_COMMAND ||
       turn_signal.command == TurnIndicatorsCommand::DISABLE)) {
       turn_signal.command = departure_maneuver_turn_signal.command;
    }

  return turn_signal;
}

std::pair<TurnIndicatorsCommand, double> TurnSignalDecider::getIntersectionTurnSignal(
  const PathWithLaneId & path, const Pose & current_pose, const size_t current_seg_idx,
  const RouteHandler & route_handler) const
{
  TurnIndicatorsCommand turn_signal{};
  turn_signal.command = TurnIndicatorsCommand::DISABLE;
  double distance = std::numeric_limits<double>::max();
  auto clock{rclcpp::Clock{RCL_ROS_TIME}};

  if (path.points.empty()) {
    return std::make_pair(turn_signal, distance);
  }

  // Get frenet coordinate of current_pose on path
  const auto vehicle_pose_frenet =
    util::convertToFrenetCoordinate3d(path, current_pose.position, current_seg_idx);

  // Get nearest intersection and decide turn signal
  double accumulated_distance = 0;

  auto prev_point = path.points.front();
  auto lane_attribute = std::string("none");
  for (const auto & path_point : path.points) {
    const double path_point_distance =
      tier4_autoware_utils::calcDistance3d(prev_point.point, path_point.point);
    accumulated_distance += path_point_distance;
    prev_point = path_point;
    const double distance_from_vehicle_front =
      accumulated_distance - vehicle_pose_frenet.length - base_link2front_;
    if (distance_from_vehicle_front > intersection_search_distance_) {
      if (turn_signal.command == TurnIndicatorsCommand::DISABLE) {
        distance = std::numeric_limits<double>::max();
      }
      return std::make_pair(turn_signal, distance);
    }
    // TODO(Horibe): Route Handler should be a library.
    for (const auto & lane : route_handler.getLaneletsFromIds(path_point.lane_ids)) {
      // judgement of lighting of turn_signal
      bool lighting_turn_signal = false;
      if (lane.attributeOr("turn_direction", std::string("none")) != lane_attribute) {
        if (
          distance_from_vehicle_front >= 0.0 &&
          distance_from_vehicle_front <
            lane.attributeOr("turn_signal_distance", intersection_search_distance_) &&
          path_point_distance > 0.0) {
          lighting_turn_signal = true;
        }
      } else {
        if (
          lane.hasAttribute("turn_direction") &&
          distance_from_vehicle_front < path_point_distance && distance_from_vehicle_front > 0) {
          lighting_turn_signal = true;
        }
      }
      lane_attribute = lane.attributeOr("turn_direction", std::string("none"));

      if (lighting_turn_signal) {
        if (lane_attribute == std::string("left")) {
          turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
          distance = distance_from_vehicle_front;
        } else if (lane_attribute == std::string("right")) {
          turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
          distance = distance_from_vehicle_front;
        } else if (lane_attribute == std::string("straight") || lane_attribute == std::string("none")) {
          turn_signal.command = TurnIndicatorsCommand::DISABLE;
        }
      }
    }
  }
  if (turn_signal.command == TurnIndicatorsCommand::DISABLE) {
    distance = std::numeric_limits<double>::max();
  }
  return std::make_pair(turn_signal, distance);
}

std::pair<TurnIndicatorsCommand, double>
TurnSignalDecider::getGoalPoseTurnSignal(const PathWithLaneId &path, const Pose &current_pose,
                                         const RouteHandler &route_handler) const {

    // This function is designed for right-handed traffic, it will automatically enable Right turn signal
    // to approach goal points
    TurnIndicatorsCommand turn_signal{};
    turn_signal.command = TurnIndicatorsCommand::DISABLE;
    double distance = std::numeric_limits<double>::max();
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    if (path.points.empty()) {
        return std::make_pair(turn_signal, distance);
    }

    auto goal_pose = route_handler.getGoalPose();

    const double to_goal_point_distance =
            tier4_autoware_utils::calcDistance3d(current_pose.position, goal_pose.position);

    const double distance_from_vehicle_front =
            to_goal_point_distance - base_link2front_;

    // TODO: Define this threshold as a parameter
    if(distance_from_vehicle_front <= 30.0){
        turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
        distance = distance_from_vehicle_front;
    }else{
        turn_signal.command = TurnIndicatorsCommand::DISABLE;
        distance = std::numeric_limits<double>::max();
    }
    return std::make_pair(turn_signal, distance);
    }

std::pair<TurnIndicatorsCommand, double>
TurnSignalDecider::getDepartureTurnSignal(const PathWithLaneId &path, const Pose &current_pose,
                                          const size_t current_seg_idx) const {

    // This function is designed for right-handed traffic, it will activate required signal when merging into
    // the path
    TurnIndicatorsCommand turn_signal{};
    turn_signal.command = TurnIndicatorsCommand::DISABLE;
    double distance = std::numeric_limits<double>::max();
    auto clock{rclcpp::Clock{RCL_ROS_TIME}};
    if (path.points.empty()) {
        return std::make_pair(turn_signal, distance);
    }

    // Get frenet coordinate of current_pose on path
    const auto vehicle_pose_frenet =
            util::convertToFrenetCoordinate3d(path, current_pose.position, current_seg_idx);

    // Define distance as a dummy value since we won't need it
//    if(vehicle_pose_frenet.distance >= 1.0){
//        turn_signal.command = TurnIndicatorsCommand::ENABLE_RIGHT;
//        distance = 0;
//    }
    if(vehicle_pose_frenet.distance <= -1.0){
        turn_signal.command = TurnIndicatorsCommand::ENABLE_LEFT;
        distance = 0;
    }
    else{
        turn_signal.command = TurnIndicatorsCommand::DISABLE;
        distance = std::numeric_limits<double>::max();
    }
    return std::make_pair(turn_signal, distance);
}
}  // namespace behavior_path_planner
