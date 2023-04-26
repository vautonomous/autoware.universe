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

#include "behavior_path_planner/scene_module/lane_following/lane_following_module.hpp"

#include "behavior_path_planner/path_utilities.hpp"
#include "behavior_path_planner/utilities.hpp"

#include <lanelet2_extension/utility/utilities.hpp>
#include <motion_utils/motion_utils.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <memory>
#include <string>

namespace behavior_path_planner
{
LaneFollowingModule::LaneFollowingModule(
  const std::string & name, rclcpp::Node & node, const LaneFollowingParameters & parameters)
: SceneModuleInterface{name, node}, parameters_{parameters}
{
  initParam();
}

void LaneFollowingModule::initParam()
{
  clearWaitingApproval();  // no need approval
}

bool LaneFollowingModule::isExecutionRequested() const { return true; }

bool LaneFollowingModule::isExecutionReady() const { return true; }

BT::NodeStatus LaneFollowingModule::updateState()
{
  current_state_ = BT::NodeStatus::SUCCESS;
  return current_state_;
}

BehaviorModuleOutput LaneFollowingModule::plan()
{
  BehaviorModuleOutput output;

  auto ref_path = getReferencePath();

  if (!isSafePath(ref_path)) {
    insertZeroVelocity(ref_path);
  }

  output.path = std::make_shared<PathWithLaneId>(ref_path);

  return output;
}
CandidateOutput LaneFollowingModule::planCandidate() const
{
  return CandidateOutput(getReferencePath());
}
void LaneFollowingModule::onEntry()
{
  initParam();
  current_state_ = BT::NodeStatus::RUNNING;
  RCLCPP_DEBUG(getLogger(), "LANE_FOLLOWING onEntry");
}
void LaneFollowingModule::onExit()
{
  initParam();
  current_state_ = BT::NodeStatus::IDLE;
  RCLCPP_DEBUG(getLogger(), "LANE_FOLLOWING onExit");
}

void LaneFollowingModule::setParameters(const LaneFollowingParameters & parameters)
{
  parameters_ = parameters;
}

PathWithLaneId LaneFollowingModule::getReferencePath() const
{
  PathWithLaneId reference_path{};

  const auto & route_handler = planner_data_->route_handler;
  const auto current_pose = planner_data_->self_pose->pose;
  const auto p = planner_data_->parameters;

  // Set header
  reference_path.header = route_handler->getRouteHeader();

  lanelet::ConstLanelet current_lane;
  if (!planner_data_->route_handler->getClosestLaneletWithinRoute(current_pose, &current_lane)) {
    RCLCPP_ERROR_THROTTLE(
      getLogger(), *clock_, 5000, "failed to find closest lanelet within route!!!");
    return reference_path;  // TODO(Horibe)
  }

  // For current_lanes with desired length
  lanelet::ConstLanelets current_lanes = planner_data_->route_handler->getLaneletSequence(
    current_lane, current_pose, p.backward_path_length, p.forward_path_length);

  if (current_lanes.empty()) {
    return reference_path;
  }

  // calculate path with backward margin to avoid end points' instability by spline interpolation
  constexpr double extra_margin = 10.0;
  const double backward_length =
    std::max(p.backward_path_length, p.backward_path_length + extra_margin);
  const auto current_lanes_with_backward_margin =
    util::calcLaneAroundPose(route_handler, current_pose, p.forward_path_length, backward_length);
  reference_path = util::getCenterLinePath(
    *route_handler, current_lanes_with_backward_margin, current_pose, backward_length,
    p.forward_path_length, p);

  // clip backward length
  const size_t current_seg_idx = findEgoSegmentIndex(reference_path.points);
  util::clipPathLength(
    reference_path, current_seg_idx, p.forward_path_length, p.backward_path_length);

  {
    double optional_lengths{0.0};
    const auto isInIntersection = util::checkLaneIsInIntersection(
      *route_handler, reference_path, current_lanes, p, optional_lengths);

    if (isInIntersection) {
      reference_path = util::getCenterLinePath(
        *route_handler, current_lanes, current_pose, p.backward_path_length, p.forward_path_length,
        p, optional_lengths);
    }

    // buffer for min_lane_change_length
    const double buffer = p.backward_length_buffer_for_end_of_lane + optional_lengths;
    const int num_lane_change =
      std::abs(route_handler->getNumLaneToPreferredLane(current_lanes.back()));
    const double lane_change_buffer = num_lane_change * (p.minimum_lane_change_length + buffer);

    reference_path = util::setDecelerationVelocity(
      *route_handler, reference_path, current_lanes, parameters_.lane_change_prepare_duration,
      lane_change_buffer);
  }

  if (parameters_.expand_drivable_area) {
    lanelet::ConstLanelets expand_lanes{};
    for (const auto & current_lane : current_lanes) {
      const std::string r_type =
        current_lane.rightBound().attributeOr(lanelet::AttributeName::Type, "none");
      const std::string l_type =
        current_lane.leftBound().attributeOr(lanelet::AttributeName::Type, "none");

      const double r_offset =
        r_type.compare("road_border") != 0 ? -parameters_.right_bound_offset : 0.0;
      const double l_offset =
        l_type.compare("road_border") != 0 ? parameters_.left_bound_offset : 0.0;

      expand_lanes.push_back(lanelet::utils::getExpandedLanelet(current_lane, l_offset, r_offset));
    }

    current_lanes = expand_lanes;
  }

  reference_path.drivable_area = util::generateDrivableArea(
    reference_path, current_lanes, p.drivable_area_resolution, p.vehicle_length, planner_data_);

  return reference_path;
}

bool LaneFollowingModule::isSafePath(const PathWithLaneId & path) const
{
  if (path.points.empty()) {
    return false;
  }

  const auto & p = parameters_;

  if (!p.enable_safety_check) {
    return true;  // if safety check is disabled, it always return safe.
  }

  const auto & ego_pose = planner_data_->self_pose->pose;

  // Calc lateral deviation from path to ego
  const auto ego_closest_index = motion_utils::findNearestIndex(path.points, ego_pose.position);
  const auto ego_closest_pose = path.points.at(ego_closest_index).point.pose;
  const auto lateral_dev =
    tier4_autoware_utils::calcLateralDeviation(ego_closest_pose, ego_pose.position);

  const auto & ego_speed = planner_data_->self_odometry->twist.twist.linear.x;

  if (
    ego_speed > p.threshold_speed_ego_is_stopped ||
    std::abs(lateral_dev) < p.ego_path_lateral_dev_to_execute) {
    return true;
  }

  const auto & rh = planner_data_->route_handler;

  lanelet::ConstLanelet current_lane;
  if (!rh->getClosestLaneletWithinRoute(ego_pose, &current_lane)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("behavior_path_planner").get_child("lane_following"),
      "failed to find closest lanelet within route!!!");
    return false;
  }

  const auto & backward_check_distance = p.safety_check_backward_distance;

  const auto ego_succeeding_lanes =
    rh->getLaneletSequence(current_lane, ego_pose, backward_check_distance, 10.0);

  const auto moving_objects = util::filterObjectsByVelocity(
    *planner_data_->dynamic_object, p.threshold_speed_object_is_stopped, 50.0);

  const auto related_objects_index =
    util::filterObjectIndicesByLanelets(moving_objects, ego_succeeding_lanes);

  // clip path
  PathWithLaneId clipped_path;
  for (size_t j = ego_closest_index; j < path.points.size(); ++j) {
    const auto & path_point = path.points.at(j);
    clipped_path.points.push_back(path_point);
  }

  for (const auto & i : related_objects_index) {
    const auto & object = moving_objects.objects.at(i);

    const auto distance = calcLongitudinalByClosestFootprint(path, object, ego_pose.position);

    if (distance > 10.0) continue;

    // check for collision between object predicted path and ego path
    if (util::isCollisionPredictedObjectPath(object, clipped_path, ego_succeeding_lanes)) {
      return false;
    }
  }
  return true;
}

double LaneFollowingModule::calcLongitudinalByClosestFootprint(
  const PathWithLaneId & path, const PredictedObject & object, const Point & ego_pos)
{
  tier4_autoware_utils::Polygon2d object_poly{};
  util::calcObjectPolygon(object, &object_poly);

  const double distance = motion_utils::calcSignedArcLength(
    path.points, ego_pos, object.kinematics.initial_pose_with_covariance.pose.position);
  double min_distance = distance;
  double max_distance = distance;
  for (const auto & p : object_poly.outer()) {
    const auto point = tier4_autoware_utils::createPoint(p.x(), p.y(), 0.0);
    const double arc_length = motion_utils::calcSignedArcLength(path.points, ego_pos, point);
    min_distance = std::min(min_distance, arc_length);
    max_distance = std::max(max_distance, arc_length);
  }
  return min_distance;
}

void LaneFollowingModule::insertZeroVelocity(PathWithLaneId & path) const
{
  for (auto & point : path.points) {
    point.point.longitudinal_velocity_mps = 0.0;
  }
}

}  // namespace behavior_path_planner
