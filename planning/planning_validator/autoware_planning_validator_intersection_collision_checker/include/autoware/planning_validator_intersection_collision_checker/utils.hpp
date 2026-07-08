// Copyright 2025 TIER IV, Inc.
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

#ifndef AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__UTILS_HPP_
#define AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__UTILS_HPP_

#include "autoware/planning_validator/types.hpp"
#include "autoware/planning_validator_intersection_collision_checker/types.hpp"

#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <lanelet2_core/Forward.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BoundingBox.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

namespace autoware::planning_validator::collision_checker_utils
{

void set_trajectory_lanelets(
  const TrajectoryPoints & trajectory_points, const RouteHandler & route_handler,
  const geometry_msgs::msg::Pose & ego_pose, EgoLanelets & lanelets);

void set_right_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const std::shared_ptr<PlanningValidatorContext> & context,
  const intersection_collision_checker_node::Params & params, const EgoLanelets & lanelets,
  TargetLaneletsMap & target_lanelets,
  const double time_horizon = std::numeric_limits<double>::max());

void set_left_turn_target_lanelets(
  const EgoTrajectory & ego_traj, const std::shared_ptr<PlanningValidatorContext> & context,
  const intersection_collision_checker_node::Params & params, const EgoLanelets & lanelets,
  TargetLaneletsMap & target_lanelets,
  const double time_horizon = std::numeric_limits<double>::max());

MarkerArray get_lanelets_marker_array(const DebugData & debug_data);
MarkerArray get_objects_marker_array(const DebugData & debug_data);

/// Outcome of decoding + validating the obstacle-grid intake contract. Any non-kOk value is a
/// contract violation that the caller must treat as data-unavailable (never as a spurious "clear").
enum class GridContractStatus { kOk, kWrongFrame, kUndecodable, kMissingLayer };

/// Decodes and validates the obstacle-grid intake contract (base_link frame; point_count /
/// min_height / low_max_height layers present). On kOk, @p out_grid holds the decoded grid with the
/// default start index. Pure and node-free (no clock / tf / logger), so a contract failure is
/// always distinguishable from a decoded-but-empty grid (a genuine clear); the caller logs and
/// abstains.
GridContractStatus decode_obstacle_grid(
  const grid_map_msgs::msg::GridMap & msg, grid_map::GridMap & out_grid);

/// Returns the four corner points (in the grid's own frame, i.e. base_link) of every obstacle-grid
/// cell that qualifies as a real in-band obstacle. A cell qualifies iff:
///   - it has at least @p min_point_count_cell raw returns (point_count layer, pre-voxel count),
///   AND
///   - its tallest in-band return low_max_height is finite and >= @p height_floor
///     (rejects ground residue and, because the floor is on low_max_height rather than max_height,
///     rejects an overhead gantry that merely shares a cell with ground residue), AND
///   - its lowest return min_height is finite and <= @p z_band_top
///     (rejects a purely-overhead cell whose lowest return is already above the ego height band).
/// The emitted z of every corner is the cell's low_max_height. Corners (not the center) are emitted
/// so that per-lanelet polygon membership stays edge-conservative: a cell whose center is just
/// outside a target lanelet but whose footprint intrudes is still retained.
/// This function is pure and transform-free; the caller applies the single base_link->map
/// transform.
std::vector<geometry_msgs::msg::Point> qualifying_cell_corners(
  const grid_map::GridMap & grid, std::uint32_t min_point_count_cell, double height_floor,
  double z_band_top);

}  // namespace autoware::planning_validator::collision_checker_utils

#endif  // AUTOWARE__PLANNING_VALIDATOR_INTERSECTION_COLLISION_CHECKER__UTILS_HPP_
