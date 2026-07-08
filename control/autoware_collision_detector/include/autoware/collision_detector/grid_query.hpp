// Copyright 2026 The Autoware Contributors
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

#ifndef AUTOWARE__COLLISION_DETECTOR__GRID_QUERY_HPP_
#define AUTOWARE__COLLISION_DETECTOR__GRID_QUERY_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <rclcpp/time.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

#include <optional>
#include <utility>

namespace autoware::collision_detector
{
/// (distance-to-ego, position) pair shared by every obstacle source (grid, dynamic object).
using Obstacle = std::pair<double /* distance */, geometry_msgs::msg::Point>;

/// Required grid frame (the producer publishes in base_link and a grid cannot be re-framed
/// cheaply).
inline constexpr char kGridFrameId[] = "base_link";
/// The two layers this 2D consumer's Gate{1, 0} reads (validate exactly what we use).
inline constexpr char kPointCountLayer[] = "point_count";
inline constexpr char kMaxHeightLayer[] = "max_height";

/// True when the grid stamp is older than timeout_sec relative to now. The polling subscriber
/// returns the last received grid forever and the producer stays silent on its failure paths, so a
/// stale grid must read as "unavailable" (diagnostic held), never as "clear". Boundary: an age of
/// exactly timeout_sec is NOT stale (strict greater-than). Both times must share a clock type
/// (RCL_ROS_TIME here); rclcpp::Time subtraction throws otherwise, matching the caller's contract.
bool is_grid_stale(const rclcpp::Time & now, const rclcpp::Time & stamp, double timeout_sec);

/// Validate the obstacle-grid contract and convert the message to a grid_map::GridMap.
/// Returns std::nullopt on ANY contract violation (frame != base_link, unconvertible message, a
/// missing required layer); the caller treats that as "grid unavailable", never as "clear".
/// Logging is left to the caller (this function is pure so it stays unit-testable). A valid but
/// all-NaN grid (the producer's alive heartbeat) converts successfully and is NOT a violation.
std::optional<grid_map::GridMap> validate_obstacle_grid(const grid_map_msgs::msg::GridMap & msg);

/// Nearest qualifying-cell obstacle for the ego polygon, using the shared 2D Gate{1, 0} query
/// (any cell with >= 1 return and max_height >= 0 qualifies; no overhead-structure discrimination).
/// The max_height >= 0 floor IS a height gate: a cell whose returns all lie below base_link z=0 is
/// dropped (the producer also crops z to [-1, 3] m), whereas the removed per-point loop had no z
/// filter and counted those sub-ground far-field returns - so the grid is strictly more
/// conservative there (decision-level parity verified on real vehicle data). The returned Obstacle
/// carries the winning cell CENTER as its position (z = 0.0, honest 2D evidence) for the debug
/// marker; the distance is edge-aware (to the cell footprint box). Returns std::nullopt when no
/// cell qualifies within the ROI ("clear within ROI").
std::optional<Obstacle> nearest_obstacle_in_grid(
  const grid_map::GridMap & grid, const autoware_utils_geometry::Polygon2d & ego_polygon);

/// Merge two obstacle candidates by minimum distance. On an exact distance tie the second argument
/// wins (strict-less comparison), matching the pre-migration pointcloud-vs-object merge order.
/// std::nullopt inputs mean "that source found nothing"; nullopt out means neither source hit.
std::optional<Obstacle> nearest_of(
  const std::optional<Obstacle> & grid_obstacle, const std::optional<Obstacle> & object_obstacle);

}  // namespace autoware::collision_detector

#endif  // AUTOWARE__COLLISION_DETECTOR__GRID_QUERY_HPP_
