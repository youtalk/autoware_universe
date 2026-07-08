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

#include "autoware/collision_detector/grid_query.hpp"

#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <optional>
#include <utility>

namespace autoware::collision_detector
{

bool is_grid_stale(const rclcpp::Time & now, const rclcpp::Time & stamp, const double timeout_sec)
{
  // Strict greater-than: an age of exactly timeout_sec is still fresh.
  return (now - stamp).seconds() > timeout_sec;
}

std::optional<grid_map::GridMap> validate_obstacle_grid(const grid_map_msgs::msg::GridMap & msg)
{
  // A grid published in the wrong frame is a wiring error: it cannot be re-framed cheaply the way a
  // point cloud can, so reject rather than silently reinterpret coordinates.
  if (msg.header.frame_id != kGridFrameId) {
    return std::nullopt;
  }
  grid_map::GridMap grid;
  if (!grid_map::GridMapRosConverter::fromMessage(msg, grid)) {
    return std::nullopt;
  }
  // Reject before any layer access so a missing layer can never become an out-of-bounds read.
  if (!grid.exists(kPointCountLayer) || !grid.exists(kMaxHeightLayer)) {
    return std::nullopt;
  }
  return grid;
}

std::optional<Obstacle> nearest_obstacle_in_grid(
  const grid_map::GridMap & grid, const autoware_utils_geometry::Polygon2d & ego_polygon)
{
  // Gate{1, 0}: single-return sensitivity plus a max_height >= 0 floor. That floor IS a height
  // gate: a cell whose returns all lie below base_link z=0 is dropped (the producer also crops z to
  // [-1, 3] m), whereas the removed per-point loop had no z filter and counted those sub-ground
  // far-field returns - so the grid is strictly more conservative there (decision-level parity
  // verified on real vehicle data). Fixed policy shared across the surround/collision consumers,
  // deliberately not exposed as a parameter.
  const auto nearest = autoware::obstacle_grid_utils::nearest_cell(grid, ego_polygon, {1, 0.0});
  if (!nearest) {
    return std::nullopt;
  }
  // Carry the winning cell CENTER (base_link, z = 0.0) as the obstacle position: honest 2D evidence
  // for the debug marker. nearest_cell's distance is edge-aware (to the cell footprint box).
  const auto position =
    autoware_utils::create_point(nearest->position.x(), nearest->position.y(), 0.0);
  return std::make_pair(nearest->distance, position);
}

std::optional<Obstacle> nearest_of(
  const std::optional<Obstacle> & grid_obstacle, const std::optional<Obstacle> & object_obstacle)
{
  if (!grid_obstacle && !object_obstacle) {
    return std::nullopt;
  }
  if (!grid_obstacle) {
    return object_obstacle;
  }
  if (!object_obstacle) {
    return grid_obstacle;
  }
  return grid_obstacle->first < object_obstacle->first ? grid_obstacle : object_obstacle;
}

}  // namespace autoware::collision_detector
