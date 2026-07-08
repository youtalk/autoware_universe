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

#include "autoware/planning_validator_intersection_collision_checker/utils.hpp"

#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator::collision_checker_utils
{
namespace
{
// Builds a 4x4 base_link grid message (all layers empty/NaN) with the requested frame and layer
// set, so decode_obstacle_grid() can be exercised without a node / TF / clock.
grid_map_msgs::msg::GridMap make_msg(
  const std::string & frame_id,
  const std::vector<std::string> & layers = {"point_count", "min_height", "low_max_height"})
{
  grid_map::GridMap grid(layers);
  grid.setGeometry(grid_map::Length(2.0, 2.0), 0.5, grid_map::Position(0.0, 0.0));
  for (const auto & layer : layers) {
    grid[layer].setConstant(std::numeric_limits<float>::quiet_NaN());
  }
  grid_map_msgs::msg::GridMap msg = *grid_map::GridMapRosConverter::toMessage(grid);
  msg.header.frame_id = frame_id;
  return msg;
}
}  // namespace

// A grid published in a frame other than base_link is a contract violation: it must read as
// data-unavailable (kWrongFrame), never fold into an indistinguishable "empty" that the caller
// would treat as a clear intersection.
TEST(ObstacleGridContract, WrongFrameIsUnavailable)
{
  grid_map::GridMap decoded;
  EXPECT_EQ(
    decode_obstacle_grid(make_msg("velodyne_top"), decoded), GridContractStatus::kWrongFrame);
}

// A base_link grid missing a required layer (here low_max_height) is unavailable, not clear.
TEST(ObstacleGridContract, MissingLayerIsUnavailable)
{
  grid_map::GridMap decoded;
  EXPECT_EQ(
    decode_obstacle_grid(make_msg("base_link", {"point_count", "min_height"}), decoded),
    GridContractStatus::kMissingLayer);
}

// A well-formed base_link grid with no populated cell decodes cleanly (kOk) AND yields no
// qualifying corners: "empty because clear" is distinct from "empty because broken", the
// distinction the fail-open fix relies on.
TEST(ObstacleGridContract, ValidEmptyGridIsClearNotUnavailable)
{
  grid_map::GridMap decoded;
  ASSERT_EQ(decode_obstacle_grid(make_msg("base_link"), decoded), GridContractStatus::kOk);
  EXPECT_TRUE(
    qualifying_cell_corners(decoded, 1U, /*height_floor*/ 0.5, /*z_band_top*/ 2.5).empty());
}
}  // namespace autoware::planning_validator::collision_checker_utils
