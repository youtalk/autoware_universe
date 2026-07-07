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

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <boost/geometry/algorithms/correct.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

namespace autoware::collision_detector
{
namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// A 6 m x 6 m grid at 1 m resolution centered on base_link origin, all layers NaN (empty grid).
// Cell centers fall on the half-integers +/-0.5, +/-1.5, +/-2.5 on each axis.
grid_map::GridMap make_empty_grid()
{
  grid_map::GridMap grid({std::string(kPointCountLayer), std::string(kMaxHeightLayer)});
  grid.setFrameId(kGridFrameId);
  grid.setGeometry(grid_map::Length(6.0, 6.0), 1.0, grid_map::Position(0.0, 0.0));
  grid[kPointCountLayer].setConstant(kNaN);
  grid[kMaxHeightLayer].setConstant(kNaN);
  return grid;
}

// Axis-aligned ego footprint box [xmin, xmax] x [ymin, ymax] in base_link.
autoware_utils_geometry::Polygon2d make_ego_box(
  const double xmin, const double xmax, const double ymin, const double ymax)
{
  autoware_utils_geometry::Polygon2d ego;
  ego.outer().emplace_back(xmin, ymin);
  ego.outer().emplace_back(xmin, ymax);
  ego.outer().emplace_back(xmax, ymax);
  ego.outer().emplace_back(xmax, ymin);
  boost::geometry::correct(ego);
  return ego;
}

Obstacle make_obstacle(const double distance, const double x)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = 0.0;
  p.z = 0.0;
  return std::make_pair(distance, p);
}

grid_map_msgs::msg::GridMap to_message(const grid_map::GridMap & grid)
{
  return *grid_map::GridMapRosConverter::toMessage(grid);
}
}  // namespace

// --- nearest_obstacle_in_grid ------------------------------------------------------------------

TEST(NearestObstacleInGrid, ReturnsDistanceAndCellCenter)
{
  auto grid = make_empty_grid();
  // Occupy the cell centered at (2.5, 0.5): footprint x in [2.0, 3.0], y in [-0.0, 1.0].
  grid.atPosition(kPointCountLayer, grid_map::Position(2.5, 0.5)) = 3.0F;
  grid.atPosition(kMaxHeightLayer, grid_map::Position(2.5, 0.5)) = 1.0F;

  // Ego unit box x,y in [-0.5, 0.5]. y-ranges overlap, so the distance is the pure x-gap:
  // 2.0 (footprint left edge) - 0.5 (ego right edge) = 1.5. Hand-computed, not via bg::distance.
  const auto result = nearest_obstacle_in_grid(grid, make_ego_box(-0.5, 0.5, -0.5, 0.5));
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->first, 1.5, 1e-9);
  EXPECT_NEAR(result->second.x, 2.5, 1e-9);
  EXPECT_NEAR(result->second.y, 0.5, 1e-9);
  EXPECT_DOUBLE_EQ(result->second.z, 0.0);
}

TEST(NearestObstacleInGrid, EmptyGridReturnsNullopt)
{
  const auto grid = make_empty_grid();
  EXPECT_FALSE(nearest_obstacle_in_grid(grid, make_ego_box(-0.5, 0.5, -0.5, 0.5)).has_value());
}

TEST(NearestObstacleInGrid, AllNanHeartbeatReturnsNullopt)
{
  // A fresh but fully-NaN grid is the producer's alive heartbeat: it must read as "clear within
  // ROI" (nullopt), distinct from a missing/stale grid which the node short-circuits earlier.
  auto grid = make_empty_grid();
  // Explicitly set every cell NaN in both layers (heartbeat), then assert no obstacle.
  grid[kPointCountLayer].setConstant(kNaN);
  grid[kMaxHeightLayer].setConstant(kNaN);
  EXPECT_FALSE(nearest_obstacle_in_grid(grid, make_ego_box(-0.5, 0.5, -0.5, 0.5)).has_value());
}

TEST(NearestObstacleInGrid, GateRejectsZeroPointCount)
{
  auto grid = make_empty_grid();
  // Gate{1, 0}: min_point_count_cell == 1, so a cell with point_count 0 (== N-1) never qualifies.
  grid.atPosition(kPointCountLayer, grid_map::Position(2.5, 0.5)) = 0.0F;
  grid.atPosition(kMaxHeightLayer, grid_map::Position(2.5, 0.5)) = 1.0F;
  EXPECT_FALSE(nearest_obstacle_in_grid(grid, make_ego_box(-0.5, 0.5, -0.5, 0.5)).has_value());
}

TEST(NearestObstacleInGrid, GateAcceptsSinglePointCount)
{
  auto grid = make_empty_grid();
  // point_count 1 (== N) is the qualifying boundary for Gate{1, 0}.
  grid.atPosition(kPointCountLayer, grid_map::Position(2.5, 0.5)) = 1.0F;
  grid.atPosition(kMaxHeightLayer, grid_map::Position(2.5, 0.5)) = 1.0F;
  EXPECT_TRUE(nearest_obstacle_in_grid(grid, make_ego_box(-0.5, 0.5, -0.5, 0.5)).has_value());
}

TEST(NearestObstacleInGrid, GateRejectsMaxHeightBelowFloor)
{
  auto grid = make_empty_grid();
  // Gate{1, 0}: min_height == 0, so max_height just below 0 fails cell_qualifies.
  grid.atPosition(kPointCountLayer, grid_map::Position(2.5, 0.5)) = 5.0F;
  grid.atPosition(kMaxHeightLayer, grid_map::Position(2.5, 0.5)) = -0.001F;
  EXPECT_FALSE(nearest_obstacle_in_grid(grid, make_ego_box(-0.5, 0.5, -0.5, 0.5)).has_value());
}

TEST(NearestObstacleInGrid, GateAcceptsMaxHeightAtFloor)
{
  auto grid = make_empty_grid();
  // max_height exactly at the 0 floor qualifies (>= comparison).
  grid.atPosition(kPointCountLayer, grid_map::Position(2.5, 0.5)) = 5.0F;
  grid.atPosition(kMaxHeightLayer, grid_map::Position(2.5, 0.5)) = 0.0F;
  EXPECT_TRUE(nearest_obstacle_in_grid(grid, make_ego_box(-0.5, 0.5, -0.5, 0.5)).has_value());
}

TEST(NearestObstacleInGrid, CellTouchingEgoEdgeIsZeroDistance)
{
  auto grid = make_empty_grid();
  // Cell centered at (1.5, 0.0): footprint x in [1.0, 2.0]. Ego right edge is at x = 1.0, so the
  // footprint's left edge coincides with the ego edge => edge-aware distance is exactly 0.
  grid.atPosition(kPointCountLayer, grid_map::Position(1.5, 0.0)) = 2.0F;
  grid.atPosition(kMaxHeightLayer, grid_map::Position(1.5, 0.0)) = 1.0F;

  const auto result = nearest_obstacle_in_grid(grid, make_ego_box(-1.0, 1.0, -1.0, 1.0));
  ASSERT_TRUE(result.has_value());
  EXPECT_NEAR(result->first, 0.0, 1e-9);
}

// --- validate_obstacle_grid --------------------------------------------------------------------

TEST(ValidateObstacleGrid, ValidGridConverts)
{
  const auto grid = make_empty_grid();
  grid_map_msgs::msg::GridMap msg = to_message(grid);

  const auto validated = validate_obstacle_grid(msg);
  ASSERT_TRUE(validated.has_value());
  EXPECT_TRUE(validated->exists(kPointCountLayer));
  EXPECT_TRUE(validated->exists(kMaxHeightLayer));
}

TEST(ValidateObstacleGrid, WrongFrameRejected)
{
  const auto grid = make_empty_grid();
  grid_map_msgs::msg::GridMap msg = to_message(grid);
  msg.header.frame_id = "map";
  EXPECT_FALSE(validate_obstacle_grid(msg).has_value());
}

TEST(ValidateObstacleGrid, MissingPointCountLayerRejected)
{
  grid_map::GridMap grid({std::string(kMaxHeightLayer)});
  grid.setFrameId(kGridFrameId);
  grid.setGeometry(grid_map::Length(6.0, 6.0), 1.0, grid_map::Position(0.0, 0.0));
  grid[kMaxHeightLayer].setConstant(kNaN);
  grid_map_msgs::msg::GridMap msg = to_message(grid);
  EXPECT_FALSE(validate_obstacle_grid(msg).has_value());
}

TEST(ValidateObstacleGrid, MissingMaxHeightLayerRejected)
{
  grid_map::GridMap grid({std::string(kPointCountLayer)});
  grid.setFrameId(kGridFrameId);
  grid.setGeometry(grid_map::Length(6.0, 6.0), 1.0, grid_map::Position(0.0, 0.0));
  grid[kPointCountLayer].setConstant(kNaN);
  grid_map_msgs::msg::GridMap msg = to_message(grid);
  EXPECT_FALSE(validate_obstacle_grid(msg).has_value());
}

// --- nearest_of (pointcloud-vs-object merge tie-break) ------------------------------------------

TEST(NearestOf, BothNulloptReturnsNullopt)
{
  EXPECT_FALSE(nearest_of(std::nullopt, std::nullopt).has_value());
}

TEST(NearestOf, OnlyGridReturnsGrid)
{
  const auto result = nearest_of(make_obstacle(2.0, 10.0), std::nullopt);
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->first, 2.0);
  EXPECT_DOUBLE_EQ(result->second.x, 10.0);
}

TEST(NearestOf, OnlyObjectReturnsObject)
{
  const auto result = nearest_of(std::nullopt, make_obstacle(3.0, 20.0));
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->first, 3.0);
  EXPECT_DOUBLE_EQ(result->second.x, 20.0);
}

TEST(NearestOf, GridCloserWins)
{
  const auto result = nearest_of(make_obstacle(1.0, 10.0), make_obstacle(4.0, 20.0));
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->first, 1.0);
  EXPECT_DOUBLE_EQ(result->second.x, 10.0);
}

TEST(NearestOf, ObjectCloserWins)
{
  const auto result = nearest_of(make_obstacle(5.0, 10.0), make_obstacle(2.0, 20.0));
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->first, 2.0);
  EXPECT_DOUBLE_EQ(result->second.x, 20.0);
}

TEST(NearestOf, ExactTieObjectWins)
{
  // Strict-less comparison => on an exact tie the dynamic-object candidate wins, preserving the
  // pre-migration merge order. Positions disambiguate which candidate was returned.
  const auto result = nearest_of(make_obstacle(2.0, 10.0), make_obstacle(2.0, 20.0));
  ASSERT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result->first, 2.0);
  EXPECT_DOUBLE_EQ(result->second.x, 20.0);
}

// --- is_grid_stale -----------------------------------------------------------------------------
// All times are RCL_ROS_TIME (the default) so the rclcpp::Time subtraction matches the node's
// clock contract. now is fixed at 100.0 s; the stamp age is 100.0 - stamp.

TEST(IsGridStale, FreshGridIsNotStale)
{
  // age = 0.3 s < 0.5 s timeout => fresh.
  EXPECT_FALSE(is_grid_stale(rclcpp::Time(100, 0), rclcpp::Time(99, 700000000), 0.5));
}

TEST(IsGridStale, StaleGridIsStale)
{
  // age = 0.6 s > 0.5 s timeout => stale.
  EXPECT_TRUE(is_grid_stale(rclcpp::Time(100, 0), rclcpp::Time(99, 400000000), 0.5));
}

TEST(IsGridStale, ExactTimeoutBoundaryIsNotStale)
{
  // age = exactly 0.5 s == 0.5 s timeout => NOT stale (strict greater-than). 0.5 s is exactly
  // representable (5e8 ns), so this boundary is deterministic.
  EXPECT_FALSE(is_grid_stale(rclcpp::Time(100, 0), rclcpp::Time(99, 500000000), 0.5));
}

}  // namespace autoware::collision_detector

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
