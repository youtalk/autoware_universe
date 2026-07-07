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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::planning_validator::collision_checker_utils
{
namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kResolution = 0.5;  // -> half cell = 0.25
constexpr double kFloor = 0.5;       // low_max_height floor (pointcloud.min_height)
constexpr double kZBandTop = 2.5;    // vehicle_height + height_buffer

// A 4x4 grid centred at the origin (span [-1, 1] on each axis), all layers NaN (empty).
grid_map::GridMap make_grid(
  const std::vector<std::string> & layers = {"point_count", "min_height", "low_max_height"})
{
  grid_map::GridMap grid(layers);
  grid.setGeometry(grid_map::Length(2.0, 2.0), kResolution, grid_map::Position(0.0, 0.0));
  for (const auto & layer : layers) {
    grid[layer].setConstant(kNaN);
  }
  return grid;
}

void set_cell(
  grid_map::GridMap & grid, const grid_map::Index & idx, const float count, const float min_height,
  const float low_max_height)
{
  grid.at("point_count", idx) = count;
  grid.at("min_height", idx) = min_height;
  grid.at("low_max_height", idx) = low_max_height;
}
}  // namespace

// A single in-band cell qualifies; exactly four corners are emitted at the cell's footprint
// corners, each at z == low_max_height. Oracles (half cell = 0.25, z = 1.5) are independent
// literals.
TEST(QualifyingCellCorners, SingleCellEmitsFourCornersAtLowMaxHeight)
{
  auto grid = make_grid();
  const grid_map::Index idx(1, 1);
  set_cell(grid, idx, 5.0F, 0.3F, 1.5F);
  grid_map::Position center;
  grid.getPosition(idx, center);

  const auto corners = qualifying_cell_corners(grid, 1U, kFloor, kZBandTop);

  ASSERT_EQ(corners.size(), 4U);
  for (const auto & c : corners) {
    EXPECT_DOUBLE_EQ(c.z, 1.5);
    EXPECT_DOUBLE_EQ(std::abs(c.x - center.x()), 0.25);
    EXPECT_DOUBLE_EQ(std::abs(c.y - center.y()), 0.25);
  }
}

// Two spatially separated qualifying cells each emit their own four corners (per-cell semantics; no
// clustering merges or drops them).
TEST(QualifyingCellCorners, TwoSeparatedCellsEmitEightCorners)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(0, 0), 3.0F, 0.3F, 1.0F);
  set_cell(grid, grid_map::Index(3, 3), 3.0F, 0.3F, 1.0F);

  const auto corners = qualifying_cell_corners(grid, 1U, kFloor, kZBandTop);

  EXPECT_EQ(corners.size(), 8U);
}

// low_max_height just below the floor rejects the cell.
TEST(QualifyingCellCorners, BelowFloorRejected)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 5.0F, 0.3F, static_cast<float>(kFloor) - 0.05F);

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// low_max_height exactly at the floor qualifies (the comparison is >=).
TEST(QualifyingCellCorners, FloorBoundaryInclusive)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 5.0F, 0.3F, static_cast<float>(kFloor));

  EXPECT_EQ(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).size(), 4U);
}

// The floor is applied to low_max_height, not max_height: a cell holding ground residue
// (low_max_height below the floor) is rejected even though taller (overhead) returns exist.
TEST(QualifyingCellCorners, GroundResidueUnderOverheadRejected)
{
  auto grid = make_grid();
  // min_height low (ground), low_max_height below floor: the tallest in-band return is ground.
  set_cell(grid, grid_map::Index(1, 1), 20.0F, 0.05F, 0.05F);

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// A purely overhead cell (lowest return above the z-band top) is rejected by the ceiling check.
TEST(QualifyingCellCorners, OverheadOnlyRejectedByCeiling)
{
  auto grid = make_grid();
  // min_height above the band top; low_max_height finite and above the floor.
  set_cell(grid, grid_map::Index(1, 1), 8.0F, 3.0F, 3.2F);

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// min_height exactly at the band top qualifies (the comparison is <=).
TEST(QualifyingCellCorners, CeilingBoundaryInclusive)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 8.0F, static_cast<float>(kZBandTop), 2.6F);

  EXPECT_EQ(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).size(), 4U);
}

// Density gate boundary: count == N-1 is rejected, count == N qualifies.
TEST(QualifyingCellCorners, DensityGateBoundary)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 2.0F, 0.3F, 1.5F);
  EXPECT_TRUE(qualifying_cell_corners(grid, 3U, kFloor, kZBandTop).empty());

  set_cell(grid, grid_map::Index(1, 1), 3.0F, 0.3F, 1.5F);
  EXPECT_EQ(qualifying_cell_corners(grid, 3U, kFloor, kZBandTop).size(), 4U);
}

// An all-NaN grid (empty grid / fresh heartbeat) yields no corners.
TEST(QualifyingCellCorners, AllNaNGridYieldsNothing)
{
  auto grid = make_grid();
  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// A grid missing a required layer is a contract violation and yields no corners (never a "clear").
TEST(QualifyingCellCorners, MissingRequiredLayerYieldsNothing)
{
  auto grid = make_grid({"point_count", "min_height"});  // no low_max_height
  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

}  // namespace autoware::planning_validator::collision_checker_utils
