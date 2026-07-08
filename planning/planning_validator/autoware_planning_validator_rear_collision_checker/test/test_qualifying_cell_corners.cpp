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

#include "utils.hpp"

#include <grid_map_core/grid_map_core.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace autoware::planning_validator::utils
{
namespace
{
constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr double kResolution = 0.5;  // -> half cell = 0.25
constexpr double kFloor = 0.3;       // low_max_height floor (pointcloud.grid.z_floor)
constexpr double kZBandTop = 2.5;    // vehicle_height + z_band_top_offset

// A 4x4 grid centred at the origin (span [-1, 1] on each axis), all layers NaN (empty heartbeat).
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

// A single in-band cell qualifies; exactly four corners are emitted at the cell footprint corners,
// each at z == low_max_height. Oracles (half cell = 0.25, z = 1.5) are independent literals.
TEST(QualifyingCellCorners, SingleCellEmitsFourCornersAtLowMaxHeight)
{
  auto grid = make_grid();
  const grid_map::Index idx(1, 1);
  set_cell(grid, idx, 5.0F, 0.2F, 1.5F);
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

// The four emitted corners are exactly the footprint corners of the qualifying cell (hand-listed
// coordinates, independent of the SUT). Cell (0,0) of a default-start 4x4 grid spanning [-1,1] with
// resolution 0.5 is centred at (0.75, 0.75); its corners are (0.5|1.0, 0.5|1.0).
TEST(QualifyingCellCorners, CornerCoordinatesMatchHandOracle)
{
  auto grid = make_grid();
  grid.convertToDefaultStartIndex();
  const grid_map::Index idx(0, 0);
  set_cell(grid, idx, 3.0F, 0.2F, 1.0F);
  grid_map::Position center;
  grid.getPosition(idx, center);
  ASSERT_DOUBLE_EQ(center.x(), 0.75);
  ASSERT_DOUBLE_EQ(center.y(), 0.75);

  const auto corners = qualifying_cell_corners(grid, 1U, kFloor, kZBandTop);
  ASSERT_EQ(corners.size(), 4U);

  auto has_xy = [&](const double x, const double y) {
    return std::any_of(corners.begin(), corners.end(), [&](const auto & c) {
      return std::abs(c.x - x) < 1e-9 && std::abs(c.y - y) < 1e-9;
    });
  };
  EXPECT_TRUE(has_xy(0.5, 0.5));
  EXPECT_TRUE(has_xy(0.5, 1.0));
  EXPECT_TRUE(has_xy(1.0, 1.0));
  EXPECT_TRUE(has_xy(1.0, 0.5));
}

// Two spatially separated qualifying cells each emit their own four corners (per-cell semantics; no
// clustering merges or drops them).
TEST(QualifyingCellCorners, TwoSeparatedCellsEmitEightCorners)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(0, 0), 3.0F, 0.2F, 1.0F);
  set_cell(grid, grid_map::Index(3, 3), 3.0F, 0.2F, 1.0F);

  EXPECT_EQ(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).size(), 8U);
}

// low_max_height just below the floor rejects the cell.
TEST(QualifyingCellCorners, BelowFloorRejected)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 5.0F, 0.2F, static_cast<float>(kFloor) - 0.05F);

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// low_max_height exactly at the floor qualifies (the comparison is >=).
TEST(QualifyingCellCorners, FloorBoundaryInclusive)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 5.0F, 0.2F, static_cast<float>(kFloor));

  EXPECT_EQ(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).size(), 4U);
}

// The floor is applied to low_max_height, not max_height: a cell holding ground residue
// (low_max_height below the floor) is rejected even though taller (overhead) returns exist.
TEST(QualifyingCellCorners, GroundResidueUnderOverheadRejected)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 20.0F, 0.05F, 0.05F);

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// A purely overhead cell (lowest return above the z-band top) is rejected by the ceiling check.
TEST(QualifyingCellCorners, OverheadOnlyRejectedByCeiling)
{
  auto grid = make_grid();
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
  set_cell(grid, grid_map::Index(1, 1), 2.0F, 0.2F, 1.5F);
  EXPECT_TRUE(qualifying_cell_corners(grid, 3U, kFloor, kZBandTop).empty());

  set_cell(grid, grid_map::Index(1, 1), 3.0F, 0.2F, 1.5F);
  EXPECT_EQ(qualifying_cell_corners(grid, 3U, kFloor, kZBandTop).size(), 4U);
}

// A NaN point_count cell never qualifies even if the height layers are in-band.
TEST(QualifyingCellCorners, NaNCountNeverQualifies)
{
  auto grid = make_grid();
  grid.at("min_height", grid_map::Index(1, 1)) = 0.2F;
  grid.at("low_max_height", grid_map::Index(1, 1)) = 1.5F;
  // point_count left NaN.

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// A cell that clears the density gate but whose low_max_height is NaN is rejected: the tallest
// in-band return is unknown, so the floor gate cannot be satisfied. This is distinct from
// BelowFloorRejected (a finite low_max below the floor); here the isnan(low_max) guard fires.
TEST(QualifyingCellCorners, NaNLowMaxHeightRejected)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 5.0F, 0.2F, kNaN);

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
}

// A cell that clears both the density gate and the finite in-band low_max_height floor but whose
// min_height is NaN is rejected: the lowest return is unknown, so the band-top gate cannot be
// satisfied. Distinct from OverheadOnlyRejectedByCeiling (a finite min above the top); here the
// isnan(min_height) guard fires.
TEST(QualifyingCellCorners, NaNMinHeightRejected)
{
  auto grid = make_grid();
  set_cell(grid, grid_map::Index(1, 1), 5.0F, kNaN, 1.5F);

  EXPECT_TRUE(qualifying_cell_corners(grid, 1U, kFloor, kZBandTop).empty());
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

}  // namespace autoware::planning_validator::utils
