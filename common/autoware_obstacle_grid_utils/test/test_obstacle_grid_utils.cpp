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
#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>
#include <vector>

namespace autoware::obstacle_grid_utils
{
namespace bg = boost::geometry;

// A 0.2 m grid centered at (0,0). Mark one occupied cell containing position (cx, cy).
// NOTE: with center (0,0) and resolution 0.2, cell centers fall on odd multiples of 0.1
// (..., 1.9, 2.1, ...), so callers pass true cell-center coordinates (e.g. 2.1, 1.1) to
// keep the independent oracle exact.
grid_map::GridMap make_grid(double cx, double cy, float count, float zmin, float zmax)
{
  grid_map::GridMap g({"max_height", "min_height", "point_count"});
  g.setFrameId("base_link");
  g.setGeometry(grid_map::Length(10.0, 10.0), 0.2, grid_map::Position(0.0, 0.0));
  g["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["min_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  grid_map::Index idx;
  g.getIndex(grid_map::Position(cx, cy), idx);
  g.at("point_count", idx) = count;
  g.at("max_height", idx) = zmax;
  g.at("min_height", idx) = zmin;
  return g;
}

Polygon2d ego_point()  // degenerate ego polygon at the origin
{
  Polygon2d p;
  bg::append(p.outer(), Point2d(0.0, 0.0));
  bg::correct(p);
  return p;
}

bool qualifies_at(const grid_map::GridMap & g, double x, double y, const Gate & gate)
{
  grid_map::Index idx;
  g.getIndex(grid_map::Position(x, y), idx);
  return cell_qualifies(g, idx, gate);
}

// An all-NaN (heartbeat) 0.2 m grid centered at (0,0); mark individual cells by grid index below.
grid_map::GridMap make_empty_grid()
{
  grid_map::GridMap g({"max_height", "min_height", "point_count"});
  g.setFrameId("base_link");
  g.setGeometry(grid_map::Length(10.0, 10.0), 0.2, grid_map::Position(0.0, 0.0));
  g["point_count"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["max_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  g["min_height"].setConstant(std::numeric_limits<float>::quiet_NaN());
  return g;
}

void set_cell(grid_map::GridMap & g, const grid_map::Index & idx, float count)
{
  g.at("point_count", idx) = count;
  g.at("max_height", idx) = 1.5f;
  g.at("min_height", idx) = 0.5f;
}

TEST(ObstacleGridUtils, EdgeAwareDistanceUsesCellFootprint)
{
  // Occupied cell centered at the true cell center (2.1, 1.1); its 0.2 m footprint box is
  // [2.0,2.2] x [1.0,1.2], whose near corner (2.0, 1.0) is closer to ego than the center.
  const auto g = make_grid(2.1, 1.1, 40, 0.5, 1.5);
  const double d = nearest_distance(g, ego_point(), Gate{1u, 0.0});
  // Independent oracle: distance to the near corner of the footprint, hand-computed.
  EXPECT_LT(d, std::hypot(2.1, 1.1));          // edge-aware: strictly less than center distance
  EXPECT_NEAR(d, std::hypot(2.0, 1.0), 1e-6);  // distance to the near footprint corner
}

TEST(ObstacleGridUtils, NearestDistanceIsInfWhenNothingQualifies)
{
  const auto g = make_grid(2.1, 1.1, 5, 0.5, 1.5);  // count 5 < gate 10 -> nothing qualifies
  const double d = nearest_distance(g, ego_point(), Gate{10u, 0.0});
  EXPECT_TRUE(std::isinf(d));
}

TEST(ObstacleGridUtils, GateRejectsBelowCount)
{
  EXPECT_FALSE(qualifies_at(make_grid(2.1, 1.1, 9, 0.5, 1.5), 2.1, 1.1, Gate{10u, 0.0}));
  EXPECT_TRUE(qualifies_at(make_grid(2.1, 1.1, 10, 0.5, 1.5), 2.1, 1.1, Gate{10u, 0.0}));
}

TEST(ObstacleGridUtils, GateRejectsBelowHeight)
{
  // tall enough passes the height gate; a flat ground-height sliver fails it.
  EXPECT_TRUE(qualifies_at(make_grid(2.1, 1.1, 50, 0.0, 1.2), 2.1, 1.1, Gate{10u, 0.3}));
  EXPECT_FALSE(qualifies_at(make_grid(2.1, 1.1, 50, 0.0, 0.1), 2.1, 1.1, Gate{10u, 0.3}));
}

TEST(ObstacleGridUtils, CellsInPolygonCollectsQualifyingCells)
{
  const auto g = make_grid(2.1, 0.1, 40, 0.5, 1.5);
  Polygon2d lane;  // a box [1,3] x [-1,1] around the occupied cell
  for (auto xy :
       {std::pair{1.0, -1.0}, std::pair{1.0, 1.0}, std::pair{3.0, 1.0}, std::pair{3.0, -1.0}}) {
    bg::append(lane.outer(), Point2d(xy.first, xy.second));
  }
  bg::correct(lane);
  EXPECT_EQ(cells_in_polygon(g, lane, Gate{1u, 0.0}).size(), 1u);
  // an empty (no-detection) lane crop yields no cells
  const auto empty = make_grid(2.1, 0.1, std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f);
  EXPECT_EQ(cells_in_polygon(empty, lane, Gate{1u, 0.0}).size(), 0u);
}
TEST(ObstacleGridUtils, CellCornersAreLiteralFootprintCorners)
{
  // center (2.1, 1.1), resolution 0.2 -> half 0.1 -> box [2.0,2.2] x [1.0,1.2].
  const auto corners = cell_corners(grid_map::Position(2.1, 1.1), 0.2);
  ASSERT_EQ(corners.size(), 4u);
  EXPECT_NEAR(corners[0].x(), 2.0, 1e-9);  // min-min
  EXPECT_NEAR(corners[0].y(), 1.0, 1e-9);
  EXPECT_NEAR(corners[1].x(), 2.0, 1e-9);  // min-max
  EXPECT_NEAR(corners[1].y(), 1.2, 1e-9);
  EXPECT_NEAR(corners[2].x(), 2.2, 1e-9);  // max-max
  EXPECT_NEAR(corners[2].y(), 1.2, 1e-9);
  EXPECT_NEAR(corners[3].x(), 2.2, 1e-9);  // max-min
  EXPECT_NEAR(corners[3].y(), 1.0, 1e-9);
}

TEST(ObstacleGridUtils, NearestCellMatchesNearestDistanceAndReportsWhere)
{
  // Same fixture as EdgeAwareDistanceUsesCellFootprint: one cell centered at (2.1, 1.1).
  const auto g = make_grid(2.1, 1.1, 40, 0.5, 1.5);
  const auto nc = nearest_cell(g, ego_point(), Gate{1u, 0.0});
  ASSERT_TRUE(nc.has_value());
  // Equivalence with the scalar helper (edge-aware distance to the near footprint corner).
  EXPECT_NEAR(nc->distance, nearest_distance(g, ego_point(), Gate{1u, 0.0}), 1e-12);
  EXPECT_NEAR(nc->distance, std::hypot(2.0, 1.0), 1e-6);  // independent oracle
  // WHERE: cell center and grid index of the occupied cell.
  EXPECT_NEAR(nc->position.x(), 2.1, 1e-6);
  EXPECT_NEAR(nc->position.y(), 1.1, 1e-6);
  grid_map::Index expected;
  g.getIndex(grid_map::Position(2.1, 1.1), expected);
  EXPECT_EQ(nc->index(0), expected(0));
  EXPECT_EQ(nc->index(1), expected(1));
}

TEST(ObstacleGridUtils, NearestCellIsNulloptWhenNothingQualifies)
{
  // count 5 < gate 10: nothing qualifies -> nullopt (the nullopt <-> +inf analog).
  const auto g = make_grid(2.1, 1.1, 5, 0.5, 1.5);
  EXPECT_FALSE(nearest_cell(g, ego_point(), Gate{10u, 0.0}).has_value());
  EXPECT_TRUE(std::isinf(nearest_distance(g, ego_point(), Gate{10u, 0.0})));
}

TEST(ObstacleGridUtils, NearestCellPicksTheCloserOfTwoOccupiedCells)
{
  auto g = make_empty_grid();
  grid_map::Index near_idx;
  grid_map::Index far_idx;
  g.getIndex(grid_map::Position(2.1, 1.1), near_idx);
  g.getIndex(grid_map::Position(4.1, 3.1), far_idx);
  set_cell(g, near_idx, 40);
  set_cell(g, far_idx, 40);
  const auto nc = nearest_cell(g, ego_point(), Gate{1u, 0.0});
  ASSERT_TRUE(nc.has_value());
  EXPECT_NEAR(nc->position.x(), 2.1, 1e-6);
  EXPECT_NEAR(nc->position.y(), 1.1, 1e-6);
  EXPECT_EQ(nc->index(0), near_idx(0));
  EXPECT_EQ(nc->index(1), near_idx(1));
}

// --- connected_components -------------------------------------------------------------------
// Indices below are grid indices (row, col); connected_components does not gate, so the caller
// supplies the qualifying list directly and we assert component structure and point_count sums.

TEST(ObstacleGridUtils, ConnectedComponentsEmptyInputEmptyOutput)
{
  const auto g = make_empty_grid();
  EXPECT_TRUE(connected_components(g, {}).empty());
}

TEST(ObstacleGridUtils, ConnectedComponentsSingleCellSumsPointCount)
{
  auto g = make_empty_grid();
  const grid_map::Index a(10, 10);
  set_cell(g, a, 30);
  const auto comps = connected_components(g, {a});
  ASSERT_EQ(comps.size(), 1u);
  EXPECT_EQ(comps[0].cells.size(), 1u);
  EXPECT_NEAR(comps[0].point_sum, 30.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsStraightLineIsOneComponent)
{
  auto g = make_empty_grid();
  const grid_map::Index a(10, 10);
  const grid_map::Index b(10, 11);
  const grid_map::Index c(10, 12);
  set_cell(g, a, 10);
  set_cell(g, b, 20);
  set_cell(g, c, 30);
  const auto comps = connected_components(g, {a, b, c});
  ASSERT_EQ(comps.size(), 1u);
  EXPECT_EQ(comps[0].cells.size(), 3u);
  EXPECT_NEAR(comps[0].point_sum, 60.0, 1e-6);  // 10 + 20 + 30
}

TEST(ObstacleGridUtils, ConnectedComponentsLShapeIsOneComponent)
{
  auto g = make_empty_grid();
  const grid_map::Index a(10, 10);
  const grid_map::Index b(11, 10);
  const grid_map::Index c(11, 11);
  set_cell(g, a, 5);
  set_cell(g, b, 5);
  set_cell(g, c, 5);
  const auto comps = connected_components(g, {a, b, c});
  ASSERT_EQ(comps.size(), 1u);
  EXPECT_EQ(comps[0].cells.size(), 3u);
  EXPECT_NEAR(comps[0].point_sum, 15.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsDiagonalPairIsOneComponentUnder8Connectivity)
{
  auto g = make_empty_grid();
  const grid_map::Index a(10, 10);
  const grid_map::Index b(11, 11);  // diagonal neighbor
  set_cell(g, a, 12);
  set_cell(g, b, 8);
  const auto comps = connected_components(g, {a, b});
  ASSERT_EQ(comps.size(), 1u);  // 8-connected: diagonal touches
  EXPECT_EQ(comps[0].cells.size(), 2u);
  EXPECT_NEAR(comps[0].point_sum, 20.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsGapSeparatesIntoTwoComponents)
{
  auto g = make_empty_grid();
  const grid_map::Index a(10, 10);
  const grid_map::Index c(10, 12);  // (10,11) deliberately not qualifying -> not adjacent to a
  set_cell(g, a, 7);
  set_cell(g, c, 9);
  const auto comps = connected_components(g, {a, c});
  ASSERT_EQ(comps.size(), 2u);
  EXPECT_EQ(comps[0].cells.size(), 1u);
  EXPECT_EQ(comps[1].cells.size(), 1u);
  EXPECT_NEAR(comps[0].point_sum, 7.0, 1e-6);  // first-seen order
  EXPECT_NEAR(comps[1].point_sum, 9.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsBorderCellsNoOutOfRangeProbe)
{
  // Cells on the (0,0) corner: neighbors like (-1,-1) are off-grid and must not be probed nor
  // alias an in-grid cell via the linear key. a and b are adjacent -> one component.
  auto g = make_empty_grid();
  const grid_map::Index a(0, 0);
  const grid_map::Index b(0, 1);
  set_cell(g, a, 3);
  set_cell(g, b, 4);
  const auto comps = connected_components(g, {a, b});
  ASSERT_EQ(comps.size(), 1u);
  EXPECT_EQ(comps[0].cells.size(), 2u);
  EXPECT_NEAR(comps[0].point_sum, 7.0, 1e-6);
}

TEST(ObstacleGridUtils, ConnectedComponentsRowWrapDoesNotFalselyConnect)
{
  // 10 m / 0.2 m = 50 columns. The last cell of row r, (r, 49), and the first cell of row r+1,
  // (r+1, 0), are NOT 8-adjacent but a naive row*cols+col key would alias (r,50) onto (r+1,0).
  // The off-grid bounds guard must keep them as two separate components.
  auto g = make_empty_grid();
  ASSERT_EQ(g.getSize()(1), 50);
  const grid_map::Index a(10, 49);
  const grid_map::Index b(11, 0);
  set_cell(g, a, 6);
  set_cell(g, b, 6);
  EXPECT_EQ(connected_components(g, {a, b}).size(), 2u);
}

}  // namespace autoware::obstacle_grid_utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
