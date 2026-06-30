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

#include <cmath>
#include <limits>
#include <utility>

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
}  // namespace autoware::obstacle_grid_utils

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
