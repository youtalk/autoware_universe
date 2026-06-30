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
#ifndef AUTOWARE__OBSTACLE_GRID_UTILS__OBSTACLE_GRID_UTILS_HPP_
#define AUTOWARE__OBSTACLE_GRID_UTILS__OBSTACLE_GRID_UTILS_HPP_

#include <autoware_utils_geometry/boost_geometry.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include <boost/geometry.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace autoware::obstacle_grid_utils
{
using autoware_utils_geometry::Point2d;
using autoware_utils_geometry::Polygon2d;

/// A cell qualifies iff point_count >= min_point_count_cell && max_height >= min_height.
struct Gate
{
  std::uint32_t min_point_count_cell;
  double min_height;
};

inline bool cell_qualifies(
  const grid_map::GridMap & grid, const grid_map::Index & idx, const Gate & gate)
{
  const float cnt = grid.at("point_count", idx);
  if (std::isnan(cnt) || static_cast<std::uint32_t>(cnt) < gate.min_point_count_cell) {
    return false;
  }
  const float hi = grid.at("max_height", idx);
  return !std::isnan(hi) && static_cast<double>(hi) >= gate.min_height;
}

/// Footprint box of one cell (areas, so distance to it is edge-aware by construction).
inline Polygon2d cell_footprint(const grid_map::Position & c, double res)
{
  const double h = 0.5 * res;
  Polygon2d box;
  boost::geometry::append(box.outer(), Point2d(c.x() - h, c.y() - h));
  boost::geometry::append(box.outer(), Point2d(c.x() - h, c.y() + h));
  boost::geometry::append(box.outer(), Point2d(c.x() + h, c.y() + h));
  boost::geometry::append(box.outer(), Point2d(c.x() + h, c.y() - h));
  boost::geometry::correct(box);
  return box;
}

/// 2D distance from ego_polygon to the nearest qualifying cell footprint. Returns
/// +inf when nothing qualifies inside the ROI (caller treats that as "clear within ROI").
inline double nearest_distance(
  const grid_map::GridMap & grid, const Polygon2d & ego_polygon, const Gate & gate)
{
  double best = std::numeric_limits<double>::infinity();
  const double res = grid.getResolution();
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    if (!cell_qualifies(grid, *it, gate)) {
      continue;
    }
    grid_map::Position c;
    grid.getPosition(*it, c);
    best = std::min(best, boost::geometry::distance(ego_polygon, cell_footprint(c, res)));
  }
  return best;
}

/// Qualifying cell centers inside a polygon (each lanelet/traj consumer re-crops here).
inline std::vector<grid_map::Position> cells_in_polygon(
  const grid_map::GridMap & grid, const Polygon2d & lane_polygon, const Gate & gate)
{
  grid_map::Polygon gm;
  gm.setFrameId(grid.getFrameId());
  for (const auto & pt : lane_polygon.outer()) {
    gm.addVertex(grid_map::Position(pt.x(), pt.y()));
  }
  std::vector<grid_map::Position> out;
  for (grid_map::PolygonIterator it(grid, gm); !it.isPastEnd(); ++it) {
    if (!cell_qualifies(grid, *it, gate)) {
      continue;
    }
    grid_map::Position c;
    grid.getPosition(*it, c);
    out.push_back(c);
  }
  return out;
}
}  // namespace autoware::obstacle_grid_utils
#endif  // AUTOWARE__OBSTACLE_GRID_UTILS__OBSTACLE_GRID_UTILS_HPP_
