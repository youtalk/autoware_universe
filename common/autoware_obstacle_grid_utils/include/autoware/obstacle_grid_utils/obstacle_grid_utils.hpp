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
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <unordered_map>
#include <unordered_set>
#include <utility>
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

/// The 4 corner points of a cell footprint box, in the same winding as cell_footprint
/// (min-min, min-max, max-max, max-min). Emitting corners rather than the center keeps
/// corridor/lane membership edge-conservative: a cell counts as inside if any corner is.
inline std::array<Point2d, 4> cell_corners(const grid_map::Position & center, double resolution)
{
  const double h = 0.5 * resolution;
  return {
    Point2d(center.x() - h, center.y() - h), Point2d(center.x() - h, center.y() + h),
    Point2d(center.x() + h, center.y() + h), Point2d(center.x() + h, center.y() - h)};
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

/// Nearest qualifying cell together with WHERE it is. distance is edge-aware (to the cell
/// footprint box, identical semantics to nearest_distance); position is the cell center and
/// index its grid index, for debug markers / SafetyFactor.points / StopObstacle.nearest_point.
struct NearestCell
{
  double distance;
  grid_map::Position position;  // cell center
  grid_map::Index index;
};

/// std::nullopt when nothing qualifies inside the ROI (the nullopt <-> +inf analog of
/// nearest_distance; a consumer treats nullopt as "clear within ROI"). On a distance tie the
/// first cell encountered by the grid iterator wins (either-of; callers must not rely on which).
inline std::optional<NearestCell> nearest_cell(
  const grid_map::GridMap & grid, const Polygon2d & ego_polygon, const Gate & gate)
{
  std::optional<NearestCell> best;
  const double res = grid.getResolution();
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    if (!cell_qualifies(grid, *it, gate)) {
      continue;
    }
    grid_map::Position c;
    grid.getPosition(*it, c);
    const double d = boost::geometry::distance(ego_polygon, cell_footprint(c, res));
    if (!best || d < best->distance) {
      best = NearestCell{d, c, *it};
    }
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

/// One 8-connected component of qualifying cells; point_sum is the sum of the point_count layer
/// over its cells (the grid analog of a Euclidean cluster's point total, gated by the caller).
struct CellComponent
{
  std::vector<grid_map::Index> cells;
  double point_sum;
};

/// 8-connected labeling over the SUPPLIED qualifying cell indices. This function does NOT gate:
/// the caller applies its per-cell Gate first and passes the surviving indices; here we only
/// group them into components and sum point_count.
///
/// Adjacency is computed in grid_map's UNWRAPPED index space (via getIndexFromBufferIndex), so it
/// stays geometrically correct even when the grid's circular buffer has been move()d
/// (getStartIndex() != (0,0)): geometrically adjacent cells that straddle the buffer wrap seam are
/// still merged into one component, and cells that are merely neighbors in raw buffer-index space
/// across the seam are not falsely merged. The reported CellComponent::cells are the caller's
/// ORIGINAL buffer indices, so they remain usable directly with grid.at().
///
/// O(cells): membership and visited are hashed on an unwrapped linear key, so out-of-grid
/// neighbors of border cells are simply absent from the member set and never probed. Components
/// come out in first-seen order for stable, deterministic output.
///
/// Precondition: every supplied index is a valid in-grid cell. point_count is expected non-NaN
/// (the caller's Gate already rejects NaN point_count); a NaN contribution is skipped rather than
/// poisoning the whole component's point_sum to NaN and silently dropping a real cluster.
inline std::vector<CellComponent> connected_components(
  const grid_map::GridMap & grid, const std::vector<grid_map::Index> & qualifying)
{
  const grid_map::Size size = grid.getSize();
  const grid_map::Index start = grid.getStartIndex();
  const int rows = size(0);
  const int cols = size(1);
  const auto key = [cols](const grid_map::Index & unwrapped) {
    return static_cast<std::int64_t>(unwrapped(0)) * cols + static_cast<std::int64_t>(unwrapped(1));
  };
  // Map an unwrapped-index key to the caller's ORIGINAL buffer index (for grid.at / reporting).
  std::unordered_map<std::int64_t, grid_map::Index> members;
  members.reserve(qualifying.size());
  for (const auto & idx : qualifying) {
    members.emplace(key(grid_map::getIndexFromBufferIndex(idx, size, start)), idx);
  }
  std::unordered_set<std::int64_t> visited;
  visited.reserve(qualifying.size());
  std::vector<CellComponent> out;
  for (const auto & seed : qualifying) {
    const grid_map::Index seed_unwrapped = grid_map::getIndexFromBufferIndex(seed, size, start);
    const std::int64_t seed_key = key(seed_unwrapped);
    if (visited.count(seed_key) != 0) {
      continue;  // already absorbed into an earlier component (or a duplicate index)
    }
    CellComponent comp;
    comp.point_sum = 0.0;
    std::vector<grid_map::Index> stack{seed_unwrapped};  // unwrapped indices
    visited.insert(seed_key);
    while (!stack.empty()) {
      const grid_map::Index cur = stack.back();  // unwrapped
      stack.pop_back();
      const grid_map::Index buf = grid_map::getBufferIndexFromIndex(cur, size, start);
      comp.cells.push_back(buf);  // report the caller's buffer-index space
      const float cnt = grid.at("point_count", buf);
      if (!std::isnan(cnt)) {
        comp.point_sum += static_cast<double>(cnt);  // skip NaN rather than poison the sum
      }
      for (int dr = -1; dr <= 1; ++dr) {
        for (int dc = -1; dc <= 1; ++dc) {
          if (dr == 0 && dc == 0) {
            continue;
          }
          const grid_map::Index nb(cur(0) + dr, cur(1) + dc);  // unwrapped neighbor
          if (nb(0) < 0 || nb(0) >= rows || nb(1) < 0 || nb(1) >= cols) {
            continue;  // off-grid neighbor: skip before keying so the linear key cannot alias
          }
          const std::int64_t nb_key = key(nb);
          const auto found = members.find(nb_key);
          if (found == members.end() || visited.count(nb_key) != 0) {
            continue;
          }
          visited.insert(nb_key);
          stack.push_back(nb);  // nb is already the unwrapped index of found->second
        }
      }
    }
    out.push_back(std::move(comp));
  }
  return out;
}
}  // namespace autoware::obstacle_grid_utils
#endif  // AUTOWARE__OBSTACLE_GRID_UTILS__OBSTACLE_GRID_UTILS_HPP_
