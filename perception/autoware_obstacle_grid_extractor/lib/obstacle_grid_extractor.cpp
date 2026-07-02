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
#include "autoware/obstacle_grid_extractor/obstacle_grid_extractor.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/time.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace autoware::obstacle_grid_extractor
{
namespace
{
constexpr char kMaxHeight[] = "max_height";
constexpr char kMinHeight[] = "min_height";
constexpr char kPointCount[] = "point_count";
constexpr char kLowMaxHeight[] = "low_max_height";
}  // namespace

ObstacleGridExtractor::ObstacleGridExtractor(const ExtractorParams & params)
: params_(params), grid_({kMaxHeight, kMinHeight, kPointCount, kLowMaxHeight})
{
  grid_.setFrameId("base_link");
  grid_.setGeometry(
    grid_map::Length(params_.roi_length_x, params_.roi_length_y), params_.resolution,
    grid_map::Position(params_.roi_offset_x, 0.0));  // forward-biased rectangular ROI
}

grid_map_msgs::msg::GridMap ObstacleGridExtractor::extract(
  const pcl::PointCloud<pcl::PointXYZ> & cloud_base_link,
  const std_msgs::msg::Header & header) const
{
  const float nan = std::numeric_limits<float>::quiet_NaN();
  grid_[kMaxHeight].setConstant(nan);
  grid_[kMinHeight].setConstant(nan);
  grid_[kPointCount].setConstant(nan);  // empty cell = NaN
  grid_[kLowMaxHeight].setConstant(nan);

  auto & hi = grid_[kMaxHeight];
  auto & lo = grid_[kMinHeight];
  auto & cnt = grid_[kPointCount];
  auto & low_hi = grid_[kLowMaxHeight];

  for (const auto & p : cloud_base_link) {  // O(N) single pass
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;  // a NaN z would otherwise pass the crop and poison the cell's height layers
    }
    if (p.z < params_.crop_z_min || p.z > params_.crop_z_max) {
      continue;
    }
    grid_map::Index idx;
    if (!grid_.getIndex(grid_map::Position(p.x, p.y), idx)) {
      continue;  // outside ROI
    }
    float & c = cnt(idx(0), idx(1));
    if (std::isnan(c)) {
      c = 1.0f;
      hi(idx(0), idx(1)) = p.z;
      lo(idx(0), idx(1)) = p.z;
    } else {
      c += 1.0f;
      hi(idx(0), idx(1)) = std::max(hi(idx(0), idx(1)), p.z);
      lo(idx(0), idx(1)) = std::min(lo(idx(0), idx(1)), p.z);
    }
    if (p.z <= params_.overhead_split) {
      float & lm = low_hi(idx(0), idx(1));
      lm = std::isnan(lm) ? p.z : std::max(lm, p.z);
    }
  }

  grid_.setTimestamp(rclcpp::Time(header.stamp).nanoseconds());
  grid_map_msgs::msg::GridMap msg = *grid_map::GridMapRosConverter::toMessage(grid_);
  msg.header = header;  // base_link, exact cloud stamp -> all-NaN = heartbeat
  return msg;
}
}  // namespace autoware::obstacle_grid_extractor
