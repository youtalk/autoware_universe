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
//
// S3 size/coverage validation: on a representative dense cloud, the grid has the configured
// geometry, drops no in-ROI/in-z-band point silently, and records an informational latency.
// (Full real-data coverage on 890 x2 frames is reported in the Milestone-S eval; this is the
// CI-resident gate so the producer ships only when geometry + coverage hold.)
#include "autoware/obstacle_grid_extractor/obstacle_grid_extractor.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <string>
#include <vector>

namespace
{
using autoware::obstacle_grid_extractor::ExtractorParams;
using autoware::obstacle_grid_extractor::ObstacleGridExtractor;

ExtractorParams prodParams()
{
  return ExtractorParams{60.0, 40.0, 20.0, 0.2, -1.0f, 3.0f};
}
}  // namespace

TEST(ObstacleGridCoverage, GeometryAndNoSilentDrop)
{
  // representative dense forward cloud inside the ROI + z-band
  std::mt19937 rng(7);
  std::uniform_real_distribution<float> ux(-9.0f, 49.0f), uy(-19.0f, 19.0f), uz(-0.5f, 2.5f);
  ObstacleGridExtractor ex(prodParams());
  std_msgs::msg::Header h;
  h.frame_id = "base_link";

  std::vector<double> ms;
  for (int frame = 0; frame < 20; ++frame) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for (int i = 0; i < 12000; ++i) cloud.emplace_back(ux(rng), uy(rng), uz(rng));

    const auto t0 = std::chrono::steady_clock::now();
    const auto msg = ex.extract(cloud, h);
    ms.push_back(
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());

    grid_map::GridMap g;
    grid_map::GridMapRosConverter::fromMessage(msg, g);

    // (a) geometry
    ASSERT_EQ(g.getLayers().size(), 3u);
    EXPECT_NEAR(g.getResolution(), 0.2, 1e-9);
    EXPECT_EQ(g.getFrameId(), "base_link");

    // (b) coverage: every point inside the ROI + z-band lands in an occupied cell
    for (const auto & p : cloud) {
      if (p.z < -1.0f || p.z > 3.0f) continue;
      grid_map::Index idx;
      if (!g.getIndex(grid_map::Position(p.x, p.y), idx)) continue;  // outside ROI is fine
      EXPECT_FALSE(std::isnan(g.at("point_count", idx)));
    }
  }
  std::sort(ms.begin(), ms.end());
  // (c) informational latency (not a hard gate; full sweep in the eval report)
  // Latency is informational (recorded for trend visibility), not a hard wall-clock gate — a hard
  // bound would flake on loaded CI / sanitizer builds. The 15 ms budget is checked in the eval.
  RecordProperty("p99_ms", std::to_string(ms[static_cast<size_t>(0.99 * (ms.size() - 1))]));
  RecordProperty("max_ms", std::to_string(ms.back()));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
