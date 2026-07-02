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

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

namespace
{
using autoware::obstacle_grid_extractor::ExtractorParams;
using autoware::obstacle_grid_extractor::ObstacleGridExtractor;

ExtractorParams testParams()
{
  return ExtractorParams{/*x*/ 60.0,      /*y*/ 40.0,     /*offset*/ 20.0,        /*res*/ 0.2,
                         /*z_min*/ -1.0f, /*z_max*/ 3.0f, /*overhead_split*/ 2.5f};
}

grid_map::GridMap toGrid(const grid_map_msgs::msg::GridMap & msg)
{
  grid_map::GridMap g;
  grid_map::GridMapRosConverter::fromMessage(msg, g);
  return g;
}
}  // namespace

TEST(ObstacleGridExtractor, TwoPointsInSameCellAccumulate)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // With ROI offset_x=20 and res 0.2, the cell containing the query target (5.1, 0.1) spans
  // [5.0,5.2] x [0.0,0.2]; both points fall inside it (x=5.0/y=0.0 would be cell boundaries).
  cloud.emplace_back(5.05f, 0.05f, 0.5f);
  cloud.emplace_back(5.15f, 0.15f, 1.2f);  // same 0.2 m cell as the first

  std_msgs::msg::Header h;
  h.frame_id = "base_link";
  const auto g = toGrid(ObstacleGridExtractor(testParams()).extract(cloud, h));

  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(5.1, 0.1), idx));
  EXPECT_FLOAT_EQ(g.at("point_count", idx), 2.0f);
  EXPECT_FLOAT_EQ(g.at("max_height", idx), 1.2f);
  EXPECT_FLOAT_EQ(g.at("min_height", idx), 0.5f);
}

TEST(ObstacleGridExtractor, EmptyCellsAreNaN)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.emplace_back(5.0f, 0.0f, 0.5f);
  std_msgs::msg::Header h;
  h.frame_id = "base_link";
  const auto g = toGrid(ObstacleGridExtractor(testParams()).extract(cloud, h));

  grid_map::Index far;
  ASSERT_TRUE(g.getIndex(grid_map::Position(30.0, 15.0), far));
  EXPECT_TRUE(std::isnan(g.at("point_count", far)));  // unobserved cell stays NaN
}

TEST(ObstacleGridExtractor, NarrowObstacleOccupiesAtLeastOneCell)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  // a 0.15 m-wide vertical pole -> all points fall in >= 1 cell; integer-stepped z
  // avoids the float-accumulation drift of a `dz += 0.1f` loop (top z stays exactly 1.0 m).
  for (int i = 0; i <= 10; ++i) {
    cloud.emplace_back(8.0f, 0.0f, 0.1f * static_cast<float>(i));  // z = 0.0 .. 1.0
  }
  std_msgs::msg::Header h;
  h.frame_id = "base_link";
  const auto g = toGrid(ObstacleGridExtractor(testParams()).extract(cloud, h));

  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(8.0, 0.0), idx));
  EXPECT_FALSE(std::isnan(g.at("point_count", idx)));
  EXPECT_FLOAT_EQ(g.at("max_height", idx), 1.0f);
  EXPECT_FLOAT_EQ(g.at("min_height", idx), 0.0f);
}

TEST(ObstacleGridExtractor, PointsOutsideCropZBandAreDropped)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.emplace_back(5.0f, 0.0f, -2.0f);  // below crop_z_min (-1.0) -> dropped
  cloud.emplace_back(5.0f, 0.0f, 5.0f);   // above crop_z_max (3.0) -> dropped
  std_msgs::msg::Header h;
  h.frame_id = "base_link";
  const auto g = toGrid(ObstacleGridExtractor(testParams()).extract(cloud, h));

  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(5.0, 0.0), idx));
  EXPECT_TRUE(std::isnan(g.at("point_count", idx)));  // both points cropped -> cell empty
}

TEST(ObstacleGridExtractor, CropZBandBoundsAreInclusive)
{
  std_msgs::msg::Header h;
  h.frame_id = "base_link";

  // points exactly on the bounds are KEPT (the crop uses strict < / >); both land in cell (5.1,0.1)
  pcl::PointCloud<pcl::PointXYZ> on_bounds;
  on_bounds.emplace_back(5.05f, 0.05f, -1.0f);  // == crop_z_min
  on_bounds.emplace_back(5.15f, 0.15f, 3.0f);   // == crop_z_max
  const auto g = toGrid(ObstacleGridExtractor(testParams()).extract(on_bounds, h));
  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(5.1, 0.1), idx));
  EXPECT_FLOAT_EQ(g.at("point_count", idx), 2.0f);
  EXPECT_FLOAT_EQ(g.at("max_height", idx), 3.0f);
  EXPECT_FLOAT_EQ(g.at("min_height", idx), -1.0f);

  // points just outside the bounds are DROPPED
  pcl::PointCloud<pcl::PointXYZ> just_outside;
  just_outside.emplace_back(8.05f, 0.05f, -1.0001f);  // just below crop_z_min
  just_outside.emplace_back(8.15f, 0.15f, 3.0001f);   // just above crop_z_max
  const auto g2 = toGrid(ObstacleGridExtractor(testParams()).extract(just_outside, h));
  grid_map::Index idx2;
  ASSERT_TRUE(g2.getIndex(grid_map::Position(8.1, 0.1), idx2));
  EXPECT_TRUE(std::isnan(g2.at("point_count", idx2)));
}

TEST(ObstacleGridExtractor, LowMaxHeightExcludesOverheadReturns)
{
  std_msgs::msg::Header h;
  h.frame_id = "base_link";

  // gantry-like cell: a ground return (0.05) + an overhead return (2.8 > overhead_split 2.5).
  // low_max_height must record only the tallest IN-BAND return (0.05), while max_height still
  // reports the true top (2.8) — this is the discriminator consumers use to reject overhead-only
  // structures without losing tall in-band obstacles.
  pcl::PointCloud<pcl::PointXYZ> gantry;
  gantry.emplace_back(5.05f, 0.05f, 0.05f);
  gantry.emplace_back(5.15f, 0.15f, 2.8f);
  const auto g = toGrid(ObstacleGridExtractor(testParams()).extract(gantry, h));
  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(5.1, 0.1), idx));
  EXPECT_FLOAT_EQ(g.at("point_count", idx), 2.0f);
  EXPECT_FLOAT_EQ(g.at("max_height", idx), 2.8f);
  EXPECT_FLOAT_EQ(g.at("low_max_height", idx), 0.05f);

  // overhead-only cell: low_max_height stays NaN while point_count/max_height are populated.
  pcl::PointCloud<pcl::PointXYZ> overhead_only;
  overhead_only.emplace_back(8.05f, 0.05f, 2.8f);
  const auto g2 = toGrid(ObstacleGridExtractor(testParams()).extract(overhead_only, h));
  grid_map::Index idx2;
  ASSERT_TRUE(g2.getIndex(grid_map::Position(8.1, 0.1), idx2));
  EXPECT_FLOAT_EQ(g2.at("point_count", idx2), 1.0f);
  EXPECT_FLOAT_EQ(g2.at("max_height", idx2), 2.8f);
  EXPECT_TRUE(std::isnan(g2.at("low_max_height", idx2)));

  // wall-like cell: an in-band tall return (2.0 <= 2.5) IS recorded by low_max_height.
  pcl::PointCloud<pcl::PointXYZ> wall;
  wall.emplace_back(11.05f, 0.05f, 0.05f);
  wall.emplace_back(11.15f, 0.15f, 2.0f);
  const auto g3 = toGrid(ObstacleGridExtractor(testParams()).extract(wall, h));
  grid_map::Index idx3;
  ASSERT_TRUE(g3.getIndex(grid_map::Position(11.1, 0.1), idx3));
  EXPECT_FLOAT_EQ(g3.at("low_max_height", idx3), 2.0f);
}

TEST(ObstacleGridExtractor, NonFinitePointsAreDropped)
{
  std_msgs::msg::Header h;
  h.frame_id = "base_link";

  // a NaN-z point with finite x/y arrives FIRST in the cell; without the finiteness guard it
  // would pass the z crop (NaN comparisons are false) and permanently poison max/min_height
  // (std::max/std::min return the first, NaN, argument). The real point must fully define the cell.
  const float nan = std::numeric_limits<float>::quiet_NaN();
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.emplace_back(5.05f, 0.05f, nan);
  cloud.emplace_back(5.15f, 0.15f, 0.5f);
  const auto g = toGrid(ObstacleGridExtractor(testParams()).extract(cloud, h));
  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(5.1, 0.1), idx));
  EXPECT_FLOAT_EQ(g.at("point_count", idx), 1.0f);  // the NaN point is not counted
  EXPECT_FLOAT_EQ(g.at("max_height", idx), 0.5f);
  EXPECT_FLOAT_EQ(g.at("min_height", idx), 0.5f);
  EXPECT_FLOAT_EQ(g.at("low_max_height", idx), 0.5f);
}

TEST(ObstacleGridExtractor, EmptyCloudIsFreshHeartbeat)
{
  pcl::PointCloud<pcl::PointXYZ> empty;
  std_msgs::msg::Header h;
  h.frame_id = "base_link";
  h.stamp.sec = 42;
  const auto msg = ObstacleGridExtractor(testParams()).extract(empty, h);

  EXPECT_EQ(msg.header.stamp.sec, 42);  // fresh stamp = "alive, nothing detected"
  const auto g = toGrid(msg);
  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(5.0, 0.0), idx));
  EXPECT_TRUE(std::isnan(g.at("point_count", idx)));  // all-NaN heartbeat
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
