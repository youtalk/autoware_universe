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

namespace
{
using autoware::obstacle_grid_extractor::ExtractorParams;
using autoware::obstacle_grid_extractor::ObstacleGridExtractor;

ExtractorParams testParams()
{
  return ExtractorParams{/*x*/ 60.0, /*y*/ 40.0, /*offset*/ 20.0, /*res*/ 0.2, -1.0f, 3.0f};
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
