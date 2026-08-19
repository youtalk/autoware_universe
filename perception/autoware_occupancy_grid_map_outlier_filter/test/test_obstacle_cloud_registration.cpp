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

#include "occupancy_grid_map_outlier_filter_node.hpp"

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

// The node declares map/pointcloud sync parameters with no code-side
// defaults (the defaults live only in the launch-time yaml), so every
// construction in this test needs them supplied explicitly. The radius
// search filter and debugger are disabled to avoid pulling in their own
// parameter sets.
rclcpp::NodeOptions options_with(const std::vector<std::string> & extra_args)
{
  std::vector<std::string> args{
    "--ros-args",
    "-p",
    "map_frame:=map",
    "-p",
    "base_link_frame:=base_link",
    "-p",
    "cost_threshold:=45",
    "-p",
    "use_radius_search_2d_filter:=false",
    "-p",
    "enable_debugger:=false",
    "-p",
    "publish_processing_time_detail:=false"};
  args.insert(args.end(), extra_args.begin(), extra_args.end());
  rclcpp::NodeOptions options;
  options.arguments(args);
  return options;
}

class ObstacleCloudRegistrationTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(ObstacleCloudRegistrationTest, NonTerminalOutputConstructsFine)
{
  // ~/output/pointcloud is not remapped onto the obstacle-cloud boundary, so
  // register_publisher() must simply decline registration without throwing.
  EXPECT_NO_THROW(
    autoware::occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent{
      options_with({})});
}

TEST_F(ObstacleCloudRegistrationTest, TerminalWithHardcodedSpecQosConstructsFine)
{
  // This node creates ~/output/pointcloud with a hardcoded rclcpp::QoS{5}.reliable(),
  // which already is the ObstacleSegmentation spec profile (RELIABLE / depth 5 /
  // VOLATILE). When launch points the output at the boundary topic (the OSS
  // default: this node is the pipeline terminal), construction must still not
  // throw because the endpoint already conforms.
  EXPECT_NO_THROW(
    autoware::occupancy_grid_map_outlier_filter::OccupancyGridMapOutlierFilterComponent{
      options_with(
        {"-r", "__node:=occupancy_grid_map_outlier_filter", "-r",
         "~/output/pointcloud:=/perception/obstacle_segmentation/pointcloud"})});
}

}  // namespace
