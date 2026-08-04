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
// Node-wrapper contract tests. These drive the real subscription callback through an in-process
// executor (no live sensor / no external DDS peers) to exercise the branches the wrapper OWNS:
//  - TF lookup FAILS  -> nothing is published (the safety-critical "never emit a wrong-frame grid"
//    guard); consumers must treat a stale stamp as "unavailable", never as "clear".
//  - TF lookup SUCCEEDS (a base_link cloud resolves to identity) -> a base_link grid is published
//    at the input cloud's own stamp.
#include "make_point_cloud.hpp"
#include "obstacle_grid_extractor_node.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

namespace
{
using autoware::obstacle_grid_extractor::ObstacleGridExtractorNode;
using autoware::obstacle_grid_extractor::test::makePointCloud;
using std::chrono::steady_clock;

constexpr char kInput[] = "/obstacle_grid_extractor/input/pointcloud";
constexpr char kOutput[] = "/obstacle_grid_extractor/output/obstacle_grid";

rclcpp::NodeOptions nodeOptions()
{
  rclcpp::NodeOptions options;
  options.append_parameter_override("roi.length_x", 60.0);
  options.append_parameter_override("roi.length_y", 40.0);
  options.append_parameter_override("roi.offset_x", 20.0);
  options.append_parameter_override("resolution", 0.2);
  options.append_parameter_override("crop.z_min", -1.0);
  options.append_parameter_override("crop.z_max", 3.0);
  options.append_parameter_override("overhead_split", 2.5);
  return options;
}

// single XYZ point cloud in the given frame, stamped at `stamp`
sensor_msgs::msg::PointCloud2 makeSinglePointCloud(
  const rclcpp::Time & stamp, const std::string & frame, float x, float y, float z)
{
  auto cloud = makePointCloud(frame, {{x, y, z}});
  cloud.header.stamp = stamp;
  return cloud;
}

// spin the executor until `pred()` holds or `timeout` elapses; returns pred() at exit
template <typename Pred>
bool spinUntil(rclcpp::Executor & exec, Pred pred, std::chrono::milliseconds timeout)
{
  const auto deadline = steady_clock::now() + timeout;
  while (steady_clock::now() < deadline) {
    if (pred()) return true;
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return pred();
}
}  // namespace

TEST(ObstacleGridExtractorNode, PublishesBaseLinkGridWhenTfResolves)
{
  // Arrange: the node plus a helper node holding the input publisher and the output subscription.
  auto node = std::make_shared<ObstacleGridExtractorNode>(nodeOptions());
  auto helper = std::make_shared<rclcpp::Node>("test_helper");
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(
    kInput, rclcpp::SensorDataQoS().keep_last(1));

  grid_map_msgs::msg::GridMap::ConstSharedPtr received;
  auto sub = helper->create_subscription<grid_map_msgs::msg::GridMap>(
    kOutput, rclcpp::QoS(1).reliable(),
    [&received](grid_map_msgs::msg::GridMap::ConstSharedPtr m) { received = m; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);

  // Act: publish a base_link cloud, for which lookupTransform("base_link", "base_link") is the
  // identity, so the happy path runs. Re-publishing absorbs discovery latency (the extractor is
  // stateless, so repeated frames are idempotent); the last published stamp is the one asserted on.
  rclcpp::Time published_stamp;
  const bool got = spinUntil(
    exec,
    [&]() {
      if (!received) published_stamp = helper->now();
      pub->publish(makeSinglePointCloud(published_stamp, "base_link", 8.05f, 0.05f, 0.5f));
      return static_cast<bool>(received);
    },
    std::chrono::seconds(10));

  // Assert: a base_link grid carrying the input cloud's stamp and the input point's cell.
  ASSERT_TRUE(got) << "no grid published for a resolvable (base_link) cloud";
  EXPECT_EQ(received->header.frame_id, "base_link");
  EXPECT_EQ(rclcpp::Time(received->header.stamp), published_stamp)
    << "the grid must carry the sensing time, not the TF lookup time";
  grid_map::GridMap grid;
  grid_map::GridMapRosConverter::fromMessage(*received, grid);
  EXPECT_EQ(grid.getLayers().size(), 4u);
  grid_map::Index occupied_cell;
  ASSERT_TRUE(grid.getIndex(grid_map::Position(8.05, 0.05), occupied_cell));
  EXPECT_FLOAT_EQ(grid.at("point_count", occupied_cell), 1.0f);
}

TEST(ObstacleGridExtractorNode, DropsCloudWhenTfLookupFails)
{
  // Arrange: same wiring, and wait for discovery first so that a later "nothing arrived" means the
  // callback dropped the cloud rather than that the link was never established.
  auto node = std::make_shared<ObstacleGridExtractorNode>(nodeOptions());
  auto helper = std::make_shared<rclcpp::Node>("test_helper");
  auto pub = helper->create_publisher<sensor_msgs::msg::PointCloud2>(
    kInput, rclcpp::SensorDataQoS().keep_last(1));

  grid_map_msgs::msg::GridMap::ConstSharedPtr received;
  auto sub = helper->create_subscription<grid_map_msgs::msg::GridMap>(
    kOutput, rclcpp::QoS(1).reliable(),
    [&received](grid_map_msgs::msg::GridMap::ConstSharedPtr m) { received = m; });

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(helper);
  ASSERT_TRUE(spinUntil(
    exec, [&]() { return pub->get_subscription_count() > 0; }, std::chrono::seconds(10)))
    << "input subscription never discovered";

  // Act: publish clouds in an UNKNOWN frame. lookupTransform to base_link has no data and throws
  // after its timeout, so onCloud takes the catch branch.
  const auto deadline = steady_clock::now() + std::chrono::milliseconds(1500);
  while (steady_clock::now() < deadline) {
    pub->publish(
      makeSinglePointCloud(helper->now(), "lidar_top_that_has_no_tf", 8.05f, 0.05f, 0.5f));
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  // Assert: not even the all-NaN heartbeat is emitted on a wrong-frame cloud.
  EXPECT_EQ(received, nullptr) << "a grid was published despite the TF lookup failing";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int rc = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return rc;
}
