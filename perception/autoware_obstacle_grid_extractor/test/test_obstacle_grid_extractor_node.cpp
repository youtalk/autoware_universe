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
// executor (no live sensor / no external DDS peers) to exercise the two branches the wrapper OWNS:
//  - TF lookup FAILS  -> nothing is published (the safety-critical "never emit a wrong-frame grid"
//    guard); consumers must treat a stale stamp as "unavailable", never as "clear".
//  - TF lookup SUCCEEDS (a base_link cloud resolves to identity) -> a base_link grid is published.
#include "obstacle_grid_extractor_node.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/rclcpp.hpp>

#include <grid_map_msgs/msg/grid_map.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace
{
using autoware::obstacle_grid_extractor::ObstacleGridExtractorNode;
using std::chrono::steady_clock;

rclcpp::NodeOptions nodeOptions()
{
  rclcpp::NodeOptions o;
  o.append_parameter_override("roi.length_x", 60.0);
  o.append_parameter_override("roi.length_y", 40.0);
  o.append_parameter_override("roi.offset_x", 20.0);
  o.append_parameter_override("resolution", 0.2);
  o.append_parameter_override("crop.z_min", -1.0);
  o.append_parameter_override("crop.z_max", 3.0);
  o.append_parameter_override("overhead_split", 2.5);
  return o;
}

// single XYZ point cloud in the given frame
sensor_msgs::msg::PointCloud2 makeCloud(
  const rclcpp::Time & stamp, const std::string & frame, float x, float y, float z)
{
  sensor_msgs::msg::PointCloud2 msg;
  msg.header.frame_id = frame;
  msg.header.stamp = stamp;
  sensor_msgs::PointCloud2Modifier mod(msg);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(1);
  sensor_msgs::PointCloud2Iterator<float> ix(msg, "x"), iy(msg, "y"), iz(msg, "z");
  *ix = x;
  *iy = y;
  *iz = z;
  return msg;
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

constexpr char kInput[] = "/obstacle_grid_extractor/input/pointcloud";
constexpr char kOutput[] = "/obstacle_grid_extractor/output/obstacle_grid";
}  // namespace

TEST(ObstacleGridExtractorNode, PublishesBaseLinkGridWhenTfResolves)
{
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

  // a base_link cloud -> lookupTransform("base_link","base_link") is identity, so the happy path
  // runs. Re-publish until the grid comes back (absorbs discovery latency; the extractor is
  // idempotent).
  const bool got = spinUntil(
    exec,
    [&]() {
      pub->publish(makeCloud(helper->now(), "base_link", 8.05f, 0.05f, 0.5f));
      return static_cast<bool>(received);
    },
    std::chrono::seconds(10));
  ASSERT_TRUE(got) << "no grid published for a resolvable (base_link) cloud";

  grid_map::GridMap g;
  grid_map::GridMapRosConverter::fromMessage(*received, g);
  EXPECT_EQ(received->header.frame_id, "base_link");  // frame overwritten to base_link on publish
  EXPECT_EQ(g.getLayers().size(), 4u);
  grid_map::Index idx;
  ASSERT_TRUE(g.getIndex(grid_map::Position(8.05, 0.05), idx));
  EXPECT_FLOAT_EQ(g.at("point_count", idx), 1.0f);  // the one input point landed in its cell
}

TEST(ObstacleGridExtractorNode, DropsCloudWhenTfLookupFails)
{
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

  // Establish the link first so a subsequent "nothing arrived" is meaningful, not just
  // non-discovery.
  ASSERT_TRUE(spinUntil(
    exec, [&]() { return pub->get_subscription_count() > 0; }, std::chrono::seconds(10)))
    << "input subscription never discovered";

  // A cloud in an UNKNOWN frame: lookupTransform to base_link has no data and throws after its
  // timeout, so onCloud takes the catch branch and publishes nothing.
  const auto deadline = steady_clock::now() + std::chrono::milliseconds(1500);
  while (steady_clock::now() < deadline) {
    pub->publish(makeCloud(helper->now(), "lidar_top_that_has_no_tf", 8.05f, 0.05f, 0.5f));
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
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
