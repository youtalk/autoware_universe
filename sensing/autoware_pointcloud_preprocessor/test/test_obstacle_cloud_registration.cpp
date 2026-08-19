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

#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

// PassThroughFilterComponent is the simplest concrete Filter: it declares no
// parameters of its own beyond what the Filter base class already declares
// with defaults, so it constructs with no extra arguments. Any concrete
// Filter subclass exercises the base-class registration path equally.
#include "autoware/pointcloud_preprocessor/passthrough_filter/passthrough_filter_node.hpp"

namespace
{

rclcpp::NodeOptions options_with(const std::vector<std::string> & args)
{
  rclcpp::NodeOptions options;
  options.arguments(args);
  return options;
}

class FilterRegistrationTest : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }
};

TEST_F(FilterRegistrationTest, NonTerminalOutputConstructsFine)
{
  EXPECT_NO_THROW(
    autoware::pointcloud_preprocessor::PassThroughFilterComponent{
      options_with({"--ros-args", "-p", "max_queue_size:=5"})});
}

TEST_F(FilterRegistrationTest, TerminalWithSpecQosConstructsFine)
{
  // This is exactly the qos_overrides configuration Task 3 installs from
  // launch for a Filter remapped onto the obstacle-cloud boundary.
  EXPECT_NO_THROW(
    autoware::pointcloud_preprocessor::PassThroughFilterComponent{options_with(
      {"--ros-args", "-r", "output:=/perception/obstacle_segmentation/pointcloud", "-p",
       "qos_overrides./perception/obstacle_segmentation/pointcloud.publisher.reliability:=reliable",
       "-p", "qos_overrides./perception/obstacle_segmentation/pointcloud.publisher.depth:=5", "-p",
       "qos_overrides./perception/obstacle_segmentation/"
       "pointcloud.publisher.history:=keep_last"})});
}

TEST_F(FilterRegistrationTest, TerminalWithoutOverrideFailsStartup)
{
  // Without the qos_overrides, the output publisher keeps its default
  // rclcpp::SensorDataQoS(), i.e. BEST_EFFORT, which does not conform to the
  // spec (RELIABLE) — construction must throw.
  EXPECT_THROW(
    autoware::pointcloud_preprocessor::PassThroughFilterComponent{
      options_with({"--ros-args", "-r", "output:=/perception/obstacle_segmentation/pointcloud"})},
    std::runtime_error);
}

}  // namespace
