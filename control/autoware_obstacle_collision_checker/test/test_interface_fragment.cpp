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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <autoware/component_interface_specs/perception.hpp>
#include <autoware/component_interface_utils/rclcpp.hpp>
#include <autoware/component_interface_utils/testing/manifest_drift.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>

// Pins the required-record content admission consumes (spec name, type, QoS,
// domain version) against the committed fragment. The production node's own
// registration is by construction: its subscription is created through
// create_subscription<Spec>, the layer that registers.
TEST(InterfaceFragment, RequiredObstacleSegmentationMatchesCommittedFragment)
{
  rclcpp::init(0, nullptr);
  {
    auto node = std::make_shared<rclcpp::Node>("obstacle_collision_checker_node");
    autoware::component_interface_utils::NodeAdaptor adaptor(node.get());
    auto sub =
      adaptor
        .create_subscription<autoware::component_interface_specs::perception::ObstacleSegmentation>(
          nullptr);
    const auto fragment =
      ament_index_cpp::get_package_share_directory("autoware_obstacle_collision_checker") +
      "/config/interface_manifest_fragment.json";
    autoware::component_interface_utils::testing::expect_manifest_matches(
      adaptor, "obstacle_collision_checker_node", fragment);
  }
  rclcpp::shutdown();
}
