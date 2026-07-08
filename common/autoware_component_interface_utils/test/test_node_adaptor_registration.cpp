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

#include "autoware/component_interface_utils/rclcpp.hpp"

#include <autoware/component_interface_specs/localization.hpp>
#include <autoware/component_interface_specs/version.hpp>
#include <rclcpp/rclcpp.hpp>

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

namespace specs = autoware::component_interface_specs;
namespace utils = autoware::component_interface_utils;

using KinematicState = specs::localization::KinematicState;  // topic: nav_msgs/msg/Odometry
using Initialize = specs::localization::Initialize;          // service

// KinematicState and Initialize both live in the localization domain, whose spec version is 0.1.0.
constexpr std::uint16_t kMajor = 0;
constexpr std::uint16_t kMinor = 1;
constexpr std::uint16_t kPatch = 0;

class NodeAdaptorRegistration : public ::testing::Test
{
protected:
  void SetUp() override { rclcpp::init(0, nullptr); }
  void TearDown() override { rclcpp::shutdown(); }

  static std::shared_ptr<rclcpp::Node> make_node(
    const std::string & name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  {
    return std::make_shared<rclcpp::Node>(name, options);
  }

  // A no-op service callback for Initialize.
  static void on_initialize(
    Initialize::Service::Request::SharedPtr, Initialize::Service::Response::SharedPtr)
  {
  }
};

TEST_F(NodeAdaptorRegistration, freshNodeHasEmptyManifest)
{
  const auto node = make_node("fresh_node");
  const utils::NodeAdaptor adaptor(node.get());

  const auto & manifest = adaptor.manifest();
  EXPECT_EQ(manifest.owner, "autowarefoundation");
  EXPECT_TRUE(manifest.provided.empty());
  EXPECT_TRUE(manifest.required.empty());
}

TEST_F(NodeAdaptorRegistration, createPublisherRegistersProvidedRole)
{
  const auto node = make_node("provider_node");
  const utils::NodeAdaptor adaptor(node.get());
  const auto pub = adaptor.create_publisher<KinematicState>();

  const auto & manifest = adaptor.manifest();
  EXPECT_EQ(manifest.node_name, "/provider_node");
  ASSERT_EQ(manifest.provided.size(), 1u);
  EXPECT_TRUE(manifest.required.empty());

  const auto & record = manifest.provided.front();
  EXPECT_EQ(record.ns, "localization");
  EXPECT_EQ(record.interface_name, "/localization/kinematic_state");
  EXPECT_EQ(record.resolved_name, "/localization/kinematic_state");
  EXPECT_EQ(record.type_name, "nav_msgs/msg/Odometry");
  EXPECT_EQ(record.major, kMajor);
  EXPECT_EQ(record.minor, kMinor);
  EXPECT_EQ(record.patch, kPatch);
}

TEST_F(NodeAdaptorRegistration, createSubscriptionRegistersRequiredRoleWithDefaultAcceptance)
{
  const auto node = make_node("consumer_node");
  const utils::NodeAdaptor adaptor(node.get());
  const auto sub =
    adaptor.create_subscription<KinematicState>([](KinematicState::Message::ConstSharedPtr) {});

  const auto & manifest = adaptor.manifest();
  EXPECT_TRUE(manifest.provided.empty());
  ASSERT_EQ(manifest.required.size(), 1u);

  const auto & record = manifest.required.front();
  EXPECT_EQ(record.ns, "localization");
  EXPECT_EQ(record.interface_name, "/localization/kinematic_state");
  EXPECT_EQ(record.resolved_name, "/localization/kinematic_state");
  EXPECT_EQ(record.type_name, "nav_msgs/msg/Odometry");
  // Default acceptance = the single MAJOR built against.
  EXPECT_EQ(record.accept_major_min, kMajor);
  EXPECT_EQ(record.accept_major_max, kMajor);
  EXPECT_EQ(record.min_minor, 0u);
}

TEST_F(NodeAdaptorRegistration, createSubscriptionHonorsAcceptMajorOverride)
{
  const auto node = make_node("migrating_consumer_node");
  const utils::NodeAdaptor adaptor(node.get());
  const auto sub = adaptor.create_subscription<KinematicState>(
    [](KinematicState::Message::ConstSharedPtr) {}, specs::accept_major{2, 3});

  const auto & manifest = adaptor.manifest();
  ASSERT_EQ(manifest.required.size(), 1u);
  const auto & record = manifest.required.front();
  EXPECT_EQ(record.accept_major_min, 2u);
  EXPECT_EQ(record.accept_major_max, 3u);
}

TEST_F(NodeAdaptorRegistration, createServiceRegistersProvidedRole)
{
  const auto node = make_node("service_server_node");
  const utils::NodeAdaptor adaptor(node.get());
  const auto srv = adaptor.create_service<Initialize>(&on_initialize);

  const auto & manifest = adaptor.manifest();
  ASSERT_EQ(manifest.provided.size(), 1u);
  EXPECT_TRUE(manifest.required.empty());

  const auto & record = manifest.provided.front();
  EXPECT_EQ(record.ns, "localization");
  EXPECT_EQ(record.interface_name, "/localization/initialize");
  EXPECT_EQ(record.resolved_name, "/localization/initialize");
  EXPECT_EQ(record.type_name, "autoware_localization_msgs/srv/InitializeLocalization");
  EXPECT_EQ(record.major, kMajor);
  EXPECT_EQ(record.minor, kMinor);
  EXPECT_EQ(record.patch, kPatch);
}

TEST_F(NodeAdaptorRegistration, createClientRegistersRequiredRoleWithDefaultAcceptance)
{
  const auto node = make_node("service_client_node");
  const utils::NodeAdaptor adaptor(node.get());
  const auto cli = adaptor.create_client<Initialize>();

  const auto & manifest = adaptor.manifest();
  EXPECT_TRUE(manifest.provided.empty());
  ASSERT_EQ(manifest.required.size(), 1u);

  const auto & record = manifest.required.front();
  EXPECT_EQ(record.ns, "localization");
  EXPECT_EQ(record.interface_name, "/localization/initialize");
  EXPECT_EQ(record.resolved_name, "/localization/initialize");
  EXPECT_EQ(record.type_name, "autoware_localization_msgs/srv/InitializeLocalization");
  EXPECT_EQ(record.accept_major_min, kMajor);
  EXPECT_EQ(record.accept_major_max, kMajor);
}

TEST_F(NodeAdaptorRegistration, initHelpersAlsoRegisterTheSameRoles)
{
  const auto node = make_node("init_node");
  const utils::NodeAdaptor adaptor(node.get());

  utils::Publisher<KinematicState>::SharedPtr pub;
  utils::Subscription<KinematicState>::SharedPtr sub;
  utils::Service<Initialize>::SharedPtr srv;
  utils::Client<Initialize>::SharedPtr cli;

  adaptor.init_pub(pub);
  adaptor.init_sub(sub, [](KinematicState::Message::ConstSharedPtr) {});
  adaptor.init_srv(srv, &on_initialize);
  adaptor.init_cli(cli);

  const auto & manifest = adaptor.manifest();
  // init_pub -> provided topic, init_srv -> provided service.
  ASSERT_EQ(manifest.provided.size(), 2u);
  // init_sub -> required topic, init_cli -> required service.
  ASSERT_EQ(manifest.required.size(), 2u);

  EXPECT_EQ(manifest.provided[0].interface_name, "/localization/kinematic_state");
  EXPECT_EQ(manifest.provided[1].interface_name, "/localization/initialize");
  EXPECT_EQ(manifest.required[0].interface_name, "/localization/kinematic_state");
  EXPECT_EQ(manifest.required[0].accept_major_min, kMajor);
  EXPECT_EQ(manifest.required[0].accept_major_max, kMajor);
  EXPECT_EQ(manifest.required[1].interface_name, "/localization/initialize");
}

TEST_F(NodeAdaptorRegistration, resolvedNameReflectsRemapWhileInterfaceNameStaysTheSpecName)
{
  rclcpp::NodeOptions options;
  options.arguments({"--ros-args", "-r", "/localization/kinematic_state:=/remapped/odometry"});
  const auto node = make_node("remapped_node", options);
  const utils::NodeAdaptor adaptor(node.get());
  const auto pub = adaptor.create_publisher<KinematicState>();

  const auto & manifest = adaptor.manifest();
  ASSERT_EQ(manifest.provided.size(), 1u);
  const auto & record = manifest.provided.front();
  // The contract identity is remap-invariant, ...
  EXPECT_EQ(record.interface_name, "/localization/kinematic_state");
  // ... while the resolved name reflects the launch-time remap (section 3.4 two-layer name).
  EXPECT_EQ(record.resolved_name, "/remapped/odometry");
  EXPECT_STREQ(pub->get_topic_name(), "/remapped/odometry");
}
