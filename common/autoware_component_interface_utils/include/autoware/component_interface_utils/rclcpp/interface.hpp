// Copyright 2022 TIER IV, Inc.
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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_

#include <autoware/component_interface_admission/records.hpp>
#include <autoware/component_interface_specs/concepts.hpp>
#include <autoware/component_interface_specs/version.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosidl_runtime_cpp/traits.hpp>

#include <tier4_system_msgs/msg/service_log.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

// Constrain the spec template parameter with the C++20 concept when the compiler offers it, and
// fall back to a plain template parameter under C++17. Universe packages compile at C++17 by
// default, so every existing C++17 consumer of this header must keep parsing; the concept is an
// opt-in improvement, never a hard requirement (component-interface-versioning.md section 4.2).
#if __cplusplus >= 202002L
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_TOPIC_SPEC \
  ::autoware::component_interface_specs::InterfaceSpec
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_SERVICE_SPEC \
  ::autoware::component_interface_specs::ServiceSpec
#else
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_TOPIC_SPEC class
#define AUTOWARE_COMPONENT_INTERFACE_UTILS_SERVICE_SPEC class
#endif

namespace autoware::component_interface_utils
{

/// The record's `ns` is the first path segment of `Spec::name` (a documented convention, so that
/// e.g. "/localization/kinematic_state" -> "localization"). Leading slashes are ignored and an
/// empty or slash-only name yields "".
inline std::string interface_namespace(const std::string & interface_name)
{
  const auto begin = interface_name.find_first_not_of('/');
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = interface_name.find('/', begin);
  return interface_name.substr(begin, end == std::string::npos ? std::string::npos : end - begin);
}

struct NodeInterface
{
  using SharedPtr = std::shared_ptr<NodeInterface>;
  using ServiceLog = tier4_system_msgs::msg::ServiceLog;

  explicit NodeInterface(rclcpp::Node * node)
  {
    this->node = node;
    this->logger = node->create_publisher<ServiceLog>("/service_log", 10);

    node_name = node->get_namespace();
    if (node_name.empty() || node_name.back() != '/') {
      node_name += "/";
    }
    node_name += node->get_name();

    // The owner is a compile-time constant of the OSS standard spec package (section 3.3): a
    // provider cannot fake the version it emits, only the consumer's acceptance range is mutable.
    manifest_.owner = autoware::component_interface_specs::owner;
  }

  void log(ServiceLog::_type_type type, const std::string & name, const std::string & yaml = "")
  {
    static const auto type_text = std::unordered_map<ServiceLog::_type_type, std::string>(
      {{ServiceLog::CLIENT_REQUEST, "client call"},
       {ServiceLog::SERVER_REQUEST, "server call"},
       {ServiceLog::SERVER_RESPONSE, "server exit"},
       {ServiceLog::CLIENT_RESPONSE, "client exit"},
       {ServiceLog::ERROR_UNREADY, "client unready"},
       {ServiceLog::ERROR_TIMEOUT, "client timeout"}});
    RCLCPP_DEBUG_STREAM(node->get_logger(), type_text.at(type) << ": " << name);

    ServiceLog msg;
    msg.stamp = node->now();
    msg.type = type;
    msg.name = name;
    msg.node = node_name;
    msg.yaml = yaml;
    logger->publish(msg);
  }

  /// Record a provided topic interface (this node is a publisher of it). The provider is the
  /// version authority: it emits `spec_version<S>()`, the owner spec version it was built against.
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_TOPIC_SPEC S>
  void register_provided(const std::string & resolved_name)
  {
    ensure_manifest_identity();
    autoware::component_interface_admission::ProvidedInterface record;
    record.ns = interface_namespace(S::name);
    record.interface_name = S::name;
    record.resolved_name = resolved_name;
    record.type_name = rosidl_generator_traits::name<typename S::Message>();
    const auto version = autoware::component_interface_specs::spec_version<S>();
    record.major = version.major;
    record.minor = version.minor;
    record.patch = version.patch;
    manifest_.provided.push_back(std::move(record));
  }

  /// Record a required topic interface (this node is a subscriber of it). The consumer declares an
  /// acceptance range, not a single version (section 3.3).
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_TOPIC_SPEC S>
  void register_required(
    autoware::component_interface_specs::accept_major accept, const std::string & resolved_name)
  {
    ensure_manifest_identity();
    autoware::component_interface_admission::RequiredInterface record;
    record.ns = interface_namespace(S::name);
    record.interface_name = S::name;
    record.resolved_name = resolved_name;
    record.type_name = rosidl_generator_traits::name<typename S::Message>();
    record.accept_major_min = accept.lo;
    record.accept_major_max = accept.hi;
    record.min_minor = 0;
    manifest_.required.push_back(std::move(record));
  }

  /// Record a provided service interface (this node is a service server of it).
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_SERVICE_SPEC S>
  void register_provided_service(const std::string & resolved_name)
  {
    ensure_manifest_identity();
    autoware::component_interface_admission::ProvidedInterface record;
    record.ns = interface_namespace(S::name);
    record.interface_name = S::name;
    record.resolved_name = resolved_name;
    record.type_name = rosidl_generator_traits::name<typename S::Service>();
    const auto version = autoware::component_interface_specs::spec_version<S>();
    record.major = version.major;
    record.minor = version.minor;
    record.patch = version.patch;
    manifest_.provided.push_back(std::move(record));
  }

  /// Record a required service interface (this node is a service client of it). The default
  /// acceptance range is the single MAJOR it was built against.
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_SERVICE_SPEC S>
  void register_required_service(const std::string & resolved_name)
  {
    ensure_manifest_identity();
    autoware::component_interface_admission::RequiredInterface record;
    record.ns = interface_namespace(S::name);
    record.interface_name = S::name;
    record.resolved_name = resolved_name;
    record.type_name = rosidl_generator_traits::name<typename S::Service>();
    const auto version = autoware::component_interface_specs::spec_version<S>();
    record.accept_major_min = version.major;
    record.accept_major_max = version.major;
    record.min_minor = 0;
    manifest_.required.push_back(std::move(record));
  }

  /// The interface manifest accumulated at the create_* / init_* choke points (section 4.3):
  /// the topics and services this node provides and requires. The runtime broadcast of this
  /// manifest and the admission checker are DEFERRED to Stage 2
  /// (component-interface-versioning.md section 0.5); today the manifest is inert and simply
  /// recorded for a future Stage-1 emitter / Stage-2 trigger to consume.
  const autoware::component_interface_admission::InterfaceManifest & manifest() const
  {
    return manifest_;
  }

  rclcpp::Node * node;
  rclcpp::Publisher<ServiceLog>::SharedPtr logger;
  std::string node_name;

private:
  // Fill the manifest's node identity lazily from the node the first time a role is registered.
  // get_fully_qualified_name() is the two-layer name's node component (section 3.4).
  void ensure_manifest_identity()
  {
    if (manifest_.node_name.empty()) {
      manifest_.node_name = node->get_fully_qualified_name();
    }
  }

  autoware::component_interface_admission::InterfaceManifest manifest_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP__INTERFACE_HPP_
