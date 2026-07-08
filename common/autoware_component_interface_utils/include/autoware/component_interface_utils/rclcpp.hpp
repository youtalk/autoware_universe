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

#ifndef AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
#define AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_

#include <autoware/component_interface_admission/records.hpp>
#include <autoware/component_interface_specs/version.hpp>
#include <autoware/component_interface_utils/rclcpp/create_interface.hpp>
#include <autoware/component_interface_utils/rclcpp/interface.hpp>
#include <autoware/component_interface_utils/rclcpp/service_client.hpp>
#include <autoware/component_interface_utils/rclcpp/service_server.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_publisher.hpp>
#include <autoware/component_interface_utils/rclcpp/topic_subscription.hpp>

#include <memory>
#include <optional>
#include <utility>

namespace autoware::component_interface_utils
{

class NodeAdaptor
{
private:
  using CallbackGroup = rclcpp::CallbackGroup::SharedPtr;

  template <class SharedPtrT, class InstanceT>
  using MessagePtrCallback =
    void (InstanceT::*)(const typename SharedPtrT::element_type::SpecType::Message::ConstSharedPtr);
  template <class SharedPtrT, class InstanceT>
  using MessageRefCallback =
    void (InstanceT::*)(const typename SharedPtrT::element_type::SpecType::Message &);

  template <class SharedPtrT, class InstanceT>
  using ServiceCallback = void (InstanceT::*)(
    const typename SharedPtrT::element_type::SpecType::Service::Request::SharedPtr,
    const typename SharedPtrT::element_type::SpecType::Service::Response::SharedPtr);

  // The default consumer acceptance range is the single MAJOR the node was built against (the
  // owner spec version it compiled against). Widen it explicitly during a migration window.
  template <class SpecT>
  static autoware::component_interface_specs::accept_major default_accept_major()
  {
    const auto version = autoware::component_interface_specs::spec_version<SpecT>();
    return {version.major, version.major};
  }

public:
  /// Constructor.
  explicit NodeAdaptor(rclcpp::Node * node) { interface_ = std::make_shared<NodeInterface>(node); }

  /// Create a client wrapper for logging.
  template <class SharedPtrT>
  void init_cli(SharedPtrT & cli, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    cli = create_client_impl<SpecT>(interface_, group);
    interface_->register_required_service<SpecT>(cli->get_service_name());
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class CallbackT>
  void init_srv(SharedPtrT & srv, CallbackT && callback, CallbackGroup group = nullptr) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    srv = create_service_impl<SpecT>(interface_, std::forward<CallbackT>(callback), group);
    interface_->register_provided_service<SpecT>(srv->get_service_name());
  }

  /// Create a publisher using traits like services.
  template <class SharedPtrT>
  void init_pub(SharedPtrT & pub) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    pub = create_publisher_impl<SpecT>(interface_->node);
    interface_->register_provided<SpecT>(pub->get_topic_name());
  }

  /// Create a subscription using traits like services.
  template <class SharedPtrT, class CallbackT>
  void init_sub(SharedPtrT & sub, CallbackT && callback) const
  {
    using SpecT = typename SharedPtrT::element_type::SpecType;
    sub = create_subscription_impl<SpecT>(interface_->node, std::forward<CallbackT>(callback));
    interface_->register_required<SpecT>(default_accept_major<SpecT>(), sub->get_topic_name());
  }

  /// Create a publisher and register it as a provided interface (section 4.3, provider role).
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_TOPIC_SPEC S>
  typename Publisher<S>::SharedPtr create_publisher() const
  {
    auto publisher = create_publisher_impl<S>(interface_->node);
    interface_->register_provided<S>(publisher->get_topic_name());
    return publisher;
  }

  /// Create a subscription and register it as a required interface (section 4.3, consumer role).
  /// The acceptance range defaults to the single MAJOR built against; widen it with accept_major.
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_TOPIC_SPEC S, class CallbackT>
  typename Subscription<S>::SharedPtr create_subscription(
    CallbackT && callback,
    autoware::component_interface_specs::accept_major accept = default_accept_major<S>()) const
  {
    auto subscription =
      create_subscription_impl<S>(interface_->node, std::forward<CallbackT>(callback));
    interface_->register_required<S>(accept, subscription->get_topic_name());
    return subscription;
  }

  /// Create a service server and register it as a provided interface (section 4.3, provider role).
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_SERVICE_SPEC S, class CallbackT>
  typename Service<S>::SharedPtr create_service(
    CallbackT && callback, CallbackGroup group = nullptr) const
  {
    auto service = create_service_impl<S>(interface_, std::forward<CallbackT>(callback), group);
    interface_->register_provided_service<S>(service->get_service_name());
    return service;
  }

  /// Create a service client and register it as a required interface (section 4.3, consumer role).
  template <AUTOWARE_COMPONENT_INTERFACE_UTILS_SERVICE_SPEC S>
  typename Client<S>::SharedPtr create_client(CallbackGroup group = nullptr) const
  {
    auto client = create_client_impl<S>(interface_, group);
    interface_->register_required_service<S>(client->get_service_name());
    return client;
  }

  /// The interface manifest accumulated from the create_* / init_* calls on this node. See
  /// NodeInterface::manifest(): the runtime broadcast / admission checker are DEFERRED to Stage 2
  /// (component-interface-versioning.md section 0.5), so the manifest is inert today.
  const autoware::component_interface_admission::InterfaceManifest & manifest() const
  {
    return interface_->manifest();
  }

  /// Relay message.
  template <class P, class S>
  void relay_message(P & pub, S & sub) const
  {
    using MsgT = typename P::element_type::SpecType::Message::ConstSharedPtr;
    init_pub(pub);
    init_sub(sub, [pub](MsgT msg) { pub->publish(*msg); });
  }

  /// Relay service.
  template <class C, class S>
  void relay_service(
    C & cli, S & srv, CallbackGroup group, std::optional<double> timeout = std::nullopt) const
  {
    init_cli(cli);
    init_srv(srv, [cli, timeout](auto req, auto res) { *res = *cli->call(req, timeout); }, group);
  }

  /// Create a subscription wrapper for pointer callback.
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessagePtrCallback<SharedPtrT, InstanceT> && callback) const
  {
    using std::placeholders::_1;
    init_sub(sub, std::bind(callback, instance, _1));
  }

  /// Create a subscription wrapper for reference callback.
  template <class SharedPtrT, class InstanceT>
  void init_sub(
    SharedPtrT & sub, InstanceT * instance,
    MessageRefCallback<SharedPtrT, InstanceT> && callback) const
  {
    using std::placeholders::_1;
    init_sub(sub, std::bind(callback, instance, _1));
  }

  /// Create a service wrapper for logging.
  template <class SharedPtrT, class InstanceT>
  void init_srv(
    SharedPtrT & srv, InstanceT * instance, ServiceCallback<SharedPtrT, InstanceT> && callback,
    CallbackGroup group = nullptr) const
  {
    using std::placeholders::_1;
    using std::placeholders::_2;
    init_srv(srv, std::bind(callback, instance, _1, _2), group);
  }

private:
  // Use a node pointer because shared_from_this cannot be used in constructor.
  NodeInterface::SharedPtr interface_;
};

}  // namespace autoware::component_interface_utils

#endif  // AUTOWARE__COMPONENT_INTERFACE_UTILS__RCLCPP_HPP_
