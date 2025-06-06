// Copyright 2023 TIER IV, Inc.
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

#ifndef MANAGER_HPP_
#define MANAGER_HPP_

#include "scene.hpp"

#include <autoware/behavior_velocity_planner_common/plugin_interface.hpp>
#include <autoware/behavior_velocity_planner_common/plugin_wrapper.hpp>
#include <autoware/behavior_velocity_planner_common/scene_module_interface.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_internal_planning_msgs/msg/path_with_lane_id.hpp>

#include <functional>
#include <memory>

namespace autoware::behavior_velocity_planner
{
/**
 * @brief Constructor for the TemplateModuleManager class.
 *
 * Initializes a TemplateModuleManager instance with the provided ROS node, setting up essential
 * parameters.
 *
 * @param node A reference to the ROS node.
 */
class TemplateModuleManager
: public autoware::behavior_velocity_planner::SceneModuleManagerInterface<>
{
public:
  explicit TemplateModuleManager(rclcpp::Node & node);

  /**
   * @brief Get the name of the module.
   *
   * This method returns a constant character string representing the name of the module, which in
   * this case is "template."
   *
   * @return A pointer to a constant character string containing the module name.
   */
  const char * getModuleName() override { return "template"; }

  RequiredSubscriptionInfo getRequiredSubscriptions() const override
  {
    return RequiredSubscriptionInfo{};
  }

private:
  double dummy_parameter_{0.0};

  /**
   * @brief Launch new modules based on the provided path.
   *
   * This method is responsible for launching new modules based on the information provided in the
   * given path.
   *
   * @param path The path with lane ID information to determine module launch.
   */
  void launchNewModules(const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;

  /**
   * @brief Get a function to check module expiration.
   *
   * This method returns a function that can be used to determine whether a specific module has
   * expired based on the given path information.
   *
   * @param path The path with lane ID information for module expiration check.
   * @return A function for checking module expiration.
   */
  std::function<bool(const std::shared_ptr<SceneModuleInterface> &)> getModuleExpiredFunction(
    const autoware_internal_planning_msgs::msg::PathWithLaneId & path) override;
};

/**
 * @brief Plugin class for TemplateModuleManager.
 *
 * The TemplateModulePlugin class is used to integrate the TemplateModuleManager into the Behavior
 * Velocity Planner.
 */
class TemplateModulePlugin
: public autoware::behavior_velocity_planner::PluginWrapper<TemplateModuleManager>
{
};

}  // namespace autoware::behavior_velocity_planner

#endif  // MANAGER_HPP_
