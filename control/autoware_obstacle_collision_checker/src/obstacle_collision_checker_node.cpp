// Copyright 2020-2024 Tier IV, Inc. All rights reserved.
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

#include "autoware/obstacle_collision_checker/obstacle_collision_checker_node.hpp"

#include "autoware/obstacle_collision_checker/debug.hpp"

#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_vehicle_info_utils/vehicle_info_utils.hpp>

#include <memory>
#include <string>
#include <vector>

namespace autoware::obstacle_collision_checker
{
ObstacleCollisionCheckerNode::ObstacleCollisionCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("obstacle_collision_checker_node", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  updater_(this)
{
  using std::placeholders::_1;

  // Node Parameter
  node_param_.update_rate = declare_parameter<double>("update_rate");
  node_param_.obstacle_grid_timeout_sec = declare_parameter<double>("obstacle_grid_timeout_sec");

  // Core Parameter
  input_.param.delay_time = declare_parameter<double>("delay_time");
  input_.param.footprint_margin = declare_parameter<double>("footprint_margin");
  input_.param.max_deceleration = declare_parameter<double>("max_deceleration");
  input_.param.resample_interval = declare_parameter<double>("resample_interval");
  input_.param.search_radius = declare_parameter<double>("search_radius");

  // Dynamic Reconfigure
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(
      &autoware::obstacle_collision_checker::ObstacleCollisionCheckerNode::param_callback, this,
      _1));

  // Subscriber
  self_pose_listener_ = std::make_shared<autoware_utils::SelfPoseListener>(this);
  transform_listener_ = std::make_shared<autoware_utils::TransformListener>(this);

  // The obstacle grid contract mandates RELIABLE KEEP_LAST(1) (rclcpp::QoS{1} defaults to
  // RELIABLE) — a deliberate policy change from the raw cloud's SensorDataQoS (BEST_EFFORT).
  sub_obstacle_grid_ = create_subscription<grid_map_msgs::msg::GridMap>(
    "input/obstacle_grid", rclcpp::QoS{1},
    std::bind(&ObstacleCollisionCheckerNode::on_obstacle_grid, this, _1));
  sub_reference_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/reference_trajectory", 1,
    std::bind(&ObstacleCollisionCheckerNode::on_reference_trajectory, this, _1));
  sub_predicted_trajectory_ = create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "input/predicted_trajectory", 1,
    std::bind(&ObstacleCollisionCheckerNode::on_predicted_trajectory, this, _1));
  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", 1, std::bind(&ObstacleCollisionCheckerNode::on_odom, this, _1));

  // Publisher
  debug_publisher_ = std::make_shared<autoware_utils::DebugPublisher>(this, "debug/marker");
  time_publisher_ = std::make_shared<autoware_utils::ProcessingTimePublisher>(this);

  // Diagnostic Updater
  updater_.setHardwareID("obstacle_collision_checker");

  updater_.add(
    "obstacle_collision_checker", this, &ObstacleCollisionCheckerNode::check_lane_departure);

  // Wait for first self pose
  self_pose_listener_->wait_for_first_pose();

  // Timer
  init_timer(1.0 / node_param_.update_rate);
}

void ObstacleCollisionCheckerNode::on_obstacle_grid(
  const grid_map_msgs::msg::GridMap::SharedPtr msg)
{
  obstacle_grid_ = msg;
}

void ObstacleCollisionCheckerNode::on_reference_trajectory(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  reference_trajectory_ = msg;
}

void ObstacleCollisionCheckerNode::on_predicted_trajectory(
  const autoware_planning_msgs::msg::Trajectory::SharedPtr msg)
{
  predicted_trajectory_ = msg;
}

void ObstacleCollisionCheckerNode::on_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_twist_ = std::make_shared<geometry_msgs::msg::Twist>(msg->twist.twist);
}

void ObstacleCollisionCheckerNode::init_timer(double period_s)
{
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(period_s));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ObstacleCollisionCheckerNode::on_timer, this));
}

bool ObstacleCollisionCheckerNode::is_data_ready()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_pose...");
    return false;
  }

  if (!obstacle_grid_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for obstacle_grid msg...");
    return false;
  }

  if (!obstacle_transform_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for obstacle_transform...");
    return false;
  }

  if (!reference_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "waiting for predicted_trajectory msg...");
    return false;
  }

  if (!current_twist_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current_twist msg...");
    return false;
  }

  return true;
}

bool ObstacleCollisionCheckerNode::is_data_timeout()
{
  const auto now = this->now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp).seconds() - now.seconds();
  if (pose_time_diff > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "pose is timeout...");
    return true;
  }

  return false;
}

void ObstacleCollisionCheckerNode::on_timer()
{
  current_pose_ = self_pose_listener_->get_current_pose();

  // Reset every tick: set true only on the stale / contract-violation paths below.
  obstacle_grid_unavailable_ = false;

  if (obstacle_grid_) {
    // Staleness watchdog: the grid is cached and re-read every tick, and the producer stays silent
    // on its own failure paths, so a frozen grid must read as "unavailable", never as "clear".
    if (
      is_grid_stale(
        rclcpp::Time(obstacle_grid_->header.stamp), this->now(),
        node_param_.obstacle_grid_timeout_sec)) {
      obstacle_grid_unavailable_ = true;
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 5000 /* ms */,
        "obstacle grid is stale; treating it as unavailable");
      updater_.force_update();
      return;
    }

    const auto & header = obstacle_grid_->header;
    try {
      obstacle_transform_ = transform_listener_->get_transform(
        "map", header.frame_id, header.stamp, rclcpp::Duration::from_seconds(0.01));
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform map to %s: %s", header.frame_id.c_str(),
        ex.what());
      return;
    }
  }

  if (!is_data_ready()) {
    return;
  }

  if (is_data_timeout()) {
    return;
  }

  // Convert the obstacle grid into the synthetic corner-point cloud that feeds the unchanged
  // corridor-membership pipeline. A contract violation (wrong frame / missing layers) yields
  // nullopt and must read as "unavailable", never as "clear".
  const auto grid_pointcloud = extract_grid_obstacle_pointcloud(*obstacle_grid_);
  if (!grid_pointcloud) {
    obstacle_grid_unavailable_ = true;
    updater_.force_update();
    return;
  }

  input_.current_pose = current_pose_;
  input_.obstacle_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>(*grid_pointcloud);
  input_.obstacle_transform = obstacle_transform_;
  input_.reference_trajectory = reference_trajectory_;
  input_.predicted_trajectory = predicted_trajectory_;
  input_.current_twist = current_twist_;
  input_.vehicle_info = vehicle_info_;

  output_ = check_for_collisions(input_);

  updater_.force_update();

  debug_publisher_->publish(
    "marker_array", create_marker_array(output_, current_pose_->pose.position.z, this->now()));

  time_publisher_->publish(output_.processing_time_map);
}

rcl_interfaces::msg::SetParametersResult ObstacleCollisionCheckerNode::param_callback(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  try {
    using autoware_utils::update_param;
    // Node Parameter
    {
      auto & p = node_param_;

      // Update params
      update_param(parameters, "update_rate", p.update_rate);
      update_param(parameters, "obstacle_grid_timeout_sec", p.obstacle_grid_timeout_sec);
    }

    auto & p = input_.param;

    update_param(parameters, "delay_time", p.delay_time);
    update_param(parameters, "footprint_margin", p.footprint_margin);
    update_param(parameters, "max_deceleration", p.max_deceleration);
    update_param(parameters, "resample_interval", p.resample_interval);
    update_param(parameters, "search_radius", p.search_radius);
  } catch (const rclcpp::exceptions::InvalidParameterTypeException & e) {
    result.successful = false;
    result.reason = e.what();
  }
  return result;
}

void ObstacleCollisionCheckerNode::check_lane_departure(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  // An unavailable grid (stale or contract-violating) must read as ERROR, never as the last
  // computed will_collide value (which the diagnostic_updater otherwise republishes on its own
  // timer): staleness must never read as "clear".
  if (obstacle_grid_unavailable_) {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "obstacle grid is unavailable");
    return;
  }

  int8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  std::string msg = "OK";

  if (output_.will_collide) {
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    msg = "vehicle will collide with obstacles";
  }

  stat.summary(level, msg);
}
}  // namespace autoware::obstacle_collision_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::obstacle_collision_checker::ObstacleCollisionCheckerNode)
