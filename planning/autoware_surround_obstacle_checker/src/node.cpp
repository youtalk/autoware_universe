// Copyright 2020 Tier IV, Inc.
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

#include "node.hpp"

#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <autoware_utils_geometry/geometry.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>

#include <boost/geometry.hpp>

#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace autoware::surround_obstacle_checker
{

using autoware_perception_msgs::msg::ObjectClassification;

SurroundObstacleCheckerNode::SurroundObstacleCheckerNode(const rclcpp::NodeOptions & node_options)
: Node("surround_obstacle_checker_node", node_options)
{
  label_map_ = {
    {ObjectClassification::UNKNOWN, "unknown"},
    {ObjectClassification::CAR, "car"},
    {ObjectClassification::TRUCK, "truck"},
    {ObjectClassification::BUS, "bus"},
    {ObjectClassification::TRAILER, "trailer"},
    {ObjectClassification::MOTORCYCLE, "motorcycle"},
    {ObjectClassification::BICYCLE, "bicycle"},
    {ObjectClassification::PEDESTRIAN, "pedestrian"},
    {ObjectClassification::ANIMAL, "animal"},
    {ObjectClassification::HAZARD, "hazard"},
    {ObjectClassification::OVER_DRIVABLE, "over_drivable"},
    {ObjectClassification::UNDER_DRIVABLE, "under_drivable"}};
  // Parameters
  {
    param_listener_ = std::make_shared<surround_obstacle_checker_node::ParamListener>(
      this->get_node_parameters_interface());

    logger_configure_ = std::make_unique<autoware_utils::LoggerLevelConfigure>(this);
  }

  vehicle_info_ = autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo();

  proximity_checker_ = std::make_unique<obstacle_proximity_checker::ProximityChecker>(
    toProximityCheckerParameters(), vehicle_info_);

  // Publishers
  pub_clear_velocity_limit_ = this->create_publisher<VelocityLimitClearCommand>(
    "~/output/velocity_limit_clear_command", rclcpp::QoS{1}.transient_local());
  pub_velocity_limit_ = this->create_publisher<VelocityLimit>(
    "~/output/max_velocity", rclcpp::QoS{1}.transient_local());
  pub_processing_time_ = this->create_publisher<autoware_internal_debug_msgs::msg::Float64Stamped>(
    "~/debug/processing_time_ms", 1);

  using std::chrono_literals::operator""ms;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 100ms, std::bind(&SurroundObstacleCheckerNode::onTimer, this));

  // Stop Checker
  vehicle_stop_checker_ = std::make_unique<VehicleStopChecker>(this);

  // Debug
  odometry_ptr_ = std::make_shared<nav_msgs::msg::Odometry>();
  {
    const auto param = param_listener_->get_params();
    const auto check_distances = getCheckDistances(param.debug_footprint_label);
    debug_ptr_ = std::make_shared<SurroundObstacleCheckerDebugNode>(
      vehicle_info_, param.debug_footprint_label, check_distances.at(0), check_distances.at(1),
      check_distances.at(2), param.surround_check_hysteresis_distance, odometry_ptr_->pose.pose,
      this->get_clock(), *this);
  }
}

std::array<double, 3> SurroundObstacleCheckerNode::getCheckDistances(
  const std::string & str_label) const
{
  const auto param = param_listener_->get_params();
  const auto & obstacle_param = param.obstacle_types_map.at(str_label);
  return {
    obstacle_param.surround_check_front_distance, obstacle_param.surround_check_side_distance,
    obstacle_param.surround_check_back_distance};
}

bool SurroundObstacleCheckerNode::getUseDynamicObject() const
{
  const auto param = param_listener_->get_params();
  bool use_dynamic_object = false;
  for (const auto & label_pair : label_map_) {
    use_dynamic_object |= param.object_types_map.at(label_pair.second).enable_check;
  }
  return use_dynamic_object;
}

obstacle_proximity_checker::Parameters SurroundObstacleCheckerNode::toProximityCheckerParameters()
  const
{
  const auto param = param_listener_->get_params();

  obstacle_proximity_checker::Parameters parameters;
  parameters.pointcloud_enable_check = param.pointcloud.enable_check;

  for (const auto & [label, object_type_param] : param.object_types_map) {
    parameters.object_type_enable_check[label] = object_type_param.enable_check;
  }

  for (const auto & [label, obstacle_type_param] : param.obstacle_types_map) {
    obstacle_proximity_checker::ObstacleTypeParameters obstacle_parameters;
    obstacle_parameters.surround_check_front_distance =
      obstacle_type_param.surround_check_front_distance;
    obstacle_parameters.surround_check_side_distance =
      obstacle_type_param.surround_check_side_distance;
    obstacle_parameters.surround_check_back_distance =
      obstacle_type_param.surround_check_back_distance;
    parameters.obstacle_types_map[label] = obstacle_parameters;
  }

  return parameters;
}

void SurroundObstacleCheckerNode::onTimer()
{
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;
  stop_watch.tic();

  odometry_ptr_ = sub_odometry_.take_data();
  obstacle_grid_ptr_ = sub_obstacle_grid_.take_data();
  object_ptr_ = sub_dynamic_objects_.take_data();

  if (!odometry_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for current velocity...");
    return;
  }

  const auto param = param_listener_->get_params();
  proximity_checker_->update_parameters(toProximityCheckerParameters());
  const auto use_dynamic_object = getUseDynamicObject();

  if (param.publish_debug_footprints) {
    debug_ptr_->publishFootprints();
  }

  if (param.pointcloud.enable_check && !obstacle_grid_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for obstacle grid info...");
  }

  if (use_dynamic_object && !object_ptr_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */, "waiting for dynamic object info...");
  }

  if (!param.pointcloud.enable_check && !use_dynamic_object) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000 /* ms */,
      "Surround obstacle check is disabled for all dynamic object types and for pointcloud check.");
  }

  // Convert the obstacle grid once per tick. std::nullopt marks the grid as unavailable, which the
  // STOP branch below must not mistake for "clear".
  const auto obstacle_grid_pointcloud = toObstacleGridPointCloud();
  const bool obstacle_grid_available = obstacle_grid_pointcloud.has_value();

  const double contact_distance_threshold =
    state_ == State::STOP ? param.surround_check_hysteresis_distance : 1e-3;
  const auto proximity_result = proximity_checker_->check(
    toProximityCheckerInputs(obstacle_grid_pointcloud), contact_distance_threshold);
  const auto is_vehicle_stopped = vehicle_stop_checker_->isVehicleStopped();

  switch (state_) {
    case State::PASS: {
      bool is_stop_required = false;
      std::tie(is_stop_required, last_obstacle_found_time_) = isStopRequired(
        proximity_result.is_obstacle_found, is_vehicle_stopped, state_, last_obstacle_found_time_,
        param.state_clear_time);
      if (!is_stop_required) {
        break;
      }

      state_ = State::STOP;

      auto velocity_limit = std::make_shared<VelocityLimit>();
      velocity_limit->stamp = this->now();
      velocity_limit->max_velocity = 0.0;
      velocity_limit->use_constraints = false;
      velocity_limit->sender = "surround_obstacle_checker";

      pub_velocity_limit_->publish(*velocity_limit);

      // do not start when there is a obstacle near the ego vehicle.
      RCLCPP_WARN(get_logger(), "do not start because there is obstacle near the ego vehicle.");

      break;
    }

    case State::STOP: {
      bool is_stop_required = false;
      std::tie(is_stop_required, last_obstacle_found_time_) = isStopRequired(
        proximity_result.is_obstacle_found, is_vehicle_stopped, state_, last_obstacle_found_time_,
        param.state_clear_time);
      if (is_stop_required) {
        break;
      }

      // Fail-safe hold (no-start guard): while the pointcloud check is enabled but its obstacle
      // grid is unavailable (stale / wrong-frame / missing-layer / not-yet-received), grid silence
      // must never be read as "clear". Hold the latched stop instead of releasing it on the grid's
      // account; a fresh valid grid restores normal hysteresis-based clearing.
      if (!obstacle_grid_available) {
        break;
      }

      state_ = State::PASS;

      auto velocity_limit_clear_command = std::make_shared<VelocityLimitClearCommand>();
      velocity_limit_clear_command->stamp = this->now();
      velocity_limit_clear_command->command = true;
      velocity_limit_clear_command->sender = "surround_obstacle_checker";

      pub_clear_velocity_limit_->publish(*velocity_limit_clear_command);

      break;
    }

    default:
      break;
  }

  if (proximity_result.nearest_obstacle.has_value()) {
    debug_ptr_->pushStopObstacle(proximity_result.nearest_obstacle);
  }

  if (state_ == State::STOP) {
    debug_ptr_->pushPose(odometry_ptr_->pose.pose, PoseType::NoStart);
  }

  autoware_internal_debug_msgs::msg::Float64Stamped processing_time_msg;
  processing_time_msg.stamp = get_clock()->now();
  processing_time_msg.data = stop_watch.toc();
  pub_processing_time_->publish(processing_time_msg);

  debug_ptr_->publish();
}

std::optional<pcl::PointCloud<pcl::PointXYZ>::ConstPtr>
SurroundObstacleCheckerNode::toObstacleGridPointCloud() const
{
  const auto param = param_listener_->get_params();

  // The pointcloud check is disabled: it neither reports obstacles nor blocks clearing, so the grid
  // counts as available-and-empty. ProximityChecker short-circuits on the same flag.
  if (!param.pointcloud.enable_check) {
    return std::make_shared<const pcl::PointCloud<pcl::PointXYZ>>();
  }

  // No grid received yet: unavailable (onTimer emits the throttled "waiting" log). Silence from a
  // yet-to-arrive grid must be held as "unknown", never read as "clear".
  if (!obstacle_grid_ptr_) {
    return std::nullopt;
  }

  // Contract validation, modeled on the already-migrated AEB intake. The old raw-cloud path
  // TF-transformed a mismatched frame; a grid cannot be re-framed cheaply, so a frame mismatch is a
  // wiring error: reject loudly and treat as unavailable. Convertibility, the required layers, and
  // the staleness watchdog are likewise fail-safe -> unavailable, never clear.
  const auto & msg = *obstacle_grid_ptr_;
  if (msg.header.frame_id != "base_link") {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /* ms */,
      "obstacle grid frame is '%s', expected 'base_link'; treating it as unavailable",
      msg.header.frame_id.c_str());
    return std::nullopt;
  }
  grid_map::GridMap grid;
  if (!grid_map::GridMapRosConverter::fromMessage(msg, grid)) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /* ms */,
      "failed to convert the obstacle grid message; treating it as unavailable");
    return std::nullopt;
  }
  // cell_qualifies reads only the point_count and max_height layers, so those are the only two the
  // gate requires; a missing layer would otherwise throw std::out_of_range and kill the container.
  for (const char * layer : {"point_count", "max_height"}) {
    if (!grid.exists(layer)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000 /* ms */,
        "obstacle grid is missing the '%s' layer; treating it as unavailable", layer);
      return std::nullopt;
    }
  }
  // Staleness watchdog: the polling subscriber returns the last received grid forever, and the
  // producer deliberately publishes nothing on its failure paths, so a frozen grid must read as
  // "unavailable", never as "clear".
  const double grid_age_sec = (this->now() - rclcpp::Time(msg.header.stamp)).seconds();
  if (grid_age_sec > param.pointcloud.obstacle_grid_timeout_sec) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 5000 /* ms */,
      "obstacle grid is stale (%.2f s old > %.2f s); treating it as unavailable", grid_age_sec,
      param.pointcloud.obstacle_grid_timeout_sec);
    return std::nullopt;
  }

  // The grid is fresh and valid. The gate is a fixed, purely-2D no-regression translation of the
  // legacy per-raw-point loop: a cell qualifies as soon as it holds any return (point_count >= 1),
  // with NO height gate. The floor is lowest(), not 0.0, on purpose: the legacy loop never
  // inspected point.z, and the producing extractor deliberately retains returns down to its crop
  // floor (z_min = -1.0), so a cell whose obstacle mass sits entirely below the ground plane
  // (max_height < 0) must still count -- a 0.0 floor would silently drop exactly those sub-ground
  // obstacles the legacy path stopped for. A NaN max_height with point_count >= 1 is a producer
  // contract violation and stays non-qualifying (never crashes).
  constexpr autoware::obstacle_grid_utils::Gate gate{1U, std::numeric_limits<double>::lowest()};
  const double resolution = grid.getResolution();

  // The grid is published in base_link, so its cells need no further transform. Emit the 4 corners
  // of every qualifying cell (z = 0, purely 2D) instead of the cell center: the checker measures
  // point-to-footprint distance, so corners keep a cell that merely overlaps the footprint
  // edge-conservative rather than reporting it half a cell too far away.
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    if (!autoware::obstacle_grid_utils::cell_qualifies(grid, *it, gate)) {
      continue;
    }
    grid_map::Position center;
    grid.getPosition(*it, center);
    for (const auto & corner : autoware::obstacle_grid_utils::cell_corners(center, resolution)) {
      cloud->push_back(
        pcl::PointXYZ(static_cast<float>(corner.x()), static_cast<float>(corner.y()), 0.0f));
    }
  }

  return cloud;
}

obstacle_proximity_checker::Inputs SurroundObstacleCheckerNode::toProximityCheckerInputs(
  const std::optional<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> & obstacle_grid_pointcloud) const
{
  obstacle_proximity_checker::Inputs inputs;
  inputs.ego_pose = odometry_ptr_->pose.pose;
  inputs.objects = object_ptr_;

  // An unavailable grid leaves pointcloud_in_base_link null, which the checker reads as "no
  // pointcloud obstacle"; onTimer separately holds the latched stop so absence never clears it.
  if (obstacle_grid_pointcloud.has_value()) {
    inputs.pointcloud_in_base_link = obstacle_grid_pointcloud.value();
  }

  return inputs;
}

auto SurroundObstacleCheckerNode::isStopRequired(
  const bool is_obstacle_found, const bool is_vehicle_stopped, const State & state,
  const std::optional<rclcpp::Time> & last_obstacle_found_time, const double time_threshold) const
  -> std::pair<bool, std::optional<rclcpp::Time>>
{
  if (!is_vehicle_stopped) {
    return std::make_pair(false, std::nullopt);
  }

  if (is_obstacle_found) {
    return std::make_pair(true, this->now());
  }

  if (state != State::STOP) {
    return std::make_pair(false, std::nullopt);
  }

  // Keep stop state
  if (last_obstacle_found_time.has_value()) {
    const auto elapsed_time = this->now() - last_obstacle_found_time.value();
    if (elapsed_time.seconds() <= time_threshold) {
      return std::make_pair(true, last_obstacle_found_time.value());
    }
  }

  return std::make_pair(false, std::nullopt);
}

}  // namespace autoware::surround_obstacle_checker

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::surround_obstacle_checker::SurroundObstacleCheckerNode)
