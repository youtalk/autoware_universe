// Copyright 2025 TIER IV, Inc.
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

#include "rear_collision_checker.hpp"

#include "utils.hpp"

#include <autoware/lanelet2_utils/geometry.hpp>
#include <autoware/motion_utils/resample/resample.hpp>
#include <autoware/signal_processing/lowpass_filter_1d.hpp>
#include <autoware_lanelet2_extension/visualization/visualization.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/parameter.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <autoware_utils/transform/transforms.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <magic_enum.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <autoware_internal_planning_msgs/msg/planning_factor.hpp>
#include <autoware_internal_planning_msgs/msg/safety_factor_array.hpp>

#include <lanelet2_core/geometry/Lanelet.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <vector>

namespace autoware::planning_validator
{
using autoware_internal_planning_msgs::msg::PlanningFactor;
using autoware_internal_planning_msgs::msg::SafetyFactor;
using autoware_internal_planning_msgs::msg::SafetyFactorArray;
using autoware_utils::get_or_declare_parameter;

void RearCollisionChecker::init(
  rclcpp::Node & node, const std::string & name,
  const std::shared_ptr<PlanningValidatorContext> & context)
{
  module_name_ = name;

  clock_ = node.get_clock();

  logger_ = node.get_logger();

  context_ = context;

  last_safe_time_ = clock_->now();

  last_unsafe_time_ = clock_->now();

  param_listener_ = std::make_unique<rear_collision_checker_node::ParamListener>(
    node.get_node_parameters_interface());

  pub_grid_points_ =
    node.create_publisher<PointCloud2>("~/rear_collision_checker/debug/obstacle_grid_points", 1);

  pub_string_ = node.create_publisher<StringStamped>("~/rear_collision_checker/debug/state", 1);

  pub_debug_processing_time_detail_ = node.create_publisher<autoware_utils::ProcessingTimeDetail>(
    "~/rear_collision_checker/debug/processing_time_detail_ms", 1);

  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(pub_debug_processing_time_detail_);

  planning_factor_interface_ =
    std::make_unique<autoware::planning_factor_interface::PlanningFactorInterface>(
      &node, "rear_collision_checker");

  setup_diag();
}

void RearCollisionChecker::validate()
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto start_time = clock_->now();

  DebugData debug_data;

  context_->validation_status->is_valid_rear_collision_check = is_safe(debug_data);

  post_process();

  debug_data.processing_time_detail_ms = (clock_->now() - start_time).seconds() * 1e3;

  publish_marker(debug_data);
  publish_planning_factor(debug_data);
}

void RearCollisionChecker::setup_diag()
{
  if (!context_->diag_updater) return;

  const auto & status = context_->validation_status->is_valid_rear_collision_check;
  context_->diag_updater->add("rear_collision_check", [&](auto & stat) {
    const std::string msg = "obstacle detected behind the vehicle";
    set_diag_status(stat, status, msg);
  });
}

void RearCollisionChecker::set_diag_status(
  DiagnosticStatusWrapper & stat, const bool & is_ok, const std::string & msg) const
{
  if (is_ok) {
    stat.summary(DiagnosticStatus::OK, "validated.");
    return;
  }

  const auto invalid_count = context_->validation_status->invalid_count;
  const auto count_threshold = context_->params.diag_error_count_threshold;
  if (invalid_count < count_threshold) {
    const auto warn_msg =
      msg + " (invalid count is less than error threshold: " + std::to_string(invalid_count) +
      " < " + std::to_string(count_threshold) + ")";
    stat.summary(DiagnosticStatus::WARN, warn_msg);
    return;
  }

  stat.summary(DiagnosticStatus::ERROR, msg);
}

void RearCollisionChecker::fill_velocity(PointCloudObject & pointcloud_object)
{
  const auto p = param_listener_->get_params();

  const auto update_history = [this](const auto & pointcloud_object) {
    if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
      history_.emplace(pointcloud_object.furthest_lane.id(), pointcloud_object);
    } else {
      history_.at(pointcloud_object.furthest_lane.id()) = pointcloud_object;
    }
  };

  const auto fill_velocity = [&p, this](auto & pointcloud_object, const auto & previous_data) {
    const auto dx = previous_data.relative_distance - pointcloud_object.relative_distance;
    const auto dt = (pointcloud_object.last_update_time - previous_data.last_update_time).seconds();

    if (dt < 1e-6) {
      pointcloud_object.velocity = previous_data.velocity;
      pointcloud_object.tracking_duration = previous_data.tracking_duration;
      pointcloud_object.relative_distance_with_delay_compensation =
        pointcloud_object.relative_distance -
        pointcloud_object.velocity * p.common.pointcloud.latency;
      return;
    }

    const auto raw_velocity = dx / dt + context_->data->current_kinematics->twist.twist.linear.x;
    const auto is_reliable =
      previous_data.tracking_duration > p.common.pointcloud.velocity_estimation.observation_time;

    if (
      is_reliable && std::abs(raw_velocity - previous_data.velocity) / dt >
                       p.common.pointcloud.velocity_estimation.max_acceleration) {
      // closest point may jumped. don't use the data.
      pointcloud_object.velocity = previous_data.velocity;
      pointcloud_object.tracking_duration = previous_data.tracking_duration;
      pointcloud_object.detail = "the estimated velocity may be an outlier.";
    } else {
      // keep tracking.
      pointcloud_object.velocity =
        autoware::signal_processing::lowpassFilter(raw_velocity, previous_data.velocity, 0.5);
      pointcloud_object.tracking_duration = previous_data.tracking_duration + dt;
    }

    pointcloud_object.relative_distance_with_delay_compensation =
      pointcloud_object.relative_distance -
      pointcloud_object.velocity * p.common.pointcloud.latency;
  };

  const auto fill_moving_time = [&p, this](auto & pointcloud_object, const auto & previous_data) {
    const auto dt = (pointcloud_object.last_update_time - previous_data.last_update_time).seconds();

    if (pointcloud_object.velocity > p.common.filter.min_velocity) {
      pointcloud_object.moving_time = previous_data.moving_time + dt;
      pointcloud_object.last_stop_time = previous_data.last_stop_time;
    } else {
      pointcloud_object.moving_time = 0.0;
      pointcloud_object.last_stop_time = clock_->now();
    }
  };

  if (history_.count(pointcloud_object.furthest_lane.id()) == 0) {
    const auto previous_lanes =
      context_->data->route_handler->getPreviousLanelets(pointcloud_object.furthest_lane);
    for (const auto & previous_lane : previous_lanes) {
      if (history_.count(previous_lane.id()) != 0) {
        fill_velocity(pointcloud_object, history_.at(previous_lane.id()));
        fill_moving_time(pointcloud_object, history_.at(previous_lane.id()));
        update_history(pointcloud_object);
        return;
      }
    }

    update_history(pointcloud_object);
    return;
  }

  fill_velocity(pointcloud_object, history_.at(pointcloud_object.furthest_lane.id()));
  fill_moving_time(pointcloud_object, history_.at(pointcloud_object.furthest_lane.id()));
  update_history(pointcloud_object);
}

auto RearCollisionChecker::extract_obstacle_grid_points(
  const grid_map_msgs::msg::GridMap & msg, DebugData & debug) const
  -> std::optional<PointCloud::Ptr>
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  auto map_points = std::make_shared<PointCloud>();

  // Intake contract validation. The grid is produced in base_link with layers point_count,
  // min_height and low_max_height; any violation is treated as data-unavailable (std::nullopt) so
  // the caller abstains, never as a spurious "clear".
  if (msg.header.frame_id != "base_link") {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "obstacle grid frame_id is '%s', expected 'base_link'; skipping.",
      msg.header.frame_id.c_str());
    return std::nullopt;
  }

  grid_map::GridMap grid;
  if (!grid_map::GridMapRosConverter::fromMessage(msg, grid)) {
    RCLCPP_ERROR_THROTTLE(logger_, *clock_, 5000, "failed to decode obstacle grid; skipping.");
    return std::nullopt;
  }
  if (!grid.exists("point_count") || !grid.exists("min_height") || !grid.exists("low_max_height")) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "obstacle grid is missing required layers; skipping.");
    return std::nullopt;
  }
  grid.convertToDefaultStartIndex();

  const auto p = param_listener_->get_params().common.pointcloud;
  const double height_floor = p.grid.z_floor;
  const double z_band_top = context_->vehicle_info.vehicle_height_m + p.grid.z_band_top_offset;

  // Per-cell z-band gate + edge-conservative corner emission (base_link frame). Height gating uses
  // direct layer reads (low_max_height floor, min_height band-top) rather than the max_height-based
  // shared Gate, so an overhead structure that shares a cell with ground residue is rejected.
  const auto corners = utils::qualifying_cell_corners(
    grid, static_cast<std::uint32_t>(p.grid.min_point_count_cell), height_floor, z_band_top);
  if (corners.empty()) return map_points;

  auto base_link_points = std::make_shared<PointCloud>();
  base_link_points->reserve(corners.size());
  for (const auto & corner : corners) {
    base_link_points->push_back(
      pcl::PointXYZ(
        static_cast<float>(corner.x), static_cast<float>(corner.y), static_cast<float>(corner.z)));
  }

  // Single map<-base_link transform: the detection lanelets, centerlines and arc coordinates are
  // all in map, so the downstream projection pipeline stays unchanged. A missing/old transform at
  // the grid stamp reads as data-unavailable (std::nullopt) so the caller abstains, never as a
  // clear; it is logged with the same throttled ERROR as the other contract failures.
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = context_->tf_buffer.lookupTransform(
      "map", msg.header.frame_id, msg.header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & e) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "no transform found for obstacle grid: %s; skipping.", e.what());
    return std::nullopt;
  }

  const Eigen::Affine3f isometry = tf2::transformToEigen(transform_stamped.transform).cast<float>();
  autoware_utils::transform_pointcloud(*base_link_points, *map_points, isometry);

  const auto grid_pointcloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
  pcl::toROSMsg(*map_points, *grid_pointcloud);
  grid_pointcloud->header.stamp = msg.header.stamp;
  grid_pointcloud->header.frame_id = "map";
  debug.grid_points = grid_pointcloud;

  return map_points;
}

auto RearCollisionChecker::get_pointcloud_object(
  const rclcpp::Time & now, const PointCloud::Ptr & pointcloud_ptr,
  const DetectionAreas & detection_areas) -> std::optional<PointCloudObject>
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  std::optional<PointCloudObject> opt_object = std::nullopt;
  for (const auto & [polygon, lanes] : detection_areas) {
    const auto pointcloud = *utils::get_obstacle_points({polygon}, *pointcloud_ptr);

    const auto path = context_->data->route_handler->getCenterLinePath(
      lanes, 0.0, std::numeric_limits<double>::max());
    const auto resampled_path = autoware::motion_utils::resamplePath(path, 2.0);

    for (const auto & point : pointcloud) {
      const auto p_geom = autoware_utils::create_point(point.x, point.y, point.z);
      const size_t src_seg_idx =
        autoware::motion_utils::findNearestSegmentIndex(resampled_path.points, p_geom);
      const double signed_length_src_offset =
        autoware::motion_utils::calcLongitudinalOffsetToSegment(
          resampled_path.points, src_seg_idx, p_geom);

      const double obj_arc_length =
        autoware::motion_utils::calcSignedArcLength(
          resampled_path.points, src_seg_idx, resampled_path.points.size() - 1) -
        signed_length_src_offset;
      const auto pose_on_center_line = autoware::motion_utils::calcLongitudinalOffsetPose(
        resampled_path.points, src_seg_idx, signed_length_src_offset);

      if (!pose_on_center_line.has_value()) {
        continue;
      }

      if (!opt_object.has_value()) {
        PointCloudObject object;
        object.last_update_time = now;
        object.last_stop_time = now;
        object.pose = pose_on_center_line.value();
        object.furthest_lane = lanes.back();
        object.tracking_duration = 0.0;
        object.absolute_distance = obj_arc_length;
        object.velocity = 0.0;
        object.moving_time = 0.0;
        opt_object = object;
      } else if (opt_object.value().absolute_distance > obj_arc_length) {
        opt_object.value().last_update_time = now;
        opt_object.value().last_stop_time = now;
        opt_object.value().pose = pose_on_center_line.value();
        opt_object.value().furthest_lane = lanes.back();
        opt_object.value().tracking_duration = 0.0;
        opt_object.value().absolute_distance = obj_arc_length;
        opt_object.value().velocity = 0.0;
        opt_object.value().moving_time = 0.0;
      }
    }
  }

  return opt_object;
}

auto RearCollisionChecker::get_pointcloud_objects(
  const double grid_rear_extent,
  const std::function<std::pair<double, double>()> & func_range_calculation,
  const std::function<PointCloudObjects(const double, const double)> & func_object_filtering,
  const std::function<void(PointCloudObjects &)> & func_safety_check) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto [forward_distance, backward_distance] = func_range_calculation();

  // Fail-visible ROI guard: the obstacle grid is produced by a forward-biased extractor, so its
  // rear coverage may be shorter than the backward reach this check needs. When that happens,
  // rear-closing objects beyond the grid are invisible; log a throttled ERROR naming both numbers.
  // The launcher is responsible for rebiasing/widening the producer ROI (see README and PR body).
  if (backward_distance > grid_rear_extent) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000,
      "obstacle grid rear coverage (%.1f m) is shorter than the required backward reach (%.1f m); "
      "rear objects beyond the grid extent are not detected. Widen the producer ROI.",
      grid_rear_extent, backward_distance);
  }

  auto objects = func_object_filtering(forward_distance, backward_distance);
  func_safety_check(objects);

  return objects;
}

auto RearCollisionChecker::get_pointcloud_objects_on_adjacent_lane(
  const lanelet::ConstLanelets & current_lanes, const Behavior & shift_behavior,
  const double forward_distance, const double backward_distance,
  const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  if (shift_behavior == Behavior::NONE) {
    return objects;
  }

  const auto ego_coordinate_on_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
    current_lanes, context_->data->current_kinematics->pose.pose);

  lanelet::ConstLanelets connected_adjacent_lanes{};

  double length = 0.0;
  for (const auto & lane : current_lanes) {
    const auto current_lane_length = lanelet::geometry::length2d(lane);

    length += current_lane_length;

    const auto ego_to_furthest_point = length - ego_coordinate_on_arc.length;
    const auto residual_distance = ego_to_furthest_point - forward_distance;

    const auto opt_adjacent_lane = [&lane, &shift_behavior, this]() {
      const auto is_right = shift_behavior == Behavior::SHIFT_RIGHT;
      return is_right ? context_->data->route_handler->getRightLanelet(lane, true, true)
                      : context_->data->route_handler->getLeftLanelet(lane, true, true);
    }();

    if (opt_adjacent_lane.has_value()) {
      connected_adjacent_lanes.push_back(opt_adjacent_lane.value());
    }

    if (!connected_adjacent_lanes.empty() && residual_distance > 0.0) {
      auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        current_lanes, connected_adjacent_lanes, residual_distance,
        residual_distance + forward_distance + backward_distance, context_->data->route_handler,
        p.common.adjacent_lane.offset.left, p.common.adjacent_lane.offset.right);

      utils::cut_by_lanelets(current_lanes, detection_areas);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      time_keeper_->start_track("get_pointcloud_object");
      auto opt_pointcloud_object = get_pointcloud_object(
        context_->data->obstacle_grid->header.stamp, obstacle_pointcloud, detection_areas);
      time_keeper_->end_track("get_pointcloud_object");

      if (!opt_pointcloud_object.has_value()) {
        return objects;
      }

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(context_->vehicle_info.min_longitudinal_offset_m);

      if (
        opt_pointcloud_object.value().relative_distance <
        p.common.pointcloud.range.dead_zone - forward_distance) {
        return objects;
      }

      fill_velocity(opt_pointcloud_object.value());

      objects.push_back(opt_pointcloud_object.value());

      return objects;
    }

    if (!connected_adjacent_lanes.empty() && !opt_adjacent_lane.has_value()) {
      auto detection_areas = utils::get_previous_polygons_with_lane_recursively(
        current_lanes, connected_adjacent_lanes, 0.0,
        ego_to_furthest_point - current_lane_length + backward_distance,
        context_->data->route_handler, p.common.adjacent_lane.offset.left,
        p.common.adjacent_lane.offset.right);

      utils::cut_by_lanelets(current_lanes, detection_areas);

      {
        debug.detection_areas.insert(
          debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
      }

      connected_adjacent_lanes.clear();

      time_keeper_->start_track("get_pointcloud_object");
      auto opt_pointcloud_object = get_pointcloud_object(
        context_->data->obstacle_grid->header.stamp, obstacle_pointcloud, detection_areas);
      time_keeper_->end_track("get_pointcloud_object");

      if (!opt_pointcloud_object.has_value()) {
        continue;
      }

      opt_pointcloud_object.value().relative_distance =
        opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
        std::abs(context_->vehicle_info.min_longitudinal_offset_m);

      if (
        opt_pointcloud_object.value().relative_distance <
        p.common.pointcloud.range.dead_zone - forward_distance) {
        return objects;
      }

      fill_velocity(opt_pointcloud_object.value());

      objects.push_back(opt_pointcloud_object.value());
    }
  }

  return objects;
}

auto RearCollisionChecker::get_pointcloud_objects_at_blind_spot(
  const lanelet::ConstLanelets & current_lanes, const Behavior & turn_behavior,
  const double forward_distance, const double backward_distance,
  const PointCloud::Ptr & obstacle_pointcloud, DebugData & debug) -> PointCloudObjects
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();

  PointCloudObjects objects{};

  if (turn_behavior == Behavior::NONE) {
    return objects;
  }

  const auto half_lanes = [&current_lanes, &turn_behavior, &p, this]() {
    const auto is_right = turn_behavior == Behavior::TURN_RIGHT;
    lanelet::ConstLanelets ret{};
    for (const auto & lane : current_lanes) {
      ret.push_back(
        utils::generate_half_lanelet(
          lane, is_right,
          0.5 * context_->vehicle_info.vehicle_width_m + p.common.blind_spot.offset.inner,
          p.common.blind_spot.offset.outer));
    }
    return ret;
  }();
  const auto detection_polygon = utils::generate_detection_polygon(
    half_lanes, context_->data->current_kinematics->pose.pose, forward_distance, backward_distance);

  DetectionAreas detection_areas{};
  detection_areas.emplace_back(detection_polygon, half_lanes);

  {
    debug.detection_areas.insert(
      debug.detection_areas.end(), detection_areas.begin(), detection_areas.end());
  }

  time_keeper_->start_track("get_pointcloud_object");
  auto opt_pointcloud_object = get_pointcloud_object(
    context_->data->obstacle_grid->header.stamp, obstacle_pointcloud, detection_areas);
  time_keeper_->end_track("get_pointcloud_object");

  if (!opt_pointcloud_object.has_value()) {
    return objects;
  }

  const auto ego_coordinate_on_arc = autoware::experimental::lanelet2_utils::get_arc_coordinates(
    current_lanes, context_->data->current_kinematics->pose.pose);

  const auto ego_to_furthest_point =
    lanelet::geometry::length2d(lanelet::LaneletSequence(half_lanes)) -
    ego_coordinate_on_arc.length;

  opt_pointcloud_object.value().relative_distance =
    opt_pointcloud_object.value().absolute_distance - ego_to_furthest_point -
    std::abs(context_->vehicle_info.min_longitudinal_offset_m);

  if (
    opt_pointcloud_object.value().relative_distance <
    p.common.pointcloud.range.dead_zone - forward_distance) {
    return objects;
  }

  fill_velocity(opt_pointcloud_object.value());

  objects.push_back(opt_pointcloud_object.value());

  return objects;
}

bool RearCollisionChecker::is_safe(const PointCloudObjects & objects, DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st("is_safe_pointcloud", *time_keeper_);

  const auto p = param_listener_->get_params();

  {
    debug.pointcloud_objects = objects;
  }

  for (const auto & object : objects) {
    if (object.tracking_duration < p.common.pointcloud.velocity_estimation.observation_time) {
      continue;
    }

    if (object.ignore) {
      continue;
    }

    if (object.safe) {
      continue;
    }

    return false;
  }

  return true;
}

bool RearCollisionChecker::is_safe(DebugData & debug)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  const auto p = param_listener_->get_params();
  const auto now = clock_->now();

  constexpr double forward = 100.0;
  constexpr double backward = 100.0;
  const auto current_lanes = utils::get_current_lanes(context_, forward, backward);

  if (current_lanes.empty()) {
    debug.text = "failed to identify the current driving lane.";
    return true;
  }

  const auto is_unsafe_holding = (now - last_unsafe_time_).seconds() < p.common.off_time_buffer;
  const auto [turn_behavior, distance_to_turn] =
    utils::check_turn_behavior(current_lanes, is_unsafe_holding, context_, p, debug);
  const auto [shift_behavior, distance_to_shift] =
    utils::check_shift_behavior(current_lanes, is_unsafe_holding, context_, p, debug);

  {
    debug.current_lanes = current_lanes;
    debug.turn_behavior = turn_behavior;
    debug.shift_behavior = shift_behavior;
  }

  if (turn_behavior == Behavior::NONE && shift_behavior == Behavior::NONE) {
    return true;
  }

  {
    debug.is_active = true;
  }

  // Obstacle-grid availability + staleness watchdog. A missing or stale grid reads as
  // data-unavailable: the checker cannot run, logs a throttled ERROR, and abstains by returning
  // valid without publishing a STOP planning factor. This is an availability signal only (not a
  // veto); it never treats a stale grid as an empty/clear scene. See README "Data-unavailability is
  // an abstain, not a veto".
  if (!context_->data->obstacle_grid) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "obstacle grid is not available; skipping rear collision check.");
    return true;
  }
  const auto grid_age = (now - rclcpp::Time(context_->data->obstacle_grid->header.stamp)).seconds();
  if (grid_age > p.common.pointcloud.obstacle_grid_timeout_sec) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, 5000, "obstacle grid is stale (age %.2f s); skipping rear collision check.",
      grid_age);
    return true;
  }

  // Rear coverage of the grid measured behind the base_link origin, used by the ROI guard below.
  const auto & grid_info = context_->data->obstacle_grid->info;
  const double grid_rear_extent = grid_info.length_x * 0.5 - grid_info.pose.position.x;

  // Grid-contract / TF watchdog: extract returns std::nullopt (already logged with a throttled
  // ERROR) on wrong frame, undecodable message, missing layer, or a missing/old map<-base_link
  // transform. Treat all of these as data-unavailable and abstain, exactly like the null/stale grid
  // branches above, rather than letting an empty cloud flow through as a spurious "clear". A
  // successfully decoded grid with no qualifying cell returns an empty cloud and is a genuine
  // clear.
  const auto opt_obstacle_pointcloud =
    extract_obstacle_grid_points(*context_->data->obstacle_grid, debug);
  if (!opt_obstacle_pointcloud) {
    return true;
  }
  const auto & obstacle_pointcloud = opt_obstacle_pointcloud.value();
  PointCloudObjects pointcloud_objects{};

  {
    time_keeper_->start_track("adjacent_lane");

    const auto delay_object = p.common.adjacent_lane.participants.reaction_time;
    const auto max_deceleration_object = p.common.adjacent_lane.participants.max_deceleration;
    const auto max_velocity_object = p.common.adjacent_lane.participants.max_velocity;

    const auto func_range_calculation = std::bind(
      p.common.adjacent_lane.metric == "ttc" ? utils::get_range_for_ttc : utils::get_range_for_rss,
      context_, distance_to_shift, delay_object, max_deceleration_object, max_velocity_object, p);
    const auto func_object_filtering = std::bind(
      &RearCollisionChecker::get_pointcloud_objects_on_adjacent_lane, this, current_lanes,
      shift_behavior, _1, _2, obstacle_pointcloud, std::ref(debug));
    const auto func_safety_check = std::bind(
      p.common.adjacent_lane.metric == "ttc" ? utils::fill_time_to_collision
                                             : utils::fill_rss_distance,
      _1, context_, distance_to_shift, delay_object, max_deceleration_object, max_velocity_object,
      p);

    auto objects = get_pointcloud_objects(
      grid_rear_extent, func_range_calculation, func_object_filtering, func_safety_check);
    pointcloud_objects.insert(pointcloud_objects.end(), objects.begin(), objects.end());

    time_keeper_->end_track("adjacent_lane");
  }

  {
    time_keeper_->start_track("blind_spot");

    const auto delay_object = p.common.blind_spot.participants.reaction_time;
    const auto max_deceleration_object = p.common.blind_spot.participants.max_deceleration;
    const auto max_velocity_object = p.common.blind_spot.participants.max_velocity;

    const auto func_range_calculation = std::bind(
      p.common.blind_spot.metric == "ttc" ? utils::get_range_for_ttc : utils::get_range_for_rss,
      context_, distance_to_turn, delay_object, max_deceleration_object, max_velocity_object, p);
    const auto func_object_filtering = std::bind(
      &RearCollisionChecker::get_pointcloud_objects_at_blind_spot, this, current_lanes,
      turn_behavior, _1, _2, obstacle_pointcloud, std::ref(debug));
    const auto func_safety_check = std::bind(
      p.common.blind_spot.metric == "ttc" ? utils::fill_time_to_collision
                                          : utils::fill_rss_distance,
      _1, context_, distance_to_turn, delay_object, max_deceleration_object, max_velocity_object,
      p);

    auto objects = get_pointcloud_objects(
      grid_rear_extent, func_range_calculation, func_object_filtering, func_safety_check);
    pointcloud_objects.insert(pointcloud_objects.end(), objects.begin(), objects.end());

    time_keeper_->end_track("blind_spot");
  }

  {
    if (is_safe(pointcloud_objects, debug)) {
      if ((now - last_unsafe_time_).seconds() > p.common.off_time_buffer) {
        last_safe_time_ = now;
        return true;
      }
    } else if ((now - last_safe_time_).seconds() < p.common.on_time_buffer) {
      RCLCPP_WARN(logger_, "[RCC] Momentary collision risk detected.");
      return true;
    } else {
      last_unsafe_time_ = now;
    }

    {
      RCLCPP_ERROR(logger_, "[RCC] Continuous collision risk detected.");
      debug.text = "continuous collision risk detected.";
    }
  }

  debug.is_safe = false;
  return false;
}

void RearCollisionChecker::post_process()
{
  auto itr = history_.begin();
  while (itr != history_.end()) {
    if ((clock_->now() - itr->second.last_update_time).seconds() > 1.0) {
      itr = history_.erase(itr);
    } else {
      itr++;
    }
  }
}

void RearCollisionChecker::publish_marker(const DebugData & debug) const
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  MarkerArray msg;

  const auto add = [&msg](const MarkerArray & added) {
    autoware_utils::append_marker_array(added, &msg);
  };

  {
    add(
      utils::create_line_marker_array(
        debug.reachable_line, "reachable_line",
        autoware_utils::create_marker_color(1.0, 0.67, 0.0, 0.999)));
    add(
      utils::create_line_marker_array(
        debug.stoppable_line, "stoppable_line",
        autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));
    add(
      lanelet::visualization::laneletsAsTriangleMarkerArray(
        "detection_lanes", debug.get_detection_lanes(),
        autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.2)));
    add(
      lanelet::visualization::laneletsAsTriangleMarkerArray(
        "current_lanes", debug.current_lanes,
        autoware_utils::create_marker_color(0.16, 1.0, 0.69, 0.2)));
    add(
      utils::create_pointcloud_object_marker_array(
        debug.pointcloud_objects, "pointcloud_objects", param_listener_->get_params()));
    add(
      utils::create_polygon_marker_array(
        debug.get_detection_polygons(), "detection_areas",
        autoware_utils::create_marker_color(1.0, 0.0, 0.42, 0.999)));

    std::for_each(msg.markers.begin(), msg.markers.end(), [](auto & marker) {
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);
    });

    context_->debug_pose_publisher->pushMarkers(msg);
  }

  if (debug.grid_points) {
    pub_grid_points_->publish(*debug.grid_points);
  }

  {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2) << std::boolalpha;
    ss << "ACTIVE:" << debug.is_active << "\n";
    ss << "SAFE:" << debug.is_safe << "\n";
    ss << "TURN:" << magic_enum::enum_name(debug.turn_behavior) << "\n";
    ss << "SHIFT:" << magic_enum::enum_name(debug.shift_behavior) << "\n";
    ss << "INFO:" << debug.text << "\n";
    ss << "TRACKING OBJECTS:" << debug.pointcloud_objects.size() << "\n";
    ss << "PROCESSING TIME:" << debug.processing_time_detail_ms << "[ms]\n";

    StringStamped string_stamp;
    string_stamp.stamp = clock_->now();
    string_stamp.data = ss.str();
    pub_string_->publish(string_stamp);
  }
}

void RearCollisionChecker::publish_planning_factor(const DebugData & debug) const
{
  if (debug.is_safe) return;

  SafetyFactorArray factor_array;
  factor_array.is_safe = false;
  factor_array.detail = "possible collision with rear object";
  factor_array.header.stamp = clock_->now();

  SafetyFactor factor;
  factor.type = SafetyFactor::POINTCLOUD;
  for (const auto & obj : debug.pointcloud_objects) {
    factor.is_safe = obj.safe;
    factor.points.push_back(obj.pose.position);
    factor_array.factors.push_back(factor);
  }

  const auto & traj_points = context_->data->current_trajectory->points;
  const auto & ego_pose = context_->data->current_kinematics->pose.pose;
  planning_factor_interface_->add(
    traj_points, ego_pose, ego_pose, PlanningFactor::STOP, factor_array);
  planning_factor_interface_->publish();
}

}  // namespace autoware::planning_validator

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  autoware::planning_validator::RearCollisionChecker, autoware::planning_validator::PluginInterface)
