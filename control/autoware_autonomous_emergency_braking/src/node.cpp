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

#include <autoware/autonomous_emergency_braking/node.hpp>
#include <autoware/autonomous_emergency_braking/utils.hpp>
#include <autoware/motion_utils/marker/marker_helper.hpp>
#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>
#include <autoware_utils/autoware_utils.hpp>
#include <autoware_utils/geometry/boost_geometry.hpp>
#include <autoware_utils/geometry/boost_polygon_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/ros/marker_helper.hpp>
#include <autoware_utils/ros/update_param.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <rclcpp/node.hpp>
#include <tf2/utils.hpp>

#include <geometry_msgs/msg/polygon.hpp>

#include <boost/geometry/algorithms/convex_hull.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/version.hpp>

#if BOOST_VERSION < 107600  // Header removed in version 1.76.0 (Humble)
#include <boost/geometry/strategies/agnostic/hull_graham_andrew.hpp>
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{
using autoware::motion::control::autonomous_emergency_braking::colorTuple;
constexpr double MIN_MOVING_VELOCITY_THRESHOLD = 0.1;
// Sky blue (RGB: 0, 148, 205) - A medium-bright blue color
constexpr colorTuple IMU_PATH_COLOR = {0.0 / 256.0, 148.0 / 256.0, 205.0 / 256.0, 0.999};
// Forest green (RGB: 0, 100, 0) - A deep, dark green color
constexpr colorTuple MPC_PATH_COLOR = {0.0 / 256.0, 100.0 / 256.0, 0.0 / 256.0, 0.999};
}  // namespace

namespace autoware::motion::control::autonomous_emergency_braking
{
using autoware::motion::control::autonomous_emergency_braking::utils::convertObjToPolygon;
using autoware_utils::Point2d;
using diagnostic_msgs::msg::DiagnosticStatus;
namespace bg = boost::geometry;

void appendPointToPolygon(Polygon2d & polygon, const geometry_msgs::msg::Point & geom_point)
{
  Point2d point;
  point.x() = geom_point.x;
  point.y() = geom_point.y;

  bg::append(polygon.outer(), point);
}

Polygon2d createPolygon(
  const geometry_msgs::msg::Pose & base_pose, const geometry_msgs::msg::Pose & next_pose,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info, const double expand_width)
{
  Polygon2d polygon;

  const double longitudinal_offset = vehicle_info.max_longitudinal_offset_m;
  const double width = vehicle_info.vehicle_width_m / 2.0 + expand_width;
  const double rear_overhang = vehicle_info.rear_overhang_m;

  appendPointToPolygon(
    polygon, autoware_utils::calc_offset_pose(base_pose, longitudinal_offset, width, 0.0).position);
  appendPointToPolygon(
    polygon,
    autoware_utils::calc_offset_pose(base_pose, longitudinal_offset, -width, 0.0).position);
  appendPointToPolygon(
    polygon, autoware_utils::calc_offset_pose(base_pose, -rear_overhang, -width, 0.0).position);
  appendPointToPolygon(
    polygon, autoware_utils::calc_offset_pose(base_pose, -rear_overhang, width, 0.0).position);

  appendPointToPolygon(
    polygon, autoware_utils::calc_offset_pose(next_pose, longitudinal_offset, width, 0.0).position);
  appendPointToPolygon(
    polygon,
    autoware_utils::calc_offset_pose(next_pose, longitudinal_offset, -width, 0.0).position);
  appendPointToPolygon(
    polygon, autoware_utils::calc_offset_pose(next_pose, -rear_overhang, -width, 0.0).position);
  appendPointToPolygon(
    polygon, autoware_utils::calc_offset_pose(next_pose, -rear_overhang, width, 0.0).position);

  polygon =
    autoware_utils::is_clockwise(polygon) ? polygon : autoware_utils::inverse_clockwise(polygon);

  Polygon2d hull_polygon;
  bg::convex_hull(polygon, hull_polygon);
  bg::correct(hull_polygon);
  return hull_polygon;
}

AEB::AEB(const rclcpp::NodeOptions & node_options)
: Node("AEB", node_options),
  vehicle_info_(autoware::vehicle_info_utils::VehicleInfoUtils(*this).getVehicleInfo()),
  collision_data_keeper_(this->get_clock())
{
  // Publisher
  {
    debug_marker_publisher_ = this->create_publisher<MarkerArray>("~/debug/markers", 1);
    virtual_wall_publisher_ = this->create_publisher<MarkerArray>("~/virtual_wall", 1);
    debug_rss_distance_publisher_ =
      this->create_publisher<tier4_debug_msgs::msg::Float32Stamped>("~/debug/rss_distance", 1);
    metrics_pub_ = this->create_publisher<MetricArray>("~/metrics", 1);
  }
  // Diagnostics
  {
    updater_.setHardwareID("autonomous_emergency_braking");
    updater_.add("aeb_emergency_stop", this, &AEB::onCheckCollision);
  }
  // parameter
  publish_debug_markers_ = declare_parameter<bool>("publish_debug_markers");
  use_predicted_trajectory_ = declare_parameter<bool>("use_predicted_trajectory");
  use_imu_path_ = declare_parameter<bool>("use_imu_path");
  limit_imu_path_lat_dev_ = declare_parameter<bool>("limit_imu_path_lat_dev");
  limit_imu_path_length_ = declare_parameter<bool>("limit_imu_path_length");
  use_pointcloud_data_ = declare_parameter<bool>("use_pointcloud_data");
  use_predicted_object_data_ = declare_parameter<bool>("use_predicted_object_data");
  use_object_velocity_calculation_ = declare_parameter<bool>("use_object_velocity_calculation");
  check_autoware_state_ = declare_parameter<bool>("check_autoware_state");
  imu_path_lat_dev_threshold_ = declare_parameter<double>("imu_path_lat_dev_threshold");
  speed_calculation_expansion_margin_ =
    declare_parameter<double>("speed_calculation_expansion_margin");
  detection_range_max_height_margin_ =
    declare_parameter<double>("detection_range_max_height_margin");
  min_generated_imu_path_length_ = declare_parameter<double>("min_generated_imu_path_length");
  max_generated_imu_path_length_ = declare_parameter<double>("max_generated_imu_path_length");
  expand_width_ = declare_parameter<double>("expand_width");
  longitudinal_offset_margin_ = declare_parameter<double>("longitudinal_offset_margin");
  t_response_ = declare_parameter<double>("t_response");
  a_ego_min_ = declare_parameter<double>("a_ego_min");
  a_obj_min_ = declare_parameter<double>("a_obj_min");

  cluster_minimum_height_ = declare_parameter<double>("cluster_minimum_height");
  minimum_cluster_size_ = declare_parameter<int>("minimum_cluster_size");
  window_size_ = declare_parameter<int>("window_size");
  if (minimum_cluster_size_ > (2 * window_size_ + 1) * (2 * window_size_ + 1)) {
    RCLCPP_WARN(
      get_logger(),
      "[AEB] minimum_cluster_size (%d) exceeds the window cell count (2*window_size+1)^2 = %d; the "
      "pointcloud branch can never trigger. Increase window_size or lower minimum_cluster_size.",
      minimum_cluster_size_, (2 * window_size_ + 1) * (2 * window_size_ + 1));
  }

  imu_prediction_time_horizon_ = declare_parameter<double>("imu_prediction_time_horizon");
  imu_prediction_time_interval_ = declare_parameter<double>("imu_prediction_time_interval");
  mpc_prediction_time_horizon_ = declare_parameter<double>("mpc_prediction_time_horizon");
  mpc_prediction_time_interval_ = declare_parameter<double>("mpc_prediction_time_interval");

  {  // Object history data keeper setup
    const auto previous_obstacle_keep_time =
      declare_parameter<double>("previous_obstacle_keep_time");
    const auto collision_keeping_sec = declare_parameter<double>("collision_keeping_sec");
    collision_data_keeper_.setTimeout(collision_keeping_sec, previous_obstacle_keep_time);
  }

  // Parameter Callback
  set_param_res_ =
    add_on_set_parameters_callback(std::bind(&AEB::onParameter, this, std::placeholders::_1));

  // start time
  const double aeb_hz = declare_parameter<double>("aeb_hz");
  const auto period_ns = rclcpp::Rate(aeb_hz).period();
  timer_ = autoware::agnocast_wrapper::create_timer(
    this, this->get_clock(), period_ns, std::bind(&AEB::onTimer, this));

  debug_processing_time_detail_pub_ =
    create_publisher<autoware_utils::ProcessingTimeDetail>("~/debug/processing_time_detail_ms", 1);
  time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(debug_processing_time_detail_pub_);
}

rcl_interfaces::msg::SetParametersResult AEB::onParameter(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using autoware_utils::update_param;
  update_param<bool>(parameters, "publish_debug_markers", publish_debug_markers_);
  update_param<bool>(parameters, "use_predicted_trajectory", use_predicted_trajectory_);
  update_param<bool>(parameters, "use_imu_path", use_imu_path_);
  update_param<bool>(parameters, "limit_imu_path_lat_dev", limit_imu_path_lat_dev_);
  update_param<bool>(parameters, "limit_imu_path_length", limit_imu_path_length_);
  update_param<bool>(parameters, "use_pointcloud_data", use_pointcloud_data_);
  update_param<bool>(parameters, "use_predicted_object_data", use_predicted_object_data_);
  update_param<bool>(
    parameters, "use_object_velocity_calculation", use_object_velocity_calculation_);
  update_param<bool>(parameters, "check_autoware_state", check_autoware_state_);
  update_param<double>(parameters, "imu_path_lat_dev_threshold", imu_path_lat_dev_threshold_);
  update_param<double>(
    parameters, "speed_calculation_expansion_margin", speed_calculation_expansion_margin_);
  update_param<double>(
    parameters, "detection_range_max_height_margin", detection_range_max_height_margin_);
  update_param<double>(parameters, "min_generated_imu_path_length", min_generated_imu_path_length_);
  update_param<double>(parameters, "max_generated_imu_path_length", max_generated_imu_path_length_);
  update_param<double>(parameters, "expand_width", expand_width_);
  update_param<double>(parameters, "longitudinal_offset_margin", longitudinal_offset_margin_);
  update_param<double>(parameters, "t_response", t_response_);
  update_param<double>(parameters, "a_ego_min", a_ego_min_);
  update_param<double>(parameters, "a_obj_min", a_obj_min_);

  update_param<double>(parameters, "cluster_minimum_height", cluster_minimum_height_);
  update_param<int>(parameters, "minimum_cluster_size", minimum_cluster_size_);
  update_param<int>(parameters, "window_size", window_size_);

  update_param<double>(parameters, "imu_prediction_time_horizon", imu_prediction_time_horizon_);
  update_param<double>(parameters, "imu_prediction_time_interval", imu_prediction_time_interval_);
  update_param<double>(parameters, "mpc_prediction_time_horizon", mpc_prediction_time_horizon_);
  update_param<double>(parameters, "mpc_prediction_time_interval", mpc_prediction_time_interval_);

  {  // Object history data keeper setup
    auto [previous_obstacle_keep_time, collision_keeping_sec] = collision_data_keeper_.getTimeout();
    update_param<double>(parameters, "previous_obstacle_keep_time", previous_obstacle_keep_time);
    update_param<double>(parameters, "collision_keeping_sec", collision_keeping_sec);
    collision_data_keeper_.setTimeout(collision_keeping_sec, previous_obstacle_keep_time);
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void AEB::onTimer()
{
  updater_.force_update();
}

void AEB::onImu(const std::shared_ptr<const Imu> & input_msg)
{
  // transform imu
  const auto logger = get_logger();
  const auto transform_stamped =
    utils::getTransform("base_link", input_msg->header.frame_id, tf_buffer_, logger);
  if (!transform_stamped.has_value()) return;

  angular_velocity_ptr_ = std::make_shared<Vector3>();
  tf2::doTransform(input_msg->angular_velocity, *angular_velocity_ptr_, transform_stamped.value());
}

bool AEB::fetchLatestData()
{
  const auto missing = [this](const auto & name) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), *get_clock(), 5000, "[AEB] waiting for %s", name);
    return false;
  };

  current_velocity_ptr_ = sub_velocity_->take_data();
  if (!current_velocity_ptr_) {
    return missing("ego velocity");
  }

  if (use_pointcloud_data_) {
    const auto obstacle_grid_ptr = sub_obstacle_grid_->take_data();
    if (!obstacle_grid_ptr) {
      return missing("obstacle grid message");
    }
    obstacle_grid_ptr_ = obstacle_grid_ptr;
  } else {
    obstacle_grid_ptr_.reset();
  }

  if (use_predicted_object_data_) {
    predicted_objects_ptr_ = predicted_objects_sub_->take_data();
    if (!predicted_objects_ptr_) {
      return missing("predicted objects");
    }
  } else {
    predicted_objects_ptr_ = {};
  }

  if (!obstacle_grid_ptr_ && !predicted_objects_ptr_) {
    return missing("object detection method (obstacle grid or predicted objects)");
  }

  const bool has_imu_path = std::invoke([&]() {
    if (!use_imu_path_) return false;
    const auto imu_ptr = sub_imu_->take_data();
    if (!imu_ptr) {
      return missing("imu message");
    }
    // imu_ptr is valid
    onImu(imu_ptr);
    return (!angular_velocity_ptr_) ? missing("imu") : true;
  });

  const bool has_predicted_path = std::invoke([&]() {
    if (!use_predicted_trajectory_) {
      return false;
    }
    predicted_traj_ptr_ = sub_predicted_traj_->take_data();
    return (!predicted_traj_ptr_) ? missing("control predicted trajectory") : true;
  });

  if (!has_imu_path && !has_predicted_path) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[AEB] At least one path (IMU or predicted trajectory) is required for operation");
    return false;
  }

  autoware_state_ = sub_autoware_state_->take_data();
  if (check_autoware_state_ && !autoware_state_) {
    return missing("autoware_state");
  }

  return true;
}

void AEB::onCheckCollision(DiagnosticStatusWrapper & stat)
{
  MarkerArray debug_markers;
  MarkerArray virtual_wall_marker;
  auto metrics = MetricArray();
  checkCollision(debug_markers);

  if (!collision_data_keeper_.checkCollisionExpired()) {
    const std::string error_msg = "[AEB]: Emergency Brake";
    const auto diag_level = DiagnosticStatus::ERROR;
    stat.summary(diag_level, error_msg);
    const auto & data = collision_data_keeper_.get();
    if (data.has_value()) {
      stat.addf("RSS", "%.2f", data.value().rss);
      stat.addf("Distance", "%.2f", data.value().distance_to_object);
      stat.addf("Object Speed", "%.2f", data.value().velocity);
      if (publish_debug_markers_) {
        addCollisionMarker(data.value(), debug_markers);
      }
    }
    addVirtualStopWallMarker(virtual_wall_marker);

    {
      auto metric = Metric();
      metric.name = "decision";
      metric.value = "brake";
      metrics.metric_array.push_back(metric);
    }

  } else {
    const std::string error_msg = "[AEB]: No Collision";
    const auto diag_level = DiagnosticStatus::OK;
    stat.summary(diag_level, error_msg);
  }

  // publish debug markers
  debug_marker_publisher_->publish(debug_markers);
  virtual_wall_publisher_->publish(virtual_wall_marker);
  // publish metrics
  metrics.stamp = get_clock()->now();
  metrics_pub_->publish(metrics);
}

bool AEB::checkCollision(MarkerArray & debug_markers)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);

  // step1. check data
  if (!fetchLatestData()) {
    return false;
  }

  // if not driving, disable aeb
  if (check_autoware_state_ && autoware_state_->state != AutowareState::DRIVING) {
    return false;
  }

  // step2. create velocity data check if the vehicle stops or not
  const double current_v = current_velocity_ptr_->longitudinal_velocity;
  if (std::abs(current_v) < MIN_MOVING_VELOCITY_THRESHOLD) {
    return false;
  }

  auto get_objects_on_path = [&](
                               const auto & path, PointCloud::Ptr points_belonging_to_cluster_hulls,
                               const colorTuple & debug_colors, const std::string & debug_ns) {
    // Check which points of the cropped point cloud are on the ego path, and get the closest one
    const auto ego_polys = generatePathFootprint(path, expand_width_);
    std::vector<ObjectData> objects;
    // Crop out Pointcloud using an extra wide ego path
    if (
      use_pointcloud_data_ && points_belonging_to_cluster_hulls &&
      !points_belonging_to_cluster_hulls->empty()) {
      const auto current_time = obstacle_grid_ptr_->header.stamp;
      getClosestObjectsOnPath(path, current_time, points_belonging_to_cluster_hulls, objects);
    }
    if (use_predicted_object_data_) {
      createObjectDataUsingPredictedObjects(path, ego_polys, objects);
    }

    // Add debug markers
    if (publish_debug_markers_) {
      addMarker(
        this->get_clock()->now(), path, ego_polys, objects, collision_data_keeper_.get(),
        debug_colors, debug_ns, debug_markers);
    }
    return objects;
  };

  auto check_collision = [&](const Path & path, std::vector<ObjectData> & objects) {
    time_keeper_->start_track("has_collision");
    const auto closest_object_point = std::invoke([&]() -> std::optional<ObjectData> {
      // Attempt to find the closest object
      const auto closest_object_itr =
        std::min_element(objects.begin(), objects.end(), [](const auto & o1, const auto & o2) {
          // target objects have priority
          if (o1.is_target != o2.is_target) {
            return o1.is_target;
          }
          return o1.distance_to_object < o2.distance_to_object;
        });

      if (closest_object_itr != objects.end()) {
        // Calculate speed for the closest object
        const auto closest_object_speed = (use_object_velocity_calculation_)
                                            ? collision_data_keeper_.calcObjectSpeedFromHistory(
                                                *closest_object_itr, path, current_v)
                                            : std::make_optional<double>(0.0);

        if (closest_object_speed.has_value()) {
          closest_object_itr->velocity = closest_object_speed.value();
          return std::make_optional<ObjectData>(*closest_object_itr);
        }
      }

      return std::nullopt;
    });

    const bool has_collision =
      (closest_object_point.has_value() && closest_object_point.value().is_target)
        ? hasCollision(current_v, closest_object_point.value())
        : false;

    time_keeper_->end_track("has_collision");
    // check collision using rss distance
    return has_collision;
  };

  // step3. make function to check collision with ego path created with sensor data
  const auto ego_imu_path = (!use_imu_path_ || !angular_velocity_ptr_)
                              ? Path{}
                              : generateEgoPath(current_v, angular_velocity_ptr_->z);

  const auto ego_mpc_path = (!use_predicted_trajectory_ || !predicted_traj_ptr_)
                              ? std::nullopt
                              : generateEgoPath(*predicted_traj_ptr_);

  // The obstacle grid (sensing-side, base_link) replaces the raw cloud + voxel + cluster + hull
  // pipeline; surviving occupied-cell centers feed the unchanged getClosestObjectsOnPath corridor
  // crop. Per-path corridor cropping is recovered downstream, so no pre-crop is needed here.
  PointCloud::Ptr points_belonging_to_cluster_hulls = pcl::make_shared<PointCloud>();
  if (use_pointcloud_data_ && obstacle_grid_ptr_) {
    getCellsFromObstacleGrid(*obstacle_grid_ptr_, points_belonging_to_cluster_hulls);
  }

  const auto imu_path_objects =
    (!use_imu_path_ || !angular_velocity_ptr_)
      ? std::vector<ObjectData>{}
      : get_objects_on_path(ego_imu_path, points_belonging_to_cluster_hulls, IMU_PATH_COLOR, "imu");

  const auto mpc_path_objects =
    (!use_predicted_trajectory_ || !predicted_traj_ptr_ || !ego_mpc_path.has_value())
      ? std::vector<ObjectData>{}
      : get_objects_on_path(
          ego_mpc_path.value(), points_belonging_to_cluster_hulls, MPC_PATH_COLOR, "mpc");

  // merge object data which comes from the ego (imu) path and predicted path
  auto merge_objects =
    [&](const std::vector<ObjectData> & imu_objects, const std::vector<ObjectData> & mpc_objects) {
      std::vector<ObjectData> merged_objects = imu_objects;
      merged_objects.insert(merged_objects.end(), mpc_objects.begin(), mpc_objects.end());
      return merged_objects;
    };

  auto merged_imu_mpc_objects = merge_objects(imu_path_objects, mpc_path_objects);
  if (merged_imu_mpc_objects.empty()) return false;

  // merge path points for the collision checking
  auto merge_paths = [&](const std::optional<Path> & mpc_path, const Path & imu_path) {
    if (!mpc_path.has_value()) {
      return imu_path;
    }
    Path merged_path = imu_path;  // Start with imu_path
    merged_path.insert(
      merged_path.end(), mpc_path.value().begin(), mpc_path.value().end());  // Append mpc_path
    return merged_path;
  };

  auto merge_imu_mpc_path = merge_paths(ego_mpc_path, ego_imu_path);
  if (merge_imu_mpc_path.empty()) return false;

  // evaluate if there is a collision for merged (imu and mpc) paths
  const bool has_collision = check_collision(merge_imu_mpc_path, merged_imu_mpc_objects);

  return has_collision;
}

bool AEB::hasCollision(const double current_v, const ObjectData & closest_object)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const double rss_dist = std::invoke([&]() {
    const double & obj_v = closest_object.velocity;
    const double & t = t_response_;
    const double pre_braking_covered_distance = std::abs(current_v) * t;
    const double braking_distance = (current_v * current_v) / (2 * std::fabs(a_ego_min_));
    const double ego_stopping_distance = pre_braking_covered_distance + braking_distance;
    const double obj_braking_distance = (obj_v > 0.0)
                                          ? -(obj_v * obj_v) / (2 * std::fabs(a_obj_min_))
                                          : (obj_v * obj_v) / (2 * std::fabs(a_obj_min_));
    return ego_stopping_distance + obj_braking_distance + longitudinal_offset_margin_;
  });

  tier4_debug_msgs::msg::Float32Stamped rss_distance_msg;
  rss_distance_msg.stamp = get_clock()->now();
  rss_distance_msg.data = rss_dist;
  debug_rss_distance_publisher_->publish(rss_distance_msg);

  if (closest_object.distance_to_object > rss_dist) return false;

  // collision happens
  ObjectData collision_data = closest_object;
  collision_data.rss = rss_dist;
  collision_data_keeper_.setCollisionData(collision_data);
  return true;
}

Path AEB::generateEgoPath(const double curr_v, const double curr_w)
{
  autoware_utils::ScopedTimeTrack st(std::string(__func__) + "(IMU)", *time_keeper_);
  const double & dt = imu_prediction_time_interval_;
  const double distance_between_points = std::abs(curr_v) * dt;
  constexpr double minimum_distance_between_points{1e-2};
  // if distance between points is too small, arc length calculation is unreliable, so we skip
  // creating the path
  if (distance_between_points < minimum_distance_between_points) {
    return {};
  }

  // The initial pose is always aligned with the local reference frame.
  geometry_msgs::msg::Pose initial_pose;
  initial_pose.position = autoware_utils::create_point(0.0, 0.0, 0.0);
  initial_pose.orientation = autoware_utils::create_quaternion_from_yaw(0.0);

  const double horizon = imu_prediction_time_horizon_;
  const double base_link_to_front_offset = vehicle_info_.max_longitudinal_offset_m;
  const double rear_overhang = vehicle_info_.rear_overhang_m;
  const double vehicle_half_width = expand_width_ + vehicle_info_.vehicle_width_m / 2.0;

  // Choose the coordinates of the ego footprint vertex that will used to check for lateral
  // deviation
  const auto longitudinal_offset = (curr_v > 0.0) ? base_link_to_front_offset : -rear_overhang;
  const auto lateral_offset = (curr_v * curr_w > 0.0) ? vehicle_half_width : -vehicle_half_width;

  Path path{initial_pose};
  path.reserve(static_cast<int>(horizon / dt));
  double curr_x = 0.0;
  double curr_y = 0.0;
  double curr_yaw = 0.0;
  double path_arc_length = 0.0;
  double t = 0.0;

  while (true) {
    curr_x = curr_x + curr_v * std::cos(curr_yaw) * dt;
    curr_y = curr_y + curr_v * std::sin(curr_yaw) * dt;
    curr_yaw = curr_yaw + curr_w * dt;
    geometry_msgs::msg::Pose current_pose =
      autoware_utils::calc_offset_pose(initial_pose, curr_x, curr_y, 0.0, curr_yaw);

    t += dt;
    path_arc_length += distance_between_points;
    const auto edge_of_ego_vehicle =
      autoware_utils::calc_offset_pose(current_pose, longitudinal_offset, lateral_offset, 0.0)
        .position;

    const bool basic_path_conditions_satisfied =
      (t > horizon) && (path_arc_length > min_generated_imu_path_length_);
    const bool path_length_threshold_surpassed =
      limit_imu_path_length_ && path_arc_length > max_generated_imu_path_length_;
    const bool lat_dev_threshold_surpassed =
      limit_imu_path_lat_dev_ && std::abs(edge_of_ego_vehicle.y) > imu_path_lat_dev_threshold_;

    if (
      basic_path_conditions_satisfied || path_length_threshold_surpassed ||
      lat_dev_threshold_surpassed) {
      break;
    }

    path.push_back(current_pose);
  }
  return path;
}

std::optional<Path> AEB::generateEgoPath(const Trajectory & predicted_traj)
{
  autoware_utils::ScopedTimeTrack st(std::string(__func__) + "(MPC)", *time_keeper_);
  if (predicted_traj.points.empty()) {
    return std::nullopt;
  }
  const auto logger = get_logger();
  const auto transform_stamped =
    utils::getTransform("base_link", predicted_traj.header.frame_id, tf_buffer_, logger);
  if (!transform_stamped.has_value()) return std::nullopt;
  // create path
  time_keeper_->start_track("createPath");
  Path path;
  path.reserve(predicted_traj.points.size());
  constexpr double minimum_distance_between_points{1e-2};
  for (size_t i = 0; i < predicted_traj.points.size(); ++i) {
    geometry_msgs::msg::Pose map_pose;
    tf2::doTransform(predicted_traj.points.at(i).pose, map_pose, transform_stamped.value());

    // skip points that are too close to the last point in the path
    if (autoware_utils::calc_distance2d(path.back(), map_pose) < minimum_distance_between_points) {
      continue;
    }

    path.push_back(map_pose);

    if (i * mpc_prediction_time_interval_ > mpc_prediction_time_horizon_) {
      break;
    }
  }
  time_keeper_->end_track("createPath");
  return (!path.empty()) ? std::make_optional(path) : std::nullopt;
}

void AEB::generatePathFootprint(
  const Path & path, const double extra_width_margin, std::vector<Polygon2d> & polygons)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (path.empty()) {
    return;
  }
  for (size_t i = 0; i < path.size() - 1; ++i) {
    polygons.push_back(
      createPolygon(path.at(i), path.at(i + 1), vehicle_info_, extra_width_margin));
  }
}

std::vector<Polygon2d> AEB::generatePathFootprint(
  const Path & path, const double extra_width_margin)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (path.empty()) {
    return {};
  }
  std::vector<Polygon2d> polygons;
  for (size_t i = 0; i < path.size() - 1; ++i) {
    polygons.push_back(
      createPolygon(path.at(i), path.at(i + 1), vehicle_info_, extra_width_margin));
  }
  return polygons;
}

void AEB::createObjectDataUsingPredictedObjects(
  const Path & ego_path, const std::vector<Polygon2d> & ego_polys,
  std::vector<ObjectData> & object_data_vector)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  if (predicted_objects_ptr_->objects.empty() || ego_polys.empty()) return;

  const double current_ego_speed = current_velocity_ptr_->longitudinal_velocity;
  const auto & objects = predicted_objects_ptr_->objects;
  const auto & stamp = predicted_objects_ptr_->header.stamp;

  // Ego position
  const auto current_p = [&]() {
    const auto & first_point_of_path = ego_path.front();
    const auto & p = first_point_of_path.position;
    return autoware_utils::create_point(p.x, p.y, p.z);
  }();

  auto get_object_tangent_velocity =
    [&](const PredictedObject & predicted_object, const auto & obj_pose) {
      const double obj_vel_norm = std::hypot(
        predicted_object.kinematics.initial_twist_with_covariance.twist.linear.x,
        predicted_object.kinematics.initial_twist_with_covariance.twist.linear.y);

      const auto obj_yaw = tf2::getYaw(obj_pose.orientation);
      const auto obj_idx = autoware::motion_utils::findNearestIndex(ego_path, obj_pose.position);
      const auto path_yaw = (current_ego_speed > 0.0)
                              ? tf2::getYaw(ego_path.at(obj_idx).orientation)
                              : tf2::getYaw(ego_path.at(obj_idx).orientation) + M_PI;
      return obj_vel_norm * std::cos(obj_yaw - path_yaw);
    };

  const auto logger = get_logger();
  const auto transform_stamped_opt =
    utils::getTransform("base_link", predicted_objects_ptr_->header.frame_id, tf_buffer_, logger);
  if (!transform_stamped_opt.has_value()) return;

  const auto longitudinal_offset_opt = utils::getLongitudinalOffset(
    ego_path, vehicle_info_.max_longitudinal_offset_m, vehicle_info_.rear_overhang_m);

  if (!longitudinal_offset_opt.has_value()) return;
  const auto longitudinal_offset = longitudinal_offset_opt.value();

  // Check which objects collide with the ego footprints
  std::for_each(objects.begin(), objects.end(), [&](const auto & predicted_object) {
    // get objects in base_link frame
    const auto & transform_stamped = transform_stamped_opt.value();
    const auto t_predicted_object =
      utils::transformObjectFrame(predicted_object, transform_stamped);
    const auto & obj_pose = t_predicted_object.kinematics.initial_pose_with_covariance.pose;
    const auto obj_poly = convertObjToPolygon(t_predicted_object);
    const double obj_tangent_velocity = get_object_tangent_velocity(t_predicted_object, obj_pose);

    for (const auto & ego_poly : ego_polys) {
      // check collision with 2d polygon
      std::vector<Point2d> collision_points_bg;
      bg::intersection(ego_poly, obj_poly, collision_points_bg);
      if (collision_points_bg.empty()) continue;

      // Create an object for each intersection point
      bool collision_points_added{false};
      for (const auto & collision_point : collision_points_bg) {
        const auto obj_position =
          autoware_utils::create_point(collision_point.x(), collision_point.y(), 0.0);
        const double obj_arc_length =
          autoware::motion_utils::calcSignedArcLength(ego_path, current_p, obj_position);
        if (std::isnan(obj_arc_length)) continue;

        // If the object is behind the ego, we need to use the backward long offset. The
        // distance should be a positive number in any case
        const double dist_ego_to_object = obj_arc_length - longitudinal_offset;

        ObjectData obj;
        obj.stamp = stamp;
        obj.position = obj_position;
        obj.velocity = obj_tangent_velocity;
        obj.distance_to_object = std::abs(dist_ego_to_object);
        obj.is_target = true;
        object_data_vector.push_back(obj);
        collision_points_added = true;
      }
      // The ego polygons are in order, so the first intersection points found are the closest
      // points. It is not necessary to continue iterating the ego polys for the same object.
      if (collision_points_added) break;
    }
  });
}

void AEB::getCellsFromObstacleGrid(
  const grid_map_msgs::msg::GridMap & msg, const PointCloud::Ptr points_belonging_to_cluster_hulls)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  grid_map::GridMap grid;
  grid_map::GridMapRosConverter::fromMessage(msg, grid);
  // Normalize the circular buffer so raw (row,col) index arithmetic in the window scan below equals
  // spatial adjacency regardless of the publisher's start index.
  grid.convertToDefaultStartIndex();

  namespace gu = autoware::obstacle_grid_utils;
  // per-cell gate: at least one point and max_height >= cluster_minimum_height (z floor)
  const gu::Gate gate{1u, cluster_minimum_height_};
  const double z_band_top = vehicle_info_.vehicle_height_m + detection_range_max_height_margin_;
  const int k = window_size_;  // half-window of the (2k+1)^2 occupied-cell neighborhood

  // 1) per-cell qualification (count + height band)
  const auto size = grid.getSize();
  grid_map::Matrix qual(size(0), size(1));
  qual.setZero();
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    const auto & idx = *it;
    if (
      gu::cell_qualifies(grid, idx, gate) &&
      static_cast<double>(grid.at("min_height", idx)) <= z_band_top) {
      qual(idx(0), idx(1)) = 1.0f;
    }
  }
  // 2) window min-occupied-cells filter (~ minimum_cluster_size) + emit surviving cell centers
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    const auto & idx = *it;
    if (qual(idx(0), idx(1)) == 0.0f) {
      continue;
    }
    int neighbours = 0;
    for (int di = -k; di <= k; ++di) {
      for (int dj = -k; dj <= k; ++dj) {
        const int i = idx(0) + di;
        const int j = idx(1) + dj;
        if (i >= 0 && j >= 0 && i < qual.rows() && j < qual.cols() && qual(i, j) > 0.0f) {
          ++neighbours;
        }
      }
    }
    if (neighbours < minimum_cluster_size_) {
      continue;  // approximate the per-cluster minimum size gate
    }
    grid_map::Position c;
    grid.getPosition(idx, c);
    points_belonging_to_cluster_hulls->push_back(
      pcl::PointXYZ(c.x(), c.y(), grid.at("max_height", idx)));
  }
}

void AEB::getClosestObjectsOnPath(
  const Path & ego_path, const rclcpp::Time & stamp,
  const PointCloud::Ptr points_belonging_to_cluster_hulls, std::vector<ObjectData> & objects)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  // check if the predicted path has a valid number of points
  if (ego_path.size() < 2 || points_belonging_to_cluster_hulls->empty()) {
    return;
  }

  const auto longitudinal_offset_opt = utils::getLongitudinalOffset(
    ego_path, vehicle_info_.max_longitudinal_offset_m, vehicle_info_.rear_overhang_m);

  if (!longitudinal_offset_opt.has_value()) return;
  const auto longitudinal_offset = longitudinal_offset_opt.value();
  const auto path_length = autoware::motion_utils::calcArcLength(ego_path);
  const auto path_width = vehicle_info_.vehicle_width_m / 2.0 + expand_width_;
  // select points inside the ego footprint path
  for (const auto & p : *points_belonging_to_cluster_hulls) {
    const auto obj_position = autoware_utils::create_point(p.x, p.y, p.z);
    auto obj_data_opt = utils::getObjectOnPathData(
      ego_path, obj_position, stamp, path_length, path_width, speed_calculation_expansion_margin_,
      longitudinal_offset, 0.0);
    if (obj_data_opt.has_value()) objects.push_back(obj_data_opt.value());
  }
}

void AEB::addMarker(
  const rclcpp::Time & current_time, const Path & path, const std::vector<Polygon2d> & polygons,
  const std::vector<ObjectData> & objects, const std::optional<ObjectData> & closest_object,
  const colorTuple & debug_colors, const std::string & ns, MarkerArray & debug_markers)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto [color_r, color_g, color_b, color_a] = debug_colors;

  auto path_marker = autoware_utils::create_default_marker(
    "base_link", current_time, ns + "_path", 0L, Marker::LINE_STRIP,
    autoware_utils::create_marker_scale(0.2, 0.2, 0.2),
    autoware_utils::create_marker_color(color_r, color_g, color_b, color_a));
  path_marker.points.reserve(path.size());
  for (const auto & p : path) {
    path_marker.points.push_back(p.position);
  }
  debug_markers.markers.push_back(path_marker);

  auto polygon_marker = autoware_utils::create_default_marker(
    "base_link", current_time, ns + "_polygon", 0, Marker::LINE_LIST,
    autoware_utils::create_marker_scale(0.03, 0.0, 0.0),
    autoware_utils::create_marker_color(color_r, color_g, color_b, color_a));
  utils::fillMarkerFromPolygon(polygons, polygon_marker);
  debug_markers.markers.push_back(polygon_marker);

  auto object_data_marker = autoware_utils::create_default_marker(
    "base_link", current_time, ns + "_objects", 0, Marker::SPHERE_LIST,
    autoware_utils::create_marker_scale(0.5, 0.5, 0.5),
    autoware_utils::create_marker_color(color_r, color_g, color_b, color_a));
  for (const auto & e : objects) {
    object_data_marker.points.push_back(e.position);
  }
  debug_markers.markers.push_back(object_data_marker);

  // Visualize planner type text
  if (closest_object.has_value()) {
    const auto & obj = closest_object.value();
    const auto color = autoware_utils::create_marker_color(0.95, 0.95, 0.95, 0.999);
    auto closest_object_velocity_marker_array = autoware_utils::create_default_marker(
      "base_link", obj.stamp, ns + "_closest_object_velocity", 0,
      visualization_msgs::msg::Marker::TEXT_VIEW_FACING,
      autoware_utils::create_marker_scale(0.0, 0.0, 0.7), color);
    closest_object_velocity_marker_array.pose.position = obj.position;
    const auto ego_velocity = current_velocity_ptr_->longitudinal_velocity;
    closest_object_velocity_marker_array.text =
      "Object velocity: " + std::to_string(obj.velocity) + " [m/s]\n";
    closest_object_velocity_marker_array.text +=
      "Object relative velocity to ego: " + std::to_string(obj.velocity - std::abs(ego_velocity)) +
      " [m/s]\n";
    closest_object_velocity_marker_array.text +=
      "Object distance to ego: " + std::to_string(obj.distance_to_object) + " [m]\n";
    closest_object_velocity_marker_array.text +=
      "RSS distance: " + std::to_string(obj.rss) + " [m]";
    debug_markers.markers.push_back(closest_object_velocity_marker_array);
  }
}

void AEB::addVirtualStopWallMarker(MarkerArray & markers)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  const auto ego_map_pose = std::invoke([this]() -> std::optional<geometry_msgs::msg::Pose> {
    const auto logger = get_logger();
    const auto tf_current_pose = utils::getTransform("map", "base_link", tf_buffer_, logger);
    if (!tf_current_pose.has_value()) return std::nullopt;
    const auto transform = tf_current_pose.value().transform;
    geometry_msgs::msg::Pose p;
    p.orientation = transform.rotation;
    p.position.x = transform.translation.x;
    p.position.y = transform.translation.y;
    p.position.z = transform.translation.z;
    return std::make_optional(p);
  });

  if (ego_map_pose.has_value()) {
    const double base_link_to_front_offset = vehicle_info_.max_longitudinal_offset_m;
    const auto ego_front_pose = autoware_utils::calc_offset_pose(
      ego_map_pose.value(), base_link_to_front_offset, 0.0, 0.0, 0.0);
    const auto virtual_stop_wall = autoware::motion_utils::createStopVirtualWallMarker(
      ego_front_pose, "autonomous_emergency_braking", this->now(), 0);
    autoware_utils::append_marker_array(virtual_stop_wall, &markers);
  }
}

void AEB::addCollisionMarker(const ObjectData & data, MarkerArray & debug_markers)
{
  autoware_utils::ScopedTimeTrack st(__func__, *time_keeper_);
  auto point_marker = autoware_utils::create_default_marker(
    "base_link", data.stamp, "collision_point", 0, Marker::SPHERE,
    autoware_utils::create_marker_scale(0.3, 0.3, 0.3),
    autoware_utils::create_marker_color(1.0, 0.0, 0.0, 0.3));
  point_marker.pose.position = data.position;
  debug_markers.markers.push_back(point_marker);
}

}  // namespace autoware::motion::control::autonomous_emergency_braking

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::motion::control::autonomous_emergency_braking::AEB)
