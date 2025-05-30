// Copyright 2024 Tier IV, Inc.
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

#include "pointcloud_based_occupancy_grid_map_node.hpp"

#include "autoware/probabilistic_occupancy_grid_map/cost_value/cost_value.hpp"
#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_fixed.hpp"
#include "autoware/probabilistic_occupancy_grid_map/costmap_2d/occupancy_grid_map_projective.hpp"
#include "autoware/probabilistic_occupancy_grid_map/utils/utils.hpp"

#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <pcl_ros/transforms.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#endif

#include <algorithm>
#include <limits>
#include <memory>
#include <sstream>
#include <string>

namespace autoware::occupancy_grid_map
{
using autoware_utils::ScopedTimeTrack;
using costmap_2d::OccupancyGridMapBBFUpdater;
using costmap_2d::OccupancyGridMapFixedBlindSpot;
using costmap_2d::OccupancyGridMapProjectiveBlindSpot;
using geometry_msgs::msg::Pose;

PointcloudBasedOccupancyGridMapNode::PointcloudBasedOccupancyGridMapNode(
  const rclcpp::NodeOptions & node_options)
: Node("pointcloud_based_occupancy_grid_map_node", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;

  /* params */
  map_frame_ = this->declare_parameter<std::string>("map_frame");
  base_link_frame_ = this->declare_parameter<std::string>("base_link_frame");
  gridmap_origin_frame_ = this->declare_parameter<std::string>("gridmap_origin_frame");
  scan_origin_frame_ = this->declare_parameter<std::string>("scan_origin_frame", "");
  use_height_filter_ = this->declare_parameter<bool>("height_filter.use_height_filter");
  min_height_ = this->declare_parameter<double>("height_filter.min_height");
  max_height_ = this->declare_parameter<double>("height_filter.max_height");
  enable_single_frame_mode_ = this->declare_parameter<bool>("enable_single_frame_mode");
  filter_obstacle_pointcloud_by_raw_pointcloud_ =
    this->declare_parameter<bool>("filter_obstacle_pointcloud_by_raw_pointcloud");
  const double map_length = this->declare_parameter<double>("map_length");
  const double map_resolution = this->declare_parameter<double>("map_resolution");

  /* Subscriber and publisher */
  obstacle_pointcloud_sub_ptr_ = this->create_subscription<PointCloud2>(
    "~/input/obstacle_pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&PointcloudBasedOccupancyGridMapNode::obstaclePointcloudCallback, this, _1));
  raw_pointcloud_sub_ptr_ = this->create_subscription<PointCloud2>(
    "~/input/raw_pointcloud", rclcpp::SensorDataQoS{}.keep_last(1),
    std::bind(&PointcloudBasedOccupancyGridMapNode::rawPointcloudCallback, this, _1));

  occupancy_grid_map_pub_ = create_publisher<OccupancyGrid>("~/output/occupancy_grid_map", 1);

  const std::string updater_type = this->declare_parameter<std::string>("updater_type");
  if (updater_type == "binary_bayes_filter") {
    occupancy_grid_map_updater_ptr_ = std::make_unique<OccupancyGridMapBBFUpdater>(
      true, map_length / map_resolution, map_length / map_resolution, map_resolution);
  } else {
    RCLCPP_WARN(
      get_logger(),
      "specified occupancy grid map updater type [%s] is not found, use binary_bayes_filter",
      updater_type.c_str());
    occupancy_grid_map_updater_ptr_ = std::make_unique<OccupancyGridMapBBFUpdater>(
      true, map_length / map_resolution, map_length / map_resolution, map_resolution);
  }

  const std::string grid_map_type = this->declare_parameter<std::string>("grid_map_type");

  if (grid_map_type == "OccupancyGridMapProjectiveBlindSpot") {
    occupancy_grid_map_ptr_ = std::make_unique<OccupancyGridMapProjectiveBlindSpot>(
      true, occupancy_grid_map_updater_ptr_->getSizeInCellsX(),
      occupancy_grid_map_updater_ptr_->getSizeInCellsY(),
      occupancy_grid_map_updater_ptr_->getResolution());
  } else if (grid_map_type == "OccupancyGridMapFixedBlindSpot") {
    occupancy_grid_map_ptr_ = std::make_unique<OccupancyGridMapFixedBlindSpot>(
      true, occupancy_grid_map_updater_ptr_->getSizeInCellsX(),
      occupancy_grid_map_updater_ptr_->getSizeInCellsY(),
      occupancy_grid_map_updater_ptr_->getResolution());
  } else {
    RCLCPP_WARN(
      get_logger(),
      "specified occupancy grid map type [%s] is not found, use OccupancyGridMapFixedBlindSpot",
      grid_map_type.c_str());
    occupancy_grid_map_ptr_ = std::make_unique<OccupancyGridMapFixedBlindSpot>(
      true, occupancy_grid_map_updater_ptr_->getSizeInCellsX(),
      occupancy_grid_map_updater_ptr_->getSizeInCellsY(),
      occupancy_grid_map_updater_ptr_->getResolution());
  }

  cudaStreamCreateWithFlags(&stream_, cudaStreamNonBlocking);
  raw_pointcloud_.stream = stream_;
  obstacle_pointcloud_.stream = stream_;
  occupancy_grid_map_ptr_->setCudaStream(stream_);
  occupancy_grid_map_updater_ptr_->setCudaStream(stream_);

  device_rotation_ = autoware::cuda_utils::make_unique<Eigen::Matrix3f>();
  device_translation_ = autoware::cuda_utils::make_unique<Eigen::Vector3f>();

  occupancy_grid_map_ptr_->initRosParam(*this);
  occupancy_grid_map_updater_ptr_->initRosParam(*this);

  // initialize debug tool
  {
    using autoware_utils::DebugPublisher;
    using autoware_utils::StopWatch;
    stop_watch_ptr_ = std::make_unique<StopWatch<std::chrono::milliseconds>>();
    debug_publisher_ptr_ =
      std::make_unique<DebugPublisher>(this, "pointcloud_based_occupancy_grid_map");
    stop_watch_ptr_->tic("cyclic_time");
    stop_watch_ptr_->tic("processing_time");

    // time keeper setup
    bool use_time_keeper = declare_parameter<bool>("publish_processing_time_detail");
    if (use_time_keeper) {
      detailed_processing_time_publisher_ =
        this->create_publisher<autoware_utils::ProcessingTimeDetail>(
          "~/debug/processing_time_detail_ms", 1);
      auto time_keeper = autoware_utils::TimeKeeper(detailed_processing_time_publisher_);
      time_keeper_ = std::make_shared<autoware_utils::TimeKeeper>(time_keeper);
    }
  }

  processing_time_tolerance_ms_ = this->declare_parameter<double>("processing_time_tolerance_ms");
  processing_time_consecutive_excess_tolerance_ms_ =
    this->declare_parameter<double>("processing_time_consecutive_excess_tolerance_ms");
  diagnostics_interface_ptr_ = std::make_unique<autoware_utils::DiagnosticsInterface>(
    this, "pointcloud_based_probabilistic_occupancy_grid_map");
}

void PointcloudBasedOccupancyGridMapNode::obstaclePointcloudCallback(
  const PointCloud2::ConstSharedPtr & input_obstacle_msg)
{
  obstacle_pointcloud_.fromROSMsgAsync(input_obstacle_msg);

  if (obstacle_pointcloud_.header.stamp == raw_pointcloud_.header.stamp) {
    onPointcloudWithObstacleAndRaw();
  }
}

void PointcloudBasedOccupancyGridMapNode::rawPointcloudCallback(
  const PointCloud2::ConstSharedPtr & input_raw_msg)
{
  raw_pointcloud_.fromROSMsgAsync(input_raw_msg);

  if (obstacle_pointcloud_.header.stamp == raw_pointcloud_.header.stamp) {
    onPointcloudWithObstacleAndRaw();
  }
}

void PointcloudBasedOccupancyGridMapNode::checkProcessingTime(double processing_time_ms)
{
  static rclcpp::Time last_normal_time = this->get_clock()->now();
  const bool is_processing_time_within_range =
    (processing_time_ms <= processing_time_tolerance_ms_);

  // Update timestamp when latency is normal
  if (is_processing_time_within_range) {
    last_normal_time = this->get_clock()->now();
  }

  // Calculate duration of abnormal latency
  const double processing_consecutive_excess_time =
    (this->get_clock()->now() - last_normal_time).seconds() * 1000.0;

  uint8_t level;
  std::string status_str;
  std::string message;

  if (is_processing_time_within_range) {
    level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status_str = "OK";
  } else if (
    processing_consecutive_excess_time > processing_time_consecutive_excess_tolerance_ms_) {
    status_str = "ERROR";
    level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    message = "Processing time exceeded the warning threshold of " +
              std::to_string(processing_time_tolerance_ms_) + "ms for " +
              std::to_string(processing_consecutive_excess_time / 1000.0) + "s  (Threshold " +
              std::to_string(processing_time_consecutive_excess_tolerance_ms_ / 1000.0) + "s)";
  } else {
    status_str = "WARN";
    level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    message = "Processing time exceeds the warning threshold of " +
              std::to_string(processing_time_tolerance_ms_) + " ms.";
  }

  diagnostics_interface_ptr_->clear();
  diagnostics_interface_ptr_->add_key_value("processing time(ms)", processing_time_ms);
  diagnostics_interface_ptr_->add_key_value(
    "is processing time within threshold", is_processing_time_within_range);
  diagnostics_interface_ptr_->add_key_value(
    "processing time consecutive excess duration(ms)", processing_consecutive_excess_time);
  diagnostics_interface_ptr_->add_key_value(
    "is processing time consecutive excess duration within threshold",
    processing_consecutive_excess_time <= processing_time_consecutive_excess_tolerance_ms_);
  diagnostics_interface_ptr_->update_level_and_message(level, "[" + status_str + "] " + message);
  diagnostics_interface_ptr_->publish(raw_pointcloud_.header.stamp);
}

void PointcloudBasedOccupancyGridMapNode::onPointcloudWithObstacleAndRaw()
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  if (stop_watch_ptr_) {
    stop_watch_ptr_->toc("processing_time", true);
  }

  // if scan_origin_frame_ is "", replace it with raw_pointcloud_.header.frame_id
  if (scan_origin_frame_.empty()) {
    scan_origin_frame_ = raw_pointcloud_.header.frame_id;
  }

  // Prepare for applying height filter
  if (use_height_filter_) {
    // Make sure that the frame is base_link
    if (raw_pointcloud_.header.frame_id != base_link_frame_) {
      if (!utils::transformPointcloudAsync(
            raw_pointcloud_, *tf2_, base_link_frame_, device_rotation_, device_translation_)) {
        return;
      }
    }
    if (obstacle_pointcloud_.header.frame_id != base_link_frame_) {
      if (!utils::transformPointcloudAsync(
            obstacle_pointcloud_, *tf2_, base_link_frame_, device_rotation_, device_translation_)) {
        return;
      }
    }
    occupancy_grid_map_ptr_->setHeightLimit(min_height_, max_height_);
  } else {
    occupancy_grid_map_ptr_->setHeightLimit(
      -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity());
  }

  // Get from map to sensor frame pose
  Pose robot_pose{};
  Pose gridmap_origin{};
  Pose scan_origin{};
  try {
    robot_pose = utils::getPose(raw_pointcloud_.header.stamp, *tf2_, base_link_frame_, map_frame_);
    gridmap_origin =
      utils::getPose(raw_pointcloud_.header.stamp, *tf2_, gridmap_origin_frame_, map_frame_);
    scan_origin =
      utils::getPose(raw_pointcloud_.header.stamp, *tf2_, scan_origin_frame_, map_frame_);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN_STREAM(get_logger(), ex.what());
    return;
  }

  {  // create occupancy grid map and publish it
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("create_occupancy_grid_map", *time_keeper_);

    // Create single frame occupancy grid map
    occupancy_grid_map_ptr_->resetMaps();
    occupancy_grid_map_ptr_->updateOrigin(
      gridmap_origin.position.x - occupancy_grid_map_ptr_->getSizeInMetersX() / 2,
      gridmap_origin.position.y - occupancy_grid_map_ptr_->getSizeInMetersY() / 2);
    occupancy_grid_map_ptr_->updateWithPointCloud(
      raw_pointcloud_, obstacle_pointcloud_, robot_pose, scan_origin);
  }

  if (enable_single_frame_mode_) {
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr = std::make_unique<ScopedTimeTrack>("publish_occupancy_grid_map", *time_keeper_);

    occupancy_grid_map_ptr_->copyDeviceCostmapToHost();

    // publish
    occupancy_grid_map_pub_->publish(OccupancyGridMapToMsgPtr(
      map_frame_, raw_pointcloud_.header.stamp, robot_pose.position.z,
      *occupancy_grid_map_ptr_));  // (todo) robot_pose may be altered with gridmap_origin
  } else {
    std::unique_ptr<ScopedTimeTrack> inner_st_ptr;
    if (time_keeper_)
      inner_st_ptr =
        std::make_unique<ScopedTimeTrack>("update_and_publish_occupancy_grid_map", *time_keeper_);

    // Update with bayes filter
    occupancy_grid_map_updater_ptr_->update(*occupancy_grid_map_ptr_);
    occupancy_grid_map_updater_ptr_->copyDeviceCostmapToHost();

    // publish
    occupancy_grid_map_pub_->publish(OccupancyGridMapToMsgPtr(
      map_frame_, raw_pointcloud_.header.stamp, robot_pose.position.z,
      *occupancy_grid_map_updater_ptr_));
  }

  if (debug_publisher_ptr_ && stop_watch_ptr_) {
    const double cyclic_time_ms = stop_watch_ptr_->toc("cyclic_time", true);
    const double processing_time_ms = stop_watch_ptr_->toc("processing_time", true);
    const double pipeline_latency_ms =
      std::chrono::duration<double, std::milli>(
        std::chrono::nanoseconds(
          (this->get_clock()->now() - raw_pointcloud_.header.stamp).nanoseconds()))
        .count();
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/cyclic_time_ms", cyclic_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/processing_time_ms", processing_time_ms);
    debug_publisher_ptr_->publish<autoware_internal_debug_msgs::msg::Float64Stamped>(
      "debug/pipeline_latency_ms", pipeline_latency_ms);

    checkProcessingTime(processing_time_ms);
  }
}

OccupancyGrid::UniquePtr PointcloudBasedOccupancyGridMapNode::OccupancyGridMapToMsgPtr(
  const std::string & frame_id, const Time & stamp, const float & robot_pose_z,
  const Costmap2D & occupancy_grid_map)
{
  std::unique_ptr<ScopedTimeTrack> st_ptr;
  if (time_keeper_) st_ptr = std::make_unique<ScopedTimeTrack>(__func__, *time_keeper_);

  auto msg_ptr = std::make_unique<OccupancyGrid>();

  msg_ptr->header.frame_id = frame_id;
  msg_ptr->header.stamp = stamp;
  msg_ptr->info.resolution = occupancy_grid_map.getResolution();

  msg_ptr->info.width = occupancy_grid_map.getSizeInCellsX();
  msg_ptr->info.height = occupancy_grid_map.getSizeInCellsY();

  double wx{};
  double wy{};
  occupancy_grid_map.mapToWorld(0, 0, wx, wy);
  msg_ptr->info.origin.position.x = occupancy_grid_map.getOriginX();
  msg_ptr->info.origin.position.y = occupancy_grid_map.getOriginY();
  msg_ptr->info.origin.position.z = robot_pose_z;
  msg_ptr->info.origin.orientation.w = 1.0;

  msg_ptr->data.resize(msg_ptr->info.width * msg_ptr->info.height);

  unsigned char * data = occupancy_grid_map.getCharMap();
  for (unsigned int i = 0; i < msg_ptr->data.size(); ++i) {
    msg_ptr->data[i] = cost_value::cost_translation_table[data[i]];
  }
  return msg_ptr;
}

}  // namespace autoware::occupancy_grid_map

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::occupancy_grid_map::PointcloudBasedOccupancyGridMapNode)
