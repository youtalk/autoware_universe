// Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include "autoware/obstacle_collision_checker/obstacle_collision_checker.hpp"

#include <autoware/obstacle_grid_utils/obstacle_grid_utils.hpp>
#include <autoware_utils/geometry/geometry.hpp>
#include <autoware_utils/math/normalization.hpp>
#include <autoware_utils/math/unit_conversion.hpp>
#include <autoware_utils/system/stop_watch.hpp>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <pcl_ros/transforms.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <boost/geometry.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include <optional>
#include <vector>

namespace
{
pcl::PointCloud<pcl::PointXYZ> get_transformed_point_cloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg,
  const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}

pcl::PointCloud<pcl::PointXYZ> filter_point_cloud_by_trajectory(
  const pcl::PointCloud<pcl::PointXYZ> & pointcloud,
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double radius)
{
  pcl::PointCloud<pcl::PointXYZ> filtered_pointcloud;
  for (const auto & point : pointcloud.points) {
    for (const auto & trajectory_point : trajectory.points) {
      const double dx = trajectory_point.pose.position.x - point.x;
      const double dy = trajectory_point.pose.position.y - point.y;
      if (std::hypot(dx, dy) < radius) {
        filtered_pointcloud.points.push_back(point);
        break;
      }
    }
  }
  return filtered_pointcloud;
}

double calc_braking_distance(
  const double abs_velocity, const double max_deceleration, const double delay_time)
{
  const double idling_distance = abs_velocity * delay_time;
  const double braking_distance = (abs_velocity * abs_velocity) / (2.0 * max_deceleration);
  return idling_distance + braking_distance;
}

}  // namespace

namespace autoware::obstacle_collision_checker
{
Output check_for_collisions(const Input & input)
{
  Output output;
  autoware_utils::StopWatch<std::chrono::milliseconds> stop_watch;

  // resample trajectory by braking distance
  constexpr double min_velocity = 0.01;
  const auto & raw_abs_velocity = std::abs(input.current_twist->linear.x);
  const auto abs_velocity = raw_abs_velocity < min_velocity ? 0.0 : raw_abs_velocity;
  const auto braking_distance =
    calc_braking_distance(abs_velocity, input.param.max_deceleration, input.param.delay_time);
  output.resampled_trajectory = cut_trajectory(
    resample_trajectory(*input.predicted_trajectory, input.param.resample_interval),
    braking_distance);
  output.processing_time_map["resampleTrajectory"] = stop_watch.toc(true);

  // resample pointcloud
  const auto obstacle_pointcloud =
    get_transformed_point_cloud(*input.obstacle_pointcloud, input.obstacle_transform->transform);
  const auto filtered_obstacle_pointcloud = filter_point_cloud_by_trajectory(
    obstacle_pointcloud, output.resampled_trajectory, input.param.search_radius);

  output.vehicle_footprints =
    create_vehicle_footprints(output.resampled_trajectory, input.param, input.vehicle_info);
  output.processing_time_map["createVehicleFootprints"] = stop_watch.toc(true);

  output.vehicle_passing_areas = create_vehicle_passing_areas(output.vehicle_footprints);
  output.processing_time_map["createVehiclePassingAreas"] = stop_watch.toc(true);

  output.will_collide = will_collide(filtered_obstacle_pointcloud, output.vehicle_passing_areas);
  output.processing_time_map["willCollide"] = stop_watch.toc(true);

  return output;
}

autoware_planning_msgs::msg::Trajectory resample_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double interval)
{
  autoware_planning_msgs::msg::Trajectory resampled;
  resampled.header = trajectory.header;

  resampled.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size() - 1; ++i) {
    const auto & point = trajectory.points.at(i);

    const auto distance =
      autoware_utils::calc_distance2d(resampled.points.back(), point.pose.position);
    if (distance > interval) {
      resampled.points.push_back(point);
    }
  }
  resampled.points.push_back(trajectory.points.back());

  return resampled;
}

autoware_planning_msgs::msg::Trajectory cut_trajectory(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const double length)
{
  autoware_planning_msgs::msg::Trajectory cut;
  cut.header = trajectory.header;

  double total_length = 0.0;
  cut.points.push_back(trajectory.points.front());
  for (size_t i = 1; i < trajectory.points.size(); ++i) {
    const auto & point = trajectory.points.at(i);

    const auto p1 = autoware_utils::from_msg(cut.points.back().pose.position);
    const auto p2 = autoware_utils::from_msg(point.pose.position);

    const auto points_distance = boost::geometry::distance(p1, p2);
    const auto remain_distance = length - total_length;

    // Over length
    if (remain_distance <= 0.0) {
      break;
    }

    // Require interpolation
    if (remain_distance <= points_distance) {
      const Eigen::Vector3d p_interpolated = p1 + remain_distance * (p2 - p1).normalized();

      autoware_planning_msgs::msg::TrajectoryPoint p;
      p.pose.position.x = p_interpolated.x();
      p.pose.position.y = p_interpolated.y();
      p.pose.position.z = p_interpolated.z();
      p.pose.orientation = point.pose.orientation;

      cut.points.push_back(p);
      break;
    }

    cut.points.push_back(point);
    total_length += points_distance;
  }

  return cut;
}

std::vector<LinearRing2d> create_vehicle_footprints(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const Param & param,
  const autoware::vehicle_info_utils::VehicleInfo & vehicle_info)
{
  // Create vehicle footprint in base_link coordinate
  const auto local_vehicle_footprint = vehicle_info.createFootprint(param.footprint_margin);

  // Create vehicle footprint on each TrajectoryPoint
  std::vector<LinearRing2d> vehicle_footprints;
  for (const auto & p : trajectory.points) {
    vehicle_footprints.push_back(
      autoware_utils::transform_vector<autoware_utils::LinearRing2d>(
        local_vehicle_footprint, autoware_utils::pose2transform(p.pose)));
  }

  return vehicle_footprints;
}

std::vector<LinearRing2d> create_vehicle_passing_areas(
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  // Create hull from two adjacent vehicle footprints
  std::vector<LinearRing2d> areas;
  for (size_t i = 0; i < vehicle_footprints.size() - 1; ++i) {
    const auto & footprint1 = vehicle_footprints.at(i);
    const auto & footprint2 = vehicle_footprints.at(i + 1);
    areas.push_back(create_hull_from_footprints(footprint1, footprint2));
  }

  return areas;
}

LinearRing2d create_hull_from_footprints(const LinearRing2d & area1, const LinearRing2d & area2)
{
  autoware_utils::MultiPoint2d combined;
  for (const auto & p : area1) {
    combined.push_back(p);
  }
  for (const auto & p : area2) {
    combined.push_back(p);
  }
  LinearRing2d hull;
  boost::geometry::convex_hull(combined, hull);
  return hull;
}

bool will_collide(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const std::vector<LinearRing2d> & vehicle_footprints)
{
  for (size_t i = 1; i < vehicle_footprints.size(); i++) {
    // skip first footprint because surround obstacle checker handle it
    const auto & vehicle_footprint = vehicle_footprints.at(i);
    if (has_collision(obstacle_pointcloud, vehicle_footprint)) {
      RCLCPP_WARN(rclcpp::get_logger("obstacle_collision_checker"), "willCollide");
      return true;
    }
  }

  return false;
}

bool has_collision(
  const pcl::PointCloud<pcl::PointXYZ> & obstacle_pointcloud,
  const LinearRing2d & vehicle_footprint)
{
  for (const auto & point : obstacle_pointcloud.points) {
    if (boost::geometry::within(autoware_utils::Point2d{point.x, point.y}, vehicle_footprint)) {
      RCLCPP_WARN(
        rclcpp::get_logger("obstacle_collision_checker"), "Collide to Point x: %f y: %f", point.x,
        point.y);
      return true;
    }
  }

  return false;
}

std::optional<sensor_msgs::msg::PointCloud2> extract_grid_obstacle_pointcloud(
  const grid_map_msgs::msg::GridMap & msg)
{
  static rclcpp::Clock clock{RCL_ROS_TIME};
  const auto logger = rclcpp::get_logger("obstacle_collision_checker");

  // Contract validation. A grid cannot be cheaply re-framed, so a frame mismatch is a wiring error:
  // reject loudly and return nullopt so the caller reads it as "unavailable", never as "clear".
  if (msg.header.frame_id != "base_link") {
    RCLCPP_ERROR_THROTTLE(
      logger, clock, 5000, "obstacle grid frame is '%s', expected 'base_link'; ignoring the grid",
      msg.header.frame_id.c_str());
    return std::nullopt;
  }

  grid_map::GridMap grid;
  if (!grid_map::GridMapRosConverter::fromMessage(msg, grid)) {
    RCLCPP_ERROR_THROTTLE(logger, clock, 5000, "failed to convert the obstacle grid message");
    return std::nullopt;
  }

  // cell_qualifies reads only the point_count and max_height layers, so those are the only two the
  // gate requires; a missing layer would otherwise throw std::out_of_range and kill the container.
  for (const char * layer : {"point_count", "max_height"}) {
    if (!grid.exists(layer)) {
      RCLCPP_ERROR_THROTTLE(
        logger, clock, 5000, "obstacle grid is missing the '%s' layer; ignoring the grid", layer);
      return std::nullopt;
    }
  }

  // Purely 2D density gate: a cell is occupied iff it holds at least one point whose max_height is
  // at or above the 0.0 floor. No upper height band, matching the "no z-gate" policy for 2D grid
  // consumers.
  //
  // Conservatism caveat vs the legacy raw-cloud path (which used only x/y and ignored z entirely):
  //  - Lower floor (0.0, base_link frame): this assumes base_link z=0 sits at/near the ground so a
  //    standing obstacle reports max_height > 0. It is strictly LESS sensitive than legacy for a
  //    short obstacle whose entire top sits below the base_link horizontal plane (e.g. a low object
  //    on a steep downslope): that cell reports max_height < 0, does not qualify, and is missed.
  //  - No upper band: any qualifying cell emits corners regardless of point height, so this relies
  //    on the grid PRODUCER height-cropping before counting (point_count must exclude overhead
  //    structures such as gantries, signs, or low branches). If the producer does not height-crop,
  //    an overhead-only cell becomes a phantom collision that a height-cropped legacy cloud
  //    avoided.
  constexpr autoware::obstacle_grid_utils::Gate gate{1U, 0.0};
  const double resolution = grid.getResolution();

  pcl::PointCloud<pcl::PointXYZ> cloud;
  for (grid_map::GridMapIterator it(grid); !it.isPastEnd(); ++it) {
    if (!autoware::obstacle_grid_utils::cell_qualifies(grid, *it, gate)) {
      continue;
    }
    grid_map::Position center;
    grid.getPosition(*it, center);
    // Edge-conservative: emit the 4 cell corners (z = 0, purely 2D) so a cell overlapping the
    // vehicle footprint always contributes at least one point inside it.
    for (const auto & corner : autoware::obstacle_grid_utils::cell_corners(center, resolution)) {
      cloud.push_back(
        pcl::PointXYZ(static_cast<float>(corner.x()), static_cast<float>(corner.y()), 0.0f));
    }
  }

  sensor_msgs::msg::PointCloud2 out;
  pcl::toROSMsg(cloud, out);
  out.header = msg.header;  // base_link frame, source-cloud stamp
  return out;
}

bool is_grid_stale(
  const rclcpp::Time & grid_stamp, const rclcpp::Time & now, const double timeout_sec)
{
  return (now - grid_stamp).seconds() > timeout_sec;
}
}  // namespace autoware::obstacle_collision_checker
