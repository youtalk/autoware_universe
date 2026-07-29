# Collision Detector

## Purpose

This module publishes an `ERROR` diagnostic if a collision is detected with the current ego footprint.

## Inner-workings / Algorithms

### Flow chart

1. Check input data.
2. Filter dynamic objects.
3. Find nearest object and its distance to ego.
4. Publish an `ERROR` diagnostic depending on the recent collision detection results.

### Algorithms

### Check data

Check that `collision_detector` receives the obstacle grid and/or dynamic objects.
When `use_pointcloud` is enabled, the node validates the incoming obstacle grid contract (frame
`base_link`, the required `point_count` / `max_height` layers) and watchdogs its stamp age against
`obstacle_grid_timeout_sec`. A missing, stale, or contract-violating grid is treated as unavailable,
never as "clear": the collision check returns without summarizing, so `diagnostic_updater` publishes
its default `ERROR` status with the message `No message was set` for that cycle.

### Object Filtering

#### Recognition Assumptions

1. If the classification changes but it's considered the same object, the uuid does not change.
2. It's possible for the same uuid to be recognized after being lost for a few frames.
3. Once an object is determined to be excluded, it continues to be excluded for a certain period of time.

#### Filtering Process

1. Initial Recognition and Exclusion:
   - The system checks if a newly recognized object's classification is listed in `nearby_object_type_filters`.
   - Supported labels follow `autoware_perception_msgs/msg/ObjectClassification.msg`:
     `UNKNOWN`, `CAR`, `TRUCK`, `BUS`, `TRAILER`, `MOTORCYCLE`, `BICYCLE`, `PEDESTRIAN`,
     `ANIMAL`, `HAZARD`, `OVER_DRIVABLE`, and `UNDER_DRIVABLE`.
   - If so, and the object is within the `nearby_filter_radius`, it is marked for exclusion.

2. New Object Determination:
   - An object is considered "new" based on its UUID.
   - If the UUID is not found in recent frame data, the object is treated as new.

3. Exclusion Mechanism:
   - Newly excluded objects are recorded by their UUID.
   - These objects continue to be excluded for a set period (`keep_ignoring_time`) as long as they maintain the classification specified in `nearby_object_type_filters` and remain within the `nearby_filter_radius`.

### Get distance to nearest object

Calculate distance between ego vehicle and the nearest object.
For the obstacle grid, the minimum distance is taken between the ego footprint polygon and the footprint box of every qualifying cell (a cell qualifies with `point_count >= 1` and `max_height >= 0`; the `max_height >= 0` floor is a height gate that drops cells whose returns all lie below `base_link` z=0, and the producer additionally crops z to `[-1, 3]` m. Unlike the pre-migration per-point loop, which had no z filter, the grid therefore drops those sub-ground far-field returns and is strictly more conservative in the far field). For dynamic objects, the minimum distance is taken between the ego footprint polygon and the object polygons. The smaller of the two candidates is used.
If the minimum distance is lower than the `collision_distance` parameter, then a collision is detected.

### Time buffer and distance hysteresis

Before publishing an `ERROR` diagnostic, a collision must be detected for at least a duration set by the parameter `time_buffer.on`.
Once an `ERROR` diagnostic is published, the `time_buffer.off_distance_hysteresis` parameter is used to make the ego footprint larger,
making it easier to detect a collision.
To stop publishing the `ERROR` diagnostic, no collision must be detected for at least a duration set by the parameter `time_buffer.off`.

## Inputs / Outputs

### Input

| Name                                           | Type                                              | Description                                                                   |
| ---------------------------------------------- | ------------------------------------------------- | ----------------------------------------------------------------------------- |
| `/sensing/obstacle_segmentation/obstacle_grid` | `grid_map_msgs::msg::GridMap`                     | Obstacle grid (base_link) whose qualifying cells the ego should stop or avoid |
| `/perception/object_recognition/objects`       | `autoware_perception_msgs::msg::PredictedObjects` | Dynamic objects                                                               |
| `/tf`                                          | `tf2_msgs::msg::TFMessage`                        | TF                                                                            |
| `/tf_static`                                   | `tf2_msgs::msg::TFMessage`                        | TF static                                                                     |

### Output

| Name              | Type                                    | Description   |
| ----------------- | --------------------------------------- | ------------- |
| `/diagnostics`    | `diagnostic_msgs::msg::DiagnosticArray` | Diagnostics   |
| `~/debug_markers` | `visualization_msgs::msg::MarkerArray`  | Debug markers |

## Parameters

| Name                                  | Type                    | Description                                                                                                                                                                                                                                                                                                                                    | Default value                                                                                                                              |
| :------------------------------------ | :---------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :----------------------------------------------------------------------------------------------------------------------------------------- |
| `use_pointcloud`                      | `bool`                  | Use the obstacle grid as obstacle check                                                                                                                                                                                                                                                                                                        | `false`                                                                                                                                    |
| `use_dynamic_object`                  | `bool`                  | Use dynamic object as obstacle check                                                                                                                                                                                                                                                                                                           | `true`                                                                                                                                     |
| `obstacle_grid_timeout_sec`           | `double`                | Obstacle grid older than this is treated as unavailable (fail-safe), never as clear. [s]                                                                                                                                                                                                                                                       | 0.5                                                                                                                                        |
| `collision_distance`                  | `double`                | Distance threshold at which an object is considered a collision. [m]                                                                                                                                                                                                                                                                           | 0.15                                                                                                                                       |
| `nearby_filter_radius`                | `double`                | Distance range for filtering objects. Objects within this radius are considered. [m]                                                                                                                                                                                                                                                           | 5.0                                                                                                                                        |
| `keep_ignoring_time`                  | `double`                | Time to keep filtering objects that first appeared in the vicinity [sec]                                                                                                                                                                                                                                                                       | 10.0                                                                                                                                       |
| `nearby_object_type_filters`          | `object of bool values` | Specifies which object types to filter. Supported keys are `filter_unknown`, `filter_car`, `filter_truck`, `filter_bus`, `filter_trailer`, `filter_motorcycle`, `filter_bicycle`, `filter_pedestrian`, `filter_animal`, `filter_hazard`, `filter_over_drivable`, and `filter_under_drivable`. Only objects with `true` value will be filtered. | `{filter_unknown: true, filter_animal: true, filter_hazard: true, filter_over_drivable: true, filter_under_drivable: true, others: false}` |
| `ignore_behind_rear_axle`             | `bool`                  | If true, collisions detected behind the rear axle of the ego vehicle are ignored                                                                                                                                                                                                                                                               | `true`                                                                                                                                     |
| `time_buffer.on`                      | `double`                | [s] minimum consecutive detection time before triggering the ERROR diagnostic                                                                                                                                                                                                                                                                  | 0.2                                                                                                                                        |
| `time_buffer.off`                     | `double`                | [s] minimum consecutive time without collision detection (including the hysteresis) before releasing the ERROR diagnostic                                                                                                                                                                                                                      | 5.0                                                                                                                                        |
| `time_buffer.off_distance_hysteresis` | `double`                | [m] extra distance used to detect collisions once the diagnostic is triggered                                                                                                                                                                                                                                                                  | 1.0                                                                                                                                        |

## Assumptions / Known limits

- This module is based on `surround_obstacle_checker`
