# Rear Collision Checker

The `rear_collision_checker` is a plugin module of the `autoware_planning_validator` node. It validates the planned trajectory by verifying that it does **not** lead to a collision with other road users (primarily vehicles, but also pedestrian/cyclists/motorcycles) approaching from lateral/rear directions. In particular, it is designed to detect potential rear-end collisions and entrapment risks when the ego vehicle is making a right/left turn or merging into an adjacent lane.

## Inner Workings

The module operates by:

1. **Identifying conflict lanes**
   - Uses the current route and lanelet topology to determine lanes that the ego vehicle’s trajectory may intersect with during right/left turns or lane changes.
   - Focuses on lanes where vehicles, bicycles, or motorcycles could approach from behind or from a lateral direction, posing a potential rear-end or entrapment risk.
   - Considers the `turn_direction` of the approaching lanelets to filter out irrelevant lanes.

2. **Filtering perception data**
   - Reads the shared 2.5D obstacle grid (`grid_map_msgs/GridMap`, base_link, layers `point_count` / `min_height` / `low_max_height`) that the parent `autoware_planning_validator` node provides on `~/input/obstacle_grid`. There is no dedicated subscription in this plugin.
   - Applies a per-cell density and z-band gate (see the Obstacle grid Parameters below), emits the qualifying cells' corner points and transforms them into the map frame.
   - Filters those points by the conflict region and computes the **nearest face of each obstacle along the lane direction** (the previous Euclidean clustering and convex-hull step is removed; the grid is already density-aggregated).
   - Applies configurable range gates (forward/backward, lateral, height) and basic outlier rejection to discard distant or irrelevant points.

3. **Estimating motion**
   - For each selected object, determines its motion relative to the lane direction.
   - Estimates speed and direction based on frame-to-frame position changes along the lane’s centerline.
   - Maintains a per-lane tracking list to stabilize velocity estimation and reduce noise from perception flicker.

4. **Calculating metrics**
   - **The calculation method and decision logic vary based on the selected collision assessment metric (e.g., Time-to-Collision).** The module calculates the chosen metric and uses the result to determine if a collision risk exists.

5. **Judging collision risk**
   - Evaluates the selected metric results against predefined thresholds to determine whether a collision is likely.
   - Applies hysteresis logic:
     - **`on_time_buffer`**: Hazardous state must persist for this duration before marking the trajectory unsafe.
     - **`off_time_buffer`**: Safe state must persist for this duration before clearing the unsafe flag.
   - If `check_on_unstoppable=false`, the module will skip collision warnings in cases where the ego vehicle cannot realistically stop before the conflict area (to prevent unnecessary alerts when stopping is already infeasible).

### Flowchart

WIP

## General Parameters

| Name                   | Unit | Type   | Description                                                                                                                                                                                                                                                                 | Default value |
| :--------------------- | ---- | ------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------- |
| `on_time_buffer`       | [s]  | double | Time buffer before enabling detection after a relevant condition is met                                                                                                                                                                                                     | 0.5           |
| `off_time_buffer`      | [s]  | double | Time buffer before disabling detection after the condition clears                                                                                                                                                                                                           | 1.5           |
| `check_on_unstoppable` | [-]  | bool   | If `true`, the module continues collision checking even when the ego vehicle cannot stop before entering a potential collision area, and outputs an **ERROR** if a collision risk is detected. If `false`, checking is skipped in such cases to avoid unnecessary warnings. | false         |

### Obstacle grid Parameters

The data source is the 2.5D obstacle grid (`grid_map_msgs/GridMap`, base_link, layers `point_count` / `min_height` / `low_max_height`). The parameter group key is kept as `pointcloud` to minimize downstream churn.

!!! warning "Producer ROI must cover the rear reach"

    This checker queries obstacles far behind ego (its backward RSS reach for a fast adjacent-lane participant is on the order of tens of metres). The stock obstacle-grid extractor is forward-biased, so its rear coverage can be shorter than that reach. When the received grid's rear extent is smaller than the required backward distance, cells beyond the grid are invisible and a throttled **ERROR** is logged naming both numbers. The launch integration is responsible for rebiasing/widening the producer ROI (e.g. `x` in `[-100, +50]`) so the grid covers the rear reach; see the campaign's launch step.

!!! note "Data-unavailability is an abstain, not a veto"

    When the obstacle grid is absent, stale (older than `obstacle_grid_timeout_sec`), in the wrong frame, undecodable, missing a required layer, or the `map <- base_link` transform for the grid stamp is unavailable, the module cannot run and **abstains**: `validate()` reports the rear-collision check as valid (no STOP planning factor is published) and a throttled **ERROR** is logged for observability. This is deliberately the same behaviour as the parent's other data-unavailability paths and is an availability signal only — it does not by itself hold or stop the ego. It is distinct from a successfully decoded grid that simply contains no qualifying cell, which is a genuine "clear". Downstream logic that must react to a rear-sensor outage should consume the ERROR/diagnostic rather than the boolean check result.

The height gate mirrors the AEB pattern: the floor `z_floor` is applied to the cell's tallest in-band return (`low_max_height`), and the ceiling `vehicle_height + z_band_top_offset` to the cell's lowest return (`min_height`). Because the floor is on `low_max_height` rather than `max_height`, an overhead structure that merely shares a cell with ground residue is rejected. The legacy per-cluster `min_cluster_height` (0.1 m) is subsumed by `z_floor` (0.3 m) and needs no separate parameter.

| Name                                              | Unit    | Type   | Description                                                                                                          | Default value |
| :------------------------------------------------ | ------- | ------ | -------------------------------------------------------------------------------------------------------------------- | ------------- |
| `pointcloud.range.dead_zone`                      | [m]     | double | Distance in front of the ego vehicle ignored for collision detection                                                 | 0.3           |
| `pointcloud.range.buffer`                         | [m]     | double | Additional margin around detection range                                                                             | 1.0           |
| `pointcloud.grid.min_point_count_cell`            | [-]     | int    | Per-cell raw (pre-voxel) return floor; a cell must hold at least this many returns. NOT the old cluster `min_size=5` | 1             |
| `pointcloud.grid.z_floor`                         | [m]     | double | Height floor applied to the cell's tallest in-band return (`low_max_height`); legacy `crop_box_filter.z.min`         | 0.3           |
| `pointcloud.grid.z_band_top_offset`               | [m]     | double | Offset added to the ego vehicle height to form the `min_height` band ceiling; legacy `crop_box_filter.z.max`         | -1.0          |
| `pointcloud.obstacle_grid_timeout_sec`            | [s]     | double | Staleness watchdog; a grid older than this reads as data-unavailable and the check abstains (see the note below)     | 0.5           |
| `pointcloud.velocity_estimation.observation_time` | [s]     | double | Time window used for velocity estimation                                                                             | 0.3           |
| `pointcloud.velocity_estimation.max_acceleration` | [m/s^2] | double | Maximum allowed acceleration in velocity estimation                                                                  | 10.0          |
| `pointcloud.latency`                              | [s]     | double | Assumed system latency for obstacle grid processing                                                                  | 0.3           |

### Object Filtering

| Name                  | Unit  | Type   | Description                                                             | Default value |
| :-------------------- | ----- | ------ | ----------------------------------------------------------------------- | ------------- |
| `filter.min_velocity` | [m/s] | double | Minimum velocity threshold for objects to be considered moving          | 1.0           |
| `filter.moving_time`  | [s]   | double | Minimum time duration an object must be moving to be considered as such | 0.5           |

### TTC

| Name                       | Unit | Type   | Description                                              | Default value |
| :------------------------- | ---- | ------ | -------------------------------------------------------- | ------------- |
| `time_to_collision.margin` | [s]  | double | Additional margin added to time-to-collision calculation | 2.0           |

### Ego Behavior

| Name                        | Unit    | Type   | Description                                                         | Default value |
| :-------------------------- | ------- | ------ | ------------------------------------------------------------------- | ------------- |
| `ego.reaction_time`         | [s]     | double | Reaction time of the ego vehicle                                    | 1.2           |
| `ego.min_velocity`          | [m/s]   | double | Minimum considered velocity of the ego vehicle                      | 1.38          |
| `ego.max_velocity`          | [m/s]   | double | Maximum considered velocity of the ego vehicle                      | 16.6          |
| `ego.max_acceleration`      | [m/s^2] | double | Maximum considered acceleration of the ego vehicle                  | 1.5           |
| `ego.max_deceleration`      | [m/s^2] | double | Maximum considered deceleration of the ego vehicle (negative value) | -4.0          |
| `ego.max_positive_jerk`     | [m/s^3] | double | Maximum allowed positive jerk for the ego vehicle                   | 5.0           |
| `ego.max_negative_jerk`     | [m/s^3] | double | Maximum allowed negative jerk for the ego vehicle                   | -5.0          |
| `ego.nominal_deceleration`  | [m/s^2] | double | Nominal deceleration used for calculations                          | -1.5          |
| `ego.nominal_positive_jerk` | [m/s^3] | double | Nominal positive jerk used for calculations                         | 0.6           |
| `ego.nominal_negative_jerk` | [m/s^3] | double | Nominal negative jerk used for calculations                         | -0.6          |

### Collision Check For Blind Spot

| Name                                       | Unit    | Type   | Description                                                  | Default value |
| :----------------------------------------- | ------- | ------ | ------------------------------------------------------------ | ------------- |
| `blind_spot.lookahead_time`                | [s]     | double | Lookahead time for blind spot detection                      | 4.0           |
| `blind_spot.metric`                        | [-]     | string | Metric used for blind spot detection (`ttc`, etc.)           | ttc           |
| `blind_spot.check.front`                   | [-]     | bool   | Whether to check for blind spot in the front                 | false         |
| `blind_spot.check.left`                    | [-]     | bool   | Whether to check for blind spot on the left                  | true          |
| `blind_spot.check.right`                   | [-]     | bool   | Whether to check for blind spot on the right                 | false         |
| `blind_spot.check.yaw_th`                  | [rad]   | double | Yaw threshold for blind spot detection                       | 0.78          |
| `blind_spot.offset.inner`                  | [m]     | double | Inner offset for blind spot detection zone                   | 0.1           |
| `blind_spot.offset.outer`                  | [m]     | double | Outer offset for blind spot detection zone                   | 0.3           |
| `blind_spot.participants.reaction_time`    | [s]     | double | Reaction time of participants in blind spot detection        | 1.2           |
| `blind_spot.participants.max_velocity`     | [m/s]   | double | Maximum velocity of participants in blind spot detection     | 5.5           |
| `blind_spot.participants.max_deceleration` | [m/s^2] | double | Maximum deceleration of participants in blind spot detection | -2.0          |

### Collision Check For Adjacent Lane

| Name                                          | Unit    | Type   | Description                                                               | Default value |
| :-------------------------------------------- | ------- | ------ | ------------------------------------------------------------------------- | ------------- |
| `adjacent_lane.lookahead_time`                | [s]     | double | Lookahead time for adjacent lane collision detection                      | 4.0           |
| `adjacent_lane.metric`                        | [-]     | string | Metric used for adjacent lane collision detection (`rss`, etc.)           | rss           |
| `adjacent_lane.check.front`                   | [-]     | bool   | Whether to check for adjacent lane collision in the front                 | true          |
| `adjacent_lane.offset.left`                   | [m]     | double | Left offset for adjacent lane collision detection                         | -0.5          |
| `adjacent_lane.offset.right`                  | [m]     | double | Right offset for adjacent lane collision detection                        | -0.5          |
| `adjacent_lane.participants.reaction_time`    | [s]     | double | Reaction time of participants in adjacent lane collision detection        | 1.2           |
| `adjacent_lane.participants.max_velocity`     | [m/s]   | double | Maximum velocity of participants in adjacent lane collision detection     | 16.6          |
| `adjacent_lane.participants.max_deceleration` | [m/s^2] | double | Maximum deceleration of participants in adjacent lane collision detection | -2.0          |
