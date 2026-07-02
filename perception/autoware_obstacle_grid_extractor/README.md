# autoware_obstacle_grid_extractor

A stateless, `base_link`, per-cloud producer that rasterizes the no-ground obstacle point cloud into a single 2.5D occupancy/height grid (`grid_map_msgs/msg/GridMap`) for the perception-independent last-resort safety path.

## Purpose

The last-resort emergency-stop consumers (AEB pointcloud branch, collision/surround/obstacle-collision checkers, planning-validator collision checkers, trajectory-modifier obstacle-stop) each used to re-implement the same crop + voxel + Euclidean cluster + convex-hull pipeline on the raw no-ground cloud. This node collapses that duplicated preprocessing into one O(N) single-pass rasterizer and gives the last-resort path logical independence from the perception modeling stack (it subscribes only to the no-ground cloud and the static sensor→`base_link` extrinsic — never to objects, tracking, occupancy, map, trajectory, or the localization pose).

## Algorithm

For each input cloud (already in / transformed to `base_link`):

1. Drop non-finite points, then points outside `[crop.z_min, crop.z_max]`.
2. Compute the single grid cell index of each remaining point (`grid_map::getIndex`).
3. Update that cell's `max_height` (max z), `min_height` (min z), `point_count` (+1), and — for points at or below `overhead_split` — `low_max_height` (max in-band z).

No KdTree, no Euclidean clustering, no convex hull. Empty cells stay `NaN`. An empty cloud yields an all-`NaN` grid with a fresh stamp — the "alive, nothing detected" heartbeat. On a TF failure nothing is published at all, so consumers must treat a stale grid stamp as "unavailable" (fail-safe), never as "clear". Each consumer re-applies its own per-cell size/height gate downstream, so no central tuning is imposed.

`low_max_height` exists because a per-cell `(min_height, max_height)` pair cannot distinguish a wall (continuous mass from the ground up) from an overhead structure sharing its cell with ground returns (a gantry above ground residue) — both give the same signature. Consumers that brake on cell height gate on `low_max_height` (the tallest return at or below `overhead_split`) so overhead-only content can never qualify a cell, while tall in-band obstacles still do.

## I/O

| Direction | Topic                                                                     | Type                      | QoS                            |
| --------- | ------------------------------------------------------------------------- | ------------------------- | ------------------------------ |
| sub       | `~/input/pointcloud` (`/perception/obstacle_segmentation/pointcloud`)     | `sensor_msgs/PointCloud2` | `SensorDataQoS` `KEEP_LAST(1)` |
| pub       | `~/output/obstacle_grid` (`/sensing/obstacle_segmentation/obstacle_grid`) | `grid_map_msgs/GridMap`   | `RELIABLE` `KEEP_LAST(1)`      |

Layers (`float32`, empty = `NaN`): `max_height`, `min_height`, `point_count`, `low_max_height`.

## Parameters

See [`config/obstacle_grid_extractor.param.yaml`](config/obstacle_grid_extractor.param.yaml) and [`schema/obstacle_grid_extractor.schema.json`](schema/obstacle_grid_extractor.schema.json). Defaults: forward-biased ROI 60×40 m at 0.2 m cell, z-band `[-1.0, 3.0]` m, `overhead_split` 2.5 m (must be at least every consumer's own z-band top).
