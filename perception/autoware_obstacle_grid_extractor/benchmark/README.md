# Obstacle grid — size/coverage & latency validation (S3)

The producer cost is `O(N)` over the input points, so it is never the bottleneck on the last-resort
path; there is no "producer must not exceed summed consumer cost" gate. What is validated instead:

- **Geometry** — the published grid has the configured ROI extent, 0.2 m resolution, and the three
  layers (`max_height`, `min_height`, `point_count`) in `base_link`.
- **Coverage** — every input point inside the ROI and z-band maps to an occupied (non-`NaN`
  `point_count`) cell: no points are silently dropped.
- **Latency** — informational P99 / max, recorded as gtest properties (not a hard gate).

`test/test_grid_coverage.cpp` is the CI-resident gate (synthetic dense clouds). The full real-vehicle
validation runs the **same `ObstacleGridExtractor`** over recorded obstacle
frames:

| quantity                                                              | result (real sensor data, 890 frames)                  |
| --------------------------------------------------------------------- | ------------------------------------------------------ |
| producer `extract()` per ~12 k-point real cloud                       | 0.14–0.27 ms @0.2 m                                    |
| producer `extract()` worst-case ~200 k-point cloud (synthetic)        | ~2.1 ms (≪ 15 ms budget)                               |
| existing sensing producer baseline (scan_ground_filter + concatenate) | ~32 ms/frame                                           |
| coverage (in-ROI point → occupied cell)                               | ~1.000                                                 |
| grid-vs-raw nearest distance                                          | conservative on 100 % of frames; error ≤ 0.26 m @0.2 m |

See the Milestone-S evaluation for the full methodology, the consumer behavior-equivalence numbers, and
how to regenerate the figures.
