# Algorithms

## 1. Pseudo-Scan Accumulation (single-point → point cloud)

The TFmini-S returns one range per frame. To use point-cloud SLAM, readings are
transformed to world coordinates using the current heading/position estimate and
accumulated into a `scan_buffer`. When the buffer reaches `buffer_size` points it
is emitted as a *pseudo-scan* for registration. The buffer is hard-capped at
`2 × buffer_size` to bound memory if processing stalls.

## 2. ICP Registration (SLAM)

`app/slam_engine.py` uses Open3D point-to-plane ICP:

1. Downsample source + target to `voxel_size` (voxel grid).
2. Estimate normals (hybrid KD-tree search).
3. Run `registration_icp` with `max_correspondence_distance`,
   `max_iterations`, and convergence on relative fitness/RMSE (1e-6).
4. Accept if `result.fitness > icp_threshold`.

**Motion pre-check:** if the current scan centroid has moved less than
`motion_threshold` from the reference, ICP is skipped and identity is reused —
this avoids re-registering near-duplicate stationary scans.

**Accumulation cap:** the global cloud is voxel-downsampled when it exceeds
500k points to keep memory bounded.

Reference: P. Besl and N. McKay, *A Method for Registration of 3-D Shapes*,
IEEE PAMI, 1992.

## 3. Data Quality Filtering

`app/data_quality.py` rejects unreliable readings **before** SLAM:

1. **Physical bounds:** `min_range ≤ d ≤ max_range`, `signal ≥ signal_threshold`.
2. **Rate-of-change:** reject if `|Δd| / Δt > max_rate_m_per_s` (physically
   impossible jump).
3. **Z-score:** reject if `|d − mean| / std > z_threshold` over the sliding
   window.
4. **IQR:** reject if `d` falls outside `[Q1 − k·IQR, Q3 + k·IQR]`.

A rolling **Data Quality Score** (fraction accepted over recent decisions) acts
as a calibration-drift / environment-degradation signal.

Optional temperature-drift compensation:
`corrected = raw · (1 + coeff · (T − T_baseline))`, disabled by default.

## 4. Object Detection

`app/object_detection.py` combines two strategies:

**a. Distance pattern analysis** (`DistancePatternAnalyzer`) over a sliding
window classifies the local surface:
- `flat_surface` — low variance (std < 0.05 m)
- `edge_detected` — |Δd| > 0.3 m between consecutive readings
- `irregular_surface` — many derivative sign changes (oscillation)

Patterns map to classes (wall/pipe/rock/debris/obstacle) via rule tables with
signal-strength ranges; confidence scales inversely with distance variance.

**b. Spatial clustering** — a DBSCAN-style expansion (`_simple_cluster`) groups
accumulated points within `clustering_eps` requiring `clustering_min_samples`.
Clusters passing size gates become `obstacle` detections. Nearby detections
(<0.5 m) are merged with a moving-average confidence.

Reference: M. Ester et al., *A Density-Based Algorithm for Discovering Clusters*,
KDD, 1996.

## 5. Profile Navigation

`ProfileNavigator` compares live position/heading against recorded waypoints:
- `distance_to_waypoint` = Euclidean distance to the active waypoint
- `heading_error` = normalized angle difference (−180°..180°)
- status transitions: `on_track` → `approaching` → `reached` (advance) /
  `off_course` when heading error exceeds 2× tolerance

Waypoints are quality-gated at record time (signal/range) so a polluted profile
cannot misguide playback.

## Performance Characteristics

| Stage | Cost driver | Mitigation |
|-------|-------------|------------|
| ICP | point count, iterations | voxel downsample, motion pre-check |
| Clustering | O(n²) naive expansion | buffer cap (500), bounding pre-filter |
| Accumulated map | unbounded growth | 500k-point voxel downsample |
| Read→process | slow pipeline blocking UART | bounded queue + worker thread + frame drop |
