# Configuration Reference

All configuration lives in [`app/config.py`](../app/config.py) as dataclasses,
with environment-variable overrides for deployment-specific values.

## Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `DEBUG` | `false` | Enables Flask debug + `allow_unsafe_werkzeug`. **Never** enable in production. |
| `DATA_DIR` | `/app/data` | Root for `maps/`, `profiles/`, `objects/`, `logs/`. |
| `WEB_HOST` | `0.0.0.0` | Bind address. |
| `WEB_PORT` | `5000` | HTTP port. |
| `LIDAR_PORT` | `/dev/ttyUSB0` | Serial device path. |
| `LIDAR_API_TOKEN` | *(auto)* | Persistent API token; auto-generated + logged if unset. |
| `CORS_ORIGINS` | `http://localhost:5000,http://127.0.0.1:5000,http://blueos.local` | Comma-separated allowed origins. |
| `REQUIRE_WS_AUTH` | `false` | Require a valid token on WebSocket connect. |

## LiDAR (`LiDARConfig`)

| Param | Default | Unit | Notes |
|-------|---------|------|-------|
| `port` | `/dev/ttyUSB0` | — | Overridden by `LIDAR_PORT`. |
| `baudrate` | `115200` | baud | TFmini-S default. |
| `timeout` | `1.0` | s | Serial read timeout. |
| `frequency` | `100` | Hz | Output rate, 1–1000. |
| `max_range` | `4.0` | m | Readings above are rejected. Overridden by `LIDAR_MAX_RANGE_M`. 12.0 m is the TFmini-S in-air spec; underwater at 850nm no real return exists past a few meters (see [`PHYSICS_AUDIT.md`](../PHYSICS_AUDIT.md) C2) — this default is a moderate-water-clarity compromise, tune to your site. |
| `min_range` | `0.1` | m | Readings below are rejected (sensor blind zone). |
| `signal_threshold` | `100` | — | Minimum signal strength to accept a frame (also the TFmini-S datasheet floor). |
| `medium_refractive_index` | `1.333` | — | ToF range assumes propagation at `c/n`; underwater at 850nm `n≈1.333` (fresh) / `1.339` (sea). Uncorrected, ranges read ~33% long. Set to `1.0` for bench/air testing via `LIDAR_MEDIUM_INDEX`. See `PHYSICS_AUDIT.md` C1. |

**Tuning:** In turbid water, raise `signal_threshold` (e.g. 150–200) to reject
weak, scattered returns. Lower `frequency` if the pipeline reports dropped
frames (see `/api/status` → `pipeline.dropped_frames`).

## SLAM (`SLAMConfig`)

| Param | Default | Unit | Notes |
|-------|---------|------|-------|
| `voxel_size` | `0.05` | m | Downsampling resolution for ICP. |
| `icp_threshold` | `0.6` | fitness | Min ICP fitness to accept a registration. |
| `max_correspondence_distance` | `0.5` | m | ICP correspondence cutoff. |
| `max_iterations` | `50` | — | ICP iteration cap. |
| `buffer_size` | `1000` | points | Points accumulated before a pseudo-scan. |
| `motion_threshold` | `0.01` | m | Below this centroid displacement, ICP is skipped (CPU saving). |

**Tuning:** If registration frequently fails (`slam.total_scans` not
increasing), lower `icp_threshold` or raise `max_correspondence_distance`. If
CPU is high on a stationary vehicle, raise `motion_threshold`.

## Navigation (`NavigationConfig`)

| Param | Default | Unit | Notes |
|-------|---------|------|-------|
| `profile_sample_rate` | `10.0` | Hz | Waypoint sampling rate. |
| `waypoint_distance_threshold` | `0.5` | m | "Reached" distance + min waypoint spacing. |
| `heading_tolerance` | `5.0` | deg | Off-course threshold is 2×. |
| `speed_limit` | `1.0` | m/s | Advisory. |
| `obstacle_distance_warning` | `2.0` | m | Warning band. |
| `obstacle_distance_critical` | `0.5` | m | Critical band. |

## Object Detection (`ObjectDetectionConfig`)

| Param | Default | Unit | Notes |
|-------|---------|------|-------|
| `enabled` | `true` | — | Master switch. |
| `min_object_size` | `0.1` | m | Cluster size floor. |
| `max_object_size` | `5.0` | m | Cluster size ceiling. |
| `detection_threshold` | `0.3` | conf | Min confidence to report. |
| `clustering_eps` | `0.2` | m | Neighborhood radius. |
| `clustering_min_samples` | `5` | — | Min points per cluster. |

## Localization (`LocalizationConfig`)

| Param | Default | Unit | Notes |
|-------|---------|------|-------|
| `enable_icp_refinement` | `true` | — | Refine pose with ICP. |
| `confidence_threshold` | `0.7` | — | Below this, matches are rejected. |
| `map_matching_distance` | `2.0` | m | Match search radius. |
| `update_frequency` | `10` | Hz | Localization update rate. |
| `lost_threshold` | `10` | — | Consecutive failures before "lost". |

## Data Quality (`DataQualityValidator`, constructed in `main.py`)

| Param | Default | Notes |
|-------|---------|-------|
| `window_size` | `50` | Sliding window for IQR/Z-score. |
| `iqr_multiplier` | `1.5` | IQR fence multiplier. |
| `z_threshold` | `3.0` | Z-score rejection threshold. |
| `max_rate_m_per_s` | `15.0` | Rate-of-change gate. |
| `temp_coefficient` | `0.0` | Temperature drift correction (0 = off). |

Signal/range values are inherited from `LiDARConfig`.
