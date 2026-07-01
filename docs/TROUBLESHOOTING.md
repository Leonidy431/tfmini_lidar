# Troubleshooting

## LiDAR Connection

**Symptom:** `lidar.connected = false` in `/api/status`, or "Failed to connect".

- Check the device exists: `ls -l /dev/ttyUSB0`.
- Confirm permissions: the container user must be in `dialout`, or the host
  user for local runs (`sudo usermod -aG dialout $USER`, then re-login).
- Verify baud rate is 115200 (TFmini-S default). A wrong rate yields checksum
  errors — watch `lidar.total_errors` climbing while `total_readings` stays 0.
- Swap TX/RX if wired backwards (see README wiring table).
- The driver auto-reconnects with exponential backoff; `reconnect_attempts` in
  status shows progress. If it never recovers, the device path is wrong or the
  cable is dead.

**Symptom:** readings arrive but `data_quality.quality_score` is low.

- Raise `signal_threshold` for turbid water.
- Check `data_quality.rejected_by` to see which filter dominates
  (`signal`, `range`, `iqr`, `zscore`, `rate`).

## SLAM Quality

**Symptom:** `slam.total_scans` not increasing during mapping.

- ICP registration is failing. Lower `slam.icp_threshold` (e.g. 0.4) or raise
  `max_correspondence_distance`.
- Feature-poor environments (flat, empty water) give ICP nothing to lock onto —
  this is a fundamental limitation of single-point LiDAR SLAM.

**Symptom:** map looks smeared / drifting.

- `slam.drift_estimate` shows per-scan displacement; large values indicate poor
  registration. Reduce vehicle speed or increase reading `frequency`.

**Symptom:** high CPU during mapping.

- Raise `motion_threshold` so stationary scans skip ICP.
- Increase `voxel_size` to downsample more aggressively.

## Localization

**Symptom:** position jumps or "lost" state.

- Ensure the loaded map matches the current environment (check map metadata
  timestamp — a stale map won't align).
- Raise `map_matching_distance` if the initial guess is far off.
- `localization.current_confidence` below `confidence_threshold` means matches
  are being rejected — the map may be too sparse.

## Pipeline / Performance

**Symptom:** `pipeline.dropped_frames` increasing.

- The processing worker can't keep up with the sensor rate. Lower LiDAR
  `frequency`, increase `voxel_size`, or raise `motion_threshold`.
- Sustained saturation with a fast sensor is expected to drop frames by design
  (the read thread is never blocked).

## Web UI

**Symptom:** control buttons return 401.

- Set the API token on the dashboard (**API Token** field) or via
  `LIDAR_API_TOKEN`.

**Symptom:** WebSocket keeps disconnecting.

- Check `CORS_ORIGINS` includes the origin you're browsing from.
- If `REQUIRE_WS_AUTH=true`, the browser must have a valid token stored (set it
  on the dashboard first).

## Container

**Symptom:** healthcheck failing / container restarting.

- `curl http://localhost:5000/api/health` inside the container. A `503` means
  the sensor is failed while the app expects it running — check the LiDAR.
- Confirm `curl` is installed (it is, in the runtime image).

## Diagnostics quick reference

```bash
# Full status snapshot
curl -s localhost:5000/api/status | python -m json.tool

# Health only
curl -s localhost:5000/api/health
```

Interpretation:
- `health.state`: `healthy` / `degraded` / `failed`
- `lidar.error_rate`: errors per reading; > 0.1 signals a wiring/baud problem
- `lidar.average_strength`: low values suggest weak returns / dirty optics
