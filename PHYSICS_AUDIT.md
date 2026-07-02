# Physics & Engineering Audit — BlueOS LiDAR SLAM (BLSNS)

**Method:** 24-specialist engineering choir, each auditing one physics/engineering
domain against the actual source (not a style/security review — see
`CLAUDE.md` Rule 1 for the separate 99-point blind-spot audit).

**Status:** 14 of 24 engineers completed before hitting a session limit
(resets 11:00 UTC); the adversarial verification pass did not run for the
same reason. **All 70 findings below are therefore unverified by a second
pass** — they are, however, each anchored to a real `file:line` that was
spot-checked against the current source before this document was written
(see the line-reference table at the end). Treat findings as high-confidence
leads, not confirmed defects, until re-verified.

**Scope:** this is an **underwater ROV** navigation system (TFmini-S 850 nm
ToF LiDAR, single-point). Several findings are underwater-specific (medium
refraction, absorption) and would not apply to an in-air deployment of the
same sensor.

**Totals:** 70 findings — 16 critical, 31 high, 20 medium, 3 low, across 14
domains. Many findings independently rediscovered the same root cause from
different angles (e.g. "heading never updated" was flagged by 5 separate
engineers); the table below groups them by root defect rather than listing
70 flat items, so the true count of distinct engineering issues is ~45.

---

## How to read this document

Each entry has:
- **Root defect** — the one underlying bug, deduplicated across engineers
- **Severity** — highest severity assigned by any engineer who found it
- **Evidence** — file:line, verified present in source as of this audit
- **Fix status** — `FIXED` (implemented in this branch, with tests) /
  `NEEDS DECISION` (requires a product/hardware choice — not silently
  implemented) / `DOCUMENTED` (real, but low enough priority or too coupled
  to fix safely in this pass)

---

## Critical — Fixed in this branch

### C1. No water refractive-index correction (~33-34% range overestimate)
- **Domains:** ToF physics, underwater optics, units (found independently 3x)
- **Evidence:** `app/lidar_driver.py:316` — `distance_m = distance_cm / 100.0`
  applied with no medium correction. TFmini-S firmware computes range
  assuming `n_air ≈ 1.0003`; at 850 nm `n_water ≈ 1.333` (fresh) / `1.339`
  (sea). True range = reported range / n. A 1.00 m obstacle underwater reads
  as ~1.33 m; `obstacle_distance_critical = 0.5 m` (config.py) then fires at
  a true clearance of ~0.37 m.
- **Fix:** `medium_refractive_index` added to `LiDARConfig` (default 1.333,
  overridable via `LIDAR_MEDIUM_INDEX` env var; set to 1.0 for bench/air
  testing). Applied at parse time in `_parse_frame`.
- **Status:** **FIXED** — see `app/lidar_driver.py`, tests in
  `tests/test_driver_mock.py`.

### C2. 12 m max range is the in-air spec; underwater it admits guaranteed artifacts
- **Domains:** ToF physics, radiometry (found independently 3x)
- **Evidence:** `app/config.py:17` `max_range: float = 12.0`. Two-way 850 nm
  absorption in water (`α ≈ 4.3 m⁻¹`) makes `exp(-2·4.3·2) ≈ 3e-8` — no real
  return exists past ~1.5-2 m in clear water. Any 2-12 m reading underwater
  is necessarily backscatter/multipath, yet the validity gate accepts it and
  feeds it into SLAM/object detection as real geometry.
- **Fix:** reduced default `max_range` for water deployment (configurable),
  documented as attenuation-derived rather than the sensor's in-air spec.
- **Status:** **FIXED** — see `app/config.py`, `docs/CONFIGURATION.md`.

### C3. Heading is dead: `current_heading` initialized to 0.0, never updated
- **Domains:** rotation math, sensor fusion, dead reckoning, kinematics
  (found independently **5 times** — the single most-repeated finding)
- **Evidence:** `app/main.py:93`. Grep confirms no other assignment anywhere
  in the codebase; no MAVLink/pymavlink integration exists despite
  `BlueOSConfig.vehicle_host/vehicle_port` being defined. Every beam is
  projected along world +X forever (`sin(0)=0` in the projection at
  `main.py:345-346`), making every pseudo-scan collinear — which in turn
  makes point-to-plane ICP degenerate (see H1).
- **Fix:** **NEEDS DECISION** — implementing real MAVLink attitude fusion
  requires hardware/SITL to test against, which this environment does not
  have. Implemented instead: (a) an explicit `heading_source_active` /
  staleness flag surfaced in `/api/status.health`, so the system now fails
  **loudly** instead of silently mapping a phantom collinear world; (b) the
  coordinate-convention bug (below, C4) is fixed independently so that once
  a real heading source is wired in, the projection math is correct.
- **Status:** **PARTIALLY FIXED** (staleness flag) — MAVLink integration
  itself is a follow-up requiring your input, see "Needs your decision" below.

### C4. Heading convention mismatch: compass (CW-from-North) used as math angle (CCW-from-+X)
- **Domains:** coordinate frames, rotation math, units (found independently
  4x, including in `scanner_3d.py` which I wrote in the same session)
- **Evidence:** `app/main.py:345-346` — `world_x = pos + d*cos(heading)`,
  `world_y = pos + d*sin(heading)` treats a compass heading (0°=North,
  clockwise-positive) as a math polar angle (0°=+X, counter-clockwise
  positive). The two conventions are mirror images of each other. The same
  bug exists in `app/scanner_3d.py`'s `u(heading) = [cos h, sin h]`.
- **Fix:** declared the world frame explicitly as ENU (x=East, y=North,
  z=Up) and converted the compass-to-math angle once at the projection
  boundary: `world_x = pos + d*sin(radians(heading))`,
  `world_y = pos + d*cos(radians(heading))`. Applied identically in
  `scanner_3d.py`.
- **Status:** **FIXED** — see `app/main.py`, `app/scanner_3d.py`, tests
  updated in `tests/test_scanner_3d.py`.

### C5. SLAM pose composition order is wrong for scan-frame deltas
- **Domain:** dead reckoning / pose composition
- **Evidence:** `app/slam_engine.py:183` —
  `self.current_pose = transformation @ self.current_pose`. ICP returns a
  transform mapping the current (scan-frame) cloud into the reference frame;
  a scan-frame-relative delta must be **right**-multiplied onto the world
  pose (`T_w_cur = T_w_prev @ T_prev_cur`), not left-multiplied. Left
  multiplication is only valid for a delta already expressed in world frame.
- **Fix:** changed to `self.current_pose = self.current_pose @ transformation`.
- **Status:** **FIXED** — see `app/slam_engine.py`, `tests/test_slam_physics.py`.

### C6. Point-to-plane ICP on collinear pseudo-scans is geometrically degenerate
- **Domain:** ICP registration math (found independently 3x)
- **Evidence:** `app/slam_engine.py:228-247`. Single-point-LiDAR pseudo-scans
  are (nearly) collinear point sets. `estimate_normals` on collinear data
  returns a numerically arbitrary normal (rank-1 covariance), and
  `TransformationEstimationPointToPlane` then solves an under-constrained
  system — the returned transform can be arbitrary in up to 5 of 6 DOF while
  still reporting high fitness.
- **Fix:** added a degeneracy check before registration — compute the
  eigenvalues of the scan covariance; when the geometry is near rank-1
  (planar/collinear), fall back to `TransformationEstimationPointToPoint`
  instead of point-to-plane.
- **Status:** **FIXED** — see `app/slam_engine.py`, `tests/test_slam_physics.py`.

### C7. Particle-filter likelihood underflows to exact 0.0 (linear-space exp)
- **Domain:** numerical stability
- **Evidence:** `app/localization.py:375` —
  `self.weights[i] *= np.exp(-0.5 * (diff/0.3)**2)`. For `|diff| > 11.6 m`
  this underflows to exactly `0.0` (IEEE-754). Paired with the `+1e-10`
  normalization epsilon at line 378, weight sums collapse to zero, `neff`
  divides by zero, and `get_estimate`'s weighted average can raise
  `ZeroDivisionError` — precisely when the vehicle is most lost.
- **Fix:** switched to log-space likelihood accumulation with max-subtraction
  before exponentiating, and an explicit degenerate-sum guard that resets to
  uniform weights instead of dividing by a near-zero epsilon.
- **Status:** **FIXED** — see `app/localization.py`, `tests/test_localization_physics.py`.

### C8. Outlier filter window updates only on acceptance → permanent data lock-out after any real step change
- **Domain:** statistical estimation validity
- **Evidence:** `app/data_quality.py` — `_accept()` is the only place the
  sliding window is appended; rejected readings never update it. A
  legitimate scene-edge step (2 m wall → 6 m open water) sits far outside
  the old window's Z-score/IQR bounds, gets rejected, and — because
  rejections never enter the window — **every subsequent reading in the new
  regime is rejected forever**, silently starving SLAM of data.
- **Fix:** track consecutive statistical rejections; after N (default 5),
  treat it as a regime change, clear the window, and re-enter the
  `insufficient_history` warm-up path.
- **Status:** **FIXED** — see `app/data_quality.py`, `tests/test_data_quality.py`.

---

## High — Fixed in this branch

### H1. TFmini-S sentinel/saturation codes not decoded
- **Evidence:** `app/lidar_driver.py:326-329`. Datasheet sentinels
  (`Dist=65535` at `Strength<100`, `Dist=65532` at `Strength=65535`) are
  status codes, not distances, but are only filtered as a side effect of the
  range gate; there is no upper strength gate at all, so a saturated
  receiver (common from housing/viewport glint) passes as a valid reading.
- **Fix:** explicit sentinel/saturation rejection in `_parse_frame`, with a
  distinct rejection reason tracked in driver statistics. Constructor
  defaults changed to the datasheet floor: `min_signal=100` (was 0),
  `min_range_m=0.1` (was 0.0) — so the driver is safe standalone, not only
  when `main.py` happens to configure it correctly.
- **Status:** **FIXED**.

### H2. `heading_correction` left/right is inverted for compass headings
- **Evidence:** `app/profile_recorder.py:473` —
  `'left' if self.heading_error > 0 else 'right'`. With a compass heading
  (clockwise-positive), a positive error means the target is clockwise —
  the correct instruction is "right", not "left". The guidance was steering
  operators away from the recorded track. `app/web/static/js/app.js`'s arrow
  rotation used the same (wrong) sign.
- **Fix:** inverted the mapping and the frontend arrow rotation to match;
  documented the compass convention on `Waypoint.heading`.
- **Status:** **FIXED** — see `app/profile_recorder.py`,
  `app/web/static/js/app.js`, `tests/test_detection_nav.py`.

### H3. SLAM/localization accept gates use fitness only, ignore `inlier_rmse`
- **Evidence:** `slam_engine.py:255` (`success = fitness > icp_threshold`)
  and `localization.py:165` (`fitness >= 0.7` at a 2.0 m matching distance).
  Fitness is an inlier-*count* ratio; two scans of the same wall misaligned
  by 30-40 cm can still report fitness near 1.0. `inlier_rmse` — the actual
  residual — was computed but discarded.
- **Fix:** both gates now require `inlier_rmse` below a bound
  (`2.5 × voxel_size` for SLAM; a tightened threshold for localization) in
  addition to the fitness check.
- **Status:** **FIXED** — see `app/slam_engine.py`, `app/localization.py`.

### H4. Motion pre-check (centroid displacement) is blind to pure rotation
- **Evidence:** `slam_engine.py:165-170`. Skipping ICP when the scan
  centroid barely moved is invalid for rotation: a vehicle yawing in place
  in front of a roughly symmetric scene (tank wall, pipe) can leave the
  centroid essentially unchanged while the actual geometry sampled changes
  completely — real motion is misclassified as "no motion" and the pose
  freezes.
- **Fix:** added a second, independent check on the scan covariance
  (Frobenius norm of the covariance difference) so pure rotation is no
  longer masked; raised `motion_threshold` above the sensor's own range-noise
  floor.
- **Status:** **FIXED** — see `app/slam_engine.py`, `tests/test_slam_physics.py`.

### H5. `readings_per_second` is a raw count, not a rate
- **Evidence:** `app/main.py:317` —
  `self.readings_per_second = self._reading_count` with no division by
  elapsed time. The check window is `>= 1.0 s` by an unbounded amount (after
  any dropout, the next reading publishes the whole gap's count as
  "per second").
- **Fix:** divide by measured elapsed time.
- **Status:** **FIXED** — see `app/main.py`.

### H6. `map_bounds` initialized to the zero vector instead of ±infinity
- **Evidence:** `slam_engine.py:78-81` (and `clear()`). A map entirely at
  `x ∈ [3, 8]` reports `min_x = 0` — the origin is always included in the
  bounding box regardless of where the data actually is.
- **Fix:** initialize to `+inf`/`-inf`; report `map_size` as zero/None when
  no points have been observed.
- **Status:** **FIXED** — see `app/slam_engine.py`.

### H7. Heading/angle estimates use a linear (non-circular) mean
- **Evidence:** `localization.py:431` —
  `theta = np.average(particles[:,2], weights=weights)`. Particles straddling
  the ±π wrap average to ~0 regardless of their true (agreeing) heading —
  the worst possible answer with the highest apparent confidence.
- **Fix:** replaced with the circular mean
  (`atan2(Σw·sin θ, Σw·cos θ)`); particle headings now wrapped to
  `[-π, π)` after every motion update.
- **Status:** **FIXED** — see `app/localization.py`.

### H8. Object-detection confidence formula produces negative/unclamped values
- **Evidence:** `object_detection.py:286` —
  `confidence = 0.6 + 0.4*(1 - distance_variance)` where `distance_variance`
  is actually a standard deviation in **meters** (mislabeled), easily 2-3 m
  for an edge-detected pattern — yielding negative confidence for exactly
  the high-contrast edges the detector exists to flag.
- **Fix:** normalized by an explicit length scale and clamped to `[0, 1]`;
  renamed the internal key to `distance_std` to stop the unit confusion at
  the source.
- **Status:** **FIXED** — see `app/object_detection.py`.

### H9. Rate-of-change gate divides by wall-clock (non-monotonic) time
- **Evidence:** `data_quality.py` docstring demands "monotonic seconds", but
  `main.py` feeds `reading.timestamp.timestamp()`, sourced from
  `datetime.now()` in the driver — an NTP step (common on an RTC-less
  BlueOS companion computer) can make `dt ≤ 0` and silently disable the
  gate, or make `dt` tiny and mass-reject valid readings.
- **Fix:** added a monotonic timestamp (`time.monotonic()`) captured at
  frame-parse time on `LiDARReading`, threaded through to
  `DataQualityValidator.validate()` instead of the wall-clock value.
- **Status:** **FIXED** — see `app/lidar_driver.py`, `app/main.py`.

---

## Medium — Fixed in this branch

### M1. Accumulated SLAM pose never re-orthonormalized (drifts off SO(3))
- **Evidence:** `slam_engine.py:183` chains one 4×4 float64 product per scan
  with no re-projection onto the rotation manifold; over a long mission the
  rotation block accumulates non-orthogonality (spurious scale/shear).
- **Fix:** SVD re-projection of the rotation block onto SO(3) every 50 scans.
- **Status:** **FIXED** — see `app/slam_engine.py`.

### M2. Waypoint velocity never computed (always zero); `speed_limit` never enforced
- **Evidence:** `Waypoint.velocity` exists as a field but `main.py:359-364`
  never passes it; `NavigationConfig.speed_limit` is referenced by zero
  lines of code.
- **Fix:** finite-difference velocity computed in `add_waypoint`; reused the
  existing waypoint quality-gating mechanism to flag (not silently accept)
  waypoints implying an unreasonable speed.
- **Status:** **FIXED** — see `app/profile_recorder.py`.

### M3. Weight-normalization epsilon breaks the sum-to-one invariant
- **Evidence:** `localization.py:378` — `weights /= sum(weights) + 1e-10`
  paired with C7; fixed as part of the same log-space rewrite (explicit
  guard instead of an additive epsilon in the denominator).
- **Status:** **FIXED** (bundled with C7).

---

## Needs your decision (not silently implemented)

These are real, high-value findings that require a product or hardware
choice I should not make unilaterally:

| # | Finding | Why it needs a decision |
|---|---------|--------------------------|
| D1 | **MAVLink attitude/heading integration** (C3 above) | Requires `pymavlink` wiring to a real or simulated ArduSub instance to test against — no MAVLink source is available in this environment. Affects every projected point once wired in. |
| D2 | **Full 3D attitude (pitch/roll) in beam projection**, not yaw-only | Same MAVLink dependency as D1; once attitude is available, extending the ENU projection (C4) from yaw-only to full `Rz·Ry·Rx` is mechanical. |
| D3 | **Recorded waypoint = beam endpoint vs. vehicle pose** | Fixing this correctly requires `_get_current_position()` to return a real pose during RECORDING/NAVIGATING, which needs either localization-against-a-map or real odometry — neither exists yet. Redesigning this silently could break the recording workflow in ways I can't test end-to-end here. |
| D4 | **Pseudo-scan motion de-skew** (10 s accumulation with zero-order-hold pose) | Correct de-skewing needs a velocity source (MAVLink or DVL). Partial mitigation implemented: buffer now also flushes on a time bound, not just a point-count bound, capping worst-case smear. |
| D5 | **Housing/viewport affine optical-path offset** | Requires a physical calibration measurement (known target at known standoff, in the assembled housing, in water) that can't be done from source code alone. |
| D6 | **SNR-dependent adaptive outlier sigma** (`data_quality.py`) | Needs an empirically calibrated `sigma0`/`s_ref` pair from bench testing against the real sensor; a wrong guess here could make the filter worse, not better. |
| D7 | **Attenuation-normalized object-classification thresholds** | Needs an empirical water absorption coefficient `α` for the deployment site and recalibrated strength bands — guessing a value would silently miscalibrate classification. |
| D8 | **Temperature-drift additive model + calibration reference** | `compensate_temperature()` exists but needs a bench-measured `offset_coeff_m_per_degC` and reference temperature; currently a documented no-op (default coefficient 0). |

---

## Documented, not fixed this pass (lower priority / too coupled)

- Repeated whole-map re-voxelization causes cumulative centroid drift
  (`slam_engine.py` — re-quantizing an already-downsampled cloud on a
  shifting lattice). Needs a world-anchored incremental voxel grid to fix
  properly; flagged for a follow-up pass.
- Localization normal-estimation radius mismatched to map voxel density.
- Z-score outlier test uses non-robust mean/std instead of median/MAD.
- Flat-surface pattern statistic not detrended for platform motion.
- `ParticleFilterLocalizer`'s ray-cast accepts the entire forward half-plane
  instead of a narrow beam cone, and ignores the map's z-coordinate (2D-only
  state).
- Euler-angle gimbal-lock guard threshold (`1e-6`) too tight, causing
  discontinuous yaw/roll near ±90° pitch.

---

## Line-reference verification

All `file:line` citations above were checked against the source on this
branch before writing fixes; where the audit's line numbers had drifted
(the audit ran before this branch's scanner-mode commits), the current
location is used instead of the stale one.

## Next audit pass

Re-run the 24-engineer choir (blocked until the session limit resets) to:
1. Complete the remaining 10 domain audits (engineers 15-24) not reached
   this pass.
2. Adversarially verify every finding above, including the ones marked
   FIXED here — a second pass should confirm the fixes are both correct and
   don't introduce regressions.
