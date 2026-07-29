# P9 Simulation Validation Report (D1/D2/D3/D4/D8)

**Scope**: The simulation half of P9-Calibration per TECHNICAL_SPECIFICATION.md, which defines P9 as "Empirical tuning on ROV hardware **or validated simulation**". Ground truth is the physically-grounded environment model in `tests/emulation_server.py` (Beer-Lambert attenuation, n(depth, salinity, temperature), signal-strength degradation).

**Test suite**: `tests/test_p9_simulation.py` (9 tests, all passing; run `pytest tests/test_p9_simulation.py -s` to reproduce the numbers below).

**What this does and does not validate**: it validates the *calibration procedures* (do they recover a known ground-truth model?) and the *end-to-end error-reduction claims* (does the corrected pipeline beat the uncorrected one, by how much?). It does **not** validate the real ocean — field P9 on hardware (IMU, turbidity tank, depth pool, oven) remains open and is tracked in `DEVELOPMENT_BACKLOG.md`.

---

## Measured Results

### D3 — Depth-dependent refractive index n(z)

| Metric | Value | Acceptance |
|---|---|---|
| Fitted coefficients vs emulator truth | a=1.332900, b=2.00e-05, c≈0 (matches emulator model exactly) | — |
| Max held-out residual \|n_fit − n_true\| | 4.4e-16 (numerical precision) | < 1e-6 ✅ |
| Mean distance error @ 3.0 m, constant n=1.333 | 1.013 mm | — (baseline) |
| Mean distance error @ 3.0 m, fitted n(z) | ~0.000 mm | < 0.1 mm ✅ |

`EnvironmentalCorrector.calibrate_depth_model()` (least-squares degree-2 fit) exactly recovers the emulator's ground-truth n(z) from 6 calibration depths and beats the constant-n correction on held-out depths. The absolute improvement is small (~1 mm at 3 m) because n varies weakly with depth — consistent with D3's MEDIUM priority in the ТЗ.

### D4 — Temperature compensation

| Metric | Value | Acceptance |
|---|---|---|
| Simulated sensor drift slope | −0.0004 /°C (within TFmini-S ±0.05 %/°C envelope) | — |
| Fitted correction slope (6-point oven procedure) | +0.000398 /°C | \|fit − truth\| < 5e-5 ✅ |
| Max held-out residual @ 1.0 m | 0.021 mm | < 0.5 mm ✅ |

The oven-calibration procedure from the ТЗ (6 temperatures, linear ratio fit) recovers the inverse drift slope to first order; residual is O(drift²) as expected.

### D2 — Multipath detection @ 3 NTU equivalent

| Metric | Before fixes | After fixes | Acceptance |
|---|---|---|---|
| Detection rate (n=200) | 100 % | **100 %** | > 85 % ✅ |
| False-positive rate (n=200) | **16.5 %** ❌ | **0.0 %** | < 5 % ✅ |

**P9-sim finding (fixed in this commit)**: the initial EM fit initialized component means by a median split. When the scattered cluster is a small minority far below the direct cluster, EM converged to a local optimum that split the *direct* cluster in two, flagging ~16 % of legitimate direct-path readings as multipath. Two mechanical fixes, both in `app/multipath_detector.py`:

1. **Percentile initialization** — component means now initialize at the 10th/90th percentiles (n_iter default raised 10→25), which converges to the true clusters in the minority-cluster regime.
2. **Bimodality guard (Ashman's D)** — flagging now additionally requires `D = √2·(μ₁−μ₀)/√(σ₀²+σ₁²) > 2`. When the window is effectively unimodal (no scattered population present), nothing is flagged, eliminating false positives against degenerate splits.

All 11 pre-existing D2 unit tests still pass unchanged.

### D8 — 9-DOF EKF fusion

| Metric | Value | Acceptance |
|---|---|---|
| Position RMSE, raw measurements (σ=0.15 m, 3-axis) | 0.251 m | — (baseline) |
| Position RMSE, fused (default tuning) | 0.127 m | — |
| **Position improvement** | **49.5 %** | > 15 % ✅ |
| Attitude RMSE raw (σ=0.02 rad, 3-axis) | 0.0357 rad | — (baseline) |
| Attitude RMSE fused, default q=0.05 | 0.0334 rad (must not be worse) | < raw ✅ |
| Attitude RMSE fused, tuned q=0.001 | 0.0180 rad | — |
| **Attitude improvement (tuned)** | **49.7 %** | > 30 % ✅ |

**P8 guidance measured in sim**: the default `process_noise_attitude=0.05` is tuned to *track maneuvers*, so its smoothing margin on a station-keeping trajectory is deliberately small (~6 %). Lowering it to 0.001 for slowly-varying attitude buys ~50 % RMSE reduction. Field P8 should pick the operating point from real maneuver data; `EKF_PROCESS_NOISE_ATTITUDE` env var is the knob.

### D1 — 3D beam projection geometry

| Metric | Value |
|---|---|
| Pitched-beam endpoint vs closed-form geometry (30° pitch, 90° yaw, 3 m) | exact (< 1e-9 m) ✅ |
| 1D yaw-only projection endpoint error @ 30° pitch, 3 m range | **1.55 m (52 % of range)** |
| 3D projection endpoint error, same case | 0.00 m |
| End-to-end via `_project_beam()` with injected MAVLink attitude | matches closed form < 1e-9 ✅ |

This quantifies the error budget D1 removes: a tilted ROV mapping with heading-only projection misplaces points by up to half the measured range.

---

## Phase-status impact

| Decision | P9-sim | P9-field |
|---|---|---|
| D1 | ✅ this report | ⏳ needs IMU + known-angle dives |
| D2 | ✅ this report (incl. 2 fixes) | ⏳ needs turbidity tank |
| D3 | ✅ this report | ⏳ needs depth pool/dives |
| D4 | ✅ this report | ⏳ needs oven/water bath |
| D8 | ✅ this report (incl. P8 tuning guidance) | ⏳ needs reference trajectories |

Full suite after this campaign: **267/267 tests passing** (258 prior + 9 P9-sim).
