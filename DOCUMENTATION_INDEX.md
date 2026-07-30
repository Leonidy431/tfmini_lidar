# Documentation Index

Single entry point into all BLSNS documentation. Grouped by purpose, not by directory — most docs live at the repo root, a few live under `docs/`. Update this file whenever a top-level `.md` is added or removed (Rule 6/7 housekeeping).

---

## Start Here

| Doc | What it's for |
|---|---|
| [`README.md`](README.md) | Project overview, quick start, hardware requirements, API summary |
| [`CLAUDE.md`](CLAUDE.md) | Mandatory project rules (Rules 1-7): blind-spot audits, FTO, golden signals, data quality, risk management, session continuity, the 12-phase algorithm HLD |
| [`.clauderc`](.clauderc) | 99 engineering-discipline rules ("Посох"), imported into every session via `@.clauderc` in CLAUDE.md; composes with Rules 1 and 7 rather than replacing them |
| [`DOCUMENTATION_INDEX.md`](DOCUMENTATION_INDEX.md) | This file |

---

## Physics & Algorithm Engineering

| Doc | What it's for |
|---|---|
| [`PHYSICS_AUDIT.md`](PHYSICS_AUDIT.md) | The original 70-finding audit (16 critical, 31 high, 20 medium, 8 deferred as D1-D8) that this whole engineering track descends from |
| [`docs/ALGORITHMS.md`](docs/ALGORITHMS.md) | ICP, clustering, data-quality internals reference |
| [`docs/ALGORITHM_PHYSICS_AUDIT_12PHASE_HLD.md`](docs/ALGORITHM_PHYSICS_AUDIT_12PHASE_HLD.md) | Retrospective application of the Rule 7 12-phase HLD to the original physics-audit fixes (log-space likelihood, ENU heading, regime-change detection) |
| [`docs/ALGORITHM_DECISION_LOG.md`](docs/ALGORITHM_DECISION_LOG.md) | Per-decision records in the Rule 7 format, plus a reusable template for future decisions |
| [`TECHNICAL_SPECIFICATION.md`](TECHNICAL_SPECIFICATION.md) | ТЗ for D1-D8 (the 8 deferred decisions): requirements, literature, approach synthesis, specialist voting, and a phase-status table (P7 code vs. P9 hardware-blocked) |

**Reading order for a new algorithmic decision**: `CLAUDE.md` Rule 7 → `docs/ALGORITHM_DECISION_LOG.md` (see the template at the bottom) → write your own entry there once P1-P12 are complete.

---

## Deferred Decisions (D1-D8) — Code + Status

| Module | Decision | Status |
|---|---|---|
| [`app/mavlink_imu.py`](app/mavlink_imu.py) | D1: MAVLink 3D attitude | Code done (P7/P10/P11/P12); P8/P9 hardware-blocked |
| [`app/multipath_detector.py`](app/multipath_detector.py) | D2: Multipath/turbidity detection | Code done (P7/P10/P11/P12); P8/P9 hardware-blocked |
| [`app/environmental_correction.py`](app/environmental_correction.py) | D3+D4: Depth-dependent refraction + temperature compensation | Code done (P7/P8-partial/P10/P11/P12); P9 hardware-blocked |
| [`app/ekf_3d_attitude.py`](app/ekf_3d_attitude.py) | D8: 9-DOF position+attitude EKF fusion | Code done (P7/P10/P11/P12); P8/P9 hardware-blocked |
| — | D5: Vibration filtering | Spec only (`TECHNICAL_SPECIFICATION.md`), low priority |
| — | D6: Velocity profile modeling | Spec only, low priority |
| — | D7: Viscosity tuning | Spec only, low priority |

Corresponding tests: `tests/test_mavlink_imu.py`, `tests/test_multipath_detector.py`, `tests/test_environmental_correction.py`, `tests/test_ekf_3d_attitude.py`, `tests/test_deferred_decisions_integration.py` (wiring into `app/main.py`).

All four are wired into `app/main.py` behind `Config` feature flags that default to `false` — see [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) for the `ENABLE_*` env vars once documented there, or `app/config.py` directly (`MAVLinkAttitudeConfig`, `MultipathConfig`, `EnvironmentalCorrectionConfig`, `EKFConfig`).

---

## Planning & Process

| Doc | What it's for |
|---|---|
| [`DEVELOPMENT_BACKLOG.md`](DEVELOPMENT_BACKLOG.md) | 8-week sprint plan: Blind Spot Audit Round 2, D1-D8 implementation tasks (with live status), technical debt, resource allocation, risk register |
| [`SPRINT_1_WORKPLAN.md`](SPRINT_1_WORKPLAN.md) | Day-by-day Sprint 1 breakdown (D1/D2 P1-P6 + audit domains 15-19) |
| [`BLIND_SPOT_AUDIT_R2_PLAN.md`](BLIND_SPOT_AUDIT_R2_PLAN.md) | Execution plan for the 10 remaining specialist domains (15-24): roles, checklists, expected finding counts |
| [`CORRESPONDENCE_LOG.md`](CORRESPONDENCE_LOG.md) | Chronological request → outcome log for the whole session (Rule 6 continuity); read this first after a context reset to see what was asked and what was delivered, in order |
| [`SESSION_COMPLETION_REPORT.md`](SESSION_COMPLETION_REPORT.md) | Point-in-time snapshot report from the end of the physics-audit + Orin Nano phase of work |

**Reading order after a session break**: `CORRESPONDENCE_LOG.md` (what happened, in order) → `DEVELOPMENT_BACKLOG.md` (what's left, with status) → the specific doc for whatever you're picking up.

---

## Hardware & Deployment (NVIDIA Orin Nano Super 8GB)

| Doc | What it's for |
|---|---|
| [`docs/ORIN_NANO_SETUP.md`](docs/ORIN_NANO_SETUP.md) | 8-part guide: JetPack flashing, wiring, dependencies, testing, systemd/Docker deployment, troubleshooting, performance benchmarks, BlueOS companion integration |
| [`BUILDING.md`](BUILDING.md) | Build/deploy procedures: Docker buildx cross-compile, native build, docker-compose (with Prometheus/Grafana) |
| [`Dockerfile.arm64`](Dockerfile.arm64) | Multi-arch production image (NVIDIA L4T PyTorch base) |
| [`tests/emulation_server.py`](tests/emulation_server.py) | Multi-protocol hardware emulator (TFmini-S UART, MAVLink IMU, MS5837 depth) with a physically-grounded underwater environment model — lets the whole stack run without real sensors |
| [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md) | BlueOS extension installation, Docker, production WSGI |
| [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) | Every config parameter, ranges, tuning |
| [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) | Connection, SLAM, localization issue diagnosis |

---

## API & Protocol Reference

| Doc | What it's for |
|---|---|
| [`docs/API.md`](docs/API.md) | Full REST reference |
| [`docs/WEBSOCKET_PROTOCOL.md`](docs/WEBSOCKET_PROTOCOL.md) | Event names + payloads |
| [`docs/SCANNER_3D.md`](docs/SCANNER_3D.md) | 3D object scanner (orbit-scan) mode |
| [`docs/TESTING.md`](docs/TESTING.md) | Test layout, CI notes, which suites need the full scientific stack |

---

## Compliance, IP, Safety

| Doc | What it's for |
|---|---|
| [`RISK_MANAGEMENT.md`](RISK_MANAGEMENT.md) | ISO 14971 hazard analysis (Rule 5) |
| [`PATENT.md`](PATENT.md) | Patent declaration and IP documentation |
| [`docs/FTO.md`](docs/FTO.md) | Freedom-to-operate analysis (Rule 2) |
| [`docs/COMPETITOR_IP.md`](docs/COMPETITOR_IP.md) | Prior-art / competitor IP notes |
| [`docs/EXPORT_CONTROL.md`](docs/EXPORT_CONTROL.md) | Export control classification notes |
| [`LICENSES.md`](LICENSES.md) | Third-party attributions |
| [`blueos-lidar-module.md`](blueos-lidar-module.md) | BlueOS extension manifest notes |

---

## Directory Map (for orientation, not exhaustive)

```
app/
  main.py                    REST API + WebSocket + orchestration
  lidar_driver.py             TFmini-S UART driver (auto-reconnect)
  slam_engine.py               ICP registration (Open3D)
  localization.py               Map-based position estimation
  mavlink_imu.py                 D1: MAVLink 3D attitude
  multipath_detector.py           D2: Turbidity/multipath detection
  environmental_correction.py      D3+D4: Depth + temperature correction
  ekf_3d_attitude.py                D8: 9-DOF EKF fusion
  map_manager.py               Map persistence (PLY/PCD/NPY/H5)
  profile_recorder.py           Waypoint recording + playback
  object_detection.py            Pattern + clustering classification
  scanner_3d.py                   3D object scanner (orbit rings)
  data_quality.py                  IQR/Z-score/rate outlier filtering
  security.py                       Auth, rate limiting, path safety
  config.py                          Dataclass configuration (incl. D1-D8 flags)
  web/                                 Frontend (HTML/CSS/JS, Three.js)
tests/                        Pytest suites (258 tests as of this session)
docs/                         Deep-dive reference docs (see tables above)
```

---

## Maintenance Note

This index is a snapshot, not a database — if it drifts from the actual file list, trust `ls *.md docs/*.md` over this table and fix the table. Update it as part of any commit that adds/removes a top-level doc, per `DEVELOPMENT_BACKLOG.md` Section 5.
