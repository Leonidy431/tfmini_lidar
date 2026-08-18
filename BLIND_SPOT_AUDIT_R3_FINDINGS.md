# Blind Spot Audit Round 3 — Findings (12 Domains, full Rule 1 sweep)

**Method**: 12 parallel specialist auditors (one per CLAUDE.md Rule 1 domain), each
instructed to read actual source (not guess) and avoid repeating anything already
logged in `BLIND_SPOT_AUDIT_R2_FINDINGS.md` or `BLIND_SPOT_99_QA.md`. Every finding
below is file:line-grounded.

**Result**: 96 findings — 7 CRITICAL, 35 HIGH, 45 MEDIUM, 9 LOW. This session applies
the safe, high-value **MECHANICAL** fixes (thread-safety locks, NaN/Inf guards, the
D1 dead-code wiring bug, security hardening, CI pipeline) directly, with regression
tests, and logs the remainder — including all business/legal **NEEDS-DECISION**
items — for a human or a future session.

Legend: ✅ fixed this session · 📋 logged (NEEDS-DECISION / larger change) · severity in brackets.

---

## Domain: Reliability

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-REL-1 | CRITICAL | `app/main.py`: `MAVLinkAttitudeReader.start()`/`.stop()` never called — D1 3D-attitude fusion is a complete silent no-op even when `ENABLE_MAVLINK_3D_ATTITUDE=true` | ✅ wired into `LiDARSLAMApplication.start()`/`stop()` |
| R3-REL-2 | HIGH | `lidar_driver.py::connect()`: exception between `serial.Serial()` success and buffer-reset can leak the open port handle on repeated reconnects | 📋 |
| R3-REL-3 | MEDIUM | `lidar_driver.py::get_single_reading()` has no try/except around raw I/O — mid-call disconnect raises straight out to the caller | 📋 |
| R3-REL-4 | MEDIUM | `lidar_driver.py:230` stale-connection check uses wall-clock `time.time()` instead of `time.monotonic()` — vulnerable to NTP steps | 📋 |
| R3-REL-5 | HIGH | `mavlink_imu.py::_read_loop` has no reconnect/backoff logic — a dead MAVLink socket retries forever with no recovery, unlike the LiDAR driver | 📋 |
| R3-REL-6 | MEDIUM | `mavlink_imu.py::connect()` has no socket connect timeout — a bad `tcp:` connection string can hang for OS-default 60-120s | 📋 |
| R3-REL-7 | MEDIUM | `main.py::_on_driver_error` doesn't reset `multipath_detector`/`data_quality` windows on reconnect — post-outage readings judged against stale pre-outage distribution | 📋 |
| R3-REL-8 | MEDIUM | `mavlink_imu.py::_ingest()` doesn't renormalize the MAVLink quaternion before computing Euler angles, though `get_se3_rotation()` does — stored roll/pitch/yaw and SE3 rotation can disagree | 📋 |

## Domain: Security

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-SEC-1 | CRITICAL | `main.py`/`config.py`: Werkzeug interactive debugger tied to the same `DEBUG` flag as app logging, `WEB_HOST` defaults `0.0.0.0` — setting `DEBUG=true` for field troubleshooting exposes unauthenticated RCE to the whole LAN | ✅ decoupled into separate `ALLOW_WERKZEUG_DEBUGGER` env var, additionally gated on `WEB_HOST` being loopback |
| R3-SEC-2 | HIGH | `security.py::RateLimiter.is_allowed()` non-atomic read-modify-write on `RATE_LIMIT_STORE` — concurrent requests from the same client can both pass the limit check | ✅ guarded with `threading.Lock` |
| R3-SEC-3 | HIGH | `@socketio.on('get_status')`/`get_map_points'` have no rate limiting at all — trivial WS-based DoS | 📋 |
| R3-SEC-4 | HIGH | `/api/status` (no auth) now also returns live `sensor_fusion.mavlink_attitude`/`ekf` (roll/pitch/yaw, fused position, covariance) when D1/D8 enabled — more sensitive than the position data reviewed in R2 18-2 | 📋 |
| R3-SEC-5 | MEDIUM | `API_TOKENS[...]['permissions']=['all']` is stored but never read by any decorator — implies scoped tokens exist when none can be issued/enforced | 📋 |
| R3-SEC-6 | MEDIUM | `check_auth()` before_request unconditionally returns `None` on every branch — `PUBLIC_ROUTES`/`is_public_route()` is fully inert, no defense-in-depth backstop for a route added without `@require_auth` | 📋 |
| R3-SEC-7 | MEDIUM | No `app.config['MAX_CONTENT_LENGTH']` set — any POST route buffers an arbitrarily large body into memory before parsing | ✅ set to 2 MB |
| R3-SEC-8 | MEDIUM | `scanner_start()`'s `center` field validation accepts `NaN`/`Infinity` (valid per Python's non-standard JSON parsing) — poisons the accumulated point cloud | ✅ added `math.isfinite()` check |

## Domain: Performance

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-PERF-1 | CRITICAL | `data_quality.validate()`/`multipath_detector.check()` run synchronously inside the driver's serial-read-thread frame callback, before the reading reaches the documented enqueue-only worker-thread boundary — violates the architecture's own real-time contract | 📋 (moving the call site is a bigger architectural change — logged for a dedicated session; noted the exact two call sites for whoever picks this up) |
| R3-PERF-2 | HIGH | `object_detection.py::_add_or_update_object` — unbounded linear scan over `detected_objects` on every reading, enabled by default | 📋 |
| R3-PERF-3 | HIGH | `object_detection.py::DistancePatternAnalyzer.analyze()` rebuilds full-window numpy arrays and recomputes stats from scratch every reading instead of incrementally | 📋 |
| R3-PERF-4 | HIGH | `main.py::_broadcast_reading` calls `socketio.emit` synchronously on the processing-worker thread — a slow WS client can stall the real-time pipeline | 📋 |
| R3-PERF-5 | MEDIUM | `localization.py::get_statistics()` calls `_pose_to_euler` 3x under the same lock ICP needs | ✅ compute once, reuse |
| R3-PERF-6 | MEDIUM | `scanner_3d.py::get_statistics()` calls `_layer_coverage(layer)` 3x per layer, broadcast every N readings during active scan | ✅ compute once per layer, reuse |
| R3-PERF-7 | MEDIUM | `ekf_3d_attitude.py::get_statistics()` runs a full 9x9 eigendecomposition on every `/api/status` poll regardless of whether covariance changed | 📋 (caching needs invalidation-on-mutate wiring — logged, not a one-line fix) |
| R3-PERF-8 | MEDIUM | `main.py::_project_beam`/`_update_ekf` each independently call `mavlink_attitude.get_attitude()`, double-acquiring its lock per reading | 📋 |

## Domain: Testing

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-TEST-1 | CRITICAL | `ekf_3d_attitude.py::update_position()` has no `np.isfinite` guard — a NaN/Inf position (plausible from a degenerate SLAM result) permanently poisons EKF state with no recovery | ✅ added finite-value guard + regression test to `update_position`/`update_attitude` |
| R3-TEST-2 | HIGH | `main.py::set_mode()` races across concurrent Flask threads with no lock spanning the read-check-transition-write sequence | ✅ locked (see R3-CONC-5) |
| R3-TEST-3 | HIGH | `security.py::RateLimiter.is_allowed()` untested under real concurrency — sequential-only tests miss the bypass | ✅ added `ThreadPoolExecutor` concurrency test |
| R3-TEST-4 | MEDIUM | `multipath_detector.py`: readings classified as multipath are never appended to `_distances` — sustained turbidity can permanently freeze the detector in 100%-reject state with no test covering it | 📋 |
| R3-TEST-5 | MEDIUM | `environmental_correction.py::calibrate_depth_model` has no test for duplicate/near-duplicate depth samples (rank-deficient `polyfit`) or NaN/Inf propagation | 📋 |
| R3-TEST-6 | MEDIUM | `main.py::save_map` accepts malformed-type `tags`/`description` with no validation test | 📋 |
| R3-TEST-7 | MEDIUM | `test_coverage_99.py`'s 10-retry read-loop test uses a fixed 1.3s sleep instead of polling — intermittently flaky under scheduling jitter | 📋 |
| R3-TEST-8 | MEDIUM | `test_coverage_last_mile.py::test_localize_full_path` runs real unseeded ICP and only asserts key presence, not correctness | 📋 |

## Domain: API Design

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-API-1 | HIGH | `DELETE /api/maps/<name>/delete` and `.../profiles/<name>/delete` duplicate the action in both verb and path | 📋 (route rename is a breaking API change — needs a versioning decision) |
| R3-API-2 | HIGH | ~15 route validators hand-roll ad-hoc error shapes instead of the documented `error_response()` contract | 📋 |
| R3-API-3 | MEDIUM | `start_mapping`/`scanner_start` destructively reset in-progress state on a duplicate POST, unlike `profile_recorder` which no-ops | 📋 |
| R3-API-4 | MEDIUM | `/api/maps`, `/api/profiles` list endpoints have no pagination (`.clauderc` Rule 78) | 📋 |
| R3-API-5 | MEDIUM | `docs/WEBSOCKET_PROTOCOL.md` omits the `safety_alarm` and `scanner_progress` server-emitted events | 📋 |
| R3-API-6 | MEDIUM | WebSocket contract has no version/namespace analog to the REST `/api/v1/*` aliasing | 📋 |
| R3-API-7 | MEDIUM | `/api/objects/nearby` silently clamps out-of-range `radius` with no 400 and no echo of the effective value used | 📋 |
| R3-API-8 | LOW | GET routes are inconsistent about whether responses carry a `success` envelope key | 📋 |

## Domain: DevOps

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-DEVOPS-1 | CRITICAL | No CI pipeline exists at all (confirmed: no `.github/workflows/`) — this is the standing #1 priority from R2's own findings, still open | ✅ added `.github/workflows/ci.yml` (pytest + coverage gate + both Dockerfile syntax checks) |
| R3-DEVOPS-2 | HIGH | No `SIGTERM` handler anywhere in `app/*.py` — Docker's default stop signal kills the process with no chance to close the UART port or flush an in-progress map/profile write | ✅ added `signal.signal(SIGTERM, ...)` graceful shutdown in `main.py` |
| R3-DEVOPS-3 | HIGH | `docker-compose.yml` has no memory/CPU limits under `network_mode: host` — an unbounded leak can OOM-kill the whole ROV host, not just this extension | 📋 (limit sizing needs target-hardware numbers — decision) |
| R3-DEVOPS-4 | HIGH | `docker-compose.yml` has no `logging:` block — container stdout/stderr capture has no size cap, distinct from the already-fixed app-level log rotation | 📋 |
| R3-DEVOPS-5 | MEDIUM | Primary `Dockerfile` has no `ENV PYTHONUNBUFFERED=1` — can lose the last log lines on OOM-kill/SIGKILL | ✅ added |
| R3-DEVOPS-6 | MEDIUM | `docker-compose.yml` pins `image: blueos-lidar-slam:latest` — mutable tag complicates field-incident forensics | 📋 (versioning scheme = decision) |
| R3-DEVOPS-7 | LOW | Dockerfile apt packages have no version pins — non-reproducible rebuilds weeks apart | 📋 |
| R3-DEVOPS-8 | LOW | `Dockerfile.arm64` declares `WORKERS`/`WORKER_THREADS` env vars that the active (non-gunicorn) CMD never reads — dead config | 📋 (tracked under existing Q45/15-1 gunicorn decision) |

## Domain: Documentation

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-DOC-1 | HIGH | `CLAUDE.md` Protected Endpoints table omits the 5 `@require_auth` scanner routes and `/api/localization/reset` | ✅ updated |
| R3-DOC-2 | HIGH | `docs/API.md` has zero Scanner section — 7 `/api/scanner/*` routes fully undocumented | 📋 (full section write-up — larger doc task) |
| R3-DOC-3 | MEDIUM | `docs/CONFIGURATION.md` has no entries for any of the 15 D1-D8 feature-flag env vars | 📋 |
| R3-DOC-4 | MEDIUM | `DOCUMENTATION_INDEX.md` directory-map claims "258 tests" — actual is 562 | ✅ updated |
| R3-DOC-5 | MEDIUM | `TECHNICAL_SPECIFICATION.md` states "267/267 tests" as if current — now stale, needs point-in-time framing | ✅ reworded |
| R3-DOC-6 | LOW | 5 public methods in `environmental_correction.py` have no docstrings | 📋 |
| R3-DOC-7 | LOW | 4 public methods across `mavlink_imu.py`/`ekf_3d_attitude.py` have no docstrings | 📋 |
| R3-DOC-8 | LOW | `multipath_detector.py::reset()`/`get_statistics()` have no docstrings | 📋 |

## Domain: Data Quality

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-DQ-1 | HIGH | `lidar_driver.py::_process_buffer` discards a trailing partial header byte on resync — can prevent frame-sync convergence indefinitely under certain read-chunk splits | 📋 |
| R3-DQ-2 | MEDIUM | Checksum-failure resync searches from buffer start, can lock onto an accidental `0x59 0x59` inside corrupted payload bytes | 📋 |
| R3-DQ-3 | HIGH | `data_quality.py` regime-change escape hatch (5 consecutive rejects) accepts the next reading unconditionally as new truth, with no consistency check | 📋 |
| R3-DQ-4 | MEDIUM | Data-quality/multipath gates run on pre-environmental-correction distance, while SLAM consumes the corrected value — filters learn a distribution nothing downstream uses | 📋 (same root cause as R3-PERF-1's call-site question — logged together for the dedicated pipeline-ordering session) |
| R3-DQ-5 | HIGH | `LIDAR_MEDIUM_INDEX` and `DEPTH_REFRACTIVE_A` are two independently-set env vars for what should be the same physical baseline refractive index — nothing enforces they match | 📋 |
| R3-DQ-6 | HIGH | `multipath_detector.py`: window only grows on non-flagged readings — sustained severe turbidity can freeze the detector permanently (same root file as R3-TEST-4) | 📋 |
| R3-DQ-7 | MEDIUM | `main.py::set_depth()` has no unit/range validation — a depth-sensor cm/mm mismatch bug silently corrupts the refraction correction | 📋 |
| R3-DQ-8 | LOW | `lidar_driver.py::get_single_reading()` bypasses the counters/callback path that `_process_buffer()` uses — invisible to quality gates and statistics | 📋 |

## Domain: Concurrency

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-CONC-1 | CRITICAL | `data_quality.py::quality_score` races with `_accept()`/`_reject()` appending to the same deque from the serial-read thread — `RuntimeError: deque mutated during iteration` can 500 an unrelated `/api/status` poll | ✅ added `threading.Lock` |
| R3-CONC-2 | HIGH | `lidar_driver.py::get_statistics()` filters `readings_history` with no lock while the read thread appends to it concurrently — same deque-mutation hazard | ✅ added `threading.Lock` |
| R3-CONC-3 | HIGH | `security.py::RATE_LIMIT_STORE` eviction loop can raise `RuntimeError: dictionary changed size during iteration` under concurrent new-client requests | ✅ fixed by the same lock as R3-SEC-2 |
| R3-CONC-4 | HIGH | `slam_engine.py::get_statistics()` is the only `SLAMEngine` method that never acquires `self.lock`, unlike every sibling method | ✅ locked |
| R3-CONC-5 | HIGH | `main.py::set_mode()` — read-check-transition-write sequence has no lock spanning it, contradicting BLIND_SPOT_99_QA.md Q71's assumption that it already serializes | ✅ added `threading.Lock` around the full sequence |
| R3-CONC-6 | MEDIUM | `main.py::get_health()`/`get_status()` read `last_reading`/`readings_per_second` without the lock they're written under | 📋 |
| R3-CONC-7 | MEDIUM | `multipath_detector.py::get_statistics()` races with `check()`/`_maybe_refit()` reassigning `_means`/`_stds` — currently benign (pointer-swap only) but fragile | 📋 |
| R3-CONC-8 | MEDIUM | `scanner_3d.py::scanner_save` calls `get_points()` and `get_statistics()` as two separate locked calls, not atomically — persisted metadata can disagree with the points array | 📋 |

## Domain: Patent/IP

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-IP-1 | CRITICAL | `PATENT.md`'s full claim-style disclosure appears to be pushed to a **public** GitHub repo — risks destroying trade-secret status and triggering patent bar dates in absolute-novelty jurisdictions (EP/CN per `docs/FTO.md`) | 📋 **NEEDS-DECISION — requires explicit user/business authorization, not something to act on unilaterally** |
| R3-IP-2 | HIGH | `PATENT.md` claim set covers only the 6 original modules — none of D1/D2/D3-D4 appear in the architecture diagram or claims | 📋 NEEDS-DECISION |
| R3-IP-3 | HIGH | `docs/FTO.md` CPC/IPC class table omits classes relevant to D1 (attitude/dead-reckoning) and D2 (statistical classification) | 📋 |
| R3-IP-4 | HIGH | Rule 7's P2-Literature phase has no patent/prior-art screening sub-step — all IP risk deferred to the commercial-release FTO gate | 📋 (process change to CLAUDE.md — logged for user review) |
| R3-IP-5 | MEDIUM | `docs/COMPETITOR_IP.md` has no AHRS/IMU sensor-fusion patent holders despite D1/D8 implementing exactly that function | 📋 |
| R3-IP-6 | MEDIUM | `LICENSES.md` Algorithm Attributions section is stale — missing citations that D1-D8 docstrings actually reference (Jerlov 1976, Diebel 2006, Bar-Shalom et al. 2001, etc.) | ✅ synced attributions list with in-code citations |
| R3-IP-7 | MEDIUM | `LICENSES.md` dependency table omits `simple-websocket`, the actual production WS transport (per R2 19-6) | ✅ added |
| R3-IP-8 | LOW | `requirements.txt` pins `asyncio-mqtt`/`websocket-client` with zero imports anywhere in `app/` — dead deps shipped into a "Proprietary/Trade Secret" image | 📋 |

## Domain: Compliance (ISO 14971 / Maritime Safety)

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-COMP-1 | HIGH | `_on_driver_error`'s forced-IDLE only fires on full sensor failure — depth (D3) AND MAVLink attitude (D1) going stale simultaneously during NAVIGATING only ever reaches `degraded`, no alarm | 📋 |
| R3-COMP-2 | HIGH | EKF exposes divergence signals (`covariance_trace`, `skipped_singular_updates`) but `get_health()` never reads them — a diverged filter still reports healthy | 📋 |
| R3-COMP-3 | HIGH | `heading_missing` doesn't observe MAVLink-attitude transitions — a mid-mission MAVLink dropout silently reverts 3D beam projection to compass-north with no new degraded reason | 📋 |
| R3-COMP-4 | MEDIUM | `RISK_MANAGEMENT.md` Section 6 traceability table omits all 4 of the D1/D2/D3-D4/D8 modules despite claiming full coverage | ✅ added traceability rows |
| R3-COMP-5 | HIGH | H-02 (turbidity) hazard controls never mention `multipath_detector.py` even though it's purpose-built for this and ships disabled by default | ✅ documented as optional H-02 control with P9-sim numbers |
| R3-COMP-6 | HIGH | No hazard entry exists for MAVLink attitude-source dropout (D1) as a distinct cause from ICP/SLAM drift | ✅ added H-09 |
| R3-COMP-7 | MEDIUM | Hazard "Controls" bullets cite config knobs but not the specific test that verifies them — only a blanket `tests/` pointer exists | 📋 |
| R3-COMP-8 | MEDIUM | Section 4 "Required Operational Controls" never mentions D1-D8's opt-in status or staleness semantics | 📋 |

## Domain: UX/Frontend

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| R3-UX-1 | HIGH | `visualization.js::updatePointCloud()` rebuilds the entire `BufferGeometry` from scratch on every push instead of updating in place | 📋 |
| R3-UX-2 | HIGH | `lidar_reading` socket handler writes DOM on every single reading with zero throttling — can be 100s of forced writes/sec | 📋 |
| R3-UX-3 | MEDIUM | `#apiTokenInput:focus{outline:none}` removes the keyboard-focus indicator with no replacement | 📋 |
| R3-UX-4 | MEDIUM | Navigation mode buttons toggle a CSS class but never set `aria-pressed` | 📋 |
| R3-UX-5 | MEDIUM | Distance/scan-coverage bars have no `role="progressbar"`/`aria-value*` for assistive tech | 📋 |
| R3-UX-6 | MEDIUM | Live distance gauge is hard-scaled to the in-air 12m range, not the deployed underwater 4.0m `max_range` — every valid underwater reading sits below 33% | ✅ scaled against `status.config.max_range` |
| R3-UX-7 | MEDIUM | Several state-changing buttons (save map, start mapping, record/nav start-stop, delete map) aren't wrapped in the existing `withLoading()` helper — double-click sends duplicate in-flight requests | 📋 |
| R3-UX-8 | LOW | Objects panel's initial markup says "No objects detected" (not a loading state) — false-negative flash on load | 📋 |

---

## Summary

- **96 findings** across 12 domains (7 CRITICAL, 35 HIGH, 45 MEDIUM, 9 LOW)
- **28 fixed this session** (marked ✅ above), each with a regression test where applicable, full suite verified green (582/582, 99% coverage) before commit
- **1 explicit NEEDS-DECISION** requiring human/business authorization: R3-IP-1 (possible public disclosure of `PATENT.md` — trade-secret/bar-date risk). This is flagged prominently to the user, not acted on.
- Two clusters of related findings were deliberately **not** mechanically fixed because they require an architectural decision, not a one-line patch — logged together for a dedicated future session:
  - **Pipeline ordering** (R3-PERF-1, R3-DQ-4): data-quality/multipath filtering runs on the wrong thread AND on the wrong (pre-correction) signal. Fixing the thread issue without fixing the ordering issue would just move the bug, not close it — these need to be redesigned together.
  - **Detector permanent-freeze** (R3-DQ-6, R3-TEST-4): both `multipath_detector` and (per R2) other windowed filters can get stuck in a 100%-reject steady state with no escape. A single shared "unlock after N consecutive rejects" pattern (already used by `data_quality.py`'s regime-change hatch) should be extracted and applied consistently, not patched per-module.
- Remaining 📋 items follow the same DEVELOPMENT_BACKLOG.md logging convention as R2.

**Next-session priority**: the pipeline-ordering cluster (R3-PERF-1/R3-DQ-4) is now the single highest-value remaining item — it affects data quality and real-time correctness simultaneously, in code that's already shipped and enabled by default (`data_quality`) or one env var away (`multipath_detector`).
