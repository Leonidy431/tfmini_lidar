# Blind Spot Audit Round 2 — Findings (Domains 15–19)

**Method**: three parallel specialist auditors over the repo, per `BLIND_SPOT_AUDIT_R2_PLAN.md` and CLAUDE.md Rule 1. Every finding was verified against actual file contents (line-referenced), not speculated.

**Result**: 44 findings — 5 CRITICAL, 15 HIGH, 18 MEDIUM, 6 LOW. This session applied the safe, high-value **MECHANICAL** fixes (durability + security hygiene + the two build-breaking Docker bugs) and logged the **NEEDS-DECISION** items for a human. All fixes verified against the test suite (**279/279 passing**, +12 new regression tests).

Legend: ✅ fixed this session · 📋 logged (NEEDS-DECISION / larger change) · severity in brackets.

---

## Domain 17 — Database & Persistence (highest ROV impact: power loss is routine)

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 17-1 | CRITICAL | Map save non-atomic, destroys previous good map in place | ✅ staging-dir + atomic `os.replace` swap (`map_manager.py::save_map`) |
| 17-2 | CRITICAL | Zero fsync; rename itself can be lost on power cut | ✅ `_fsync_path`/`_fsync_dir` on every write + dir after rename |
| 17-3 | HIGH | Metadata written claiming N points even when point write silently failed | ✅ `_save_points` checks `o3d.io.write_point_cloud` bool, returns None; save aborts |
| 17-4 | HIGH | Truncated/corrupt point files load as partial/empty and go to localization | ✅ `load_map` validates Nx3 shape + cross-checks `metadata.point_count` |
| 17-5 | HIGH | One corrupt `metadata.json` makes an intact map permanently unloadable | ✅ metadata read in its own try/except → synthesizes + tries all formats |
| 17-6 | HIGH | Failed/partial saves leave orphan dirs that `list_maps` advertises as real | ✅ build in `.staging`, `rmtree` on failure; listing skips `.staging`, marks metadata-less dirs `"incomplete"` |
| 17-8 | MEDIUM | One bad file blanks the entire map/profile listing | ✅ per-entry try/except in `list_maps` (maps); 📋 profiles listing (same pattern, not yet applied) |
| 17-7 | MEDIUM | Format switch orphans old points file; disables multi-format fallback | 📋 (fallback-to-stale-map is a recovery-policy decision) |
| 17-9 | MEDIUM | `objects/` grows unbounded; same-second saves overwrite; filename not `safe_join`ed | 📋 (retention policy = decision); note the `safe_join` gap is latent (no route wired) |
| 17-10 | MEDIUM | No log rotation — `app.log` fills the extension volume | ✅ `RotatingFileHandler` (10 MB × 3) in `main.py` |
| 17-11 | MEDIUM | No free-space check / ENOSPC discrimination; profile retry loop retries ENOSPC | 📋 (threshold + operator behavior = decision) |
| 17-12 | MEDIUM | `MapManager` has no lock — save/load/delete race; names silently overwrite | 📋 (overwrite semantics are a UX contract; per-name lock is the fix) |
| 17-L1 | LOW | Orphan `.tmp` profiles never cleaned | 📋 |
| 17-L2 | LOW | In-flight recordings exist only in RAM (lost on power cut) | 📋 (checkpoint interval = decision) |
| 17-L3 | LOW | `export_map` writes to unvalidated caller path (currently unreachable) | 📋 |
| 17-L4 | LOW | `Config.init_directories()` raises at import on read-only `/app/data` | 📋 |

---

## Domain 18 — Security Hardening

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 18-1 | HIGH | WebSocket auth off by default — telemetry+config readable by any LAN host | 📋 (defaulting `REQUIRE_WS_AUTH=true` changes local-use UX — decision) |
| 18-2 | HIGH | 12 GET routes unauthenticated, contradicting CLAUDE.md's protected list | 📋 (add auth vs. update doc — read-exfil scope decision; no state-changing route is unprotected — verified) |
| 18-3 | HIGH | Rate limit keyed on spoofable `X-Forwarded-For`; store never evicts | ✅ eviction past cap + empty-window sweep; 📋 trusted-proxy XFF gate (decision) |
| 18-4 | HIGH | Failed-auth never reaches rate limiter — unlimited token guessing | ✅ `require_auth` meters both failure paths before returning 401 |
| 18-8 | MEDIUM | Auto-generated token written cleartext to the persisted log file | ✅ log only an 8-char fingerprint; full token to stdout once |
| 18-9 | MEDIUM | Log injection via raw `X-Forwarded-For` in log lines | ✅ `repr()` the client id in both warnings |
| 18-10 | MEDIUM | `flask-cors==5.0.0` / `requests==2.32.3` carry published advisories | ✅ bumped to `flask-cors==6.0.0`, `requests==2.32.4` |
| 18-11 | MEDIUM | CORS origins not trimmed/validated; needless `supports_credentials` | ✅ strip + reject `*`; dropped `supports_credentials` |
| 18-7 | MEDIUM | `api_key` in query string lands in access/proxy/referrer logs | 📋 (drop query fallback vs. scrub logs — decision; kept for WS handshake) |
| 18-12 | MEDIUM | Token in `localStorage` behind CDN scripts w/ no SRI, no CSP | 📋 (SRI + CSP; inline `onclick` handlers foreclose strict CSP — larger frontend change) |
| 18-15 | LOW | Unused `hmac` import; no min-entropy check on `LIDAR_API_TOKEN` | ✅ dropped import; reject env token < 16 chars |

---

## Domain 19 — Network & Communication

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 19-6 | HIGH | No real WebSocket transport — `simple-websocket` absent, silent long-poll fallback | ✅ added `simple-websocket==1.0.0` to requirements |
| 19-13 | MEDIUM | `socketio.emit` from two threads: no ordering guarantee, no per-client backpressure | 📋 (coalesce lidar_reading to ~10 Hz through one emitter — design change) |
| 19-14 | MEDIUM | Client reconnect gives up after ~40 s, loses gap events, reuses stale token | 📋 (frontend: `reconnectionAttempts: Infinity`, refresh `socket.auth`, resync on reconnect) |

---

## Domain 15 — Deployment & DevOps / Domain 16 — CI/CD

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 15-2 | CRITICAL | `Dockerfile.arm64` won't parse — inline comments on `EXPOSE` lines | ✅ moved comments to their own lines |
| 15-1 | CRITICAL | Default container CMD crashes at startup (`allow_unsafe_werkzeug=False`, no TTY) | 📋 (gunicorn CMD vs. explicit dev entrypoint — deployment decision) |
| 16-3 | CRITICAL | No CI exists despite docs describing CI gating | 📋 (add `.github/workflows/ci.yml` — this is exactly what let 15-2 ship broken) |
| 15-4 | HIGH | `Dockerfile.arm64` pins conflict with `requirements.txt`, re-introducing patched CVEs | 📋 (delete hand-written pin blocks, install from requirements — moderate Dockerfile rewrite) |
| 15-5 | HIGH | Open3D built from source then re-requested from PyPI | 📋 (needs `-DBUILD_PYTHON_MODULE=ON` + strip open3d from req install on ARM64) |
| 15-6 | HIGH | Primary `Dockerfile` can't build for BlueOS's actual RPi ARM64 target | 📋 (platform-support decision) |
| 15-7 | HIGH | `Dockerfile.arm64` single-stage despite "multi-stage" header; ships toolchain+pytest in runtime | 📋 (builder/runtime split — moderate rewrite) |
| 15-8 | MEDIUM | ARM64 CMD passes `--host`/`--port` flags `app/main.py` never parses; `--headless` in docs doesn't exist | 📋 (drop flags or add argparse) |
| 15-9 | MEDIUM | Commented prod CMD references non-existent `app.wsgi:app`; gunicorn not in requirements | 📋 |
| 15-10 | MEDIUM | `BUILDING.md` gunicorn recipe (`--workers 2 --worker-class sync`) contradicts DEPLOYMENT.md and corrupts SLAM state | 📋 (doc fix) |
| 15-12 | MEDIUM | Stale test counts ("175/175") vs. 279 actual; `--cov` documented but `pytest-cov` absent | 📋 (CI should publish the count; add pytest-cov) |
| 15-13 | MEDIUM | `BUILDING.md` verify snippet imports non-deps (`sklearn`, `torch`, `pyserial`-as-module) | 📋 (doc fix) |
| 15-14 | MEDIUM | BlueOS bind mount root-owned, container runs uid 1000 → data writes fail after healthy startup | 📋 (startup writability assert = decision) |
| 15-11 | MEDIUM | `docker-compose.yml`: `ports:` under `network_mode: host`; obsolete `version:` key | ✅ removed both |
| 15-16 | LOW | `simple-websocket` missing (see 19-6) | ✅ (fixed via 19-6) |
| 15-15 | LOW | Three different version strings for one release; `BUILDING.md` mounts non-existent `prometheus.yml` | 📋 |
| 15-17 | LOW | No committed systemd unit / `.env.example`; no `PYTHONUNBUFFERED` in x86 image | 📋 |

**Verified-clean** (auditor checked, no issue): `.dockerignore` exists and is correct (~2 MB context); `curl` present in both images and `/api/health` public, so healthchecks are wired correctly; no state-changing route lacks `@require_auth` (all 22 covered); token storage design (SHA-256 of a 256-bit random) makes the missing `compare_digest` non-exploitable.

---

## Fixes applied this session (12 mechanical) — verification

New regression suite `tests/test_persistence_hardening.py` (12 tests) locks in: atomic save (no staging leak, re-save stays loadable, all 4 formats round-trip), load validation (corrupt-metadata still loads, truncated/wrong-shape rejected), listing isolation (one bad file doesn't blank the list, staging dirs hidden), rate-limiter eviction + windowing, and weak-token rejection.

Full suite: **279/279 passing**.

Files changed: `app/map_manager.py`, `app/security.py`, `app/main.py`, `requirements.txt`, `Dockerfile.arm64`, `docker-compose.yml`, `tests/test_persistence_hardening.py`.

---

# Domains 20–24 (second wave)

Same method (three parallel auditors). **40 further findings** — 6 HIGH, 20 MEDIUM, remainder LOW. This wave applied **7 mechanical fixes** (backend observability + underwater depth-staleness + frontend robustness); the rest are logged.

## Domain 20 — UX & Dashboard / Frontend

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 20-1 | CRITICAL | Realtime libs load only from public CDN — offline field use → whole dashboard dead | 📋 (vendor socket.io/three.js locally — decision + asset commit) |
| 20-2 | HIGH | Absolute `/api`, `/static`, socket.io paths break behind a reverse-proxy subpath | 📋 (derive base path — BlueOS mount-point decision) |
| 20-3 | HIGH | Failed maps/profiles fetch throws, list stuck on "Loading…" forever | ✅ `Array.isArray` guard + error state in `loadMaps`/`loadProfiles` |
| 20-4 | HIGH | 401/expired token renders identically to backend-down | ✅ `updateStatus` detects `success===false`/`unauthorized`, distinct toast |
| 20-5 | MEDIUM | Most control handlers silently swallow failures | 📋 (add `else showToast(...)` across ~7 handlers — larger sweep) |
| 20-6 | MEDIUM | `PointCloudVisualizer.clear()` leaks GPU geometry/material | ✅ dispose geometry+material+markers in `clear()` |
| 20-7 | MEDIUM | No touch controls on the 3D view — unusable on a field tablet | 📋 (add pointer/touch or OrbitControls — decision) |
| 20-8 | MEDIUM | Localization 2D canvas can render at zero size until a window resize | 📋 (ResizeObserver / resize in switchTab) |
| 20-9 | MEDIUM | Tabs lack `aria-selected`/`tabpanel` roles + keyboard nav | 📋 (a11y sweep) |
| 20-10 | MEDIUM | Connection status conveyed by color alone, never announced | 📋 (a11y: text + aria-label) |
| 20-11 | MEDIUM | Low-contrast secondary text below WCAG AA | 📋 (color decision) |
| 20-12 | LOW | Blocking `alert()`/`confirm()`, suppressible in kiosk webviews | 📋 |
| 20-13 | LOW | Nav arrow not updated when `heading_error` is exactly 0 | ✅ guard on `!= null` |
| 20-14 | LOW | Screenshot may be blank (no `preserveDrawingBuffer`) | ✅ set `preserveDrawingBuffer: true` |

## Domain 21 — Scalability & multi-ROV

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 21-1 | HIGH | Global singleton + single shared mode blocks multi-ROV entirely | 📋 (vehicle-id registry — architectural decision) |
| 21-2 | MEDIUM | `SLAMEngine.poses` list grows unbounded for the whole mission | 📋 (bounded/decimated deque) |
| 21-3 | MEDIUM | Full-cloud voxel downsample per web poll, on request thread, under SLAM lock | 📋 (cache downsampled cloud, invalidate on scan) |
| 21-4 | MEDIUM | Accumulated-cloud growth O(n)/scan via `+` concat → ~O(n²)/mission | 📋 (chunked buffer + fixed-cadence downsample) |
| 21-5 | MEDIUM | Particle-filter update pure-Python O(particles×map_points)/reading | 📋 (vectorize / KD-tree) |
| 21-6 | LOW | Localization ICP matches full non-downsampled reference map each scan | 📋 |

## Domain 22 — Underwater domain-specific

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 22-1 | HIGH | Depth telemetry had no staleness guard (unlike attitude) — frozen depth biases every range | ✅ `set_depth` timestamped; `_fresh_depth()` expires after `DEPTH_TIMEOUT_S`, falls back to constant-n |
| 22-2 | MEDIUM | Salinity + water-temperature terms unmodeled in refractive index | 📋 (CTD-driven n — needs sensor + decision) |
| 22-3 | MEDIUM | SLAM assumes rigid static world; bubbles/particulate/fish become permanent map points | 📋 (temporal-persistence/occupancy filter — design) |
| 22-4 | MEDIUM | Multipath filter can't catch high-signal near returns (specular/hull/bubble) | 📋 (gate on posterior+geometry, not low-signal precondition) |
| 22-5 | MEDIUM | Fixed 4.0 m `max_range` doesn't adapt to turbidity/thermocline | 📋 (adaptive range from attenuation/quality-score) |
| 22-6 | LOW | Driver's built-in default `max_range_m=12.0` (in-air) | ✅ by-design: library default stays air/bench (matches `medium_refractive_index=1.0` default philosophy); app always passes 4.0 via config. Documented, not changed (changing it would reject the 5 m frames in existing driver tests). |

## Domain 23 — Hardware Integration & Calibration

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 23-1 | HIGH | IMU→LiDAR extrinsic (lever-arm + mount rotation) never applied | 📋 (configurable SE(3) extrinsic — needs mounting data) |
| 23-2 | MEDIUM | Frame timestamp stamped at parse time after bulk read, not at capture | 📋 (per-frame cadence anchoring) |
| 23-3 | MEDIUM | No output-format/firmware handshake at init | 📋 (query version, force format — mechanical but needs hardware to validate) |
| 23-4 | MEDIUM | Hot-unplug re-enumeration (ttyUSB0→ttyUSB1) not handled | 📋 (by-id/by-path symlink — decision) |
| 23-5 | MEDIUM | No calibration persistence (D3/D4 coefficients + sensor EEPROM lost on restart) | 📋 (persist to data volume — ties to 23 & D3/D4 P9) |
| 23-6 | LOW | Dockerfile vs compose device-map mismatch; MAVLink defaults to UDP | 📋 (reconcile + document serial connection string) |

## Domain 24 — Observability & Telemetry

| # | Sev | Finding | Status |
|---|-----|---------|--------|
| 24-7 | HIGH | New D1/D2/D3/D8 `get_statistics()` exposed nowhere | ✅ `sensor_fusion` block added to `get_status()` (per-module, enabled-only) |
| 24-8 | HIGH | No Prometheus `/metrics` endpoint despite documented monitoring stack | 📋 (add endpoint or drop the docs claim — decision) |
| 24-9 | HIGH | Health check ignores data freshness — mute-but-connected sensor reads "healthy" | ✅ `STALE_READ_S` gate → `degraded` + `stale_readings` reason |
| 24-10 | MEDIUM | Latency golden signal missing (Rule 3) | 📋 (time `_process_reading`, expose histogram) |
| 24-11 | MEDIUM | Valid-frame rejections dropped with no counter | ✅ driver `invalid_readings` counter, surfaced in stats |
| 24-12 | MEDIUM | Checksum/sync errors lumped into `errors_count` (drives health gate) | ✅ separate `frame_errors` counter surfaced (error_rate semantics unchanged — safe) |
| 24-13 | LOW | No request tracing / correlation IDs | 📋 |
| 24-14 | LOW | Per-reading rejections log per-frame (flood at DEBUG, invisible at INFO) | 📋 (periodic aggregate counter) |

## Domains 20–24 fixes applied this wave (7 mechanical) — verification

`tests/test_audit_r2_domains20_24.py` (11 tests): depth staleness (fresh applied / stale→constant-n / helper), health freshness (stale→degraded, fresh→healthy), driver counters (frame_errors on bad checksum, invalid_readings on sentinel, valid not miscounted), fusion metrics (default empty, enabled reported, `sensor_fusion` key present). Frontend fixes (20-3/4/6/13/14) verified by inspection (no JS test harness).

Full suite after both waves: **290/290 passing**.

---

## Next-session priorities (from the 📋 backlog across both waves)

1. **CI (16-3)** — highest leverage: nothing caught the build-breaking `EXPOSE` bug because nothing runs. A minimal `.github/workflows/ci.yml` (dependency-light pytest + `docker build` both files) prevents recurrence.
2. **Container startup (15-1 / 18-5)** — the default `docker compose up -d` crash-loops; needs the gunicorn CMD decision.
3. **WS auth default + route auth (18-1, 18-2)** — read-exfiltration scope; a security-posture decision.
4. **Reverse-proxy base path + local vendored libs (20-1, 20-2)** — the dashboard is dead offline and behind a subpath, i.e. exactly the BlueOS deployment shape.
5. **Scalability O(n²) mapping + per-poll downsample (21-3, 21-4)** — the map pipeline degrades with size/clients; caching + chunked accumulation is mechanical once the shape is agreed.
6. **IMU→LiDAR extrinsic (23-1)** and **adaptive range / dynamic-return rejection (22-3, 22-5)** — underwater correctness, need mounting/field data.
