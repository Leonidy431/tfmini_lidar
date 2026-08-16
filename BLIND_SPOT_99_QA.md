# 99 Blind Spot Questions & Answers

**Method**: 99 questions across the 12 specialist domains from CLAUDE.md
Rule 1 (Blind Spot Audit). For each question, exactly **one** answer is
selected — the decision is justified by naming the specific parameter(s)
from CLAUDE.md Rule 7's 48-parameter evaluation matrix (Accuracy,
Latency, Resource, Robustness, Generalization, Compatibility,
Maintainability, Scientific) that drove the choice, not by re-running a
full 12-phase HLD per question — that machinery is reserved for the
handful of decisions big enough to earn `docs/ALGORITHM_DECISION_LOG.md`
entries. This document is the fast, single-pass sibling: one question, one
answer, one cited reason.

**Relationship to prior audits**: `BLIND_SPOT_AUDIT_R2_FINDINGS.md` (84
findings, 10 domains, code-level) and this document overlap in domain
coverage but not in content — R2 found concrete bugs by reading code; this
document asks forward-looking design/policy questions (many genuinely
undecided, several extending R2's own NEEDS-DECISION items) and commits to
one answer each, so they stop being open questions. Where a question's
answer duplicates an R2 finding's fix, that's cross-referenced rather than
re-derived.

**Status of answers**: an "answer" here is a committed *design decision*,
not necessarily *implemented code* — where implementation is pending, the
answer says so and points at the relevant backlog location. Committing to
an answer is still valuable on its own: it converts a standing "we haven't
decided" into a stated position that can be executed, contested, or
revisited, rather than silently re-litigated every time it comes up.

---

## Domain 1: Security (9 questions)

**Q1. Should `REQUIRE_WS_AUTH` default to `true`?**
**A**: Yes, once a companion sensor mode exists — for now, stays `false` with a loud startup log warning when unset on a non-localhost `WEB_HOST`. Defaulting secure-by-default is right in principle, but a silent behavior flip would break every existing BlueOS deployment mid-upgrade with no error, only a working-but-insecure state that looks identical to before. *[Robustness: graceful degradation; Compatibility: backward compatibility]*

**Q2. Should the 12 unauthenticated GET routes (`BLIND_SPOT_AUDIT_R2_FINDINGS.md` 18-2) get `@require_auth`, or should CLAUDE.md's route table be corrected instead?**
**A**: Add `@require_auth` to the map/profile/scanner read routes. The map point cloud and live position are the actual sensitive payload (vehicle location, obstacle layout); read-only isn't the same as harmless here, given the underwater/security context this project already treats seriously elsewhere (rate limiting, path traversal guards). *[Robustness: failure mode count; Scientific: reproducibility — a documented threat model beats an undocumented "public by accident"]*

**Q3. `api_key` as a query parameter — keep, restrict to WS handshake only, or drop entirely?**
**A**: Restrict to the WebSocket handshake only (where header auth isn't available pre-connection) and drop the REST fallback. REST clients can always send a header; the query-string convenience isn't worth the log/referrer leakage risk documented in 18-7. *[Robustness: failure mode count; Compatibility: backward compatibility — deprecate with a log warning for one release before removing]*

**Q4. Should `LIDAR_API_TOKEN` enforce a minimum entropy beyond length (currently just `len >= 16`)?**
**A**: No — length is the right proxy here. A charset/entropy check adds complexity and false confidence (a 16-char dictionary phrase with high char diversity can still be weaker than a 20-char low-diversity random string); the actual guidance already documented (`secrets.token_urlsafe(32)`) is the real fix, enforced by example, not by a brittle heuristic. *[Maintainability: code clarity; Robustness: failure mode count]*

**Q5. Should the rate limiter's `X-Forwarded-For` trust be fixed now (18-3's remaining half) via a trusted-proxy allowlist?**
**A**: Yes, gated behind a `TRUSTED_PROXY_IPS` env var defaulting to empty (i.e., don't trust XFF at all by default — fall back to `remote_addr`). BlueOS deployments are typically direct, not behind a reverse proxy, so the safe default is "don't trust a header nothing is expected to set." *[Robustness: failure mode count; Compatibility: backward compatibility]*

**Q6. Should map/profile names allow Unicode (currently ASCII-only via `validate_path_component`)?**
**A**: No. The regex's ASCII-only restriction is deliberate defense-in-depth against path-traversal/encoding tricks (see the `.` / `..` / null-byte guards next to it) — Unicode filenames are a UX nicety this project's threat model doesn't need to accept the risk for. *[Robustness: failure mode count]*

**Q7. Should CSP/security headers (18-12, still open) be added via Flask's `after_request`, or via a reverse proxy in front of the app?**
**A**: `after_request` in `app/main.py`, not a reverse proxy — this project has no assumed reverse-proxy layer (BlueOS extensions are typically exposed directly), so the header must be set by the app itself or it won't exist in most deployments. *[Compatibility: dependency footprint — no new infra requirement; Robustness: graceful degradation]*

**Q8. Should the auto-generated token rotate periodically, or stay static for the container's lifetime?**
**A**: Static for the container's lifetime. Auto-rotation without a distribution mechanism (no operator UI push, no paired client) would lock out the dashboard on every rotation with no recovery path — worse than a long-lived token an operator can see once and store. Rotation is a `LIDAR_API_TOKEN`-holder's manual choice (restart with a new env value), not the app's job. *[Robustness: graceful degradation; Generalization: unknown-object handling — an "unknown" client can't be notified of a silent rotation]*

**Q9. Should failed-auth rate limiting (already fixed per 18-4) also trigger a temporary IP-level lockout after N failures, beyond the existing rolling rpm cap?**
**A**: No — a fixed lockout is a self-inflicted DoS vector (an attacker who knows the lockout window can lock out the legitimate operator by spoofing their IP in requests, if XFF trust is ever misconfigured per Q5). The existing 429 rate limit is sufficient; anything stronger belongs at the network layer (firewall), not the app. *[Robustness: failure mode count; Maintainability: code clarity]*

---

## Domain 2: Reliability (8 questions)

**Q10. Should the driver auto-reconnect indefinitely, or give up after N attempts and require an operator restart?**
**A**: Indefinitely, with the existing exponential backoff capped at `reconnect_delay_max` — an underwater ROV mid-dive has no operator standing by to restart a container; a permanently-given-up driver is worse than a slowly-retrying one. Already the implemented behavior (`_attempt_reconnect`'s counter resets at the cap rather than stopping) — this question confirms it as intentional, not accidental. *[Robustness: graceful degradation, failure mode count]*

**Q11. Should `_on_driver_error`'s safety-mode-drop (forcing IDLE on sensor failure during an active mode) apply to SCANNING mode too, or only MAPPING/LOCALIZING/NAVIGATING/RECORDING?**
**A**: Yes, include SCANNING — it's already in `ACTIVE_MODES`. A scan built on a failed sensor mid-orbit is just as unsafe to trust as a map or navigation route built the same way; there's no principled reason to exempt it. *[Robustness: graceful degradation; Accuracy: outlier rejection rate — a failed-sensor scan is pure noise]*

**Q12. Should `Scanner3D.max_points` (500,000 cap) degrade gracefully (drop oldest) or hard-stop the scan on hit?**
**A**: Hard-stop with a clear `'capacity'` rejection reason (current behavior) — an orbit scan is a bounded, operator-supervised task, not a continuous stream; silently dropping old points mid-scan would corrupt the geometry an operator is actively watching build, which is worse than a visible stop. *[Robustness: graceful degradation; Accuracy: bias — silent point-dropping introduces sampling bias]*

**Q13. Should the processing-queue drop-oldest policy (`main.py`'s `queue.Full` handler) apply the same way in all modes, or should RECORDING mode never drop frames (waypoints are cumulative, losing one shifts the whole profile)?**
**A**: Same drop-oldest policy everywhere, including RECORDING — the alternative (blocking the UART reader) risks a full driver stall, which loses far more data than one dropped waypoint. `_dropped_frames` is already tracked and exposed; that's the right mitigation (visibility), not mode-specific queue logic that adds complexity for a rare case. *[Robustness: graceful degradation; Maintainability: code clarity — one policy, not N mode-specific ones]*

**Q14. Should map save failures (`map_manager.save_map` returning `False`) trigger an automatic retry, matching `profile_recorder.save_profile`'s retry loop?**
**A**: Yes — this is a real asymmetry. `save_map`'s atomic staging-dir writes have the same transient-I/O-failure exposure `save_profile`'s retry loop was built for; add the same `max_retries` pattern. **Logged as an actionable backlog item** (mechanical, low-risk) rather than implemented here since it touches `app/map_manager.py` and needs its own test pass per `validation_protocol.md`. *[Robustness: failure mode count; Maintainability: test coverage]*

**Q15. Should `DataQualityValidator`'s regime-change counter (`regime_change_after=5`) be configurable per-deployment, or stay a fixed constant?**
**A**: Configurable via `Config.data_quality` (currently hardcoded) — turbid/clear water have genuinely different noise regimes, and 5 consecutive rejections is a reasonable default but not a physically-derived constant; an operator tuning for a specific dive site should be able to adjust it without a code change. *[Generalization: turbidity effects, unknown-object handling]*

**Q16. Should the EKF's `skipped_singular_updates` counter trigger any automated response (e.g. forced re-init), or stay purely observational?**
**A**: Purely observational for now, surfaced via `get_statistics()` (already done). A singular update is rare and self-recovering (the state simply doesn't update that cycle); auto-reinitializing on a threshold risks discarding good accumulated state over a transient numerical blip. Revisit only if field data (D8 P9) shows it happening often enough to matter. *[Robustness: underflow prevention; Scientific: reproducibility — need real data before adding response logic]*

**Q17. Should a watchdog process restart the whole container if the processing thread dies silently, or is `_processing_loop`'s own exception-swallowing sufficient?**
**A**: The exception-swallowing (`try/except` around `_process_reading` inside `_processing_loop`) is sufficient — it already prevents the thread itself from dying on any single reading's failure. A container-level watchdog is out of scope for the app and belongs in the deployment layer (systemd's `Restart=on-failure`, already documented in `docs/ORIN_NANO_SETUP.md`). *[Robustness: graceful degradation; Compatibility: dependency footprint]*

---

## Domain 3: Performance (8 questions)

**Q18. Should the per-poll full-cloud voxel downsample (`21-3`, still open) be fixed via caching, or by moving the computation off the request thread only?**
**A**: Cache the downsampled cloud, invalidate on new accepted scan — moving it off the request thread alone still means every poll (and every client) recomputes redundantly; caching fixes the actual O(n) waste, thread placement alone doesn't. *[Latency: p95/p99 latency, throughput; Resource: CPU utilization]*

**Q19. Should the O(n²)-ish accumulated-cloud growth (`21-4`) be fixed by chunked storage, or by reducing the downsample threshold from 500,000 points?**
**A**: Chunked/fixed-cadence downsampling, not a lower threshold — lowering the threshold trades map fidelity for speed without addressing the underlying per-scan cost of the `+` concatenation; a smaller threshold just makes the same O(n²) pattern hit its downsample floor sooner, still degrading before that point. *[Latency: throughput; Accuracy: repeatability — fidelity shouldn't be the lever pulled to fix an algorithmic complexity problem]*

**Q20. Should the particle-filter's O(particles × map_points) ray-cast (`21-5`) be vectorized with NumPy, or replaced with a KD-tree lookup?**
**A**: KD-tree — `scipy.spatial.cKDTree` is already a project dependency (used in `object_detection.py`'s clustering) and gives O(log n) per query versus NumPy vectorization's still-O(map_points) per particle. Reuses an existing, already-tested dependency rather than adding a new one. *[Latency: median/p95 latency; Compatibility: dependency footprint]*

**Q21. Should `readings_per_second`'s 1-second rate window be shortened for more responsive UI feedback, or kept at 1s for stability?**
**A**: Kept at 1s. A shorter window (e.g. 200ms) would make the displayed rate visibly jittery at the LiDAR's actual ~10Hz cadence — the current window already fixed the opposite bug (H5, raw-count-as-rate); over-correcting toward responsiveness reintroduces instability in a different form. *[Latency: jitter; Maintainability: code clarity]*

**Q22. Should the multipath detector's `refit_every=5` cadence be tuned down for faster adaptation, or up for lower CPU cost?**
**A**: Kept at 5 — P9-sim validation (100% detection, 0% FP) was measured at this cadence; changing it without re-running that validation would invalidate the only empirical evidence this parameter is well-tuned. Revisit only alongside new P9-sim runs, not in isolation. *[Scientific: reproducibility; Latency: throughput]*

**Q23. Should the driver's UART read loop poll at a fixed interval, or stay purely reactive on `in_waiting`?**
**A**: Stay reactive — the existing `time.sleep(0.001)` fallback already caps busy-spin CPU cost when idle, and a fixed poll interval would add latency on the common case (data already waiting) to save nothing, since the reactive check is already cheap. *[Latency: max blockage time; Resource: CPU utilization]*

**Q24. Should Open3D operations (ICP, voxel downsample) be offloaded to GPU on the Orin Nano, given CUDA is available?**
**A**: Not yet — `docs/ORIN_NANO_SETUP.md` already benchmarks CPU-only ICP at 25ms per 100-point scan, well within the 10Hz budget; GPU offload adds a CUDA dependency and complexity for a problem that isn't currently the bottleneck. Revisit only if map sizes grow enough that CPU ICP starts missing the real-time budget. *[Resource: power consumption, code size; Compatibility: dependency footprint]*

**Q25. Should the EKF run on every reading, or only when both position and attitude data are simultaneously fresh?**
**A**: Every reading — `predict()` should run on every cycle regardless of measurement availability (that's what a Kalman filter's predict/update split is for); `update_position()`/`update_attitude()` already independently no-op-skip when their source is stale. Gating the whole EKF on "both fresh" would throw away the position-only fusion benefit whenever MAVLink drops out. *[Accuracy: convergence speed; Robustness: graceful degradation]*

---

## Domain 4: Testing (8 questions)

**Q26. Should the flaky ICP-registration test pattern (fixed in `test_coverage_99.py`'s reorthonormalization test) become a documented anti-pattern, or was it a one-off?**
**A**: Documented — already added to `validation_protocol.md`'s "Anti-patterns this protocol exists to prevent" table. Chained-random-real-ICP-registration is a general trap (the "aperture problem" applies to any lateral-translation test scenario), not specific to that one test; future SLAM/localization tests should default to the "arrange internal state one step from the threshold" pattern instead. *[Maintainability: test coverage; Scientific: reproducibility]*

**Q27. Should mutation testing (`.clauderc` Rule 43) be added to this project's CI, once CI exists?**
**A**: Not initially — mutation testing is valuable but expensive (multiplies test-suite runtime by the number of mutants), and this project's 99% line coverage plus the extensive edge-case/branch tests already written give strong confidence without it. Revisit once a CI budget exists and 99% coverage has been sustained for a while as a "next tier" hardening step. *[Resource: I/O operations, code size; Maintainability: test coverage]*

**Q28. Should the 40 remaining uncovered lines (99% → 100%) be chased, or left as documented exceptions?**
**A**: Left documented (already done in `CORRESPONDENCE_LOG.md` Entry 13 and the coverage commit) — they're the `if __name__ == '__main__':` guard and a few defensive except-branches whose mock setup cost exceeds their value. `.clauderc` Rule 26 itself says "target... not necessarily 100%"; chasing the last 1% here would be optimizing a vanity metric over real risk reduction. *[Maintainability: test coverage; Resource: code size — test complexity has a cost too]*

**Q29. Should the emulation server (`tests/emulation_server.py`) be extended to simulate D1-D8 hardware failure modes (MAVLink dropout, turbidity spikes) for more field-like P9-sim coverage?**
**A**: Yes — this is a genuine gap. `test_p9_simulation.py` already validates steady-state behavior; a dropout/spike-injection mode in the emulator would let P9-sim extend to the adversarial cases P6 already enumerated on paper but never exercised end-to-end. **Logged as a backlog item** (`DEVELOPMENT_BACKLOG.md`), not implemented here — scoped work, not a quick fix. *[Robustness: NTP-step immunity, saturation handling; Scientific: reproducibility]*

**Q30. Should `test_api_routes.py`'s Flask-test-client tests run against a real (not mocked) driver in CI, using the emulation server?**
**A**: No, keep the driver mocked for route tests — route tests exist to verify HTTP/auth/routing contracts, not sensor behavior (that's `test_driver_mock.py`/`test_p9_simulation.py`'s job). Mixing concerns would make route-test failures ambiguous (routing bug vs. emulator timing flake) and slower to diagnose. *[Maintainability: code clarity; Latency: throughput — test suite speed matters for iteration]*

**Q31. Should property-based testing (`.clauderc` Rule 37, e.g. Hypothesis) be added for the physics/math modules (quaternion math, EKF, environmental correction)?**
**A**: Yes, specifically for `app/mavlink_imu.py`'s quaternion round-trip and `app/ekf_3d_attitude.py`'s wrap_angle — these have clean mathematical invariants (unit quaternion norm, angle wrapping to [-π,π)) that property-based testing is a natural fit for, beyond the fixed test cases already written. **Logged as backlog** (adds a new dependency, `hypothesis`, needing its own Compatibility review). *[Scientific: reproducibility, novelty score; Robustness: failure mode count]*

**Q32. Should test execution be parallelized (`pytest-xdist`) given the suite now takes ~45s?**
**A**: Not yet — 45s for 562 tests is still well within comfortable iteration speed; adding `pytest-xdist` introduces its own test-isolation risk (shared fixtures, the `Config.slam` singleton bug from Entry 13 would become much harder to debug under parallel execution) for a problem that doesn't yet exist. Revisit if the suite grows past ~2-3 minutes. *[Latency: throughput; Robustness: failure mode count — parallelism amplifies isolation bugs]*

**Q33. Should the 3 pre-existing dependency-light test files (`test_security.py`, `test_data_quality.py`, `test_driver_mock.py` per `docs/TESTING.md`) still be maintained as a separate "no numpy/Open3D" tier, now that the full suite runs fine together?**
**A**: Yes, keep the tier distinction documented — it exists for exactly the CI scenario this project doesn't have yet (a fast pre-check before the full scientific-stack suite runs), and removing the distinction now would have to be re-added when CI finally lands (Backlog Section 7 priority #1). *[Compatibility: dependency footprint; Resource: I/O operations]*

---

## Domain 5: API Design (8 questions)

**Q34. Should the `/api/v1/*` aliases eventually deprecate the unversioned `/api/*` routes, or stay dual forever?**
**A**: Stay dual — `_register_versioned_aliases()` already documents `/api/v1` as "canonical going forward" while keeping the unversioned path "for backward compatibility." With no evidence of external unversioned-path consumers yet (single-operator dashboard, same origin), there's no forcing function to deprecate, and no cost to keeping both beyond the trivial `add_url_rule` loop. *[Compatibility: backward compatibility; Maintainability: code clarity]*

**Q35. Should scanner/mapping/localization "get points" endpoints support pagination, given `.clauderc` Rule 78 requires it for list-returning endpoints?**
**A**: No — these aren't paginated lists in the REST sense, they're already-bounded point-cloud snapshots (`voxel_size=0.1` downsample for mapping, `max_points=50000` stride for scanner). Rule 78's intent (avoid unbounded response size) is already satisfied by domain-appropriate downsampling/striding, which is the correct mechanism for spatial data — pagination would fragment a single coherent point cloud across requests for no benefit. *[Resource: memory peak; Latency: throughput]*

**Q36. Should the standardized error shape (`{"error": {"code", "message", "details"?}}`) include a request-ID for correlation, closing `24-13`?**
**A**: Yes — add a `before_request`-generated UUID, included in both the error response and the log line. This is a small, low-risk addition that directly closes a documented observability gap (Blind Spot Audit R2 24-13) without touching any business logic. **Logged as backlog** (mechanical, scoped for a dedicated commit + test). *[Maintainability: code clarity; Robustness: failure mode count]*

**Q37. Should `/api/health` return more detail (per-subsystem status) for richer monitoring, or stay minimal for fast polling?**
**A**: Stay minimal — it's polled by Docker's `HEALTHCHECK` and external monitors at a fixed interval; `/api/status` already exists for the detailed view (including the new `sensor_fusion` block). Splitting these two concerns (liveness-check vs. full status) is the right shape; merging them would make the healthcheck slower and its failure mode harder to diagnose (which field regressed vs. a clean 503). *[Latency: median latency; Maintainability: code clarity]*

**Q38. Should DELETE routes (`delete_map`, `delete_profile`) require a confirmation token/second call, given they're irreversible?**
**A**: No — the frontend already gates destructive actions behind a `confirm()` dialog (Blind Spot Audit R2 20-12 notes this should eventually move to an in-page modal, but the confirmation step itself exists), and the API layer requiring a second round-trip would only protect against programmatic misuse, which `@require_auth` already gates. Adding server-side confirmation flow is complexity without a matching threat. *[Maintainability: code clarity; Robustness: failure mode count]*

**Q39. Should scanner/mapping modes expose a WebSocket-only "live points" stream instead of the current polling model, to reduce redundant `/api/*/points` calls?**
**A**: Partially yes — `lidar_reading` is already broadcast via SocketIO; extending that pattern to periodic map/scanner point deltas (not full snapshots) would reduce redundant polling, but this overlaps directly with the still-open `19-13` (coalesce emits, one emitter thread) finding, so implementation should happen together with that fix, not before it. *[Latency: throughput; Resource: I/O operations]*

**Q40. Should the REST API support bulk operations (e.g. delete multiple maps in one call)?**
**A**: No — this is a single-operator dashboard, not a fleet-management console; bulk endpoints add surface area (partial-failure semantics, transactional guarantees) for a use case that doesn't exist yet. Revisit only alongside `21-1`'s multi-ROV registry work, where bulk operations across vehicles would become a genuine need. *[Maintainability: code clarity; Robustness: failure mode count]*

**Q41. Should object-detection classes (`Config.object_detection.classes`) be extensible via API, or stay a fixed config-time list?**
**A**: Stay fixed config-time — the classification logic (`_classify_object`'s confidence formula) isn't a general-purpose classifier that accepts arbitrary new classes at runtime; it's tuned to the six documented categories. Runtime extensibility would need matching classifier logic that doesn't exist, so exposing the knob would be a promise the code can't keep. *[Generalization: unknown-object handling; Scientific: novelty score]*

---

## Domain 6: DevOps (9 questions)

**Q42. Should CI (`16-3`, priority #1 in the backlog) be GitHub Actions or a self-hosted runner?**
**A**: GitHub Actions — the repo already lives on GitHub, needs no new infrastructure, and the dependency-light test tier (Q33) is specifically designed to run fast on a standard hosted runner without special hardware. Self-hosting only makes sense if hardware-in-the-loop testing (real TFmini-S/MAVLink) becomes a CI requirement, which it isn't yet (P9-field is explicitly deferred). *[Compatibility: dependency footprint; Resource: I/O operations]*

**Q43. Should CI run the full scientific-stack suite (Open3D etc.) on every push, or only the dependency-light tier + full suite on PR/merge?**
**A**: Dependency-light tier on every push (fast feedback), full suite gating merge to `main` — this mirrors the existing documented test-tier split (`docs/TESTING.md`) and keeps the common case (iterating on a feature branch) fast while still catching everything before it reaches `main`. *[Latency: throughput; Robustness: failure mode count]*

**Q44. Should `Dockerfile.arm64`'s conflicting version pins (`15-4`) be fixed by deleting the hand-written pins, or by syncing them to `requirements.txt`'s versions?**
**A**: Delete the hand-written pins and install solely from `requirements.txt` — two sources of truth for the same dependency set is the actual defect (already identified); syncing them manually just recreates the same drift risk at the next `requirements.txt` update. One file, one truth. **Logged as backlog** (Dockerfile rewrite, needs a real ARM64 build to verify — can't validate in this x86_64 environment). *[Compatibility: dependency footprint; Maintainability: code clarity]*

**Q45. Should the container's default CMD (`15-1`, crash-loops under `docker compose up -d`) switch to gunicorn, or gate `allow_unsafe_werkzeug` on an explicit env var instead of `DEBUG`?**
**A**: Switch to gunicorn as the container default — `docs/DEPLOYMENT.md` already documents the correct gunicorn invocation (`--workers 1`, `GeventWebSocketWorker`); the Werkzeug dev server was never meant to be the production path, and an env-var escape hatch (`ALLOW_WERKZEUG=true`) would just paper over using the wrong server by default. **Logged as backlog** — needs `gunicorn`/`gevent-websocket` added to `requirements.txt` and the Dockerfile CMD updated together. *[Robustness: graceful degradation; Compatibility: dependency footprint]*

**Q46. Should `Dockerfile.arm64` be split into real multi-stage build/runtime (`15-7`), given it currently ships the full toolchain in the runtime image?**
**A**: Yes, mirroring the root `Dockerfile`'s existing multi-stage pattern — this is a direct image-size and attack-surface reduction (compilers/pytest in a production image is the opposite of the hardening the root `Dockerfile` already achieves) with a clear existing template to copy. **Logged as backlog**, needs a real ARM64 build to verify Open3D's compiled artifacts survive the stage copy. *[Resource: code size, cache footprint; Robustness: failure mode count]*

**Q47. Should the primary `Dockerfile` gain Raspberry-Pi-compatible ARM64 support (`15-6`), given BlueOS's actual deployment target is RPi, not Jetson?**
**A**: Yes — this is arguably the most important open DevOps item, since `docs/DEPLOYMENT.md` currently points BlueOS at an image that can't build for BlueOS's real hardware. Requires either an Open3D binary wheel for RPi ARM64 (may not exist) or a from-source build step — genuinely uncertain which is feasible without testing on real RPi hardware, so this is flagged as **NEEDS-DECISION**, not answered with false confidence. *[Compatibility: dependency footprint; Generalization: benchtop→field transfer]*

**Q48. Should `docker-compose.yml`'s Prometheus/Grafana stanza (referencing a non-existent `prometheus.yml`, per `BLIND_SPOT_AUDIT_R2_FINDINGS.md` p.81) be completed, or removed as aspirational?**
**A**: Removed from `BUILDING.md` as a documented example, replaced with "add your own `prometheus.yml`" guidance — a `/metrics` endpoint doesn't exist yet (`24-8`, itself NEEDS-DECISION), so a working Prometheus example can't be shipped honestly until that endpoint exists. Documenting a stanza that references a file nobody will ever create is worse than omitting it. *[Maintainability: documentation quality]*

**Q49. Should the daily Rule 9 sweep (this session's own cron/Routine) eventually become a CI job instead of a session-scoped Routine?**
**A**: Yes, once CI exists (`16-3`) — a Routine survives container resets (already proven across 4 daily fires) but is still tied to one Claude session's lifecycle in a way a scheduled CI workflow isn't. This is explicitly noted in `DEVELOPMENT_BACKLOG.md` Section 7 as the reasoning behind prioritizing CI. *[Robustness: graceful degradation; Compatibility: dependency footprint]*

**Q50. Should `requirements.txt` pin exact versions everywhere, or use ranges for non-security-sensitive packages to ease future updates?**
**A**: Exact pins everywhere, as already practiced — this project already documents *why* each security-relevant pin is what it is (inline CVE comments); ranges would reintroduce the "surprise minor-version behavior change" risk `.clauderc` Rule 93 warns about, for a marginal convenience (not needing a PR to bump a version) that isn't costly given how infrequently deps actually change here. *[Compatibility: dependency footprint; Robustness: failure mode count]*

---

## Domain 7: Documentation (8 questions)

**Q51. Should `DOCUMENTATION_INDEX.md` be auto-generated from the repo's file list, or hand-maintained?**
**A**: Hand-maintained, as already practiced — the index's value is the curated one-line description and reading-order guidance per doc, which a generator can't produce; the maintenance cost (one line per new doc) is low enough that automation would add complexity to solve a problem that isn't actually painful yet. *[Maintainability: documentation quality; Resource: code size]*

**Q52. Should `PHYSICS_AUDIT.md` (the original 70-finding audit) be merged into `BLIND_SPOT_AUDIT_R2_FINDINGS.md`, now that R2 exists?**
**A**: No, keep them separate — they're genuinely different audit rounds with different methodologies (PHYSICS_AUDIT.md is physics/algorithm-focused, R2 is broader engineering-domain coverage) and merging would obscure the chronology `CORRESPONDENCE_LOG.md` already documents. Cross-reference, don't consolidate. *[Maintainability: documentation quality; Scientific: reproducibility — preserving the actual audit history has value]*

**Q53. Should the 10+ top-level markdown files be reorganized into `docs/`, given the root directory is getting crowded?**
**A**: No — `DOCUMENTATION_INDEX.md`'s existing categorization already solves the discoverability problem the reorganization would target, and moving files breaks every existing cross-reference link across `CORRESPONDENCE_LOG.md`, `DEVELOPMENT_BACKLOG.md`, etc. for a purely cosmetic gain. *[Maintainability: documentation quality; Compatibility: backward compatibility — internal links]*

**Q54. Should `README.md` be updated to mention Rule 8/9 (autonomy nervous system, DFM/DFA protocol), or stay focused on human-operator quick-start?**
**A**: Stay focused on human quick-start — `README.md`'s audience is a human standing up the extension, not an AI agent orienting itself (that's `CLAUDE.md`'s job). Mixing the two audiences in one doc would dilute both. *[Maintainability: documentation quality]*

**Q55. Should algorithm decision records (`docs/ALGORITHM_DECISION_LOG.md`) get a fourth entry for this document's own methodology (99-question format), given Rule 7 already has a template?**
**A**: No — this document's Q&A format is a lighter-weight sibling process (stated in this doc's own header), not a Rule-7-tier decision; giving it a full decision-log entry would misrepresent its rigor level. It's referenced from `DOCUMENTATION_INDEX.md` on its own terms instead. *[Maintainability: documentation quality]*

**Q56. Should `docs/API.md` be regenerated from an OpenAPI spec (tying into `AUTONOMY_HACKS.md`'s "contract before code" hack #70), or stay hand-written?**
**A**: Hand-written for now — introducing OpenAPI tooling is a real investment (spec authoring, generator pipeline, keeping it in sync) that only pays off once the API has external consumers beyond this project's own dashboard. Revisit if a third-party integration or public API commitment ever appears. *[Compatibility: dependency footprint; Maintainability: documentation quality]*

**Q57. Should `RISK_MANAGEMENT.md` (ISO 14971 hazard analysis) be updated to include the D1-D8 deferred decisions' hazards (e.g. MAVLink dropout mid-navigation)?**
**A**: Yes — this is a real gap. The hazard analysis predates D1 (MAVLink attitude fusion) and doesn't yet cover attitude-source-dropout as a distinct hazard from the existing sensor-dropout entries. **Logged as backlog**, not answered inline — a proper ISO 14971 entry needs severity/probability scoring, not a one-line addition. *[Robustness: failure mode count; Scientific: reproducibility]*

**Q58. Should `PATENT.md`/`docs/FTO.md` be revisited given the D1-D8 code additions (MAVLink fusion, multipath detection, EKF) since the original FTO analysis?**
**A**: Yes, before any commercial release (already CLAUDE.md Rule 2's standing trigger) — the EKF/multipath/depth-correction algorithms weren't part of the original FTO's CPC-class search scope. Not re-run here since Rule 2 explicitly gates this on "before any commercial deployment," which hasn't been reached. *[Scientific: novelty score; Compatibility: license compliance]*

---

## Domain 8: Data Quality (8 questions)

**Q59. Should `DataQualityValidator`'s IQR and Z-score filters run independently (either can reject) or require both to agree before rejecting?**
**A**: Independent (either rejects) — as currently implemented. Requiring both to agree would let an outlier that only one method catches through; the two methods exist precisely because they catch different failure shapes (IQR: distributional outliers, Z-score: distance-from-mean outliers), so an AND-gate would weaken the filter to the weaker of the two on any given anomaly. *[Robustness: outlier rejection rate; Accuracy: bias]*

**Q60. Should the multipath detector and the IQR/Z-score data-quality filter be merged into one pipeline stage, or stay sequential/independent?**
**A**: Stay sequential, as currently wired (`_on_lidar_reading`: data_quality first, then multipath) — they detect different physical phenomena (statistical outliers vs. scattered-light returns) with different warm-up requirements and independent enable flags; merging would couple two orthogonal concerns and make the multipath detector's optional nature (default off) harder to reason about. *[Maintainability: code clarity; Robustness: graceful degradation]*

**Q61. Should the data-quality regime-change recovery (clears window after N rejects) log a distinct event for operator visibility, beyond the existing `regime_changes` counter?**
**A**: The counter is sufficient — it's already surfaced via `get_statistics()` and visible in `/api/status`. A distinct WebSocket event would be one more thing the frontend has to handle for a diagnostic-tier signal, not an operator-actionable one; the existing counter-in-status pattern matches how other diagnostic signals (e.g. `dropped_frames`) are already exposed. *[Maintainability: code clarity; Latency: throughput — fewer WS event types]*

**Q62. Should out-of-range readings (beyond `max_range`) be logged individually, or only counted (current behavior)?**
**A**: Counted only — per-reading logging at 10Hz would flood logs exactly as `24-14` already identified for rejected readings generally; the `rejected_by['range']` counter in `get_statistics()` already gives the operator the signal that matters (how often, not which exact reading). *[Resource: I/O operations; Robustness: failure mode count]*

**Q63. Should the environmental correction's depth-staleness timeout (`DEPTH_TIMEOUT_S=2.0`) be shorter, matching MAVLink attitude's 1.0s default?**
**A**: No, keep it at 2.0s — a pressure sensor's read cadence is typically slower than an attitude source's (MAVLink streams at up to 50Hz; a depth sensor like the MS5837 is commonly polled at 1-10Hz), so a matching 1.0s timeout would false-trigger staleness on a healthy but slower sensor. The two timeouts are intentionally different for this reason, not an oversight. *[Generalization: depth dependence; Robustness: graceful degradation]*

**Q64. Should the driver's `invalid_readings` counter (sentinel/saturation) distinguish weak-signal sentinels from saturation sentinels, or stay one combined count?**
**A**: Distinguish them — this is a real gap the current implementation doesn't close. Weak-signal (turbidity/range) and saturation (too-close/too-reflective) point at opposite physical causes and opposite operator responses. **Logged as backlog** (small, mechanical addition to `lidar_driver.py`'s `_parse_frame`). *[Robustness: failure mode count; Accuracy: outlier rejection rate]*

**Q65. Should the multipath detector's bimodality guard (Ashman's D > 2, added per the P9-sim fix) have its threshold configurable, or stay a fixed constant?**
**A**: Fixed constant — 2.0 is a standard, well-cited threshold for "clearly bimodal" in the mixture-modeling literature (not an arbitrarily chosen number), and the P9-sim validation that confirmed 0% false positives was run at this exact value. Making it configurable without new validation data would let an operator silently un-fix the exact bug this threshold closed. *[Scientific: reproducibility, peer review status]*

**Q66. Should object-detection's clustering parameters (`clustering_eps=0.2`, `clustering_min_samples=5`) adapt to point density, or stay fixed?**
**A**: Stay fixed for now — adaptive clustering parameters are a legitimate future improvement but need field data (what does real point density actually look like across dive conditions?) to tune correctly; guessing at an adaptive formula without that data risks replacing one fixed-but-understood behavior with an adaptive-but-untested one. *[Generalization: unknown-object handling; Scientific: reproducibility]*

---

## Domain 9: Concurrency (8 questions)

**Q67. Should the driver's UART read thread and the processing worker thread share a lock, or stay decoupled via the queue (current design)?**
**A**: Stay decoupled via the queue — this is the entire point of the existing architecture (comment in `main.py`: "so a slow pipeline can never back up the UART buffer"); adding a shared lock would reintroduce the coupling the queue was specifically built to remove. *[Latency: max blockage time; Robustness: graceful degradation]*

**Q68. Should `socketio.emit` calls from the serial-error-callback thread and the processing-worker thread be coalesced through one emitter thread, closing `19-13`?**
**A**: Yes — this is a real, already-identified ordering bug (a `lidar_reading` can be delivered after a `safety_alarm` that logically preceded it). **Logged as backlog**, not fixed here: the right shape is a single-consumer emit queue, which is a small but non-trivial architectural change deserving its own commit and test pass, not a quick patch. *[Robustness: failure mode count; Maintainability: code clarity]*

**Q69. Should `_multipath_rejected_count` and similar cross-thread counters use `threading.Lock`, or is Python's GIL-protected int increment sufficient?**
**A**: GIL-protected increment is sufficient for these specific counters — `count += 1` on a simple int is atomic enough under CPython's GIL for monotonically-increasing diagnostic counters where losing an occasional increment under extreme race timing has zero correctness impact (it's a dashboard number, not a control-flow gate). Adding locks here would be defensive theater with a real performance cost on the hot path. *[Resource: CPU utilization; Maintainability: code clarity]*

**Q70. Should `MapManager` gain a per-name lock for save/load/delete (`17-12`, still open), or is the atomic-staging-dir pattern sufficient?**
**A**: Add a per-name lock — the staging-dir pattern (already fixing power-loss corruption) doesn't protect against two concurrent requests for the *same* map name racing each other (one's `delete` between another's `load_map`'s metadata-read and points-read, as `17-12` describes). These are different failure modes needing different fixes. **Logged as backlog** — needs its own test for the race condition, not a quick add. *[Robustness: failure mode count; Accuracy: repeatability]*

**Q71. Should `ProfileRecorder`/`ProfileNavigator` share a lock across recording and navigation happening "simultaneously" (mode switch mid-operation)?**
**A**: No — `LiDARSLAMApplication.set_mode()` already serializes mode transitions (stopping the previous mode's active operation before starting the next), so RECORDING and NAVIGATING can never genuinely run concurrently by the state machine's own design. A lock would protect against a scenario the mode FSM already prevents. *[Maintainability: code clarity; Robustness: failure mode count]*

**Q72. Should the EKF's `predict()`/`update_*()` calls be lock-protected, given `_process_reading` runs on the single processing-worker thread but `get_statistics()` may be read from the Flask request thread concurrently?**
**A**: Yes, this is a genuine unprotected read/write race — `eng.ekf`'s internal `state`/`covariance` numpy arrays can be read via `/api/status` while being mutated by the processing thread. **Logged as backlog** (needs a lock around the EKF's public methods, matching the pattern already used in `SLAMEngine`/`LocalizationEngine`/`ProfileRecorder`, which is the actual precedent to follow here). *[Robustness: failure mode count; Accuracy: repeatability]*

**Q73. Should `MultipathDetector`'s rolling window (a plain `deque`, mutated from the driver-callback thread) be lock-protected the same way?**
**A**: Not currently a real risk — `MultipathDetector.check()` is only ever called from `_on_lidar_reading`, which itself only runs on the single serial-read thread (never concurrently with itself). No cross-thread access exists to protect against, unlike the EKF case in Q72 which genuinely has a Flask-thread reader. *[Maintainability: code clarity — don't add locks without a real race to protect against]*

**Q74. Should the WebSocket connection handler (`on_connect`) validate tokens synchronously (current) or offload to a background check to avoid blocking the SocketIO event loop?**
**A**: Stay synchronous — token validation is a fast in-memory dict lookup (`hash_token()` + `dict.__contains__`), not an I/O-bound operation; offloading it to a background task would add latency and complexity to protect against a cost that's already negligible. *[Latency: median latency; Maintainability: code clarity]*

---

## Domain 10: UX/Frontend (8 questions)

**Q75. Should the CDN-loaded three.js/socket.io (`20-1`, offline breaks the whole dashboard) be vendored locally now, or wait for a broader frontend pass?**
**A**: Vendor now — this is the single highest-impact open UX finding (BlueOS is frequently used on vehicles with no internet access, exactly where a CDN dependency fails hardest) and the fix is mechanical (download + serve locally), not a design decision. **Logged as backlog** priority alongside CI, not deferred to "a broader pass" that may never come. *[Robustness: graceful degradation; Generalization: benchtop→field transfer]*

**Q76. Should touch controls (`20-7`, missing on the 3D view) use a hand-rolled implementation or adopt `THREE.OrbitControls`?**
**A**: Adopt `THREE.OrbitControls` — the existing mouse-control code (`setupControls()`) is a hand-rolled spherical-camera implementation that already duplicates what OrbitControls provides; extending it with touch support by hand would be re-solving a problem the library already solves well, adding maintenance burden for no benefit over adopting the standard tool. *[Maintainability: code clarity; Compatibility: dependency footprint — small addition, high value]*

**Q77. Should the dashboard's accessibility gaps (`20-9`/`20-10`/`20-11`) be fixed incrementally per-finding, or batched into one a11y sweep?**
**A**: Batched — these three findings (tab roles/keyboard nav, color-only status, contrast) are all touching the same CSS/markup surface (`index.html`, `style.css`); fixing them together avoids three separate review passes over overlapping code and lets contrast/color fixes be checked against the same palette decision at once. *[Maintainability: code clarity; Resource: code size — fewer, larger reviewed changes over many small ones here]*

**Q78. Should `alert()`/`confirm()` (`20-12`) be replaced with a custom modal system, or a lightweight existing library?**
**A**: Custom, reusing the existing `showToast()` infrastructure's visual language — the app already has a toast system built for exactly this kind of non-blocking feedback; a confirmation variant of the same component is a small addition, while pulling in a modal library adds a new dependency for a UI pattern (confirm dialog) that's genuinely simple to build once. *[Compatibility: dependency footprint; Maintainability: code clarity]*

**Q79. Should the reverse-proxy base-path issue (`20-2`) be fixed via a build step (inject the base path at deploy time) or runtime detection (`document.baseURI`)?**
**A**: Runtime detection — a build step would require a bundler/build pipeline this project doesn't currently have (the frontend is currently plain, unbundled JS served directly), and `document.baseURI`-based detection works with zero new tooling. Reaching for a build step here would be solving a packaging problem the project doesn't have yet to fix a routing problem it does have. *[Compatibility: dependency footprint; Generalization: benchtop→field transfer — works under any BlueOS mount point without redeployment]*

**Q80. Should the WebSocket reconnect logic (`20-14`... actually `19-14`, giving up after ~40s) retry indefinitely, or cap with a longer timeout and clear "reconnect manually" UI?**
**A**: Retry indefinitely with exponential backoff capped at a sane max interval (e.g. 30s) — an underwater dive can have connectivity gaps longer than 40s (thruster interference, depth changes), and a dead dashboard that never recovers on its own is worse than one that keeps trying quietly in the background with a visible "reconnecting" indicator. *[Robustness: graceful degradation; Generalization: unknown-object handling — unknown gap duration]*

**Q81. Should the 3D visualizer's screenshot feature (`preserveDrawingBuffer`, already fixed) support exporting the raw point cloud (not just a PNG), for offline analysis?**
**A**: Not through the screenshot button — that's a UI-capture feature, not a data-export one; point-cloud export already exists via `map_manager.export_map()`'s multiple formats (ply/pcd/npy/csv/xyz), accessible through the maps panel. Conflating "take a picture" with "export the data" in one button would confuse the two genuinely different use cases. *[Maintainability: code clarity]*

**Q82. Should the dashboard show the D1-D8 fusion-module status (`sensor_fusion` block, already in `/api/status`) in the UI, or is it API-only diagnostic data?**
**A**: API-only for now — none of D1/D2/D3/D4/D8 are enabled by default (`Config` flags all off), so there's currently nothing for an operator to see; adding UI for a feature that's off by default in every real deployment would be speculative work. Add UI once field validation (P9-hardware) makes any of these flags a realistic on-by-default candidate. *[Generalization: unknown-object handling — don't build UI for a state that can't occur yet; Maintainability: code size]*

---

## Domain 11: Patent/IP (8 questions)

**Q83. Should the D2 multipath detector's hand-rolled 2-component EM (chosen over scikit-learn) affect the FTO analysis differently than a library-based implementation would?**
**A**: No material difference — the underlying algorithm (Gaussian mixture EM, Ashman's D bimodality test) is decades-old published statistics, not novel IP regardless of implementation; the choice not to use scikit-learn was a dependency-footprint decision (`CORRESPONDENCE_LOG.md` Entry 9), not a patentability one. `docs/FTO.md`'s CPC-class scope is unaffected by this implementation detail. *[Compatibility: license compliance; Scientific: novelty score]*

**Q84. Should the EKF's Joseph-form covariance update be flagged as a novel contribution in `PATENT.md`, or treated as standard practice?**
**A**: Standard practice, not novel — the Joseph form is a well-known numerical-stability technique in the Kalman filtering literature (cited to Bar-Shalom et al. 2001 in the code's own docstring); using a textbook-standard numerically-stable formulation instead of the naive one isn't inventive, it's just correct engineering. *[Scientific: novelty score, peer review status]*

**Q85. Should the depth-dependent refractive index polynomial model (D3) be checked against existing underwater LiDAR patents before field deployment?**
**A**: Yes, as part of the standing Rule 2 FTO gate (before any commercial deployment) — but not before, since D3's P9-field hasn't happened yet and the model's actual fitted coefficients (currently placeholder `b=c=0`) don't exist to search against. Searching a not-yet-calibrated model would be premature. *[Compatibility: license compliance; Scientific: reproducibility]*

**Q86. Should the ENU-vs-NED coordinate handling (the D1 3D beam projection's NED-in, ENU-out design) be documented as project-specific know-how, or is it standard robotics practice worth citing externally?**
**A**: Standard practice worth citing, not proprietary know-how — REP 103 (ROS coordinate conventions) and general aerospace navigation literature already establish these conventions; this project's contribution is applying them correctly to this specific sensor stack (a real bug fix, per Physics Audit C4), not inventing a new convention. Already appropriately documented with citations in code comments rather than treated as a trade secret. *[Scientific: novelty score; Maintainability: documentation quality]*

**Q87. Should the atomic map-save pattern (staging-dir + fsync + os.replace) be considered for a defensive publication, given it closes a real class of bug?**
**A**: No — this is a well-established filesystem durability pattern (write-to-temp, fsync, atomic rename), documented in POSIX filesystem programming guidance for decades; it's good engineering practice correctly applied, not novel IP. *[Scientific: novelty score]*

**Q88. Should the project's dependency license audit (`.clauderc` Rule 93) extend to checking `Dockerfile.arm64`'s NVIDIA base image's licensing terms?**
**A**: Yes — `nvcr.io/nvidia/l4t-pytorch` carries NVIDIA's own container license terms (distinct from the open-source packages inside it), which hasn't been explicitly reviewed in this project's documentation. **Logged as backlog**, a licensing-review task, not answered with an assumption here. *[Compatibility: license compliance]*

**Q89. Should PATENT.md track the D1-D8 modules as separate potential filings, or as one combined "sensor fusion suite" filing scope?**
**A**: This is a business/legal decision outside engineering scope — not answered here. Flagged as **NEEDS-DECISION** for whoever owns IP strategy for this project; engineering's role (already fulfilled) is documenting the technical novelty/prior-art landscape (`docs/FTO.md`), not deciding filing strategy. *[Scientific: novelty score — informs, doesn't decide, the filing question]*

---

## Domain 12: Compliance (9 questions)

**Q90. Should the D1-D8 hazards (per Q57) be added to `RISK_MANAGEMENT.md` before or after P9-field validation?**
**A**: Before — ISO 14971 hazard analysis is meant to precede deployment, not follow it; a hazard like "MAVLink attitude source drops mid-navigation, beam projection silently reverts to 1D heading" is analyzable and controllable *today*, using the already-implemented graceful-degradation behavior as the design control, independent of whether P9-field has run yet. *[Robustness: failure mode count; Scientific: reproducibility]*

**Q91. Should the laser-safety classification (TFmini-S, already a hazard entry) be re-verified given the new depth-dependent refractive correction (D3) changes reported distances?**
**A**: No re-verification needed — D3 corrects the *reported range value*, not the sensor's actual emitted optical power or beam characteristics; the laser safety classification is a hardware property of the TFmini-S module itself, entirely unaffected by downstream software distance corrections. *[Generalization: depth dependence — affects measurement, not emission]*

**Q92. Should IEC 60945 (maritime equipment environmental testing, referenced in `get_health()`'s temperature envelope check) be extended to cover the Orin Nano companion computer, or only the LiDAR sensor?**
**A**: Extend the analysis to note it, but the actual envelope check stays sensor-focused — the TFmini-S's operating temperature is a hard physical limit affecting measurement validity (hence the existing `get_health()` check); the Orin Nano's thermal behavior is a platform reliability concern better handled by its own thermal management/monitoring, not conflated with sensor data-quality logic. *[Robustness: failure mode count; Maintainability: code clarity]*

**Q93. Should the multipath detector's turbidity handling be validated against any regulatory water-quality standard, or is P9-sim's synthetic validation sufficient for compliance purposes?**
**A**: P9-sim is sufficient for engineering validation, but not for a compliance claim — no regulatory standard governs "LiDAR multipath rejection accuracy in turbid water" as such; this is a performance/reliability characteristic, not a certifiable safety parameter. Compliance documentation should describe it as validated-in-simulation, field-pending, honestly (already the stated status), not overclaim regulatory conformance that doesn't apply. *[Scientific: reproducibility; Robustness: turbidity effects]*

**Q94. Should the depth-dependent correction (D3) be cross-checked against a certified depth reference for any eventual commercial certification?**
**A**: Yes, this is exactly what D3's P9-field phase (calibration dives with reference markers) is scoped to produce — not a new requirement, but confirmation that the existing planned validation step (`TECHNICAL_SPECIFICATION.md` D3-P9) already covers this need once hardware access exists. *[Scientific: reproducibility; Generalization: depth dependence]*

**Q95. Should this project maintain a formal Software Bill of Materials (SBOM), given `.clauderc` Rule 93's dependency-audit requirement?**
**A**: Not yet as a separate artifact — `requirements.txt`'s existing exact-pin-plus-CVE-comment practice (Q50) already substantively serves the same purpose (know exactly what's in the build, know why). A formal SBOM (e.g. CycloneDX/SPDX format) is a natural addition once this project has an external compliance audience requiring machine-readable format, not before. *[Compatibility: license compliance; Maintainability: code size]*

**Q96. Should the EKF/multipath/depth-correction modules' "field-pending" status be a hard blocker on enabling them in a production BlueOS deployment, or can an operator opt in with informed risk acceptance?**
**A**: Opt-in with informed risk acceptance — the feature flags already default off (safe), and `TECHNICAL_SPECIFICATION.md`'s phase-status table already documents exactly what "P9-sim done, P9-field pending" means; an operator who reads that and chooses to enable D1/D2/D3/D4/D8 anyway (e.g. for a low-stakes test dive) is making an informed choice the software shouldn't paternalistically block. *[Robustness: graceful degradation; Scientific: reproducibility — the honesty of the documented status is what makes opt-in acceptable]*

**Q97. Should `.clauderc`'s duplicate upload (`clauderc.md`, uploaded via GitHub web UI, merged in Entry-14-adjacent commit) be reconciled with `.clauderc`, or left as two files?**
**A**: Left as two files for now, flagged here rather than silently resolved — `clauderc.md` was uploaded directly by the repo owner outside this session's own edits; unilaterally deleting or merging someone else's deliberate upload without confirmation would violate this session's own standing discipline (investigate before overwriting user-originated changes). **NEEDS-DECISION** from the user: keep both, or explicitly authorize consolidating into `.clauderc` alone. *[Maintainability: documentation quality — genuinely ambiguous without user input, correctly deferred rather than guessed]*

**Q98. Should the daily Rule 9 sweep's no-op log entries (4 and counting in `state_journal.md`) be pruned/archived once the count grows large, per `AUTONOMY_HACKS.md`'s context-density guidance?**
**A**: Yes, once the log exceeds roughly 30 entries (a month of daily no-ops) — `state_journal.md`'s own "Older Entries" section already documents this exact archival pattern (`docs/STATE_JOURNAL_ARCHIVE.md`) for the Latest Entry; the Daily Rule 9 Sweep Log section should follow the same discipline once it's no longer a handful of lines. Not yet necessary at 4 entries. *[Resource: cache footprint — context window cost of reading a bloated journal]*

**Q99. Should this document itself (99 Q&A) be revisited/updated as answers get implemented, or treated as a point-in-time snapshot?**
**A**: Point-in-time snapshot, with implementation status tracked in `DEVELOPMENT_BACKLOG.md` instead — updating 99 answers in place every time one gets implemented would turn this document into a second, redundant backlog tracker. This document's value is the *decision*, made once; `DEVELOPMENT_BACKLOG.md` is the *execution* tracker, and duplicating status between them would create exactly the kind of drift `.clauderc` Rule 8 warns against. *[Maintainability: documentation quality; Resource: code size]*

---

## Summary

| Domain | Questions | Answered inline | Logged as new backlog items | NEEDS-DECISION (genuinely open) |
|---|---|---|---|---|
| Security | 9 | 9 | 0 | 0 |
| Reliability | 8 | 7 | 1 (Q14) | 0 |
| Performance | 8 | 8 | 0 | 0 |
| Testing | 8 | 6 | 2 (Q29, Q31) | 0 |
| API Design | 8 | 7 | 1 (Q36) | 0 |
| DevOps | 9 | 6 | 3 (Q44, Q45, Q46) | 1 (Q47) |
| Documentation | 8 | 7 | 1 (Q57) | 0 |
| Data Quality | 8 | 7 | 1 (Q64) | 0 |
| Concurrency | 8 | 5 | 3 (Q68, Q70, Q72) | 0 |
| UX/Frontend | 8 | 7 | 1 (Q75) | 0 |
| Patent/IP | 8 | 7 | 1 (Q88) | 1 (Q89) |
| Compliance | 9 | 8 | 0 | 1 (Q97) |
| **Total** | **99** | **84** | **14** | **3** |

**Genuinely undecided (3)**: Q47 (RPi ARM64 buildability — needs real hardware to answer honestly), Q89 (patent filing strategy — a business decision, not engineering's to make), Q97 (`clauderc.md` vs `.clauderc` duplication — needs explicit user authorization to touch someone else's upload).

**New backlog items surfaced (14)**: see individual answers above for scope; none implemented in this document — each needs its own `validation_protocol.md` pass (tests, coverage check) per this project's own discipline, not a bundled drive-by fix.
