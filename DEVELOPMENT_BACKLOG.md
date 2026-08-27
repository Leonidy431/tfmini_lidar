# Development Backlog (Разработческий бэклог)

**Project**: BlueOS LiDAR SLAM Navigation System (BLSNS)  
**Updated**: 2026-08-27 (Blind Spot Audit Round 3 + operator-debt self-assessment, см. `CORRESPONDENCE_LOG.md` Entry 16/17; this section previously read 2024-01-15 and was stale by several sprints — see `BLIND_SPOT_AUDIT_R3_FINDINGS.md` for how the audit rounds surface this exact class of drift)  
**Owner**: Engineering Team  
**Status**: Active Development

---

## Executive Summary

Backlog содержит 6 категорий работ (originally 4; Sections 7-8 added since):
1. **Blind Spot Audit Round 2** (all 24 domains complete — Section 1's per-domain plan is historical, see `BLIND_SPOT_AUDIT_R2_FINDINGS.md` for the actual results)
2. **Deferred Decisions (D1-D8)** из Physics Audit — **P7-P11 код реализован для D1/D2/D3/D4/D8**, P9 field calibration заблокирована на аппаратуре (P9-sim done for all four, see `TECHNICAL_SPECIFICATION.md`)
3. **Technical Debt & Enhancements** (low-priority improvements)
4. **Correspondence Log Integration + Documentation Index + Continuous Blind-Spot Monitoring** (Sections 4-6)
5. **Engineering Discipline Baseline (.clauderc) + Coverage Milestone** (Section 7 — DONE, coverage driven 71%→99%)
6. **Blind Spot Audit Round 3** (Section 8 — full 12-domain Rule 1 sweep, 96 findings, 28 fixed, 68 logged, 1 NEEDS-DECISION)

**Текущий статус (2026-08-27)**:
- ✅ `app/mavlink_imu.py`, `app/multipath_detector.py`, `app/environmental_correction.py`, `app/ekf_3d_attitude.py` реализованы, протестированы, подключены в `app/main.py` за feature-флагами (по умолчанию выключены); D1 was silently never started until R3-REL-1 fixed it (see Section 8)
- ✅ 582/582 тестов проходят стабильно, 99% coverage (3175 statements, 44 missing)
- ✅ CI pipeline live (`.github/workflows/ci.yml`) — was the standing #1 priority since Round 2, closed in Round 3
- ⏳ P9 field calibration for all four D1-D8 decisions still blocked on hardware (MAVLink IMU, turbidity tank, depth pool, oven)
- 🔴 **NEEDS-DECISION, unresolved**: `PATENT.md`'s claim disclosure appears to be in this public GitHub repo (R3-IP-1) — a business/legal call, not yet actioned

**Рекомендуемый фокус**:
- Section 8's architectural clusters (pipeline ordering, detector permanent-freeze) — highest engineering-value remaining item, needs a dedicated session
- R3-IP-1 needs the operator's explicit decision before further Patent/IP work
- Field validation D1/D2/D3/D4/D8 as soon as hardware is available (P8-P9 phases)

---

## Section 1: Blind Spot Audit Round 2

**Status**: ✅ ALL 10 DOMAINS AUDITED (15–24) — see `BLIND_SPOT_AUDIT_R2_FINDINGS.md`: **84 findings total** (44 in wave 1 + 40 in wave 2), **19 mechanical fixes applied + committed** (12 + 7), remainder logged as NEEDS-DECISION with severity + rationale. Full suite 290/290. Round 2 audit complete; the logged NEEDS-DECISION items feed the "Next-session priorities" list in the findings doc.  
**Specialist Domains Completed (14/24)**:
- ✅ Security (OWASP, injection, auth)
- ✅ Reliability (error handling, fault tolerance)
- ✅ Performance (memory, CPU, real-time constraints)
- ✅ Testing (coverage gaps, edge cases)
- ✅ API Design (REST conventions)
- ✅ DevOps (Docker, deployment)
- ✅ Documentation (missing/outdated docs)
- ✅ Data Quality (sensor validation, outliers)
- ✅ Concurrency (race conditions, thread safety)
- ✅ UX/Frontend (visualization, WebSocket)
- ✅ Patent/IP (prior art, licensing)
- ✅ Compliance (maritime safety, ISO 14971)
- ✅ Physics/Mathematics (coordinate frames, numerical stability)
- ✅ System Architecture (modularity, coupling)

**Remaining Domains (10/24)**:
1. **Deployment & DevOps (Extended)** — containerization, health checks, monitoring
2. **CI/CD Pipeline** — automated testing, build, deployment
3. **Database & Persistence** — map/profile storage, backup strategy
4. **Security Hardening** — secrets management, rate limiting edge cases
5. **Network & Comm** — WebSocket reliability, latency, packet loss handling
6. **User Experience** — dashboard UX, 3D visualization, accessibility
7. **Scalability** — multi-ROV swarms, cloud integration
8. **Underwater Domain-Specific** — sonar/acoustic interference, pressure ratings, salinity
9. **Hardware Integration** — sensor calibration, firmware updates
10. **Observability & Telemetry** — logging, metrics collection, alerting

### Audit Execution Plan

**Workflow**: Multi-agent 12-specialist audit (parallel execution)

```bash
# Execute Round 2 (10 domains × 12 specialists = 120 audit threads)
# Each specialist produces 8-9 findings → ~100 total findings expected
# Severity breakdown: ~20 critical, ~40 high, ~40 medium

python -m app.main --audit-round-2 --domains 15-24
```

**Expected Output**:
- 100 additional findings (blind spots)
- Severity distribution: 20C + 40H + 40M
- Time to complete: 2–4 hours (parallel agents)
- Documentation: PHYSICS_AUDIT.md (updated section for Round 2)

**Timeline**: Week 1 of Sprint 1

### Blind Spot Audit Tasks

| Task | Domain | Type | Effort | Priority |
|------|--------|------|--------|----------|
| BS-15 | Deployment & DevOps | AUDIT | 2h | HIGH |
| BS-16 | CI/CD Pipeline | AUDIT | 2h | HIGH |
| BS-17 | Database & Persistence | AUDIT | 2h | HIGH |
| BS-18 | Security Hardening | AUDIT | 2h | HIGH |
| BS-19 | Network & Comm | AUDIT | 2h | HIGH |
| BS-20 | UX & Dashboard | AUDIT | 2h | MEDIUM |
| BS-21 | Scalability | AUDIT | 2h | MEDIUM |
| BS-22 | Underwater Domain | AUDIT | 2h | MEDIUM |
| BS-23 | Hardware Integration | AUDIT | 2h | MEDIUM |
| BS-24 | Observability | AUDIT | 2h | MEDIUM |

---

## Section 2: Deferred Decisions (D1-D8)

**Dependency Chain**:
```
D1 (MAVLink) ─→ D8 (3D-EKF)
       ↓
D3 (Depth) ─→ D4 (Temp)
       ↓
D2 (Multipath)
       ↓
D5, D6, D7 (specialized)
```

### Sprint 1 (Weeks 1-3): Critical Path D1 + D2

#### Task D1-01: MAVLink 3D Attitude (Phase P1-P3)

| Item | Details |
|------|---------|
| **Task ID** | D1-01 |
| **Title** | MAVLink 3D Attitude Integration (Scoping + Literature) |
| **Phase** | P1-Scoping, P2-Literature, P3-Synthesis |
| **Effort** | 3 days |
| **Assignee** | Controls Engineer |
| **Status** | ✅ DONE (see TECHNICAL_SPECIFICATION.md D1, ALGORITHM_DECISION_LOG.md) |
| **Dependency** | MAVLink library availability |

**Subtasks**:
- [ ] D1-P1: Finalize requirements (roll/pitch/yaw, gimbal lock handling, sync)
- [ ] D1-P2: Collect 50+ papers (Thrun, Diebel, PX4 docs, robotics quaternions)
- [ ] D1-P3: Synthesize 300 approach variants (Euler/Quaternion/RotMatrix/DualQuat)
- [ ] D1-P3: Create comparison matrix (48 parameters × 4 approaches)

**Definition of Done**:
- Requirements doc in TECHNICAL_SPECIFICATION.md (DONE)
- 50+ paper references with annotations
- Approach matrix with scoring
- P1-P3 review meeting with 4+ specialists

---

#### Task D1-02: MAVLink 3D Attitude (Phase P4-P5)

| Item | Details |
|------|---------|
| **Task ID** | D1-02 |
| **Title** | MAVLink 3D Attitude (Specialist Voting + Decision) |
| **Phase** | P4-Evaluation, P5-Ensemble |
| **Effort** | 2 days |
| **Assignee** | Controls Engineer + Specialist Panel (12 experts) |
| **Status** | ✅ DONE — 98% consensus (11/11), quaternion representation selected |

**Subtasks**:
- [ ] D1-P4: 12-specialist panel voting (32-expert matrix per Rule 7)
- [ ] D1-P4: Each expert scores top-10 candidates on 48 params
- [ ] D1-P5: Aggregate votes (ranked-choice), compute confidence
- [ ] D1-P5: Decision record in ALGORITHM_DECISION_LOG.md

**Definition of Done**:
- Voting matrix completed (12 × 10 × 48)
- Top-3 candidates ranked with confidence > 85%
- Decision rationale documented
- Risk/trade-off analysis (if any)

---

#### Task D1-03: MAVLink 3D Attitude (Phase P6-P7)

| Item | Details |
|------|---------|
| **Task ID** | D1-03 |
| **Title** | MAVLink 3D Attitude (Adversarial + Prototyping) |
| **Phase** | P6-Adversarial, P7-Prototyping |
| **Effort** | 4 days |
| **Assignee** | Software Engineer (real-time systems) |
| **Status** | ✅ CODE DELIVERED (see CORRESPONDENCE_LOG.md Entry 9) |

**Subtasks**:
- [x] D1-P6: Stress-test gimbal lock (pitch=90°), clock skew, MAVLink timeout — `test_gimbal_lock_pitch_90_does_not_raise`, `test_stale_sample_returns_none`
- [x] D1-P6: Document failure modes and mitigation — docstrings in `app/mavlink_imu.py`
- [x] D1-P7: Implement prototype (pymavlink + quaternion library) — `app/mavlink_imu.py` (`MAVLinkAttitudeReader`, pure-NumPy quaternion math, `pymavlink` optional at runtime)
- [ ] D1-P7: Benchmark: latency (<10ms), memory (<1KB), throughput (50 Hz) — not yet measured on target hardware (Orin Nano)
- [x] D1-P7: Create test suite (unit tests for rotation math) — `tests/test_mavlink_imu.py` (20 tests)

**Definition of Done**:
- [x] Prototype code in `app/mavlink_imu.py`
- [x] All stress tests PASS
- [ ] Benchmarks documented (pending target-hardware run)
- [x] Unit test coverage > 90% (20 unit + 4 integration tests)

---

#### Task D1-04: MAVLink 3D Attitude (Phase P8-P12)

| Item | Details |
|------|---------|
| **Task ID** | D1-04 |
| **Title** | MAVLink 3D Attitude (Integration + Field Validation) |
| **Phase** | P8-P12 |
| **Effort** | 5 days |
| **Assignee** | Integration Engineer + Field Engineer |
| **Status** | 🟡 PARTIAL — P10/P11/P12 done, P8/P9 blocked on hardware |
| **Dependency** | Prototype from D1-03 |

**Subtasks**:
- [ ] D1-P8: Ablation study (hyperparameters: sync threshold, quaternion smoothing) — needs real MAVLink stream to tune against
- [ ] D1-P9: Lab calibration (rotation matrix validation) — needs IMU + known-angle fixture
- [x] D1-P10: Integration into SLAM (SE(3) pose composition) — `app/main.py::_project_beam`, `_compute_3d_beam_offset`
- [x] D1-P10: Feature flags (ENABLE_MAVLINK_3D_ATTITUDE) — `Config.mavlink_attitude` in `app/config.py`
- [x] D1-P11: Integration tests (SLAM with/without 3D attitude) — `tests/test_deferred_decisions_integration.py::TestBeam3DProjection` (4 tests)
- [x] D1-P12: Documentation (algorithm doc + code comments linking to papers) — docstrings cite Diebel (2006), Beard & McLain (2012)
- [ ] D1-P12: Field trials (2 dives with known pitch/roll angles) — hardware-blocked

**Definition of Done**:
- [ ] All 12 phases documented in ALGORITHM_DECISION_LOG.md (P8/P9 pending hardware)
- [x] 100% test pass rate (unit + integration) — 258/258
- [ ] RMSE improvement > 15% on angled approaches (measured in field) — needs field data
- [x] Graceful fallback to 1D heading if MAVLink unavailable — verified by `test_project_beam_falls_back_when_attitude_stale`

**Estimated Timeline**: Weeks 1-2 of Sprint 1 (code delivered ahead of schedule; P8/P9 remain gated on hardware access)

---

#### Task D2-01: Multipath Detection (Phase P1-P5)

| Item | Details |
|------|---------|
| **Task ID** | D2-01 |
| **Title** | Multipath Detection in Turbid Water (Scoping + Decision) |
| **Phase** | P1-P5 |
| **Effort** | 3 days |
| **Assignee** | Signal Processing Engineer |
| **Status** | ✅ DONE — 91% consensus (12/12), 2-component Gaussian mixture selected |

**Subtasks**:
- [ ] D2-P1: Finalize turbidity model (multipath vs. direct light)
- [ ] D2-P2: Literature search (mixture models, turbidity effects, Carpenter 1999)
- [ ] D2-P3: Synthesize approaches (GMM, thresholding, temporal consistency)
- [ ] D2-P4-P5: Specialist voting (Numerics + Algorithms + Physics domain experts)

**Definition of Done**:
- Approach matrix (300 variants × 48 parameters)
- Specialist consensus > 85%
- Top-3 approaches ranked

---

#### Task D2-02: Multipath Detection (Phase P6-P7)

| Item | Details |
|------|---------|
| **Task ID** | D2-02 |
| **Title** | Multipath Detection (Prototyping + Benchmarking) |
| **Phase** | P6-P7 |
| **Effort** | 5 days |
| **Assignee** | ML/Signal Processing Engineer |
| **Status** | ✅ CODE DELIVERED (see CORRESPONDENCE_LOG.md Entry 9) |

**Subtasks**:
- [x] D2-P6: Stress-test (total extinction, all particles scattered, false positives) — `test_short_range_but_strong_signal_not_flagged` (false-positive control)
- [x] D2-P7: Implement Mixture-of-Gaussians detector — `app/multipath_detector.py`. **Deviation from spec**: hand-rolled 2-component 1D EM in plain NumPy instead of `scikit-learn.GaussianMixture`, to avoid a new dependency for a well-conditioned 1D special case (see CORRESPONDENCE_LOG.md Entry 9, decision #1)
- [ ] D2-P7: Benchmark: latency (<2ms per reading), memory (<2KB), 100-sample warm-up — not yet measured on target hardware
- [x] D2-P7: Validation: 92%+ detection rate on synthetic multipath — `test_scattered_low_signal_flagged`, `test_separates_two_clear_clusters`

**Definition of Done**:
- [x] `app/multipath_detector.py` implemented
- [ ] Latency + memory budgets met (pending target-hardware benchmark)
- [x] 90%+ detection rate on test data — 11 unit + 2 integration tests, all passing

---

### Sprint 2 (Weeks 3-5): D3 + D4 Calibration

#### Task D3-01: Depth-Dependent Refractive (Phase P1-P5)

| Item | Details |
|------|---------|
| **Task ID** | D3-01 |
| **Title** | Depth-Dependent n(z) Correction (Scoping + Specialist Decision) |
| **Phase** | P1-P5 |
| **Effort** | 2 days |
| **Assignee** | Physics Engineer |
| **Status** | ✅ DONE (see TECHNICAL_SPECIFICATION.md D3) |

**Subtasks**:
- [ ] D3-P1: Finalize polynomial model order (linear vs. quadratic)
- [ ] D3-P2: Literature (Austin & Halikas 1976, seawater refractive index)
- [ ] D3-P3-P5: Specialist decision (Physics + Numerics domain)

---

#### Task D3-02: Depth Calibration

| Item | Details |
|------|---------|
| **Task ID** | D3-02 |
| **Title** | Depth-Dependent Calibration (Lab + Field) |
| **Phase** | P7-P9 |
| **Effort** | 3 days (lab) + 2 days (field dives) |
| **Assignee** | Field Engineer + Physicist |
| **Status** | 🟡 PARTIAL — P7/P11 code delivered, P9 blocked on hardware |
| **Dependency** | Calibration pool/tank access |

**Subtasks**:
- [x] D3-P7: Prototype polynomial fit — `app/environmental_correction.py::DepthCorrectedRefractive`
- [ ] D3-P9: Lab calibration: measure distances at 0m, 5m, 10m, 20m, 50m depths — needs pool/tank access
- [x] D3-P9: Fit n(depth) = a + b*z + c*z² (least-squares helper ready) — `EnvironmentalCorrector.calibrate_depth_model()`, unit-tested against a known polynomial (`test_calibrate_depth_model_recovers_known_polynomial`); awaiting real (depth, n) pairs to fit against
- [ ] D3-P9: Field validation: 3 ROV dives at varying depths — hardware-blocked
- [x] D3-P11: Unit tests for depth-corrected distances — `tests/test_environmental_correction.py` (16 tests), `tests/test_deferred_decisions_integration.py::TestEnvironmentalCorrectionIntegration` (3 tests)

**Definition of Done**:
- [ ] Polynomial coefficients extracted (a, b) — default (b=c=0) is a verified no-op pending real data
- [ ] Field RMSE improvement > 10% on deep dives — needs field data
- [x] Temperature-corrected distances validated (unit level) — see D4-01

---

#### Task D4-01: Temperature Correction (Similar to D3, 3 days total)

| Item | Details |
|------|---------|
| **Task ID** | D4-01 |
| **Title** | Temperature-Compensated Distance (Oven Calibration) |
| **Phase** | P1-P9 |
| **Effort** | 3 days |
| **Assignee** | Physics Engineer |
| **Status** | 🟡 PARTIAL — P1-P7/P11 code delivered, P9 blocked on hardware |
| **Dependency** | Lab oven or water bath |

**Subtasks**:
- [x] P7 code: `app/environmental_correction.py::TemperatureCorrection` (linear model, ref_temp_c=20.0 default, no-op at reference temperature)
- [x] P11 tests: `tests/test_environmental_correction.py::TestTemperatureCorrection` (3 tests), integration test with `Config.environmental_correction.temperature_enabled` gate
- [ ] Oven calibration: 0°C, 10°C, 15°C, 20°C (ref), 25°C, 30°C — hardware-blocked
- [ ] Fit linear model: coefficient = m*temp + b — placeholder slope (0.0005/°C) in place pending real calibration
- [ ] Field validation: temperature range on real ROV dives — hardware-blocked

---

### Sprint 3 (Weeks 5-8): D8 EKF Fusion

#### Task D8-01: 3D-Attitude EKF (Phase P1-P7)

| Item | Details |
|------|---------|
| **Task ID** | D8-01 |
| **Title** | 3D-Attitude EKF (Design + Prototyping) |
| **Phase** | P1-P7 |
| **Effort** | 5 days |
| **Assignee** | Controls Engineer |
| **Status** | ✅ CODE DELIVERED (see CORRESPONDENCE_LOG.md Entry 9) |
| **Dependency** | D1 (MAVLink) complete — ✅ satisfied (`app/mavlink_imu.py`) |

**Subtasks**:
- [x] D8-P1: Define EKF state (9-DOF: [x,y,z,roll,pitch,yaw,vx,vy,vz]) — `app/ekf_3d_attitude.py`
- [x] D8-P2: Literature (Bar-Shalom 2001, Beard & McLain 2012) — cited in module docstring
- [x] D8-P3: Synthesis — linear KF core selected (process model is linear; only the angular innovation is nonlinear/wrapped), simpler than full EKF/UKF for this state, documented rationale in module docstring
- [ ] D8-P4-P5: Specialist voting (Controls + Numerics experts) — recorded narratively in TECHNICAL_SPECIFICATION.md D8, not re-run as a formal panel this session
- [x] D8-P6: Adversarial tests (singularities, covariance explosion, numerical stability) — `test_singular_innovation_covariance_does_not_crash`, `test_covariance_stays_positive_semidefinite` (200-iteration stress test)
- [x] D8-P7: Implement prototype (NumPy-based EKF) — `app/ekf_3d_attitude.py` (~230 LOC)

**Definition of Done**:
- [x] EKF implementation in `app/ekf_3d_attitude.py`
- [ ] Latency < 5ms per update (50 Hz) — not yet measured on target hardware
- [x] Covariance matrix stability verified — Joseph-form update, PSD-preserving (see CORRESPONDENCE_LOG.md Entry 9, decision #3)
- [x] Unit tests for state propagation + measurement update — 21 unit + 3 integration tests

---

#### Task D8-02: EKF Integration & Validation

| Item | Details |
|------|---------|
| **Task ID** | D8-02 |
| **Title** | EKF Integration + Field Validation |
| **Phase** | P8-P12 |
| **Effort** | 4 days |
| **Assignee** | Integration Engineer + Field Engineer |
| **Status** | 🟡 PARTIAL — P10/P11/P12 done, P8/P9 blocked on hardware |

**Subtasks**:
- [ ] D8-P8: Ablation (process noise, measurement noise, filter gain) — needs real trajectory data to tune against
- [ ] D8-P9: Calibration dives (measure EKF RMSE vs. manual ground truth) — hardware-blocked
- [x] D8-P10: Integration (fuse LiDAR + MAVLink IMU in main loop) — `app/main.py::_update_ekf`, called from `_process_reading`
- [x] D8-P11: Regression tests (no degradation for systems without IMU) — `ekf=None` by default, `TestDefaultsDisabled` confirms unmodified behavior; 258/258 full suite passing
- [x] D8-P12: Documentation (algorithm doc, Kalman filter math) — module docstring cites Bar-Shalom (2001), Beard & McLain (2012); Joseph-form rationale documented inline

**Definition of Done**:
- [ ] Full P1-P12 HLD documented — P8/P9 pending hardware
- [ ] RMSE improvement > 15% on full-6DOF trajectories — needs field data
- [ ] Field validation on 5+ dives — hardware-blocked
- [x] Graceful fallback to 3-DOF position-only if IMU unavailable — `update_position()` works independently of `update_attitude()`; `_update_ekf()` only calls the latter when `mavlink_attitude.get_attitude()` is fresh

---

### Sprint 4 (Low Priority): D5, D6, D7

| Task | Title | Effort | Priority | Status |
|------|-------|--------|----------|--------|
| D5-01 | Vibration filtering (accelerometer) | 2 days | LOW | BACKLOG |
| D6-01 | Velocity profile modeling (multi-sensor) | 3 days | LOW | BACKLOG |
| D7-01 | Viscosity tuning (CFD/tank) | 2 days | LOW | BACKLOG |

---

## Section 3: Technical Debt & Enhancements

### Category A: Bug Fixes (Critical)

| ID | Title | Severity | Effort | Status |
|----|-------|----------|--------|--------|
| BUG-001 | Web UI connection timeout (5s > latency) | CRITICAL | 1 day | BACKLOG |
| BUG-002 | WebSocket message ordering race condition | CRITICAL | 2 days | BACKLOG |
| BUG-003 | Dependency vulnerability (PyJWT) | CRITICAL | 0.5 days | TODO |

### Category B: Enhancements (High)

| ID | Title | Impact | Effort | Status |
|----|-------|--------|--------|--------|
| ENH-001 | Health dashboard (Prometheus metrics) | HIGH | 3 days | BACKLOG |
| ENH-002 | Offline map replay & debugging | HIGH | 2 days | BACKLOG |
| ENH-003 | Multi-ROV coordination (swarm) | HIGH | 8 days | BACKLOG |
| ENH-004 | CloudSync: map/profile backup to cloud | HIGH | 4 days | BACKLOG |

### Category C: Documentation (Medium)

| ID | Title | Effort | Status |
|----|-------|--------|--------|
| DOC-001 | Field deployment guide | 1 day | BACKLOG |
| DOC-002 | Troubleshooting runbook | 1 day | BACKLOG |
| DOC-003 | Hardware integration guide | 2 days | BACKLOG |
| DOC-004 | API reference (OpenAPI/Swagger) | 1 day | BACKLOG |

---

## Resource Allocation

### Headcount & Calendar

**Total Team**: 5–6 FTE  
**Timeline**: 8 weeks (2 months)

| Role | Sprint 1 | Sprint 2 | Sprint 3 | Sprint 4 |
|------|----------|----------|----------|----------|
| Controls Eng (D1, D8) | 60% | 20% | 80% | 0% |
| Signal Proc Eng (D2) | 80% | 0% | 0% | 0% |
| Physics Eng (D3, D4) | 0% | 100% | 0% | 0% |
| Software Eng (Integration) | 40% | 30% | 30% | 10% |
| Field Engineer (Validation) | 20% | 50% | 20% | 0% |
| QA/Test Engineer (All) | 40% | 40% | 40% | 20% |

---

## Risk Register

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| MAVLink hardware unavailable | Medium | High | Prototype with simulator (pymavlink mock) |
| Turbidity meter not available | Low | Medium | Empirical training without direct turbidity measurement |
| Field scheduling conflicts | Medium | Medium | Book ROV dives 4 weeks in advance |
| Numerical instability in EKF | Low | High | Extensive unit tests, reference implementations (EKF-SLAM) |
| Scope creep (D5, D6, D7 pressure) | High | Medium | Strict backlog prioritization, deferred decision policy |

---

## Success Criteria

✅ **Audit Round 2**: 10/10 domains audited, 100+ findings identified  
✅ **D1-D2**: Implementation complete, field validated, >85% specialist consensus  
✅ **D3-D4**: Calibration coefficients extracted, >10% RMSE improvement  
✅ **D8**: EKF fused estimates, >15% RMSE improvement on 6-DOF trajectories  
✅ **Tests**: 250+/250+ passing (175 from physics audit + 75 new)  
✅ **Documentation**: All 12-phase HLDs complete with paper citations  
✅ **Field Validation**: 15+ ROV dives, ground-truth measured  

---

## Next Steps

**Week 1**:
1. Execute Blind Spot Audit Round 2 (domains 15–24)
2. Start D1-01 (MAVLink scoping + literature)
3. Start D2-01 (Multipath scoping + literature)
4. Confirm hardware dependencies (MAVLink, depth sensor, turbidity meter)

**Week 2-3**:
1. Complete D1-02, D2-02 (specialist voting + prototyping)
2. Finalize D1-03, D2-03 implementations
3. Begin D3-01 (depth calibration planning)

**Week 3-5**:
1. D3/D4 lab calibration
2. Field trials for D1/D2 validation
3. D8-01 EKF design

**Week 6-8**:
1. D8 integration + field validation
2. Regression testing (all 250+ tests)
3. PR review + deployment readiness

---

## Appendix: Velocity Tracking

**Past Velocity** (Physics Audit):
- 70 findings identified in 2 weeks
- 27 unit tests added in 1 week
- 3 major decisions fully HLD'd in 2 weeks
- **Velocity**: ~2000 LOC + documentation per week

**Projected Velocity** (D1-D8 Implementation):
- 4 major decisions (D1, D2, D3, D8) following full HLD
- ~200 LOC per decision (implementation + tests)
- ~500 lines documentation per decision
- Field validation: 2–3 days per major decision
- **Expected**: 8 weeks for Sprints 1–3

---

## Metrics Dashboard

**Health Check** (weekly):
- [ ] Blind spot audit progress (domains completed)
- [ ] D1-D8 implementation (% phase completion)
- [ ] Test pass rate (should stay 100%)
- [ ] No critical bugs introduced
- [ ] Field schedule on track

**Monthly Review**:
- Velocity trend (LOC/week)
- Specialist consensus on decisions (target: >85%)
- Risk status (any blockers?)
- Budget utilization (FTE hours)

---

## Section 4: Correspondence Log Integration

Per-request history and the engineering decisions made while implementing D1/D2/D3/D4/D8 code (Entry 9) now live in `CORRESPONDENCE_LOG.md`, cross-referenced from the task statuses above rather than duplicated inline. Read it for:
- Why the D2 mixture model is hand-rolled NumPy instead of scikit-learn
- The NED->ENU derivation behind the D1 3D beam projection and why it's not a naive quaternion rotation of the ENU vector
- Why the D8 EKF uses the Joseph-form covariance update
- The root cause and fix for a flaky multipath integration test
- Entry 10-11: Blind Spot Audit R2, all 10 domains (15-24), 84 findings, 19 mechanical fixes applied — see `BLIND_SPOT_AUDIT_R2_FINDINGS.md` for the full table and the NEEDS-DECISION backlog
- Entry 12-13: the mid-session container-reset incident (branch reverted to the original designated branch, 9 uncommitted coverage-test files lost and recreated), and the coverage push from 71% to 99% (3120 statements, 40 missing) — including two real test-isolation bugs the push surfaced (`Config.slam` shared-singleton leak, an Open3D object-aliasing bug)
- Entry 14: the autonomous-agent nervous system — `state_journal.md`, `validation_protocol.md`, `.claudeignore`, `context_map.json`, `AUTONOMY_HACKS.md`, wired into `CLAUDE.md` as Rule 8. Read `AUTONOMY_HACKS.md` for the 99-lifehack reference; read `validation_protocol.md`'s "Anti-patterns this protocol exists to prevent" table for how each of the incidents above maps to an enforceable check going forward.

---

## Section 5: Documentation Index

`DOCUMENTATION_INDEX.md` (repo root) is the single entry point into all project documentation — audit reports, the Rule 7 HLD framework, per-decision records, the technical spec, this backlog, sprint workplans, hardware setup guides, and the correspondence log. Update it whenever a new top-level `.md` file is added to the repo.

---

## Section 6: Continuous Blind-Spot Monitoring

**Status**: LAPSED — the session-scoped cron job described below was lost when the execution container was reset (see Section 7). `CronCreate` jobs are session-scoped by design and do not survive a container restart; this was always documented as the mechanism's limitation, and it materialized. If continuous monitoring is still wanted, it needs re-scheduling in a live session, or — better, given it already lapsed once — promoting to actual CI (Domain 16 in `BLIND_SPOT_AUDIT_R2_PLAN.md`), which is now also priority #1 in the NEEDS-DECISION list precisely because nothing else catches this class of gap.

Original design (for reference / re-scheduling): a recurring check every ~2 hours looking for newly-introduced blind spots (in the D1/D2/D3-D4/D8 code, and a rotating slice of the wider codebase) and closing the safe/mechanical ones directly, following the same Rule 1 severity triage used for the original 70-finding audit. Auto-fix policy: only mechanical, low-risk fixes applied and committed; anything requiring a design decision or hardware data logged instead of guessed at.

---

## Section 7: Engineering Discipline Baseline (.clauderc) + Coverage Milestone

**Status**: DONE

`.clauderc` (repo root) adds 99 numbered engineering-discipline rules — codename "Посох" — imported into every session via `@.clauderc` at the top of `CLAUDE.md`. Six sections: architecture (1-10), adversarial code review (11-25), 95%+ test coverage discipline (26-50), security/OWASP (51-70), performance (71-85), process/documentation (86-99). Rule 99 is the binding law: no code without 95%+ coverage, blind spots, or ТЗ divergence may ship. It composes with, rather than replaces, this repo's existing Rule 1 (Blind Spot Audit) and Rule 7 (12-phase HLD).

Acting on Rule 99 immediately: overall `app/` test coverage was driven from **71% → 99%** (3120 statements, 40 missing), closing essentially every remaining branch across every module — see `CORRESPONDENCE_LOG.md` Entry 13 for the full methodology, including two real test-isolation bugs the coverage push surfaced and fixed (a shared-singleton config leak in `SLAMEngine()`, and an Open3D object-aliasing bug that was silently corrupting an ICP registration target in a *test*, not production code, but worth knowing the failure mode of). Full suite: 562/562 tests, stable across repeated runs.

The 40 remaining uncovered lines are the `if __name__ == '__main__':` entry guard, near-duplicate error-handler lines, and a few defensive except branches whose mock setup cost would exceed their value — logged here rather than chased further, per the same cost/benefit judgment .clauderc Rule 26 implies ("target... not necessarily 100%").

---

## Section 8: Blind Spot Audit Round 3 (full 12-domain Rule 1 sweep)

**Status**: 28/96 findings fixed this session (mechanical, regression-tested); 68 logged below. Full findings table with severity, exact file:line, and status per finding: `BLIND_SPOT_AUDIT_R3_FINDINGS.md`.

**CI pipeline (Section 6/7's standing #1 priority) is now DONE**: `.github/workflows/ci.yml` runs pytest+coverage-gate (95% floor) on every push/PR, plus a full `docker build` of the primary `Dockerfile` and a syntax-only `docker buildx build --check` of `Dockerfile.arm64` (this is exactly the class of check that would have caught R2 15-2's EXPOSE parse bug before it shipped). Section 6's "Continuous Blind-Spot Monitoring LAPSED" note is superseded — CI is the durable replacement it recommended.

### NEEDS-DECISION — requires explicit user/business authorization

- **R3-IP-1 (CRITICAL)**: `PATENT.md`'s full claim-style disclosure appears to be pushed to a **public** GitHub repo (`Leonidy431/tfmini_lidar`). This risks destroying trade-secret status and triggering patent bar dates in absolute-novelty jurisdictions (EP/CN per `docs/FTO.md`). This was flagged to the user directly when found; no unilateral action was taken (repo visibility, provisional filing, and PATENT.md content are all business/legal decisions outside engineering scope). **Action needed**: verify repo visibility; if public, either file a provisional before any applicable bar date or formally reclassify as trade-secret-only.
- R3-IP-2/3/4: PATENT.md claims, docs/FTO.md CPC classes, and Rule 7's P2-Literature phase all need to account for D1/D2/D3-D4 — logged pending the R3-IP-1 decision above (no point re-drafting claims before knowing the disclosure posture).

### Architectural (need a dedicated session, not a one-line patch)

- **Pipeline ordering** (R3-PERF-1 CRITICAL + R3-DQ-4): `data_quality`/`multipath_detector` filtering runs on the serial-read thread itself (violates the documented enqueue-only real-time contract) AND on the pre-environmental-correction signal (SLAM consumes a different, corrected value than what the filters actually judged). These need to be fixed together — moving only the thread would just relocate the bug.
- **Detector permanent-freeze** (R3-DQ-6 + R3-TEST-4): `multipath_detector` can get stuck in a 100%-reject steady state with no escape, unlike `data_quality.py`'s existing `regime_change_after` unlock. Extract that pattern into something both modules share, rather than a per-module patch.

### Remaining mechanical/logged items by domain (see `BLIND_SPOT_AUDIT_R3_FINDINGS.md` for exact file:line + fix)

Reliability (7): R3-REL-2/3/4/5/6/7/8 — connect() leak on exception, no try/except in `get_single_reading`, wall-clock vs monotonic staleness check, MAVLink reconnect/backoff, MAVLink connect timeout, stale-window reset on reconnect, quaternion renormalization order.
Security (4): R3-SEC-3/4/5/6 — WS event rate limiting, `/api/status` sensor_fusion exposure, unused token permission scoping, inert `check_auth` backstop.
Performance (4): R3-PERF-2/3/4/8 — object-detection O(n) scan, per-reading full-window stat recompute, synchronous `socketio.emit` on the pipeline thread, duplicate `get_attitude()` lock acquisition.
Testing (5): R3-TEST-4/5/6/7/8 — multipath freeze-state test, calibration edge-case tests, malformed-type API body tests, flaky fixed-sleep test, unseeded-ICP test.
API Design (7): R3-API-1 through 8 — DELETE route naming, error-shape consistency, destructive duplicate-POST resets, list pagination, WS protocol doc gaps, WS versioning, radius clamping, success-envelope consistency.
DevOps (2): R3-DEVOPS-3/4/6/7/8 — compose resource limits, compose log driver, image tag versioning, apt pinning, dead gunicorn env vars.
Documentation (3): R3-DOC-2/3/6/7/8 — Scanner API doc section, D1-D8 CONFIGURATION.md entries, missing docstrings.
Data Quality (5): R3-DQ-1/2/3/5/7/8 — frame resync edge cases, checksum-resync accidental-lock-on, regime-change consistency check, dual refractive-index config vars, `set_depth` unit/range validation, `get_single_reading` bypassing counters.
Concurrency (2): R3-CONC-6/7/8 — unlocked `get_health` field reads, `multipath_detector` stats race (currently benign), non-atomic scanner save.
Compliance (4): R3-COMP-1/2/3/7/8 — combined-staleness escalation, EKF-divergence health signal, `attitude_3d_lost` health reason, per-hazard test citations, D1-D8 operational-controls documentation.
Patent/IP (1): R3-IP-8 — dead `asyncio-mqtt`/`websocket-client` dependencies.
UX/Frontend (6): R3-UX-1/2/3/4/5/7/8 — point-cloud buffer rebuild, unthrottled DOM writes, focus-outline removal, missing `aria-pressed`, missing `role="progressbar"`, missing `withLoading()` wrapping, false-negative loading state.

**Next-session priority**: the pipeline-ordering cluster (R3-PERF-1/R3-DQ-4) — it's the highest-value remaining item, affecting both real-time correctness and data quality simultaneously, in code already shipped and enabled by default.
