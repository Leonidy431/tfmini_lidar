# Development Backlog (Разработческий бэклог)

**Project**: BlueOS LiDAR SLAM Navigation System (BLSNS)  
**Updated**: 2024-01-15  
**Owner**: Engineering Team  
**Status**: Active Development

---

## Executive Summary

Backlog содержит 3 категории работ:
1. **Blind Spot Audit Round 2** (14/24 domains complete → 10 remaining)
2. **Deferred Decisions (D1-D8)** из Physics Audit (hardware-dependent)
3. **Technical Debt & Enhancements** (low-priority improvements)

**Рекомендуемый фокус на следующие 2 месяца**:
- Sprint 1: D1 (MAVLink) + D2 (Multipath) + Blind Spot Audit domains 15–19
- Sprint 2: D3/D4 (Calibration) + domains 20–24
- Sprint 3: D8 (EKF fusion) + regression testing + field validation

---

## Section 1: Blind Spot Audit Round 2

**Status**: PLANNING  
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
| **Status** | TODO |
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
| **Status** | TODO |

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
| **Status** | TODO |

**Subtasks**:
- [ ] D1-P6: Stress-test gimbal lock (pitch=90°), clock skew, MAVLink timeout
- [ ] D1-P6: Document failure modes and mitigation
- [ ] D1-P7: Implement prototype (pymavlink + quaternion library)
- [ ] D1-P7: Benchmark: latency (<10ms), memory (<1KB), throughput (50 Hz)
- [ ] D1-P7: Create test suite (unit tests for rotation math)

**Definition of Done**:
- Prototype code in `app/mavlink_imu.py`
- All stress tests PASS
- Benchmarks documented
- Unit test coverage > 90%

---

#### Task D1-04: MAVLink 3D Attitude (Phase P8-P12)

| Item | Details |
|------|---------|
| **Task ID** | D1-04 |
| **Title** | MAVLink 3D Attitude (Integration + Field Validation) |
| **Phase** | P8-P12 |
| **Effort** | 5 days |
| **Assignee** | Integration Engineer + Field Engineer |
| **Status** | TODO |
| **Dependency** | Prototype from D1-03 |

**Subtasks**:
- [ ] D1-P8: Ablation study (hyperparameters: sync threshold, quaternion smoothing)
- [ ] D1-P9: Lab calibration (rotation matrix validation)
- [ ] D1-P10: Integration into SLAM (SE(3) pose composition)
- [ ] D1-P10: Feature flags (ENABLE_MAVLINK_3D_ATTITUDE)
- [ ] D1-P11: Integration tests (SLAM with/without 3D attitude)
- [ ] D1-P12: Documentation (algorithm doc + code comments linking to papers)
- [ ] D1-P12: Field trials (2 dives with known pitch/roll angles)

**Definition of Done**:
- All 12 phases documented in ALGORITHM_DECISION_LOG.md
- 100% test pass rate (unit + integration)
- RMSE improvement > 15% on angled approaches (measured in field)
- Graceful fallback to 1D heading if MAVLink unavailable

**Estimated Timeline**: Weeks 1-2 of Sprint 1

---

#### Task D2-01: Multipath Detection (Phase P1-P5)

| Item | Details |
|------|---------|
| **Task ID** | D2-01 |
| **Title** | Multipath Detection in Turbid Water (Scoping + Decision) |
| **Phase** | P1-P5 |
| **Effort** | 3 days |
| **Assignee** | Signal Processing Engineer |
| **Status** | TODO |

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
| **Status** | TODO |

**Subtasks**:
- [ ] D2-P6: Stress-test (total extinction, all particles scattered, false positives)
- [ ] D2-P7: Implement Mixture-of-Gaussians detector (scikit-learn)
- [ ] D2-P7: Benchmark: latency (<2ms per reading), memory (<2KB), 100-sample warm-up
- [ ] D2-P7: Validation: 92%+ detection rate on synthetic multipath

**Definition of Done**:
- `app/multipath_detector.py` implemented
- Latency + memory budgets met
- 90%+ detection rate on test data

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
| **Status** | TODO |

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
| **Status** | TODO |
| **Dependency** | Calibration pool/tank access |

**Subtasks**:
- [ ] D3-P7: Prototype polynomial fit
- [ ] D3-P9: Lab calibration: measure distances at 0m, 5m, 10m, 20m, 50m depths
- [ ] D3-P9: Fit n(depth) = 1.333 + a*z + b*z²
- [ ] D3-P9: Field validation: 3 ROV dives at varying depths
- [ ] D3-P11: Unit tests for depth-corrected distances

**Definition of Done**:
- Polynomial coefficients extracted (a, b)
- Field RMSE improvement > 10% on deep dives
- Temperature-corrected distances validated

---

#### Task D4-01: Temperature Correction (Similar to D3, 3 days total)

| Item | Details |
|------|---------|
| **Task ID** | D4-01 |
| **Title** | Temperature-Compensated Distance (Oven Calibration) |
| **Phase** | P1-P9 |
| **Effort** | 3 days |
| **Assignee** | Physics Engineer |
| **Status** | TODO |
| **Dependency** | Lab oven or water bath |

**Subtasks**:
- [ ] Oven calibration: 0°C, 10°C, 15°C, 20°C (ref), 25°C, 30°C
- [ ] Fit linear model: coefficient = m*temp + b
- [ ] Field validation: temperature range on real ROV dives

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
| **Status** | TODO |
| **Dependency** | D1 (MAVLink) complete |

**Subtasks**:
- [ ] D8-P1: Define EKF state (9-DOF: [x,y,z,r,p,y,vx,vy,vz])
- [ ] D8-P2: Literature (Bar-Shalom 2001, Beard & McLain 2012, EKF SLAM papers)
- [ ] D8-P3: Synthesis (extended/unscented Kalman filters, factored/information filters)
- [ ] D8-P4-P5: Specialist voting (Controls + Numerics experts)
- [ ] D8-P6: Adversarial tests (singularities, covariance explosion, numerical stability)
- [ ] D8-P7: Implement prototype (NumPy-based EKF, ~500 LOC)

**Definition of Done**:
- EKF implementation in `app/ekf_3d_attitude.py`
- Latency < 5ms per update (50 Hz)
- Covariance matrix stability verified
- Unit tests for state propagation + measurement update

---

#### Task D8-02: EKF Integration & Validation

| Item | Details |
|------|---------|
| **Task ID** | D8-02 |
| **Title** | EKF Integration + Field Validation |
| **Phase** | P8-P12 |
| **Effort** | 4 days |
| **Assignee** | Integration Engineer + Field Engineer |
| **Status** | TODO |

**Subtasks**:
- [ ] D8-P8: Ablation (process noise, measurement noise, filter gain)
- [ ] D8-P9: Calibration dives (measure EKF RMSE vs. manual ground truth)
- [ ] D8-P10: Integration (fuse LiDAR + MAVLink IMU in main loop)
- [ ] D8-P11: Regression tests (no degradation for systems without IMU)
- [ ] D8-P12: Documentation (algorithm doc, Kalman filter math)

**Definition of Done**:
- Full P1-P12 HLD documented
- RMSE improvement > 15% on full-6DOF trajectories
- Field validation on 5+ dives
- Graceful fallback to 3-DOF position-only if IMU unavailable

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
