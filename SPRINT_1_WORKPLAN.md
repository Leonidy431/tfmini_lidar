# Sprint 1 Work Plan (Weeks 1-3)

**Duration**: 3 weeks (21 days)  
**Team Size**: 6 FTE (Controls Eng, Signal Proc, Software Eng, Field Eng, QA, PM)  
**Focus**: D1 (MAVLink 3D) + D2 (Multipath) + Blind Spot Audit R2 (domains 15-19)  
**Target**: All P1-P5 phases complete, ready for prototyping (P7)

---

## Week 1: Scoping + Literature (Days 1-5)

### D1-01: MAVLink 3D Attitude (Scoping + P1)

**Assignee**: Controls Engineer (40%), PM (20%)  
**Duration**: 2 days  
**Status**: TODO

#### Subtasks

**D1-P1.1**: Finalize requirements document
- [ ] Define heading vector vs 3D attitude (roll/pitch/yaw)
- [ ] Specify gimbal lock handling (quaternion format)
- [ ] Latency requirement: <10ms per update
- [ ] Synchronization tolerance: ±100ms (LiDAR @ 10 Hz)
- [ ] Graceful fallback: 1D heading if MAVLink unavailable
- [ ] Document failure modes:
  - MAVLink timeout (no message > 1s)
  - Clock skew (timestamp desync)
  - Quaternion NaN/Inf
  - Gimbal lock (pitch = ±90°)

**Output**: `docs/D1_REQUIREMENTS.md` (500 words)

**D1-P1.2**: Identify success metrics
- [ ] RMSE: <15% improvement on angled approaches vs 1D heading
- [ ] Latency: p99 < 10ms
- [ ] Zero divergence in SE(3) pose composition
- [ ] 100% test coverage (unit + integration)
- [ ] Graceful degradation (zero crashes if MAVLink unavailable)

**Output**: Metrics table in `docs/D1_REQUIREMENTS.md`

---

### D1-02: MAVLink 3D Literature Search (P2)

**Assignee**: Research Specialist / ML Engineer (60%)  
**Duration**: 2 days  
**Status**: TODO

#### Subtasks

**D1-P2.1**: Search academic sources
- [ ] PubMed: Search for "UAV attitude estimation" + "quaternion"
- [ ] IEEE Xplore: "3D pose estimation" + "underwater navigation"
- [ ] arXiv.org: "SLAM" + "IMU" + "sensor fusion"
- [ ] Google Scholar: "gimbal lock" + "quaternion kinematics"
- [ ] GitHub: PX4 Autopilot codebase (https://github.com/PX4/PX4-Autopilot)

**Target**: 50+ papers

**D1-P2.2**: Annotate bibliography
For each paper, extract:
- [ ] Problem solved
- [ ] Approach used (Euler/Quaternion/RotMatrix/DualQuat)
- [ ] Test conditions (indoor/outdoor/underwater)
- [ ] RMSE reported
- [ ] Latency/computational cost
- [ ] License/reproducibility

**D1-P2.3**: Categorize approaches
- [ ] Euler angle representation (pros/cons table)
- [ ] Quaternion representation (advantages summary)
- [ ] Rotation matrix with orthonormalization
- [ ] Dual quaternion (full SE(3) pose)
- [ ] Extended Kalman Filter (EKF) + IMU fusion
- [ ] Unscented Kalman Filter (UKF) variant

**Output**: `docs/D1_LITERATURE_REVIEW.md` (1000+ words, 50+ references)

**D1-P2.4**: Extract taxonomy
- [ ] 300 parameterized variants (different sensor configs, filter types)
- [ ] Sensitivity analysis: What changes with sampling rate?
- [ ] Robustness: How to handle gimbal lock?
- [ ] Integration complexity: How to interface with SLAM?

**Output**: Approach taxonomy table in literature review

---

### D2-01: Multipath Detection (Scoping + P1)

**Assignee**: Signal Processing Engineer (40%)  
**Duration**: 2 days  
**Status**: TODO

#### Subtasks

**D2-P1.1**: Define multipath problem
- [ ] Underwater scattering: particles reflect light
- [ ] Multiple echoes: direct path + scattered paths
- [ ] Manifests as: Low signal + distance jitter ±5-10cm
- [ ] Failure mode: SLAM outliers, navigation divergence

**D2-P1.2**: Requirements
- [ ] Detection rate: >90% multipath in turbidity 2-5 NTU
- [ ] False positive rate: <5% in clear water
- [ ] Latency: <2ms per reading (must not slow real-time pipeline)
- [ ] Memory: <2KB per detector instance
- [ ] Graceful degradation: Continue mapping if multipath detector unavailable

**Output**: `docs/D2_REQUIREMENTS.md` (400 words)

---

### Blind Spot Audit R2 Domains 15-19 (P1 Planning)

**Assignee**: PM + Lead Specialists (20% each)  
**Duration**: 1 day  
**Status**: TODO

#### Subtasks

**BS-15: Deployment & DevOps (Extended)**
- [ ] Define scope: Container orchestration, health checks, monitoring
- [ ] Assign 12 specialist team members
- [ ] Prepare audit checklist

**BS-16: CI/CD Pipeline**
- [ ] Build automation, test gating, deployment stages
- [ ] 12 specialist reviewers scheduled

**BS-17: Database & Persistence**
- [ ] Map/profile/object storage, backup strategy
- [ ] Query optimization, consistency

**BS-18: Security Hardening**
- [ ] Secrets management (API tokens, credentials)
- [ ] Rate limiting edge cases, DOS protection

**BS-19: Network & Communication**
- [ ] WebSocket reliability under packet loss
- [ ] Latency optimization, connection retry logic

**Output**: Blind Spot Audit R2 scope document

---

## Week 1 Deliverables

| Task | Deliverable | Owner | Status |
|------|-------------|-------|--------|
| D1-P1.1 | Requirements doc | Controls Eng | TODO |
| D1-P1.2 | Metrics table | Controls Eng | TODO |
| D1-P2.1 | 50+ papers | Research | TODO |
| D1-P2.2 | Annotated bibliography | Research | TODO |
| D1-P2.3-P2.4 | Taxonomy + literature review | Research | TODO |
| D2-P1.1-P1.2 | Multipath requirements | Signal Proc | TODO |
| BS-R2 Scope | Audit planning | PM + Leads | TODO |

**Quality Gate**: All deliverables peer-reviewed before Week 2

---

## Week 2: Synthesis + Specialist Prep (Days 6-10)

### D1-03: ENU Heading vs 3D Attitude (P3 Synthesis)

**Assignee**: Theorist / Controls Engineer (60%)  
**Duration**: 2 days  
**Status**: TODO

#### Subtasks

**D1-P3.1**: Synthesize 300 approach variants
- [ ] Euler angles (roll/pitch/yaw): 12 conventions (ZYX, XYZ, etc.)
- [ ] Quaternion (x,y,z,w): 4 variants (scalar-last, scalar-first, etc.)
- [ ] Rotation matrix (3×3): 5 representation styles
- [ ] Dual quaternion: 3 parameterizations
- [ ] Sensor fusion: EKF vs UKF vs particle filter (6 variants each)
- [ ] Sampling rates: 10 Hz, 50 Hz, 100 Hz variants
- [ ] Gimbal lock handling: 5 approaches per representation

**Total**: 12×4×5×3×6×10×5 = ~108,000 but reduce via independence → ~300 unique

**D1-P3.2**: Create 48-parameter evaluation matrix
```
Dimensions: 300 approaches × 48 parameters

Parameters (organized by category):
├─ Accuracy (8)
│  ├─ RMS error on reference roll/pitch
│  ├─ Max error (worst case)
│  ├─ Drift over time (10 minute mission)
│  ├─ Convergence speed
│  ├─ Repeatability (std dev)
│  ├─ Bias (systematic offset)
│  ├─ Noise immunity
│  └─ Gimbal lock recovery
├─ Latency (5)
│  ├─ Median latency (50th percentile)
│  ├─ p95 latency
│  ├─ p99 latency
│  ├─ Jitter (variance)
│  └─ Maximum blockage time
├─ Resource (6)
│  ├─ Memory peak
│  ├─ Memory average
│  ├─ CPU utilization
│  ├─ Cache footprint
│  ├─ Code size
│  └─ I/O operations
├─ Robustness (8)
│  ├─ Noise tolerance (white noise, clipping)
│  ├─ Multipath handling
│  ├─ Clock skew immunity
│  ├─ Quaternion NaN recovery
│  ├─ Gimbal lock handling
│  ├─ Saturation handling
│  ├─ Graceful degradation
│  └─ Failure mode count
├─ Generalization (5)
│  ├─ Sim-to-real transfer
│  ├─ Depth dependence (0-100m)
│  ├─ Temperature sensitivity
│  ├─ Age/calibration drift
│  └─ Unknown sensor handling
├─ Compatibility (4)
│  ├─ Backward compatibility
│  ├─ Python version requirements
│  ├─ Dependency footprint
│  └─ License compliance
├─ Maintainability (3)
│  ├─ Code clarity
│  ├─ Test coverage
│  └─ Documentation quality
└─ Scientific (3)
   ├─ Peer review status
   ├─ Reproducibility
   └─ Novelty score
```

**Output**: CSV matrix: `docs/D1_APPROACH_MATRIX.csv` (300 rows × 48 cols)

**D1-P3.3**: Scoring
- [ ] Each cell scored 0-10 (0=worst, 10=best)
- [ ] Preliminary scoring by 3 domain experts (backup scoring)
- [ ] Normalize and identify top-10 candidates for specialist voting

**Output**: Ranked candidates list (top 10) in synthesis document

---

### D2-02: Multipath Approach Synthesis (P3)

**Assignee**: Signal Processing Engineer + ML Engineer (60%)  
**Duration**: 1.5 days  
**Status**: TODO

#### Subtasks

**D2-P3.1**: Synthesize approaches
- [ ] Thresholding (signal < X): 10 threshold variants
- [ ] Variance-based: Fixed window, adaptive window (5 variants)
- [ ] Mixture-of-Gaussians: 2-component, 3-component (3 variants)
- [ ] Temporal consistency: 1-scan history, 2-scan, 3-scan (3 variants)
- [ ] Hybrid: Threshold + variance (4 combinations)

**Total**: ~15 unique approaches

**D2-P3.2**: Create matrix and score
- [ ] Accuracy (detection rate, false positive rate)
- [ ] Latency (computational cost)
- [ ] Memory (model size)
- [ ] Calibration complexity (tuning difficulty)
- [ ] Generalization (how sensitive to environment?)

**Output**: `docs/D2_APPROACH_MATRIX.md` (15 approaches × 20 parameters)

---

### Specialist Panel Preparation

**Assignee**: PM (100%)  
**Duration**: 0.5 days  
**Status**: TODO

#### Subtasks

**D1 Voting Preparation**:
- [ ] Invite 32 specialists (4 per domain, 8 domains)
- [ ] Send pre-read: D1_REQUIREMENTS.md + D1_APPROACH_MATRIX.csv
- [ ] Explain 48-parameter matrix and scoring methodology
- [ ] Schedule voting sessions (staggered if needed)
- [ ] Prepare scoring sheet template

**D2 Voting Preparation**:
- [ ] Invite 12 specialists (Numerics, Algorithms, Physics, Software)
- [ ] Send pre-read: D2_REQUIREMENTS.md + D2_APPROACH_MATRIX.md
- [ ] Schedule voting sessions

**Output**: Voting packets sent, confirmations received

---

## Week 2 Deliverables

| Task | Deliverable | Status |
|------|-------------|--------|
| D1-P3 | Approach matrix (300×48) | TODO |
| D1-P3 | Top-10 candidates ranked | TODO |
| D2-P3 | Approach matrix (15×20) | TODO |
| Specialist prep | Voting packets sent | TODO |

---

## Week 3: Specialist Voting + Adversarial (Days 11-15)

### D1-04: Specialist Voting (P4-P5)

**Assignee**: PM + Panel Moderator (100%)  
**Duration**: 2 days  
**Status**: TODO

#### Subtasks

**D1-P4.1**: Panel voting (4 hours per specialist)
- [ ] 32 specialists score top-10 approaches on 48 parameters
- [ ] Each expert rates approaches 0-10 per parameter
- [ ] Total: 32 experts × 10 approaches × 48 params = 15,360 scores

**D1-P4.2**: Aggregate voting (P5)
- [ ] Compile raw scores into matrix
- [ ] Calculate mean + std dev per approach × parameter
- [ ] Apply domain weights:
  - Physics: 30% (most important)
  - Algorithms: 20%
  - Numerics: 20%
  - Software: 15%
  - Other: 15%

**D1-P4.3**: Ranked-choice ensemble
- [ ] Rank approaches by weighted score
- [ ] Calculate confidence (% experts agreeing on top-3)
- [ ] Select top-3 candidates for P6 adversarial testing

**Output**: 
- `docs/D1_VOTING_RESULTS.md`: Matrix of scores, ranked list
- Decision record: Top-3 approaches with 90%+ confidence if possible

**Expected Result**: Quaternion (ATTITUDE_QUATERNION) should win >90% consensus

---

### D2-04: Specialist Voting (P4-P5)

**Assignee**: PM (60%), Moderator (40%)  
**Duration**: 1 day  
**Status**: TODO

#### Subtasks

**D2-P4.1-P4.3**: Same as D1, but smaller panel (12 experts vs 32)
- [ ] Experts: Numerics (4), Algorithms (4), Physics (4)
- [ ] Approaches: 15 vs D1's 10
- [ ] Parameters: 20 vs D1's 48 (simpler problem)
- [ ] Expected winner: Mixture-of-Gaussians with 85%+ consensus

**Output**: `docs/D2_VOTING_RESULTS.md`

---

### D1-05: Adversarial Testing (P6 Planning)

**Assignee**: Chaos Engineer / Reliability Specialist (60%)  
**Duration**: 1.5 days  
**Status**: TODO

#### Subtasks

**D1-P6.1**: Identify failure modes for top-3 approaches
- [ ] Gimbal lock (pitch = ±90°): Can quaternion handle it?
- [ ] Clock skew (wall-clock jump -5s): Impact on SE(3) continuity?
- [ ] MAVLink timeout (no message > 1s): How to detect and fall back?
- [ ] NaN/Inf in quaternion: Recovery procedure?
- [ ] Multiple rapid rotations: Numerical stability?

**D1-P6.2**: Design stress tests
For each failure mode:
- [ ] Input scenario (test case)
- [ ] Expected behavior (graceful degradation)
- [ ] Pass criterion (no crash, no NaN)
- [ ] Latency impact

**Example**:
```
Failure: Gimbal Lock (pitch = 90°)
Scenario: Rapidly pitch up to ±90°
Expected: Quaternion w=0.7, x=0.7, y=0, z=0 (valid)
Criterion: RMSE error < 5° on roll/yaw recovery
Latency: No spike above nominal 10ms
```

**Output**: `docs/D1_ADVERSARIAL_TESTS.md` (20+ test cases)

---

### D2-05: Adversarial Testing (P6 Planning)

**Assignee**: Reliability Specialist (60%)  
**Duration**: 0.5 days  
**Status**: TODO

#### Subtasks

**D2-P6.1-P6.2**: Stress tests for multipath detector
- [ ] All readings far from recent mean (extreme outlier cluster)
- [ ] Alternating near/far readings (multipath switching)
- [ ] Continuous saturation (all readings rejected)
- [ ] Recovery from saturation (step change to valid readings)

**Output**: `docs/D2_ADVERSARIAL_TESTS.md` (10+ test cases)

---

## Week 3 Deliverables

| Task | Deliverable | Status |
|------|-------------|--------|
| D1-P4-P5 | Voting results + top-3 ranked | TODO |
| D1-P5 | Decision confidence > 85% | TODO |
| D1-P6 | 20+ adversarial test cases | TODO |
| D2-P4-P5 | Voting results + top-3 ranked | TODO |
| D2-P6 | 10+ adversarial test cases | TODO |

**Quality Gate**: Consensus > 85% on final decisions before moving to P7

---

## Cross-Cutting Tasks (Weeks 1-3)

### Blind Spot Audit Round 2 Execution

**Parallel Timeline**: Audit domains 15-19 during Weeks 1-3

**Domain Assignment**:
```
Domain 15 (Deployment)    | Week 1-2 | 12 specialists
Domain 16 (CI/CD)         | Week 1-2 | 12 specialists
Domain 17 (Database)      | Week 2-3 | 12 specialists
Domain 18 (Security++)    | Week 2-3 | 12 specialists
Domain 19 (Network)       | Week 3   | 12 specialists
```

**Per Domain**:
- [ ] P1: Scope (8-9 blind spots per specialist)
- [ ] P2: Literature (20-30 findings per domain)
- [ ] Severity classification (C/H/M/L)
- [ ] Root cause analysis
- [ ] Design control mapping

**Expected Output**:
- 60-70 findings total (12 × 5 domains ≈ 12/domain)
- Categorized in PHYSICS_AUDIT.md (Round 2 section)

---

## Sprint 1 Success Criteria

| Criterion | Target | Owner | Status |
|-----------|--------|-------|--------|
| D1-P1: Requirements finalized | 100% | Controls Eng | TODO |
| D1-P2: Literature collected | 50+ papers | Research | TODO |
| D1-P3: Approach matrix complete | 300 × 48 | Theorist | TODO |
| D1-P4-P5: Specialist consensus | >85% | Panel | TODO |
| D1-P6: Adversarial tests designed | 20+ cases | Reliability | TODO |
| D2-P1-P5: Same pipeline | ~90% complete | Signal Proc | TODO |
| Blind Spot Audit R2 (15-19) | 60-70 findings | Audit Team | TODO |
| Documentation | All P1-P6 docs | All | TODO |

---

## Resource Allocation (Week 1-3)

| Role | Week 1 | Week 2 | Week 3 | Notes |
|------|--------|--------|--------|-------|
| Controls Eng (D1) | 40% | 60% | 40% | Scoping, synthesis, prep |
| Signal Proc Eng (D2) | 40% | 60% | 30% | Scoping, synthesis, testing |
| Research Specialist | 60% | 20% | 0% | Heavy literature work W1-W2 |
| ML Engineer | 20% | 40% | 20% | Approach synthesis, scoring |
| PM | 20% | 30% | 40% | Coordination, voting management |
| Chaos Eng | 0% | 20% | 50% | Adversarial test design |
| QA Engineer | 10% | 20% | 20% | Test documentation, review |
| Field Engineer | 10% | 10% | 10% | Feasibility input |

---

## Risk Mitigation

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Specialist unavailability | Medium | High | 2 backups per domain, asynchronous voting |
| Literature search incomplete | Low | Medium | Start search early, expand sources Week 2 |
| Consensus < 85% | Low | High | Return to P2 with different scope |
| Time overrun on P3 synthesis | Medium | Medium | Pre-synthesize top 100 approaches W1 |

---

## Definition of Done (Sprint 1)

✅ **D1**: P1-P6 complete, decision ready, 90%+ consensus, adversarial tests designed  
✅ **D2**: P1-P6 complete, decision ready, 85%+ consensus, adversarial tests designed  
✅ **Audit R2**: Domains 15-19 audited, 60-70 findings logged  
✅ **Documentation**: All specs, matrices, voting results, adversarial tests documented  
✅ **Quality**: All deliverables peer-reviewed before sign-off  

**Readiness for Sprint 2 P7 (Prototyping)**: YES

---

## Sprint 1 Sign-Off

```
Components Ready for P7:
├─ D1: Quaternion approach selected (top choice)
├─ D2: GMM approach selected (top choice)
├─ Test cases: 30+ defined for adversarial P6
└─ Specs: Ready for engineering implementation

Next Step: Sprint 1 → Sprint 2 (P7-P12 prototyping, benchmarking, field validation)
```
