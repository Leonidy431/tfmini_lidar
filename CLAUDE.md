# BlueOS LiDAR SLAM Navigation System (BLSNS)

## Project Overview

TFmini-S LiDAR integration for BlueOS underwater ROV navigation with SLAM mapping, profile recording, and object detection.

## Key Files

- `app/main.py` - Flask REST API + WebSocket server
- `app/lidar_driver.py` - TFmini-S UART driver (115200 baud, 9-byte frames, auto-reconnect)
- `app/slam_engine.py` - ICP-based point cloud registration
- `app/profile_recorder.py` - Navigation waypoint recording/playback
- `app/object_detection.py` - Pattern-based obstacle classification
- `app/scanner_3d.py` - 3D object scanner (orbit scan, ring layers, coverage)
- `app/localization.py` - Map-based position estimation
- `app/security.py` - Authentication, rate limiting, path traversal protection
- `app/data_quality.py` - IQR/Z-score outlier filtering (Rule 4)
- `Dockerfile` - BlueOS Docker extension
- `PATENT.md` - Patent declaration and IP documentation

## Security

### API Authentication

All state-changing API routes require authentication:

```bash
# Set persistent token via environment
export LIDAR_API_TOKEN="your-secure-token"

# Or use auto-generated token (shown in startup logs)
# Pass token via:
# - Authorization: Bearer <token>
# - X-API-Key: <token>
# - ?api_key=<token>
```

### Protected Endpoints

Routes requiring auth: `/api/start`, `/api/stop`, `/api/mode/*`, `/api/mapping/*`, 
`/api/maps/*/save`, `/api/maps/*/load`, `/api/maps/*/delete`, `/api/profiles/*`, 
`/api/objects/clear`, `/api/objects/save`

Public routes: `/`, `/api/health`, `/api/register_service`, `/api/status` (read-only)

### CORS Configuration

```bash
export CORS_ORIGINS="http://blueos.local,http://localhost:5000"
```

## Development Rules

### Build & Test

```bash
# Run tests
python -m pytest tests/ -v

# Run application
python -m app.main

# Build Docker
docker build -t blueos-lidar-slam .
```

### Code Style

- Python: PEP-8, type hints required
- JavaScript: ES6+, no jQuery
- Commits: Conventional commits (feat/fix/docs/refactor)

---

## Mandatory Project Rules

### Rule 1: Blind Spot Audit (99-Point Analysis)

**Trigger**: Before major releases, PRs, or when explicitly requested.

**Process**:
1. Deploy 12 specialist auditors via multi-agent workflow
2. Each specialist finds 8-9 blind spots in their domain
3. Total target: ~99 blind spots identified
4. Critical/High severity items get deep-dive verification
5. Auto-engage specialists for each weakness area

**Specialist Domains**:
| Domain | Focus |
|--------|-------|
| Security | OWASP, injection, auth, secrets |
| Reliability | Error handling, fault tolerance |
| Performance | Memory, CPU, real-time constraints |
| Testing | Coverage gaps, edge cases |
| API Design | REST conventions, versioning |
| DevOps | Docker, deployment, healthchecks |
| Documentation | Missing/outdated docs |
| Data Quality | Sensor validation, outliers |
| Concurrency | Race conditions, thread safety |
| UX/Frontend | Visualization, WebSocket |
| Patent/IP | Prior art, licensing |
| Compliance | Maritime safety, ISO 14971 |

**Auto-Connect Specialists**:
When a blind spot is found, automatically engage the matching skill from claude-skills:
- Security → `engineering-team/senior-security`
- Reliability → `engineering/chaos-engineering`
- Performance → `engineering/performance-profiler`
- Testing → `engineering-team/tdd-guide`
- API → `engineering/api-design-reviewer`
- DevOps → `engineering/docker-development`
- Documentation → `docs/documentation-standards`
- Data Quality → `engineering/data-quality-auditor`
- Concurrency → `engineering-team/senior-backend`
- UX → `engineering-team/senior-frontend`
- Patent → `research/patent`
- Compliance → `ra-qm-team/risk-management-specialist`

### Rule 2: Freedom-to-Operate (FTO) Before Commercial Release

Run patent skill FTO analysis before any commercial deployment:
- Search CPC classes: G01S17 (LiDAR), G01C21 (navigation), G05D1 (underwater vehicles)
- Document prior art in PATENT.md
- Identify blocking patents requiring licensing

### Rule 3: Golden Signals Monitoring

All sensor/SLAM modules must expose metrics:
- Latency (readings_per_second, ICP registration time)
- Traffic (total measurements, scans processed)
- Errors (invalid readings, driver errors)
- Saturation (buffer utilization, memory usage)

### Rule 4: Data Quality Validation

Before SLAM processing:
- IQR outlier detection on raw LiDAR readings
- Z-score filtering for anomalous points
- Data Quality Score tracking for calibration drift

### Rule 5: Risk Management (ISO 14971)

Maintain hazard analysis in `RISK_MANAGEMENT.md` for:
- TFmini-S sensor failures (laser safety, measurement errors)
- Underwater interference (turbidity, multipath)
- Communication dropouts
- Navigation algorithm failures

Each hazard tracks Severity × Probability, design controls (mapped to source),
and accepted residual risk. Update before every release.

### Rule 6: Session Continuity Logging (30-minute cadence)

**Purpose**: Enable fast context recovery after session breaks or interruptions without requiring full conversation replay.

**Trigger**: Every 30 minutes during active development (commits, test changes, code modifications).

**Format**: Structured continuity log entry appended to session notes or CLAUDE.md update comments covering:
1. **Completed in last 30min**: File changes, commits pushed, tests added/fixed, blockers resolved
2. **Current state**: Active branch, last commit hashes, test pass rate
3. **Pending decisions**: Items awaiting user input, hardware-dependent work, deferred improvements
4. **Modified files**: List with purpose (e.g., "app/lidar_driver.py: added mono_timestamp field for rate-gate immunity")
5. **Next steps**: Immediate tasks for next 30-minute interval

**Example entry**:
```
=== Session Continuity Log [14:30] ===
Completed: Added 9 tests to test_driver_mock.py (TestPhysicsCorrections), 175/175 pass
State: claude/physics-engineering-audit branch, commit abc123def456
Decisions pending: D1-D8 deferred (MAVLink integration, 3D attitude, housing calibration)
Modified: app/lidar_driver.py, tests/test_driver_mock.py, PHYSICS_AUDIT.md
Next: Create PR, run blind-spot audit on remaining 10 domains
```

This enables a future session to pick up immediately by reading the last log entry, 
without re-deriving the entire audit trail from git commit history.

### Rule 7: Algorithm Evaluation via Multi-Specialist Decision Framework (12-Phase HLD)

**Purpose**: Ensure algorithmic decisions are evidence-based, peer-reviewed, and scientifically grounded across 32 specialist domains. All new algorithms and physics corrections must follow this framework.

**Trigger**: When implementing new signal processing, SLAM, localization, control, or sensor algorithms; when choosing between multiple competing approaches; before major algorithmic changes.

**12-Phase HLD**:

| Phase | Task | Specialists | Output |
|-------|------|-----------|--------|
| **P1-Scoping** | Define algorithm requirements, constraints, success metrics | Domain Lead, Requirements Engineer | Requirements document with edge cases, failure modes |
| **P2-Literature** | Search PubMed, arXiv, IEEE Xplore, Scholar for scientific precedent; extract 20-50 candidate papers | Research Specialist, ML Engineer | Annotated bibliography with approach taxonomy |
| **P3-Approaches** | Synthesize candidate approaches from literature (~300 variants when parameterized); categorize by architecture, trade-offs | Theoretical Computer Scientist, Domain Expert (×3) | Comparison matrix: 48 evaluation parameters across all variants |
| **P4-Evaluation** | Run each approach through 48-parameter matrix: accuracy, latency, power, memory, robustness, drift, false-positive rate, false-negative rate, generalization, numerical stability, etc. | Specialist Panel (32 experts across 8 domains) | Scored matrix: each expert rates top-10 candidates |
| **P5-Ensemble** | Aggregate 32 expert votes using ranked-choice voting; apply domain weights (physics 30%, performance 25%, reliability 20%, etc.) | Decision Science Specialist, Panel Moderator | Ranked selection of top-3 candidates with confidence scores |
| **P6-Adversarial** | Stress-test top-3: failure modes, edge cases, scaling limits, underwater-specific challenges (turbidity, multipath, NTP drift) | Chaos Engineering, Reliability Specialist (×2) | Failure analysis: what breaks, at what conditions, residual risk |
| **P7-Prototyping** | Implement minimal viable versions of top-3 candidates; benchmark on historical data and simulation | ML Engineer, DevOps | Implementation code, benchmark results, resource profiles |
| **P8-Ablation** | For selected candidate, ablate each major component; measure sensitivity to hyperparameters | Experimental Design, Statistical Specialist | Sensitivity analysis, hyperparameter tuning guidance |
| **P9-Calibration** | Empirical tuning on ROV hardware or validated simulation; measure performance on ground-truth reference data | Field Engineer, Calibration Specialist | Calibration constants, empirical validation curves |
| **P10-Integration** | Integrate into codebase with feature flags, health metrics, fallback modes; ensure no silent failures | Backend Engineer, Security Specialist | Clean PR with tests, metrics, graceful degradation |
| **P11-Validation** | Automated + manual validation: unit tests, integration tests, bench testing, field trials; achieve >95% confidence interval on success metrics | QA, Test Engineer, Domain Specialist | Test report with coverage matrix, known limitations |
| **P12-Documentation** | Document algorithm, trade-offs, failure modes, calibration procedure, scientific precedent (with citations), decision rationale | Technical Writer, Domain Expert | Algorithm doc in `docs/ALGORITHMS.md`, comments in code linking to papers |

**Specialist Panel (32 experts across 8 domains)**:
- **Physics** (4): optics/ToF, underwater acoustics, underwater navigation, sensor calibration
- **Algorithms** (4): computer vision, SLAM/localization, signal processing, control theory
- **Numerics** (4): numerical stability, computational geometry, linear algebra, real-time constraints
- **Software** (4): performance profiling, memory optimization, embedded systems, real-time kernels
- **Hardware** (3): UART/serial protocols, GPIO/actuators, sensor integration
- **Testing** (3): statistical testing, edge-case generation, failure-mode testing
- **Safety** (3): fault tolerance, degradation modes, underwater hazard analysis
- **Integration** (3): API design, system integration, backward compatibility

**48 Evaluation Parameters**:
- Accuracy (8): RMS error, max error, outlier rejection rate, drift over time, convergence speed, repeatability, bias, variance
- Latency (5): median latency, p95/p99 latency, jitter, throughput, max blockage time
- Resource (6): memory peak/average, CPU utilization, power consumption, cache footprint, code size, I/O operations
- Robustness (8): noise tolerance, multipath handling, turbidity effects, NTP-step immunity, saturation handling, underflow prevention, graceful degradation, failure mode count
- Generalization (5): benchtop→field transfer, depth dependence, temperature sensitivity, age/drift, unknown-object handling
- Compatibility (4): backward compatibility, Python version, dependency footprint, license compliance
- Maintainability (3): code clarity, test coverage, documentation quality
- Scientific (3): peer review status, reproducibility (code/data available), novelty score

**Decision Criteria** (after P5 voting):
- **Select**: Candidate with highest ensemble score AND passes all adversarial tests (P6) AND lower than acceptable risk threshold
- **Defer**: If top-3 tied or risk too high → return to P2 with different literature scope or domain constraints
- **Accept Tradeoff**: If no perfect solution → explicitly document tradeoff (e.g., "10% accuracy loss for 50% latency gain") in decision record

**Output Artifacts**:
1. `docs/ALGORITHM_<name>_DECISION.md`: Full decision record with matrix, expert votes, failure analysis, scientific precedent
2. Code comments linking to papers: `# See [Author YEAR] https://doi.org/...`
3. Hyperparameter tuning guide in docstring
4. Fallback/graceful degradation on failure

**Example Decision Record Format**:
```markdown
# Algorithm Decision: Particle Filter Likelihood (2024-01-15)

## Problem
Underwater localization particle filter underflows in log-likelihood computation.

## Candidates (300 variants)
1. Linear-space with epsilon (proposed in RTABMap)
2. Log-space with max-subtraction (Izenman et al. 2008)
3. Mixture model with regime detection (Thrun et al. 2005)

## Expert Votes (32 panel)
- Physics: Log-space [4/4 votes]
- Numerics: Log-space [4/4 votes]
- Algorithms: Log-space, Mixture [3/4, 1/4]
- ... (other domains)
Result: Log-space wins 28/32 votes, confidence 87.5%

## Failure Modes
- All weights identical → resampled to uniform (tested)
- Extreme range outliers → clamped to [0.1m, 10m] in likelihood (tested)

## Implementation (P7)
- Benchmark: 1000 particles, depth 50m → 0.3ms/update
- Memory: 12KB particle state + 4KB likelihoods

## Calibration (P9)
- Reference: manual ground-truth measurements
- Field validation: 100 dives, RMSE 0.15m vs 0.22m (linear-space)

## Papers
- [Izenman 2008] On the Use of the Log-Likelihood Ratio...
- [Thrun et al. 2005] Probabilistic Robotics, MIT Press
- [RTABMap] https://github.com/introlab/rtabmap
```

---

## Dependencies

- Python 3.9+
- Flask 3.0+, Flask-SocketIO 5.3+
- Open3D 0.18+ (ICP registration)
- PySerial 3.5 (UART communication)
- NumPy, SciPy (signal processing)

## License

Proprietary / Trade Secret - See PATENT.md
