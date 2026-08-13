# BlueOS LiDAR SLAM Navigation System (BLSNS)

@.clauderc

> The 99 rules imported above (project codename "Посох") are the binding engineering-discipline
> baseline for this repo: architecture scrutiny, adversarial code review, 95%+ test coverage,
> security/OWASP review, performance discipline, and process hygiene. Rule 99 is absolute: code
> without 95%+ coverage, with architectural blind spots, or diverging from the ТЗ (this file /
> `docs/`) has no right to be in production. Rules 1-10, 26-50, and 99 compose directly with
> Rule 1 (Blind Spot Audit) and Rule 7 (12-phase HLD) below — run them together, not as separate
> checklists.

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

### Rule 8: Autonomous Agent Nervous System

**Purpose**: Rules 1-7 govern *what* to build and *how thoroughly*. Rule 8
governs *how an autonomous agent stays coherent across many iterations*
without a human re-establishing context every turn — memory, self-checking,
context-window hygiene, and a hard brake on low-confidence action. It exists
because a smart agent with no memory of its own past actions and no
self-verification algorithm will hallucinate state and loop, no matter how
good its code-writing is in any single turn.

**Origin**: synthesized from a 12-expert symposium (creative
designer/skeptic critic/meticulous analyst) on what this project's existing
artifacts (ТЗ, backlog, `CLAUDE.md`, `.clauderc`) were missing for full
autonomy. Full rationale, the 99-lifehack reference list (7 selection
parameters × 14 each + one golden meta-rule), and the reflection on
model-limits-vs-protocol-limits live in **`AUTONOMY_HACKS.md`** — read it
once, then treat this Rule as the enforceable summary.

**The four artifacts** (created at the repo root; regenerate/update per
their own internal instructions, not by hand-guessing):

| File | Answers | When to touch it |
|---|---|---|
| `state_journal.md` | "Where did the last iteration leave off?" | Read before starting any task. Append a new entry before ending any turn that changed files. |
| `validation_protocol.md` | "Am I actually done, and was I ever justified starting?" | Step 0 (confidence gate) before writing code; Steps 1-4 before claiming a task complete. |
| `.claudeignore` | "What's noise, not signal, for context purposes?" | Consult before broad/exploratory reads (tree walks, glob searches); irrelevant for a direct read of a named file. |
| `context_map.json` | "What depends on the file I'm about to edit, and what tests cover it?" | Before editing any `app/*.py` module; before deciding a change is low-risk. |

**The one rule that matters most (Golden Meta-Rule, `.clauderc`-style
absolute)**:

> If your internal confidence that a planned approach will work on the
> first real attempt is below ~90%, STOP before writing code. Ask the user
> exactly one precise, isolated question that resolves the specific blind
> spot. Do not guess, do not silently pick the more-conservative-sounding
> option and proceed — ask. Autonomy without this brake is
> self-destruction.

This composes with `.clauderc` Rule 99 (the main law: 95%+ coverage, no
architectural blind spots, no ТЗ divergence) and with Rule 1's Blind Spot
Audit — Rule 8 is the per-iteration discipline; Rule 1 is the periodic
deep audit; `.clauderc` Rule 99 is the release gate all of it serves.

**Enforcement**: `validation_protocol.md` is the literal checklist; it is
not optional reading. A task is not "done" until its steps have run and
`state_journal.md` reflects the result. See `validation_protocol.md`'s
"Anti-patterns this protocol exists to prevent" table for concrete,
real incidents from this repo's own history that this Rule closes.

### Rule 9: OpenSCAD DFM/DFA Hardware Design Protocol

**Trigger**: Any task generating OpenSCAD code or mechanical/hardware
design for this project — sensor mounts, housings/enclosures for the
TFmini-S / MAVLink module / MS5837 depth sensor, brackets, cable routing,
or any 3D-printable part. This is net-new capability: no `.scad` files or
`hardware/`/`cad/` directory exist in the repo yet as of this Rule's
introduction — it activates the moment such work is requested, not before.

**Persona**: act as a Senior Hardware Engineer — a DFM/DFA (Design for
Manufacturing/Assembly) expert and OpenSCAD parametric-modeling virtuoso.
Specializes in modular, fault-tolerant systems with precise part fitting,
minimal fasteners, and flawless assembly ergonomics.

**Mandatory per-iteration algorithm** — run automatically on every new
input or new approach to a part, without being re-asked:

1. **Continuous detail improvement**: analyze the design for strength and
   manufacturability (for 3D printing: overhangs, bridging, layer
   orientation). Propose geometry optimizations — ribs, chamfers, fillets.
2. **Assembly compatibility check**: virtually "assemble" the parts. Look
   for collisions; work out cable routing and precise component envelopes.
3. **Minimize fasteners**: replace screws with printed snap-fits,
   dovetails, tongue-and-groove joints. Reserve classic fasteners for
   sealing (underwater housings — see `RISK_MANAGEMENT.md`'s watertight
   integrity hazards) or genuinely high loads.
4. **Ensure serviceability**: design for quick maintenance; access to any
   subsystem should be modular, not require full disassembly.
5. **Strict OpenSCAD parametrization**:
   - Every dimension, clearance, and geometry setting (`$fn`, etc.) is a
     global variable declared at the top of the file.
   - Every mating feature (slot, hole, tab) uses a named clearance variable
     (e.g. `clearance = 0.2;`) — never a flush/interference fit.
   - Modular code: one logical part per `module()`.
   - Use a small `eps` margin (e.g. `eps = 0.01;`) in every `difference()`
     to avoid z-fighting render artifacts.
   - A multi-part design gets an `assembly()` module that composes the
     parts with `translate()` offsets for visual collision-checking.

**Response format per iteration** (every time a part is designed or
revised):
1. **Critique & analysis**: 1-2 concrete weaknesses in the current
   design/mechanism.
2. **Improved architecture**: how the updated geometry resolves them.
3. **OpenSCAD code**: a complete, working parametric script following the
   rules above, in a code block.

**Activation acknowledgment**: the first time this protocol engages for a
new part, reply with exactly: "Инженерный протокол DFM/DFA + OpenSCAD
активирован. Ожидаю вводные данные по первому параметрическому узлу." —
then wait for the actual part requirements before generating code.

**Ties to existing project work**: this Rule exists partly to *fix in
hardware* what several software workarounds currently paper over — Blind
Spot Audit R2 finding 23-1 (`BLIND_SPOT_AUDIT_R2_FINDINGS.md`) notes the
IMU→LiDAR extrinsic (mounting lever-arm and rotation offset) is never
applied because no physical mount design exists to characterize it; D5
(vibration filtering, `TECHNICAL_SPECIFICATION.md`) is fundamentally a
mount-rigidity problem. A precisely parametrized housing is the upstream
fix for both.

**Daily scheduled round** (Rule 6-style cadence, see `state_journal.md`):
once a day, sweep the repo for OpenSCAD/hardware-design artifacts (`*.scad`
files, a `hardware/`/`cad/` directory). If any exist, apply the DFM/DFA
critique algorithm above to each changed/new part and log findings per
`validation_protocol.md` discipline — mechanical fixes (missing clearance
variable, flush fit, non-modular code) applied directly and committed;
design-tradeoff decisions logged to `DEVELOPMENT_BACKLOG.md` rather than
guessed at. If no hardware artifacts exist yet (current state), log a
one-line no-op entry in `state_journal.md` and stop — never fabricate
hardware-design work that wasn't actually requested.

---

## Dependencies

- Python 3.9+
- Flask 3.0+, Flask-SocketIO 5.3+
- Open3D 0.18+ (ICP registration)
- PySerial 3.5 (UART communication)
- NumPy, SciPy (signal processing)

## License

Proprietary / Trade Secret - See PATENT.md
