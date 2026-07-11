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

---

## Dependencies

- Python 3.9+
- Flask 3.0+, Flask-SocketIO 5.3+
- Open3D 0.18+ (ICP registration)
- PySerial 3.5 (UART communication)
- NumPy, SciPy (signal processing)

## License

Proprietary / Trade Secret - See PATENT.md
