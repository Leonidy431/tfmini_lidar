# Risk Management File — BlueOS LiDAR SLAM Navigation System (BLSNS)

**Standard:** ISO 14971:2019 — Application of risk management to medical/safety-critical devices
**Scope:** TFmini-S LiDAR integration for underwater ROV navigation, SLAM mapping,
profile recording, and object detection.
**Status:** Living document — update before each release (see Rule 5, `CLAUDE.md`).

> This is an engineering hazard analysis for a marine robotics accessory. It is
> not a certified medical-device risk file. The ISO 14971 framework is adopted
> as a discipline for systematic hazard identification and mitigation tracking.

---

## 1. Risk Management Process

1. **Identify** hazards across sensor, environment, communication, and algorithm domains.
2. **Estimate** risk = Severity × Probability (pre-mitigation).
3. **Control** via design measures (mapped to code where implemented).
4. **Evaluate** residual risk (post-mitigation).
5. **Review** the full file before every release.

### Severity scale

| Level | Label | Definition |
|-------|-------|------------|
| S1 | Negligible | No mission impact; cosmetic |
| S2 | Minor | Degraded data; recoverable in software |
| S3 | Serious | Mission abort or loss of navigation reference |
| S4 | Critical | Vehicle loss, collision, or grounding |

### Probability scale

| Level | Label | Definition |
|-------|-------|------------|
| P1 | Remote | Unlikely across the product lifetime |
| P2 | Occasional | May occur several times in service |
| P3 | Frequent | Expected during normal operation |

### Risk acceptability matrix

| | P1 | P2 | P3 |
|-----|----|----|----|
| **S4** | Medium | High | High |
| **S3** | Low | Medium | High |
| **S2** | Low | Low | Medium |
| **S1** | Low | Low | Low |

Residual risk of **High** blocks release until further controls are added.

---

## 2. Hazard Analysis

### H-01 — TFmini-S sensor disconnect / cable fault
- **Cause:** USB-UART cable failure, connector corrosion, power glitch.
- **Effect:** Loss of range data; stale SLAM map; navigation blind.
- **Pre-mitigation:** S3 × P2 = **Medium**
- **Controls:**
  - Auto-reconnection with exponential backoff — `app/lidar_driver.py::_attempt_reconnect`
  - Stale-connection detection (5 s no-data watchdog) — `_read_loop`
  - `seconds_since_last_read` surfaced in `/api/status`
- **Residual:** S3 × P1 = **Low**

### H-02 — Erroneous measurement (turbidity / multipath / suspended particulate)
- **Cause:** Underwater optical scattering produces spurious short/long ranges.
- **Effect:** Phantom obstacles or corrupted map geometry.
- **Pre-mitigation:** S3 × P3 = **High**
- **Controls:**
  - IQR + Z-score outlier rejection — `app/data_quality.py`
  - Signal-strength threshold gate — `DataQualityValidator.validate`
  - Physical range bounds (min/max) enforced before processing
  - Data Quality Score degradation is observable operator signal
- **Residual:** S3 × P2 = **Medium** (residual risk accepted; operator advised to
  monitor Data Quality Score and abort if it drops below 0.7)

### H-03 — Laser safety (Class 1 eye exposure during handling)
- **Cause:** TFmini-S 850 nm emitter active during bench setup.
- **Effect:** Eye exposure. TFmini-S is Class 1 (eye-safe) per manufacturer.
- **Pre-mitigation:** S2 × P1 = **Low**
- **Controls:** Documented Class 1 rating; operators instructed not to disassemble
  or defeat optics. No firmware path increases emitter power.
- **Residual:** S2 × P1 = **Low**

### H-04 — Communication dropout (topside ↔ ROV link)
- **Cause:** Tether damage, WebSocket disconnect, network congestion.
- **Effect:** Operator loses telemetry / control of mapping session.
- **Pre-mitigation:** S3 × P2 = **Medium**
- **Controls:**
  - WebSocket auto-reconnect on client — `app/web/static/js/app.js`
  - REST polling fallback for status (2 s interval)
  - Server state is authoritative; reconnection resyncs via `get_status`
  - Rate limiting prevents a runaway client from saturating the link — `app/security.py`
- **Residual:** S3 × P1 = **Low**

### H-05 — Navigation algorithm failure (localization divergence / SLAM drift)
- **Cause:** ICP registration diverges; accumulated drift; feature-poor environment.
- **Effect:** Reported position diverges from truth; misguided navigation.
- **Pre-mitigation:** S4 × P2 = **High**
- **Controls:**
  - Localization confidence threshold with "lost" detection — `LocalizationConfig.lost_threshold`
  - Drift estimate surfaced in `/api/status`
  - Navigation guidance is advisory only; the ROV pilot retains manual authority
  - Waypoint deviation flags `off_course` status — `app/profile_recorder.py::ProfileNavigator`
- **Residual:** S4 × P1 = **Medium** (residual risk accepted; system is a navigation
  *aid*, not an autonomous controller — human-in-the-loop is a required control)

### H-06 — Unauthorized control / tampering
- **Cause:** Open API on a shared vehicle network.
- **Effect:** Malicious start/stop, map deletion, path traversal file access.
- **Pre-mitigation:** S3 × P2 = **Medium**
- **Controls:**
  - Bearer-token authentication on all state-changing routes — `app/security.py::require_auth`
  - Path traversal protection on all file operations — `safe_join`, `validate_path_component`
  - Restricted CORS origins
  - Container runs unprivileged (`Privileged: false`) with scoped device access
- **Residual:** S3 × P1 = **Low**

### H-07 — Resource exhaustion (unbounded point cloud / memory)
- **Cause:** Long mapping session accumulates points without bound.
- **Effect:** Memory saturation; process kill; loss of unsaved map.
- **Pre-mitigation:** S3 × P2 = **Medium**
- **Controls:**
  - Voxel downsampling on map export — `SLAMEngine.get_map_downsampled`
  - Bounded reading history buffers (`deque(maxlen=...)`)
  - Container HEALTHCHECK enables orchestrator restart — `Dockerfile`
- **Residual:** S2 × P2 = **Low**

### H-08 — Stale / corrupted saved map loaded for localization
- **Cause:** Loading a map recorded in a since-changed environment.
- **Effect:** Localization matches against outdated geometry.
- **Pre-mitigation:** S3 × P2 = **Medium**
- **Controls:**
  - Map metadata records creation timestamp, scan count, bounds
  - Localization confidence gate rejects poor matches
  - Operator responsibility documented: verify map currency before navigation
- **Residual:** S3 × P1 = **Low**

---

## 3. Residual Risk Summary

| Hazard | Pre | Residual | Accepted |
|--------|-----|----------|----------|
| H-01 Sensor disconnect | Medium | Low | Yes |
| H-02 Erroneous measurement | High | Medium | Yes (with operator monitoring) |
| H-03 Laser safety | Low | Low | Yes |
| H-04 Comm dropout | Medium | Low | Yes |
| H-05 Nav algorithm failure | High | Medium | Yes (human-in-the-loop required) |
| H-06 Unauthorized control | Medium | Low | Yes |
| H-07 Resource exhaustion | Medium | Low | Yes |
| H-08 Stale map | Medium | Low | Yes |

**Overall residual risk:** Acceptable for supervised operation as a navigation aid.
Two hazards retain **Medium** residual risk (H-02, H-05); both are accepted on the
explicit condition that a human operator supervises the session and retains manual
control authority.

---

## 4. Required Operational Controls (labeling)

These controls are **required** for the residual-risk acceptance above to hold:

1. The system is a navigation **aid**; the ROV pilot must retain manual control.
2. Monitor the **Data Quality Score**; abort mapping if it falls below 0.7.
3. Verify a saved map's currency (timestamp/environment) before localizing against it.
4. Set a strong `LIDAR_API_TOKEN` on any shared network.
5. Confirm sensor reconnection (`/api/status`) after any tether disturbance.

---

## 5. Maritime Standards Compliance Status

Honest status of maritime/safety standards raised in the audit. This product is
a **supervised navigation aid**, not certified bridge equipment; several
standards are therefore roadmap or not-applicable rather than met. Claiming
otherwise would be misleading.

| Standard | Scope | Status | Notes |
|----------|-------|--------|-------|
| ISO 14971 | Risk management | **Partial** | Hazard analysis in this file; formal V&V pending |
| IEC 60945 (temp envelope) | Maritime equipment env. | **Partial** | Temperature envelope monitored in health (-15..+55C); no certified enclosure |
| Safe-state on failure (DNV-CG-0264 spirit) | Fail-safe | **Implemented** | Sensor failure in active mode forces IDLE + alarm (`safety_alarm` event) |
| IMO MSC-FAL.1/Circ.3 (cyber) | Cyber risk | **Partial** | Auth, rate limiting, input validation, path safety implemented; formal cyber risk assessment pending |
| IEC 61508 (SIL) | Functional safety integrity | **Not applicable** | No SIL rating claimed; advisory-only system, human-in-the-loop |
| COLREGs / SOLAS Ch.V | Collision rules / AIS | **Not applicable** | Not an autonomous controller; no AIS integration |
| IMO MSC.302(87) alarms | Alarm priorities | **Roadmap** | `safety_alarm` uses a priority field; full alarm mgmt (ack/persistence) not implemented |
| ISO 13485 config mgmt | Config change control | **Partial** | Config in versioned source; no cryptographic parameter hashing |

**Roadmap items** must be completed and re-assessed before any safety-certified
or autonomous deployment. Until then, the required operational controls in
Section 4 (human supervision) are mandatory compensating controls.

## 6. Traceability

Every design control above maps to source. Post-market: re-run the 99-point blind
spot audit (Rule 1) before each release and fold new hazards into this file.

| Control domain | Source |
|----------------|--------|
| Reconnection / watchdog | `app/lidar_driver.py` |
| Outlier rejection | `app/data_quality.py` |
| Authn / path safety / CORS / rate limit | `app/security.py` |
| Confidence / drift / deviation | `app/localization.py`, `app/profile_recorder.py` |
| Container hardening / healthcheck | `Dockerfile` |
| Verification | `tests/` |
