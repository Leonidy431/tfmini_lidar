# State Journal (Журнал состояний)

**Purpose**: defense against hallucination and looping. Claude is required to
update the **Latest Entry** section below at the end of every work
iteration — before ending a turn that changed code, and at minimum every
~30 minutes of active work (this composes with, not replaces, CLAUDE.md
Rule 6's 30-minute continuity-log cadence; Rule 6 is the narrative version
for humans, this file is the machine-checkable state for the next agent
turn). See CLAUDE.md Rule 8 for the full protocol this file is part of.

**Rule**: Never start a new task without reading the Latest Entry first.
Never end a turn that changed files without writing a new one.

---

## Format (copy this block for each new entry, newest on top)

```
### [ISO-8601 timestamp] — <one-line summary>

**Phase**: <backlog phase/sprint, e.g. "Sprint 1: D1/D2 P1-P6" or "Blind Spot Audit R2">
**Backlog step completed**: <exact task ID or line from DEVELOPMENT_BACKLOG.md / TECHNICAL_SPECIFICATION.md>
**Branch / commit**: <branch name>, HEAD=<short sha>
**Test status**: <N/N passing, coverage %> (run: `python3 -m pytest tests/ -q --cov=app`)
**Unresolved issues**: <bulleted list, or "none">
**Files touched this iteration**: <path: one-line reason, for each file>
**Next step**: <the exact next action, specific enough that a fresh session with no other context could start immediately>
**Confidence in current approach**: <High / Medium / Low — if Low, validation_protocol.md Step 0 (golden meta-rule) should already have triggered a stop-and-ask>
```

---

## Latest Entry

### 2026-09-07T09:17:45Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review

---

### 2026-09-06T09:18:49Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review

---

### 2026-09-05T09:17:24Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review

---

### 2026-09-04T09:17:43Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review

---

### 2026-09-03T09:17:49Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review

---

### 2026-09-03T00:00:00Z — Reminder cadence consolidated to daily per operator request

**Phase**: Housekeeping (cross-cutting, not a numbered backlog sprint)
**Backlog step completed**: N/A — direct operator instruction, not a pre-planned item
**Branch / commit**: `claude/physics-engineering-audit` — verify with `git log -1 --oneline` at read time
**Test status**: 589/589 passing (unchanged — docs/scheduling only, no `app/` code touched)
**Unresolved issues**: same standing 4 NEEDS-DECISION items in `OPERATOR_DEBTS.md` Section 1, unchanged by this iteration
**Files touched this iteration**:
- `OPERATOR_DEBTS.md`: Section 4 rewritten — 3-hour reminder (job `acb800a5`) had already lapsed from the 2026-08-29 environment reset (exactly the durability caveat it documented); replaced with a daily job (`f165bac9`, ~15:07 UTC) per operator request "Все рутинные процессы запускай раз день. Остальные отмени. Оператор изучает код."
**Next step**: nothing pending from this change itself. `CronList` confirmed no other in-session jobs existed to cancel — the daily Rule 9 hardware sweep (external, durable) is unaffected and remains the other routine process.
**Confidence in current approach**: High — this is a direct, unambiguous instruction with no design judgment involved; verified via `CronList` before acting rather than assuming prior state.

---

### 2026-08-27T06:30:00Z — Operator debt report + Decision 4 (attitude_3d_lost/ekf_diverged health signals)

**Phase**: Self-assessment + Blind Spot Audit R3 follow-up (R3-COMP-2/R3-COMP-3)
**Backlog step completed**: `DEVELOPMENT_BACKLOG.md` Section 8's R3-COMP-2/R3-COMP-3 closed; new `OPERATOR_DEBTS.md` created per explicit user request ("напиши долги оператору... с напоминанием каждые три часа")
**Branch / commit**: `claude/physics-engineering-audit` — verify with `git log -1 --oneline` at read time
**Test status**: 589/589 passing (582 baseline + 7 new), 99% coverage (3186 statements, 42 missing)
**Unresolved issues**:
- Same standing NEEDS-DECISION items (now consolidated in `OPERATOR_DEBTS.md` Section 1): R3-IP-1 (public-repo patent disclosure risk), Q89 (filing strategy), Q97 (`clauderc.md` duplication), Q47 (ARM64 hardware buildability)
- Same architectural clusters deferred (pipeline ordering R3-PERF-1/R3-DQ-4; detector permanent-freeze R3-DQ-6/R3-TEST-4)
- **Reminder mechanism caveat**: the 3-hour Operator Debt Reminder uses `CronCreate` (job `acb800a5`, cron `13 */3 * * *`) because no durable-trigger tool was available when requested — unlike the Daily Rule 9 sweep's durable server-side Routine, this is session-scoped and auto-expires after 7 days. Documented in `OPERATOR_DEBTS.md` Section 4 so it doesn't silently lapse unexplained the way the original blind-spot cron did (`DEVELOPMENT_BACKLOG.md` Section 6).
**Files touched this iteration**:
- `OPERATOR_DEBTS.md`: new — NEEDS-DECISION items + self-assessment findings + reminder-cadence documentation
- `.env.example`: new — all 31 env vars this app reads, none previously documented in one place
- `docker-compose.yml`: comment pointing at `.env.example`
- `DEVELOPMENT_BACKLOG.md`: Executive Summary rewritten (was dated 2024-01-15, claimed false test/domain counts); Section 8 R3-COMP-2/3 marked done
- `context_map.json`: `coverage_summary`/`generated` updated from stale `6c94d01`/3120-stmt snapshot to current
- `docs/ALGORITHM_DECISION_LOG.md`: new Decision 4 (scoped HLD, not full 32-panel — see its own "Note on HLD depth")
- `app/config.py`: `EKFConfig.divergence_trace_threshold` (env `EKF_DIVERGENCE_TRACE_THRESHOLD`, default 50.0, explicitly documented as uncalibrated)
- `app/main.py`: `_attitude_3d_ever_active` tracking in `_project_beam()`; `get_health()` gains `attitude_3d_lost` and `ekf_diverged` reasons, and `no_heading_source` now also considers 3D attitude a valid heading source
- `RISK_MANAGEMENT.md`: H-09 gap closed (residual S3×P2 Medium → S3×P1 Low), H-05 gains an `ekf_diverged` control reference, Residual Risk Summary table updated
- `BLIND_SPOT_AUDIT_R3_FINDINGS.md`: R3-COMP-2/3 marked ✅, fixed-count 28→30
- `tests/test_main_internals.py`: 7 new tests (`TestAttitudeAndEKFHealthSignals`)
**Next step**: Next 3-hour reminder cycle may optionally advance one more Section 8 item per its own instructions (not mandatory). Highest standing priority remains the pipeline-ordering architectural cluster — needs a dedicated session, not a cycle-sized slice.
**Confidence in current approach**: High for the code changes (each new health reason has a false-positive AND false-negative regression test; the EKF threshold's uncertainty is written down rather than hidden — see Decision 4's "Calibration caveat"). Correctly did NOT invent a durable-trigger capability that wasn't actually available — used the honest fallback (`CronCreate`) and documented its limitation instead of silently overclaiming durability.

---

### 2026-08-17T00:00:00Z — Blind Spot Audit Round 3: 96 findings, 28 fixed with tests

**Phase**: Blind Spot Audit Round 3 (cross-cutting, all 12 CLAUDE.md Rule 1 domains)
**Backlog step completed**: New `DEVELOPMENT_BACKLOG.md` Section 8 created; CI pipeline (Section 6/7's standing #1 priority since R2) closed as part of this pass
**Branch / commit**: `claude/physics-engineering-audit` — verify with `git log -1 --oneline` at read time
**Test status**: 582/582 passing (562 baseline + 20 new regression tests), 99% coverage (3175 statements, 44 missing), stable across 3 repeated runs
**Unresolved issues**:
- 68 of 96 R3 findings remain logged (not implemented) in `DEVELOPMENT_BACKLOG.md` Section 8, grouped by domain
- 1 explicit NEEDS-DECISION flagged to the user, not acted on: R3-IP-1 — `PATENT.md`'s claim disclosure appears to be in a public GitHub repo, risking trade-secret/bar-date loss; this is a business/legal call
- 2 finding clusters deliberately deferred as a coordinated redesign rather than patched piecemeal: pipeline ordering (R3-PERF-1 CRITICAL + R3-DQ-4 — data-quality/multipath filtering runs on the wrong thread AND the wrong pre-correction signal) and detector permanent-freeze (R3-DQ-6 + R3-TEST-4)
- Same standing hardware-blocked P9 field-validation items as prior entries, unchanged
**Files touched this iteration**:
- `BLIND_SPOT_AUDIT_R3_FINDINGS.md`: new — full 96-finding table across 12 domains with fix status
- `app/main.py`: MAVLink attitude start/stop wiring (R3-REL-1), Werkzeug debugger decoupling (R3-SEC-1), MAX_CONTENT_LENGTH, scanner NaN/Inf validation (R3-SEC-8), set_mode() lock (R3-TEST-2/CONC-5), SIGTERM handler (R3-DEVOPS-2)
- `app/config.py`: `ALLOW_WERKZEUG_DEBUGGER` flag
- `app/security.py`: `RateLimiter` lock (R3-SEC-2/CONC-3)
- `app/data_quality.py`: `_decisions`/`quality_score` lock (R3-CONC-1)
- `app/lidar_driver.py`: `readings_history` lock (R3-CONC-2)
- `app/slam_engine.py`: `get_statistics()` lock (R3-CONC-4)
- `app/ekf_3d_attitude.py`: NaN/Inf input guard on `update_position`/`update_attitude` (R3-TEST-1)
- `app/localization.py`, `app/scanner_3d.py`: redundant-computation perf fixes (R3-PERF-5/6)
- `app/web/static/js/app.js`: distance-gauge scaling fix (R3-UX-6)
- `Dockerfile`: `PYTHONUNBUFFERED=1` (R3-DEVOPS-5)
- `.github/workflows/ci.yml`: new — pytest+coverage-gate, Dockerfile build, Dockerfile.arm64 syntax check (R3-DEVOPS-1)
- `requirements.txt`: added `pytest-cov` (previously undeclared despite being used)
- `CLAUDE.md`, `RISK_MANAGEMENT.md`, `LICENSES.md`, `DOCUMENTATION_INDEX.md`, `TECHNICAL_SPECIFICATION.md`, `DEVELOPMENT_BACKLOG.md`, `CORRESPONDENCE_LOG.md`: doc-sync fixes and backlog logging (R3-DOC-1/4/5, R3-COMP-4/5/6, R3-IP-6/7)
- `tests/test_security.py`, `tests/test_ekf_3d_attitude.py`, `tests/test_main_internals.py`, `tests/test_api_routes.py`, `tests/test_data_quality.py`, `tests/test_driver_mock.py`, `tests/test_slam_physics.py`: 20 new regression tests, one per mechanical fix
**Next step**: The pipeline-ordering cluster (R3-PERF-1/R3-DQ-4) is the highest-value remaining item — it affects real-time correctness and data quality simultaneously in code already shipped/default-enabled. Needs a dedicated session since it requires redesigning where filtering happens, not a one-line patch. Separately, R3-IP-1 needs the user's explicit read/decision before any further Patent/IP-domain work proceeds.
**Confidence in current approach**: High for all 28 fixes (each grounded in an actual file:line read by a specialist agent, verified again before editing, with a regression test proving the specific failure mode is closed). Correctly did NOT touch R3-IP-1 (repo visibility / patent filing is outside engineering authority) or the two architectural clusters (a rushed fix there would just relocate the bug) — per the golden meta-rule.

---

### 2026-08-16T09:40:00Z — 99 blind-spot Q&A document created

**Phase**: Infrastructure / documentation (cross-cutting, not a numbered backlog sprint)
**Backlog step completed**: N/A — new artifact requested directly by user, not a pre-planned backlog item; its 14 surfaced answers were themselves logged as new backlog items (see below)
**Branch / commit**: `claude/physics-engineering-audit` — verify with `git log -1 --oneline` at read time, do not trust this line's staleness
**Test status**: 562/562 passing (unchanged — docs-only iteration, no `app/` code touched, verified via `python3 -m pytest tests/ -q` before commit per `validation_protocol.md` Step 3)
**Unresolved issues**:
- Same standing items as the prior entry below (P9-hardware field validation, CI pipeline priority #1, no PR yet to `main`) — unchanged by this iteration
- 3 new genuinely-open questions surfaced: `BLIND_SPOT_99_QA.md` Q47 (RPi ARM64 buildability, needs real hardware), Q89 (patent filing strategy, a business decision), Q97 (`clauderc.md` vs `.clauderc` duplication, needs explicit user authorization before touching another party's upload)
**Files touched this iteration**:
- `BLIND_SPOT_99_QA.md`: new — 99 questions across the 12 Rule-1 domains, each answered once and tagged with the Rule-7 48-parameter framework field(s) that justified the choice
- `DOCUMENTATION_INDEX.md`: added a row for the new doc + a reading-order note
**Next step**: 14 new backlog items were surfaced by this document's answers (see its own Summary table) — none implemented inline, each needs its own `validation_protocol.md` pass. Still recommend CI pipeline (Q42/Q43/`16-3`) as the actual next piece of work, since several of the 14 new items (Q44-Q46) are Docker/CI-adjacent and would benefit from CI existing first to verify them.
**Confidence in current approach**: High for the 84 inline-answered questions (each grounded in an already-implemented pattern or an already-documented finding in this repo, not invented). Medium-by-design for the 3 flagged NEEDS-DECISION questions — correctly left open rather than guessed, per the golden meta-rule.

---

### 2026-07-30T00:00:00Z — Autonomy nervous-system artifacts created

**Phase**: Infrastructure (not a numbered backlog sprint — a cross-cutting request to build the autonomy scaffolding itself)
**Backlog step completed**: N/A (this work predates/enables backlog execution; created `state_journal.md`, `validation_protocol.md`, `.claudeignore`, `context_map.json`, `AUTONOMY_HACKS.md`, wired Rule 8 into `CLAUDE.md`)
**Branch / commit**: `claude/physics-engineering-audit`, HEAD=see `git log -1 --oneline` at read time (this file is not auto-updated by tooling — always trust `git log`/`git status` over a stale entry's claimed commit)
**Test status**: 562/562 passing, 99% coverage (3120 statements, 40 missing) as of commit `6c94d01` — unchanged by this iteration (docs/config only, no app/ code touched)
**Unresolved issues**:
- Field validation (P9-hardware) for D1/D2/D3/D4/D8 still blocked on physical hardware (MAVLink IMU, turbidity tank, depth pool, oven) — see `TECHNICAL_SPECIFICATION.md` phase table
- Blind Spot Audit R2 NEEDS-DECISION backlog (~33 items) not yet actioned — CI pipeline is priority #1 (see `DEVELOPMENT_BACKLOG.md` Section 6/7 and `BLIND_SPOT_AUDIT_R2_FINDINGS.md` "Next-session priorities")
- No PR exists yet for `claude/physics-engineering-audit` → `main` (GitHub App not connected in this environment; manual creation link is in `CORRESPONDENCE_LOG.md`)
**Files touched this iteration**:
- `state_journal.md`: this file (new)
- `validation_protocol.md`: pre-next-task checklist (new)
- `.claudeignore`: context-window noise filter (new)
- `context_map.json`: dependency map for `app/*.py`, test coverage cross-reference, deferred-decision status (new)
- `AUTONOMY_HACKS.md`: the 99-lifehack reference doc, 7 parameters × 14 + golden rule (new)
- `CLAUDE.md`: Rule 8 added, referencing all of the above
- `DOCUMENTATION_INDEX.md`: updated to list the new files
**Next step**: Resume `DEVELOPMENT_BACKLOG.md` Section 6/7 NEEDS-DECISION backlog, starting with a CI pipeline (`.github/workflows/ci.yml`) per `BLIND_SPOT_AUDIT_R2_FINDINGS.md`'s "Next-session priorities" #1 — nothing else in the repo currently catches a build-breaking regression before it merges.
**Confidence in current approach**: High (this is scaffolding/documentation work with no functional-correctness risk; the golden meta-rule doesn't apply here since there's no code to be wrong about).

---

## Daily Rule 9 Sweep Log

Append-only, one line per day this fires with nothing to review (see
CLAUDE.md Rule 9 "Daily scheduled round"). This section exists so a no-op
day doesn't force a full Latest Entry rewrite — when the sweep finds real
`.scad`/hardware artifacts to review, that work gets a proper entry in
Latest Entry per the Format above, not just a line here.

- 2026-08-13T09:18:39Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-14T09:26:55Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-15T09:18:13Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-16T09:18:15Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-17T09:25:22Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-18T09:17:59Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-19T09:18:29Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-20T09:18:20Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-21T09:18:05Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-22T09:18:01Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-23T09:18:12Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-24T09:18:14Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-25T09:17:29Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-26T09:17:35Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-27T09:27:14Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-28T09:18:28Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-29T09:17:57Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review. (Environment reset recovered this cycle — repo had reverted to `claude/blue-os-lidar-system-LoGoc`/PR#1 state and Python packages were gone; recovered via `git checkout -B claude/physics-engineering-audit origin/claude/physics-engineering-audit` + `pip install -r requirements.txt pytest-cov`, confirmed 589/589 passing before proceeding, per validation_protocol.md Step 1.)
- 2026-08-30T09:17:11Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-08-31T09:17:50Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-09-01T09:17:31Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.
- 2026-09-02T09:18:19Z — Daily Rule 9 sweep: no hardware/OpenSCAD artifacts yet, nothing to review.

---

## Older Entries

*(none yet — this is the first entry. Append new entries above this line, keep this file from growing unbounded by archiving entries older than ~10 iterations into `docs/STATE_JOURNAL_ARCHIVE.md` if it gets unwieldy — see AUTONOMY_HACKS.md Context Density #1/#6/#13 on context-window garbage collection.)*
