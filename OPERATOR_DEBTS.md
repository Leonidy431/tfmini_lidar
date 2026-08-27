# Operator Debt Report

**Purpose**: a single place listing everything that needs the *operator's*
(human's) attention — decisions only a human can make, and self-assessed
gaps in what this session needs to work efficiently. Requested 2026-08-27
("напиши долги оператору... с напоминанием каждые три часа"). A Routine
("Operator Debt Reminder", 3h cadence) re-reads this file and pings the
operator with the current NEEDS-DECISION list — see the bottom of this file
for how that Routine is wired.

**Update discipline**: this file changes only when a debt is added,
resolved, or re-prioritized — not rewritten wholesale each time. Resolved
items move to "Resolved" with the commit that closed them, they are not
deleted (so the reminder history stays auditable).

---

## 1. NEEDS-DECISION (business/legal/operator judgment — not engineering)

| # | Item | Why it needs you, not me | Source |
|---|------|---------------------------|--------|
| D1 | **`PATENT.md`'s claim disclosure appears to be in a public GitHub repo.** This risks destroying trade-secret status and, in absolute-novelty jurisdictions (EP/CN per `docs/FTO.md`), triggering a patent bar date. | Repo visibility, provisional filing timing, and whether to reclassify as trade-secret-only are legal/business calls with real deadlines — guessing wrong here is not recoverable by a later commit. | `BLIND_SPOT_AUDIT_R3_FINDINGS.md` R3-IP-1 |
| D2 | Patent filing strategy generally (whether/when to file, which jurisdictions). | Same reason as D1 — commercial strategy. | `BLIND_SPOT_99_QA.md` Q89 |
| D3 | `clauderc.md` (root, uploaded twice via GitHub web UI by the repo owner) duplicates `.clauderc`'s content. Should they be reconciled/merged, or is the duplication intentional? | It's someone else's deliberate upload, not this session's file — touching it without authorization would be presumptuous. | `BLIND_SPOT_99_QA.md` Q97 |
| D4 | RPi ARM64 buildability for `Dockerfile.arm64` can't be honestly confirmed without real hardware (CI's `--check` step only validates syntax, not a real cross-arch build). | Needs either real hardware access or an explicit risk-acceptance to ship unverified. | `BLIND_SPOT_99_QA.md` Q47 |

**None of these were acted on unilaterally** — flagging and waiting is the correct move per the golden meta-rule in `validation_protocol.md` Step 0.

---

## 2. Outstanding backlog (engineering work, not decisions — I can act on these, just haven't yet)

Full detail in `DEVELOPMENT_BACKLOG.md` Section 8. Summary:

- **2 architectural clusters** (highest value, need a dedicated session each — not a one-line patch):
  - Pipeline ordering: `data_quality`/`multipath_detector` filtering runs on the wrong thread (violates the real-time enqueue-only contract) *and* on the wrong pre-correction signal. (R3-PERF-1 CRITICAL + R3-DQ-4)
  - Detector permanent-freeze: `multipath_detector` can lock into 100%-reject with no escape, unlike `data_quality.py`'s existing regime-change unlock pattern. (R3-DQ-6 + R3-TEST-4)
- **68 smaller findings** logged by domain (Reliability, Security, Performance, Testing, API Design, DevOps, Documentation, Data Quality, Concurrency, Compliance, Patent/IP, UX/Frontend) — see `DEVELOPMENT_BACKLOG.md` Section 8 for the full grouped list with exact `file:line` pointers into `BLIND_SPOT_AUDIT_R3_FINDINGS.md`.

None of these block anything else; they're prioritized roughly by severity in Section 8.

---

## 3. Self-assessment: what I need for efficient work (guru-mode gaps)

Found via a fresh pass over environment config, ТЗ, and the backlog itself on 2026-08-27:

| Gap found | Status |
|---|---|
| No `.env.example` existed anywhere — 31 env vars read across `app/config.py`/`security.py`/`main.py` with zero single-source template | ✅ Fixed — `.env.example` added at repo root, `docker-compose.yml` points to it |
| `DEVELOPMENT_BACKLOG.md`'s Executive Summary was dated 2024-01-15, claimed "258/258 tests" and "10 domains remaining" — both false for months | ✅ Fixed — Executive Summary rewritten to current state |
| `context_map.json`'s `coverage_summary` pinned to a stale commit (`6c94d01`, 3120 stmts) | ✅ Fixed — updated to current commit/coverage |
| `state_journal.md` is ~130 lines and growing (Daily Rule 9 Sweep Log now has 15 entries) | 📋 Not yet urgent — the file's own archival threshold is "~10 iterations" for the *Latest Entry* section specifically, which still has only 2 entries; the sweep log is meant to be append-only. Revisit if it starts crowding out real signal. |
| `clauderc.md` vs `.clauderc` duplication (D3 above) | 📋 Still NEEDS-DECISION, unchanged |

**Meta-observation**: every gap found above is the same failure mode — a file whose job is to tell a future session "what's true right now" silently drifting out of sync with reality. That's exactly what Rule 8's nervous-system files exist to prevent, and exactly what periodic self-audits like this one are for. No new tooling needed; the existing structure (state_journal.md / context_map.json / validation_protocol.md) is sufficient, it just needs to actually get re-checked, which this pass did.

---

## 4. Reminder cadence

A scheduled job ("Operator Debt Reminder") fires every 3 hours (cron `13
*/3 * * *`, job id `acb800a5`), re-reads Section 1 of this file, and
messages the operator with the current NEEDS-DECISION count and a one-line
headline per item. It does **not** re-run the full self-assessment each
time (that's a periodic-audit action, not a 3-hourly one) — it just surfaces
what's still open so nothing silently goes stale again. See
`state_journal.md`'s Daily Rule 9 Sweep Log for the equivalent pattern
already in use for the hardware-design sweep.

**Important durability caveat, unlike the Daily Rule 9 sweep**: the Rule 9
hardware-design sweep runs on a *durable server-side Routine* that has
survived across container resets and many days (see `trigger_id:
trig_012baCTwPWhsNiMaFRYzAsNo` in past notifications). This 3-hour reminder
uses `CronCreate` instead, because no durable-trigger tool was available to
this session when it was requested — `CronCreate` jobs are **session-only**
(in-memory, gone if this session ends) and **auto-expire after 7 days**
even if the session stays alive. If the operator wants this reminder to
survive a session end or to run indefinitely, it needs to be re-created as
a durable Routine through whatever interface set up the Daily Rule 9 sweep
(this session doesn't have that tool). Flagging this now rather than
silently letting it lapse the way the original Blind-Spot-Audit cron did
(see `DEVELOPMENT_BACKLOG.md` Section 6's "LAPSED" note) — that exact
failure mode is why this caveat is written down instead of assumed away.

**Last full self-assessment**: 2026-08-27T05:30:00Z
**Last reminder fired**: (updated by the job itself; check `state_journal.md`/git log if this line looks stale)
