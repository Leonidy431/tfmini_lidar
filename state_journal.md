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

## Older Entries

*(none yet — this is the first entry. Append new entries above this line, keep this file from growing unbounded by archiving entries older than ~10 iterations into `docs/STATE_JOURNAL_ARCHIVE.md` if it gets unwieldy — see AUTONOMY_HACKS.md Context Density #1/#6/#13 on context-window garbage collection.)*
