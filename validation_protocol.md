# Validation Protocol (Протокол самопроверки)

**Rule**: Claude may not mark a backlog task complete, move to the next
`DEVELOPMENT_BACKLOG.md` item, or claim a fix is done, until every
applicable step below has been run and its result recorded in
`state_journal.md`. This is CLAUDE.md Rule 8's enforcement checklist —
Rule 8 says *what* the nervous system is; this file is the specific
sequence that closes each loop.

This composes with, not replaces, `.clauderc` Rules 26-50 (95%+ coverage
discipline) and Rule 99 (the binding law). Where this file and `.clauderc`
overlap, `.clauderc` sets the bar and this file sets the *order of
operations* to check it.

---

## Step 0 — Golden Meta-Rule (run this FIRST, before writing any code)

> If your internal confidence that the planned approach will work on the
> first real attempt is below ~90%, STOP. Do not write the code. Ask the
> user exactly one precise, isolated question that resolves the specific
> blind spot causing the uncertainty. Autonomy without this brake is
> self-destruction (см. AUTONOMY_HACKS.md, Параметр 0).

Confidence is genuinely low when any of these are true — check honestly:
- The task touches a module with no existing test coverage for the code path in question (cross-check `context_map.json` → `tests_covering`).
- The fix depends on hardware/data not available in this environment (cross-check `TECHNICAL_SPECIFICATION.md` P9 status — if it says "field pending," don't fabricate field results).
- Two plausible approaches exist and the codebase gives no precedent for which one this project prefers.
- The change would touch `app/main.py`'s route auth/rate-limit decorator order, `app/security.py`'s token/traversal logic, or `app/map_manager.py`'s atomic-save sequence — all three have caused real incidents documented in `BLIND_SPOT_AUDIT_R2_FINDINGS.md`; treat them as high-blast-radius by default.

If none of these apply and confidence is genuinely high, proceed.

---

## Step 1 — Read state before acting

- [ ] Read `state_journal.md`'s Latest Entry. Do not trust your own prior turn's summary over it — the journal is the source of truth for "where did I leave off."
- [ ] Run `git status` and `git log -1 --oneline`. If there are unexpected uncommitted changes or the branch doesn't match the journal's claimed branch, stop and reconcile before proceeding (this exact failure mode — a container reset silently reverting to a different branch with lost uncommitted work — happened once in this project; see `CORRESPONDENCE_LOG.md` Entry 13).
- [ ] Consult `context_map.json` for the file(s) about to be edited: what depends on them, what they depend on, which tests cover them.

## Step 2 — Before writing code

- [ ] Confirm the task against `DEVELOPMENT_BACKLOG.md` / `TECHNICAL_SPECIFICATION.md` — is this actually the next planned step, or scope creep?
- [ ] If the task is a new algorithm/physics decision (not a bugfix), CLAUDE.md Rule 7's 12-phase HLD applies — don't skip straight to code.
- [ ] Check `.claudeignore` is respected — don't read/paste generated artifacts, logs, or `__pycache__` into context to answer this question.

## Step 3 — After writing code, before claiming done

Run in this order; stop and fix at the first failure rather than batching fixes at the end (cheaper to debug one broken thing than five):

1. **Syntax/import sanity**: `python3 -c "import app.<module>"` for every module touched.
2. **Targeted tests**: run the specific test file(s) covering the change (from `context_map.json` → `tests_covering`).
   ```bash
   python3 -m pytest tests/test_<relevant>.py -q
   ```
3. **Full suite**: 
   ```bash
   python3 -m pytest tests/ -q
   ```
   Must be 100% passing. A "pre-existing failure unrelated to my change" is not an acceptable reason to skip this — either fix it or explicitly flag it to the user; never silently ignore a red suite.
4. **Coverage delta** (only if new code paths were added):
   ```bash
   python3 -m pytest tests/ -q --cov=app --cov-report=term-missing
   ```
   New code should not drop overall coverage below the current baseline (check `context_map.json` → `coverage_summary` for the last recorded number). `.clauderc` Rule 99 floor is 95%; this project is currently at 99% — don't regress it without a documented reason.
5. **Stability check** (only for anything touching threading, timing, or randomness — driver read loops, ICP registration, particle filters): run the affected test file **3 times** in a row. A test that passes once and fails on repeat is not done; see `CORRESPONDENCE_LOG.md` Entry 13 for two real examples (a shared-config-singleton leak and an aliased-Open3D-object bug) that only surfaced this way.
   ```bash
   for i in 1 2 3; do python3 -m pytest tests/test_<file>.py -q; done
   ```
6. **Security self-check** (only for changes touching `app/security.py`, `app/main.py` routes, or anything handling file paths/user input): re-read `.clauderc` Rules 51-70 and confirm none are newly violated.
7. **Container/build sanity** (only when `Dockerfile`, `Dockerfile.arm64`, `docker-compose.yml`, or `requirements.txt` changed): at minimum confirm the Dockerfile still parses (`docker build --check` if available, or a manual line-by-line read for the exact class of bug already caught once — inline `EXPOSE` comments, see `BLIND_SPOT_AUDIT_R2_FINDINGS.md` 15-2). Full `docker build` when the environment allows it.

## Step 4 — Before ending the turn

- [ ] Append a new entry to `state_journal.md` (see its Format section).
- [ ] If findings emerged that need a design decision (not a mechanical fix), log them in `DEVELOPMENT_BACKLOG.md` rather than guessing — do not silently skip them.
- [ ] Commit with a message explaining *why*, not just *what* (repo convention — see any recent commit for the expected depth). Push if the user's standing instructions for this session say to (they do, for this repo — see the branch/PR instructions in the system context).
- [ ] Never leave uncommitted work that took more than a few minutes to produce sitting only in the working tree — the container-reset incident in `CORRESPONDENCE_LOG.md` Entry 13 is the concrete cost of skipping this.

---

## Anti-patterns this protocol exists to prevent

| Anti-pattern | Real incident in this repo | Protocol step that catches it |
|---|---|---|
| Claiming a fix works without running it | (avoided so far, but the risk is generic) | Step 3.2-3.3 |
| A flaky test masquerading as "done" | Reorthonormalization test failed silently depending on run order/randomness | Step 3.5 |
| Shared mutable test state corrupting unrelated tests | `SLAMEngine()`'s shared `Config.slam` singleton leaked `buffer_size` between tests | Step 3.5 + `context_map.json`'s `Config.py` note |
| Losing uncommitted work to environment resets | 9 coverage-test files lost mid-session, had to be recreated | Step 4 (commit early, commit often) |
| A build-breaking typo shipping because nothing runs it | `Dockerfile.arm64`'s inline `EXPOSE` comments broke the build, caught only by a later audit, not CI | Step 3.7 (and the standing recommendation to add real CI — see `DEVELOPMENT_BACKLOG.md` Section 7) |
