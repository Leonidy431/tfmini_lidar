# Freedom-to-Operate (FTO) Analysis — Preliminary

> **⚠️ Legal disclaimer:** This document is a **preliminary, engineering-level
> FTO screen**, not legal advice. It records search *signal*, not a
> professional opinion. A qualified patent attorney **must** perform a formal
> FTO search and clearance before any commercial launch. See Rule 2 in
> [`CLAUDE.md`](../CLAUDE.md).

## Scope

- **Product:** single-point LiDAR SLAM navigation aid for underwater ROVs.
- **Jurisdictions of interest:** US, EP, CN (typical ROV markets).
- **Purpose:** identify potentially blocking patents and design-around needs.

## Relevant CPC / IPC Classes

| Class | Domain |
|-------|--------|
| G01S 17/89 | LiDAR systems for mapping/imaging |
| G01S 17/86 | LiDAR combined with other sensors |
| G01C 21/00 | Navigation / route determination |
| G05D 1/00 | Control of position of vehicles (incl. underwater) |
| G06T 7/30 | Image registration |
| B63G 8/00 | Submarines / underwater vehicles |

## Prior Art / Patents to Clear (flagged for attorney review)

The following were surfaced during the blind-spot audit as candidates for a
professional FTO search. **Non-infringement has not been verified.**

| Patent | Assignee (approx.) | Area | Notes |
|--------|--------------------|------|-------|
| US 10,386,479 | Velodyne | 3D LiDAR mapping | Multi-beam; our single-point approach likely distinct — verify. |
| US 9,377,311 | (navigation) | Waypoint navigation | Check claim scope vs. our profile replay. |
| US 10,852,418 | (underwater SLAM) | Underwater SLAM | Closest domain — priority for attorney review. |
| EP 3410155 | (LiDAR detection) | LiDAR object detection | Compare against our pattern/cluster method. |

## Preliminary Design-Around Posture

Where overlap is plausible, the novel/limited aspects to emphasize (and keep
claims narrow around) are:

1. **Pseudo-scan accumulation parameters** specific to *single-point* ToF
   sensors (buffer sizing, motion pre-check threshold).
2. **Distance-pattern classification thresholds** tuned for *underwater*
   object discrimination (turbidity/multipath-aware gating).
3. **Waypoint feature-storage format** for profile replay with quality gating.

Broad "system" claims should be avoided (see `PATENT.md` claim-narrowing note).

## Required Next Steps (before commercial release)

- [ ] Commission a professional FTO search covering the classes above.
- [ ] Obtain claim charts showing non-infringement or design-arounds for any
      blocking patent.
- [ ] Re-run this screen if the feature set changes materially.
- [ ] Record outcomes and attorney sign-off here.

## Audit Log

| Date | Action | Result |
|------|--------|--------|
| 2026-07-01 | Engineering screen from audit findings | Candidate patents listed; formal search pending |
