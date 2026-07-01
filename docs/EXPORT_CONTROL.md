# Export Control Assessment — Preliminary

> **⚠️ Not legal advice.** Preliminary self-assessment. A formal export
> classification must be obtained from the relevant authority (US BIS, or
> equivalent) before international distribution. (Patent/IP finding #9.)

## Product

Software extension for underwater ROV navigation using a commercial
single-point LiDAR (Benewake TFmini-S) — mapping, navigation-profile replay, and
object detection.

## Preliminary Classification Considerations

| Factor | Assessment |
|--------|------------|
| Sensor | Commercial off-the-shelf (COTS) TFmini-S, widely available |
| Software | Navigation/mapping aid; no weapons targeting or military-specific function |
| Encryption | Standard TLS via reverse proxy only; no proprietary cryptography |
| Autonomy | Advisory only; human-in-the-loop required (see RISK_MANAGEMENT.md) |

Underwater vehicles and navigation software **can** fall under controlled
categories in some jurisdictions (e.g. certain AUV navigation technologies).
This requires formal review — do **not** assume EAR99 without a determination.

## Required Actions Before International Distribution

- [ ] Obtain formal ECCN / export classification (US: BIS; other jurisdictions
      as applicable).
- [ ] Document the determination and its basis here (EAR99 vs. controlled).
- [ ] If controlled, implement screening (denied-party, destination) before
      distribution.
- [ ] Add an export-control notice to the README and distribution channel.
- [ ] Re-assess if autonomy or targeting capability is ever added.

## Notice (draft, pending classification)

> This software may be subject to export control regulations. Recipients are
> responsible for compliance with applicable export laws. Classification
> pending formal determination.

## Audit Log

| Date | Action | Result |
|------|--------|--------|
| 2026-07-01 | Preliminary self-assessment | Formal classification required before export |
