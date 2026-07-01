# Competitor IP Landscape — Preliminary

> **⚠️ Not legal advice.** Preliminary landscape notes to guide a professional
> patent search. Verify all assertions with counsel before commercial launch.

## Purpose

Map the competitive patent landscape in underwater LiDAR/navigation so blocking
claims can be designed around before launch (Patent/IP finding #8).

## Key Players & Areas to Search

| Entity | Domain | Search focus |
|--------|--------|--------------|
| Kongsberg Maritime | AUV/ROV navigation, INS | Autonomous underwater navigation patents |
| Teledyne (BlueView / RESON) | Sonar, imaging | Acoustic + optical mapping crossover |
| Blue Robotics | ROV hardware, BlueOS ecosystem | Extension/integration IP, existing platform patents |
| Velodyne / Ouster | LiDAR sensors | Multi-beam mapping (distinct from single-point) |
| Chinese research institutes (CSIC, CNOOC-affiliated) | Underwater robotics | CN-jurisdiction underwater SLAM filings |

## Benewake TFmini-S Protocol Provenance (finding #3)

- The 9-byte UART frame (`0x59 0x59` header, checksum) is documented in the
  **public Benewake TFmini-S datasheet**.
- Our driver implements the documented protocol from the datasheet; no
  proprietary Benewake SDK code is copied.
- **Action:** confirm datasheet terms permit third-party protocol
  implementation; if the SDK carries restrictions, obtain written permission or
  use only datasheet-derived framing (current approach).

## Positioning vs. Landscape

Our differentiation (to keep defensible):
- **Single-point** ToF (not multi-beam) → distinct from Velodyne/Ouster claims.
- **BlueOS extension** integration model → complements rather than competes with
  Blue Robotics platform IP.
- **Underwater-specific data-quality gating** → narrow, tunable, novel.

## Required Next Steps

- [ ] Professional landscape search per entity/class above.
- [ ] Identify any blocking claims; document design-arounds.
- [ ] Confirm Benewake datasheet licensing terms.
- [ ] Attorney review + sign-off.
