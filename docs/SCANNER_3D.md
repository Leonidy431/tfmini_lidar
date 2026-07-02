# 3D Object Scanner Mode

Turns the single-point TFmini-S into a 3D object scanner. The carrier vehicle
(surface boat, drone, or underwater ROV) **orbits the target object in a
circle** with the sensor aimed at the object center. Each full orbit produces a
ring of surface points; repeating the orbit at different depths/altitudes
stacks rings into a dense 3D point cloud — like a turntable 3D scanner, but
inverted: the sensor moves, the object stays still.

```
        orbit (radius R)
      .  -  -  -  .
    /               \        carrier P, heading h aimed at center C
   |        C        |       reading d  →  surface point S
   |     ● object    |
    \       ▲       /        S = C + (d − R) · u(h)
      ' _  _|_  _ '           u(h) = [cos h, sin h, 0]
            P →sensor
```

## Geometry

- Carrier position is **derived**, not measured: `P = C − R·u(h)` — valid as
  long as the pilot keeps the orbit radius and aims the sensor at the center.
- Surface point: `S = C + (d − R)·u(h)` at the current layer depth `z`.
- Readings with `d ≥ R` passed the center without hitting the object (a miss)
  and are rejected (`beyond_center`).

Only the **heading** is required from the vehicle (already fed from MAVLink as
`current_heading`) — no absolute positioning needed. This makes the mode
practical for boats and drones without RTK/DVL.

## Operating Procedure

1. Position the vehicle at the chosen orbit radius `R` from the object,
   sensor pointing at the object center.
2. `POST /api/scanner/start` with `orbit_radius` (and optional `center`,
   `initial_z`).
3. Orbit the object slowly, keeping distance and aim. Watch
   `layer_coverage` in `scanner_progress` events / `/api/scanner/status` —
   a ring is complete at ≥95% coverage.
4. Change depth/altitude by one `layer_height` step and
   `POST /api/scanner/layer {"z": -0.5}`. Orbit again.
5. Repeat for all layers, then `POST /api/scanner/stop` and
   `POST /api/scanner/save/<name>` — the cloud is stored as a normal map
   (PLY), tagged `3d_scan`, viewable in the Mapping tab and exportable.

## REST API

| Method | Path | Auth | Body / Notes |
|--------|------|:----:|--------------|
| POST | `/api/scanner/start` | 🔒 | `{center?: [x,y,z], orbit_radius?: m, initial_z?: m}` |
| POST | `/api/scanner/stop` | 🔒 | Stops scan; data kept until cleared/saved |
| POST | `/api/scanner/layer` | 🔒 | `{z: float}` — set current ring depth |
| GET | `/api/scanner/status` | | Coverage per layer, counts, rejections |
| GET | `/api/scanner/points` | | Point cloud (strided ≤50k) |
| POST | `/api/scanner/clear` | 🔒 | Discard scan data |
| POST | `/api/scanner/save/<name>` | 🔒 | `{description?, tags?}` → saved as map |

WebSocket: `scanner_progress` (every 25 accepted points) carries the same
payload as `/api/scanner/status`.

## Configuration (`ScannerConfig`)

| Param | Default | Notes |
|-------|---------|-------|
| `orbit_radius` | 3.0 m | Default carrier-to-center distance |
| `angular_resolution_deg` | 5.0° | Coverage bin size (72 bins/ring) |
| `layer_height` | 0.5 m | Vertical distance between rings |
| `min_distance` | 0.2 m | Reject closer readings |
| `max_points` | 500 000 | Hard cloud cap |
| `signal_threshold` | 100 | Min signal strength |
| `min_coverage_complete` | 0.95 | Ring "complete" threshold |

## Accuracy Notes & Limitations

- Point accuracy is bounded by how well the pilot holds the orbit radius and
  aim; radius error translates 1:1 into radial point error. Prefer slow,
  steady orbits.
- Heading must be calibrated (compass deviation directly rotates the model).
- Underwater: data-quality gates (signal threshold) reject most turbidity
  spikes, but very turbid water degrades the ring — watch the Data Quality
  Score.
- The mode assumes a **convex-ish, static object**; concave pockets facing
  away from the orbit are not observable from a circular path.
