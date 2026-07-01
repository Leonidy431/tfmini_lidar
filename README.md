# BlueOS LiDAR SLAM Navigation System (BLSNS)

Real-time mapping, navigation profile recording, and object detection for
underwater ROVs using a Benewake **TFmini-S** single-point LiDAR, packaged as a
[BlueOS](https://blueos.cloud/) Docker extension.

- **SLAM mapping** — ICP-based point cloud registration (Open3D)
- **Navigation profiles** — record a path, then replay it with live guidance
- **Object detection** — pattern + clustering based obstacle classification
- **Localization** — map-based position estimation
- **Web UI** — Flask REST API + WebSocket, Three.js 3D visualization

> ⚠️ This system is a **navigation aid**. A human operator must supervise
> operation and retain manual control. See [`RISK_MANAGEMENT.md`](RISK_MANAGEMENT.md).

---

## Hardware Requirements

| Item | Detail |
|------|--------|
| Sensor | Benewake TFmini-S ToF LiDAR |
| Interface | UART, 115200 baud, 8N1 |
| Frame | 9 bytes, header `0x59 0x59`, checksum = low byte of sum of first 8 |
| Range | 0.1 – 12 m |
| Connection | USB-UART adapter (`/dev/ttyUSB0`) or Pi UART (`/dev/ttyAMA0`) |

### Wiring (TFmini-S → USB-UART)

| TFmini-S | Signal | USB-UART |
|----------|--------|----------|
| Pin 1 (red) | +5V | 5V |
| Pin 2 (white) | RX | TX |
| Pin 3 (green) | TX | RX |
| Pin 4 (black) | GND | GND |

---

## Software Prerequisites

- Python 3.9+ (3.11 recommended)
- Docker (for BlueOS extension deployment)
- Dependencies pinned in [`requirements.txt`](requirements.txt)

---

## Quick Start (local, no Docker)

```bash
pip install -r requirements.txt

# Optional: set a persistent API token (otherwise one is auto-generated
# and printed to the logs on startup)
export LIDAR_API_TOKEN="your-secure-token"
export LIDAR_PORT="/dev/ttyUSB0"

python -m app.main
```

Open <http://localhost:5000>. Paste the API token into the **API Token** field
on the dashboard to enable control actions.

If no sensor is connected the app starts in demo mode (no readings).

---

## Docker / BlueOS

```bash
# Build
docker build -t blueos-lidar-slam .

# Run (compose is the easiest path)
docker compose up -d
```

The container runs **unprivileged** with only the `SYS_RAWIO` capability and
explicit serial-device mappings. See [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md)
for BlueOS extension installation and production (Gunicorn) setup.

---

## Authentication

All **state-changing** routes require a token, sent as:

- `Authorization: Bearer <token>`
- `X-API-Key: <token>`
- `?api_key=<token>` (query string)

Read-only routes (`/api/status`, `/api/health`, map/profile/object listings)
are open but rate-limited. Set `LIDAR_API_TOKEN` for a stable token, or read the
auto-generated one from the startup logs.

Enable WebSocket auth on shared networks with `REQUIRE_WS_AUTH=true`.

---

## Configuration

Key environment variables (full reference in
[`docs/CONFIGURATION.md`](docs/CONFIGURATION.md)):

| Variable | Default | Purpose |
|----------|---------|---------|
| `LIDAR_PORT` | `/dev/ttyUSB0` | Serial device |
| `LIDAR_API_TOKEN` | *(auto)* | Persistent API token |
| `CORS_ORIGINS` | localhost + blueos.local | Allowed web origins |
| `REQUIRE_WS_AUTH` | `false` | Require token on WebSocket connect |
| `DATA_DIR` | `/app/data` | Maps/profiles/objects/logs root |
| `WEB_PORT` | `5000` | HTTP port |
| `DEBUG` | `false` | Never enable in production |

---

## API Overview

REST endpoints are available at both `/api/<x>` and the canonical
`/api/v1/<x>`. Full list in [`docs/API.md`](docs/API.md); WebSocket events in
[`docs/WEBSOCKET_PROTOCOL.md`](docs/WEBSOCKET_PROTOCOL.md).

| Method | Route | Auth | Purpose |
|--------|-------|------|---------|
| GET | `/api/health` | no | Liveness + degradation state |
| GET | `/api/status` | no | Full system status |
| POST | `/api/start` / `/api/stop` | yes | Start/stop acquisition |
| POST | `/api/mapping/start` / `stop` / `clear` | yes | Mapping control |
| POST | `/api/maps/<name>/save` / `load` | yes | Persist / load maps |
| DELETE | `/api/maps/<name>/delete` | yes | Delete map |
| POST | `/api/profiles/<name>/record/start` | yes | Record profile |
| POST | `/api/profiles/<name>/navigate/start` | yes | Replay profile |

Standard error shape:

```json
{ "error": { "code": "not_found", "message": "Resource not found" } }
```

---

## Testing

```bash
python -m pytest tests/ -v
```

Dependency-light suites (`test_security.py`, `test_data_quality.py`,
`test_driver_mock.py`) run without numpy/Open3D and are CI-friendly. The full
`test_lidar.py` requires the scientific stack. See
[`docs/TESTING.md`](docs/TESTING.md).

---

## Documentation

| Doc | Contents |
|-----|----------|
| [`docs/CONFIGURATION.md`](docs/CONFIGURATION.md) | Every config parameter, ranges, tuning |
| [`docs/DEPLOYMENT.md`](docs/DEPLOYMENT.md) | BlueOS + Docker + production WSGI |
| [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md) | Connection, SLAM, localization issues |
| [`docs/ALGORITHMS.md`](docs/ALGORITHMS.md) | ICP, clustering, data quality internals |
| [`docs/WEBSOCKET_PROTOCOL.md`](docs/WEBSOCKET_PROTOCOL.md) | Event names + payloads |
| [`docs/API.md`](docs/API.md) | Full REST reference |
| [`docs/TESTING.md`](docs/TESTING.md) | Test layout and CI notes |
| [`RISK_MANAGEMENT.md`](RISK_MANAGEMENT.md) | ISO 14971 hazard analysis |
| [`PATENT.md`](PATENT.md) | IP / patent declaration |

---

## Project Layout

```
app/
  main.py            REST API + WebSocket + orchestration
  lidar_driver.py    TFmini-S UART driver (auto-reconnect)
  slam_engine.py     ICP registration (Open3D)
  localization.py    Map-based position estimation
  map_manager.py     Map persistence (PLY/PCD/NPY/H5)
  profile_recorder.py Waypoint recording + playback
  object_detection.py Pattern + clustering classification
  data_quality.py    IQR/Z-score/rate outlier filtering
  security.py        Auth, rate limiting, path safety
  config.py          Dataclass configuration
  web/               Frontend (HTML/CSS/JS, Three.js)
tests/               Pytest suites
```

## License

Proprietary / Trade Secret — see [`PATENT.md`](PATENT.md) and
[`LICENSES.md`](LICENSES.md) for third-party attributions.
