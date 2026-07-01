# Third-Party Licenses & Attributions

This project bundles / depends on the following third-party software. The
project itself is proprietary (see [`PATENT.md`](PATENT.md)); the components
below retain their own licenses and must be attributed in distributions.

> **Note on trade-secret claims:** open-source dependencies are **not** trade
> secrets. Only original, undisclosed algorithm implementations are claimed as
> proprietary. This file documents the OSS boundary.

## Python Dependencies

| Package | License | Notes |
|---------|---------|-------|
| Flask | BSD-3-Clause | Web framework |
| Werkzeug | BSD-3-Clause | WSGI toolkit |
| Flask-CORS | MIT | CORS handling |
| Flask-SocketIO | MIT | WebSocket layer |
| python-socketio / python-engineio | MIT | Socket.IO impl |
| pyserial | BSD-3-Clause | UART communication |
| NumPy | BSD-3-Clause | Numerics |
| SciPy | BSD-3-Clause | Signal processing |
| **Open3D** | **MIT** | Point cloud / ICP registration |
| h5py | BSD-3-Clause | HDF5 storage |
| pymavlink | LGPL-3.0 | MAVLink (dynamic use) |
| aiohttp | Apache-2.0 | Async HTTP |
| PyYAML | MIT | Config parsing |
| requests | Apache-2.0 | HTTP client |
| python-dateutil | Apache-2.0 / BSD | Date parsing |
| pytest | MIT | Testing |

Full license texts ship with each package's distribution. Pinned versions are
in [`requirements.txt`](requirements.txt).

## Frontend Dependencies (CDN)

| Library | License | Notes |
|---------|---------|-------|
| Socket.IO client | MIT | Loaded via cdnjs |
| Three.js (r128) | MIT | 3D visualization |

## Algorithm Attributions

The implementations are original, but the underlying algorithms are prior art:

- **ICP** — Besl & McKay, *A Method for Registration of 3-D Shapes*, IEEE PAMI, 1992.
- **DBSCAN-style clustering** — Ester et al., *A Density-Based Algorithm for
  Discovering Clusters in Large Spatial Databases with Noise*, KDD, 1996.
- **IQR / Z-score outlier detection** — standard statistical methods (public domain).

## LGPL Compliance (pymavlink)

pymavlink is LGPL-3.0. It is used as an unmodified dependency via its public API
(dynamic linking equivalent). No pymavlink source is modified. Users may replace
the library with a compatible version.

## Hardware Protocol

The TFmini-S UART frame format implemented in `app/lidar_driver.py` follows the
publicly published Benewake TFmini-S datasheet. See
[`docs/COMPETITOR_IP.md`](docs/COMPETITOR_IP.md) for protocol-provenance notes.
