# REST API Reference

Base paths: `/api/<x>` and the canonical `/api/v1/<x>` (identical behavior).

**Auth:** routes marked 🔒 require a token (`Authorization: Bearer`,
`X-API-Key`, or `?api_key=`). All routes are rate-limited.

**Errors:** standardized as
`{"error": {"code": string, "message": string, "details"?: object}}`.

## System

| Method | Path | Auth | Description |
|--------|------|:----:|-------------|
| GET | `/api/health` | | Liveness + degradation state. 503 if sensor failed. |
| GET | `/api/status` | | Full status (lidar, slam, localization, data_quality, health, pipeline). |
| GET | `/api/register_service` | | BlueOS service registration. |
| POST | `/api/start` | 🔒 | Start acquisition. |
| POST | `/api/stop` | 🔒 | Stop acquisition. |
| POST | `/api/mode/<mode>` | 🔒 | Set mode: idle/mapping/localizing/navigating/recording. |

## Mapping

| Method | Path | Auth | Description |
|--------|------|:----:|-------------|
| POST | `/api/mapping/start` | 🔒 | Clear + enter mapping mode. |
| POST | `/api/mapping/stop` | 🔒 | Stop mapping; returns stats. |
| POST | `/api/mapping/clear` | 🔒 | Clear current map. |
| GET | `/api/mapping/statistics` | | SLAM stats. |
| GET | `/api/mapping/points` | | Downsampled map points. |
| GET | `/api/mapping/trajectory` | | Trajectory positions. |

## Maps

| Method | Path | Auth | Description |
|--------|------|:----:|-------------|
| GET | `/api/maps` | | List saved maps. |
| GET | `/api/maps/<name>` | | Map info. |
| POST | `/api/maps/<name>/save` | 🔒 | Save current map. Body: `{description, tags}`. |
| POST | `/api/maps/<name>/load` | 🔒 | Load map → localizing mode. |
| DELETE | `/api/maps/<name>/delete` | 🔒 | Delete map. |

`<name>` must match `^[A-Za-z0-9][A-Za-z0-9_\-.]*$` (path-traversal safe).

## Localization

| Method | Path | Auth | Description |
|--------|------|:----:|-------------|
| GET | `/api/localization/position` | | Current position/stats. |
| POST | `/api/localization/reset` | 🔒 | Reset localization. |

## Profiles

| Method | Path | Auth | Description |
|--------|------|:----:|-------------|
| GET | `/api/profiles` | | List profiles. |
| POST | `/api/profiles/<name>/record/start` | 🔒 | Start recording. Body: `{description}`. |
| POST | `/api/profiles/record/stop` | 🔒 | Stop + save recording. |
| POST | `/api/profiles/<name>/navigate/start` | 🔒 | Replay profile. |
| POST | `/api/profiles/navigate/stop` | 🔒 | Stop navigation. |
| DELETE | `/api/profiles/<name>/delete` | 🔒 | Delete profile. |

## Objects

| Method | Path | Auth | Description |
|--------|------|:----:|-------------|
| GET | `/api/objects` | | All detected objects + stats. |
| GET | `/api/objects/nearby?x&y&z&radius` | | Objects within radius (≤100 m). |
| POST | `/api/objects/clear` | 🔒 | Clear detections. |
| POST | `/api/objects/save` | 🔒 | Persist detections to disk. |

## Status codes

| Code | Meaning |
|------|---------|
| 200 | OK |
| 400 | Bad request / invalid name / invalid coordinates |
| 401 | Missing/invalid token |
| 404 | Not found |
| 429 | Rate limit exceeded |
| 500 | Internal error |
| 503 | Sensor failed (health check) |
