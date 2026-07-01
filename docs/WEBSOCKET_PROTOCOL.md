# WebSocket Protocol

The server uses Socket.IO over the same origin/port as the REST API.

## Connection

```js
const socket = io({
  reconnection: true,
  reconnectionAttempts: 10,
  reconnectionDelay: 1000,
  reconnectionDelayMax: 5000,
  auth: { token: "<api-token>" }   // required when REQUIRE_WS_AUTH=true
});
```

When `REQUIRE_WS_AUTH=true`, the server validates `auth.token` (or an
`api_key` query param) during the handshake and rejects invalid connections.

## Client → Server events

| Event | Payload | Response event |
|-------|---------|----------------|
| `get_status` | — | `status` |
| `get_map_points` | — | `map_points` |

## Server → Client events

| Event | Payload schema | When |
|-------|----------------|------|
| `status` | full status object (see `/api/status`) | on connect, on `get_status` |
| `lidar_reading` | `{distance, signal_strength, timestamp, temperature, valid}` | each accepted reading |
| `localization` | `{success, position:[x,y,z], confidence}` | localizing mode |
| `navigation` | `{status, target_waypoint, distance_to_waypoint, heading_error, heading_correction, progress, ...}` | navigating mode |
| `detection` | `{id, class, confidence, position:[x,y,z], size:[w,h,d], distance, ...}` | object detected |
| `map_points` | `{points: [[x,y,z], ...]}` | on `get_map_points` |
| `driver_error` | `{context, message, health}` | driver serial/read error |

## Payload examples

**`lidar_reading`**
```json
{ "distance": 1.523, "signal_strength": 210, "timestamp": "2026-07-01T10:00:00.000",
  "temperature": 24.3, "valid": true }
```

**`navigation`**
```json
{ "status": "on_track", "waypoint_index": 4, "total_waypoints": 20,
  "distance_to_waypoint": 0.812, "heading_error": -3.4,
  "heading_correction": "right", "progress": 20.0 }
```

**`driver_error`**
```json
{ "context": "serial", "message": "device reports readiness to read but returned no data",
  "health": { "state": "degraded", "reasons": ["reconnecting"] } }
```

## Lifecycle notes

- On reconnect, the client should call `get_status` (or issue a REST
  `/api/status`) to resync — the reference frontend does this automatically.
- Server state is authoritative; the WebSocket stream is fire-and-forget and
  may drop frames under load (mirrors the processing pipeline's frame dropping).
