# Deployment Guide

## 1. BlueOS Extension (recommended)

BlueOS discovers extensions from a Docker image whose `LABEL permissions`
describes the device/port requirements (already set in the
[`Dockerfile`](../Dockerfile)).

1. Build and push the image to a registry reachable by the vehicle:
   ```bash
   docker build -t <registry>/blueos-lidar-slam:1.0.1 .
   docker push <registry>/blueos-lidar-slam:1.0.1
   ```
2. In BlueOS → **Extensions → Installed → +**, add the image name and tag.
3. BlueOS reads the `permissions` label and requests the serial devices and
   port. Confirm the install.
4. Open the extension; paste the API token (from container logs) on the
   dashboard.

## 2. Manual Docker

```bash
docker compose up -d
docker compose logs -f    # grab the auto-generated API token
```

The compose file:
- runs **unprivileged** with `cap_add: SYS_RAWIO`,
- maps only the serial devices (no full `/dev` mount),
- stores data in a named volume `lidar-data`,
- includes a `curl`-based healthcheck.

### Device permissions

The container user (`lidar`, uid 1000) is in the `dialout` group. On the host,
ensure the serial device is group-accessible:

```bash
ls -l /dev/ttyUSB0          # should be group 'dialout'
sudo usermod -aG dialout $USER   # host user, if running outside Docker
```

## 3. Production WSGI (outside BlueOS)

The built-in `socketio.run()` dev server is fine for BlueOS/local use. For a
hardened standalone deployment, front it with Gunicorn + gevent-websocket:

```bash
pip install gunicorn gevent gevent-websocket

gunicorn \
  --worker-class geventwebsocket.gunicorn.workers.GeventWebSocketWorker \
  --workers 1 \
  --bind 0.0.0.0:5000 \
  app.main:app
```

Notes:
- Use exactly **one** worker — application state (SLAM map, mode) is in-process.
- Keep `DEBUG=false` (guards `allow_unsafe_werkzeug`).
- Put a reverse proxy (nginx/Caddy) in front for TLS if exposed beyond the
  vehicle LAN.

## 4. Network Modes

- **Host mode** (default, BlueOS): the app binds the host's port 5000
  directly. Remove explicit port bindings if you switch to host mode manually.
- **Bridge mode**: remove `network_mode: host` and rely on the `ports:`
  mapping. Adjust `CORS_ORIGINS` accordingly.

## 5. BlueOS Compatibility

- Tested against BlueOS 1.1+ extension manifest (`permissions` label schema).
- Requires a host with the TFmini-S on a mappable `/dev/tty*` device.

## 6. Environment Checklist

| Setting | Production value |
|---------|------------------|
| `DEBUG` | `false` |
| `LIDAR_API_TOKEN` | strong, unique |
| `CORS_ORIGINS` | explicit host list |
| `REQUIRE_WS_AUTH` | `true` on shared networks |
