# ---- Build stage: compile wheels with build toolchain ----
FROM python:3.11-slim-bullseye AS builder

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY requirements.txt .
# Build wheels so the runtime image needs no compiler
RUN pip wheel --no-cache-dir --wheel-dir /wheels -r requirements.txt


# ---- Runtime stage: minimal image, no build tools ----
FROM python:3.11-slim-bullseye AS runtime

# Runtime-only shared libraries (Open3D needs GL/glib) + curl for healthcheck
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender1 \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Install prebuilt wheels
COPY requirements.txt .
COPY --from=builder /wheels /wheels
RUN pip install --no-cache-dir --no-index --find-links=/wheels -r requirements.txt \
    && rm -rf /wheels

# Copy application only (tests excluded via .dockerignore)
COPY app/ ./app/

# Run as a non-root user; grant access to serial devices via the dialout group
RUN useradd -r -u 1000 -g dialout -d /app lidar \
    && mkdir -p /app/data \
    && chown -R lidar:dialout /app
USER lidar

# Expose port
EXPOSE 5000

# Health check endpoint
HEALTHCHECK --interval=30s --timeout=10s --start-period=60s --retries=3 \
  CMD curl -f http://localhost:5000/api/health || exit 1

# BlueOS Labels
LABEL version="1.0.1"
LABEL permissions='{\
  "ExposedPorts": {\
    "5000/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": ["/usr/blueos/extensions/lidar-slam:/app/data"],\
    "Privileged": false,\
    "Devices": [\
      {"PathOnHost": "/dev/ttyUSB0", "PathInContainer": "/dev/ttyUSB0", "CgroupPermissions": "rwm"},\
      {"PathOnHost": "/dev/ttyUSB1", "PathInContainer": "/dev/ttyUSB1", "CgroupPermissions": "rwm"},\
      {"PathOnHost": "/dev/ttyAMA0", "PathInContainer": "/dev/ttyAMA0", "CgroupPermissions": "rwm"}\
    ],\
    "CapAdd": ["SYS_RAWIO"],\
    "PortBindings": {\
      "5000/tcp": [{"HostPort": "5000"}]\
    },\
    "NetworkMode": "host"\
  }\
}'
LABEL authors='["BlueOS LiDAR Team"]'
LABEL docs='https://github.com/blueos-lidar-slam'
LABEL website='https://github.com/blueos-lidar-slam'
LABEL support='https://github.com/blueos-lidar-slam/issues'
LABEL readme='https://raw.githubusercontent.com/blueos-lidar-slam/main/README.md'
LABEL links='{\
  "website": "https://github.com/blueos-lidar-slam",\
  "support": "https://github.com/blueos-lidar-slam/issues"\
}'
LABEL requirements="Benewake TFmini-S LiDAR connected via USB-UART"
LABEL type="device-integration"
LABEL tags="lidar, slam, navigation, mapping, localization, tfmini"

# Entry point
CMD ["python", "-m", "app.main"]
