FROM python:3.11-slim-bullseye

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    git \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Copy requirements first for caching
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy application
COPY app/ ./app/
COPY tests/ ./tests/

# Expose port
EXPOSE 5000

# BlueOS Labels
LABEL version="1.0.0"
LABEL permissions='{\
  "ExposedPorts": {\
    "5000/tcp": {}\
  },\
  "HostConfig": {\
    "Binds": ["/usr/blueos/extensions/lidar-slam:/app/data"],\
    "Privileged": true,\
    "Devices": [\
      {"PathOnHost": "/dev/ttyUSB0", "PathInContainer": "/dev/ttyUSB0", "CgroupPermissions": "rwm"},\
      {"PathOnHost": "/dev/ttyUSB1", "PathInContainer": "/dev/ttyUSB1", "CgroupPermissions": "rwm"},\
      {"PathOnHost": "/dev/ttyAMA0", "PathInContainer": "/dev/ttyAMA0", "CgroupPermissions": "rwm"}\
    ],\
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
