# Building & Deployment Guide

**Targets**: 
- NVIDIA Orin Nano Super 8GB (ARM64)
- Docker containers (multi-arch)
- Local development (x86_64/ARM64)

---

## Quick Start

### Option 1: Docker (Recommended for Orin Nano)

```bash
# On x86_64 host (build machine)
git clone https://github.com/Leonidy431/tfmini_lidar.git
cd tfmini_lidar

# Build for ARM64 (Orin Nano)
docker buildx build --platform linux/arm64 \
  -t blueos-lidar:arm64 \
  -f Dockerfile.arm64 \
  .

# Tag and push to registry
docker tag blueos-lidar:arm64 myregistry.azurecr.io/blueos-lidar:arm64
docker push myregistry.azurecr.io/blueos-lidar:arm64

# On Orin Nano (target device)
ssh ubuntu@<ORIN_IP>
docker pull myregistry.azurecr.io/blueos-lidar:arm64

docker run -d \
  --name blsns \
  --restart unless-stopped \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  -v /data/blsns:/app/data \
  -p 5000:5000 \
  myregistry.azurecr.io/blueos-lidar:arm64

# Check logs
docker logs -f blsns

# Test
curl http://localhost:5000/api/health
```

### Option 2: Native Build on Orin Nano

```bash
# SSH into Orin Nano
ssh ubuntu@<ORIN_IP>

# Clone and enter directory
git clone https://github.com/Leonidy431/tfmini_lidar.git
cd tfmini_lidar

# Create virtual environment
python3.10 -m venv venv
source venv/bin/activate

# Install dependencies (slow on ARM64, ~15 minutes)
pip install -r requirements.txt

# Run tests
python3.10 -m pytest tests/ -v

# Start application
python3.10 -m app.main
```

---

## Detailed Build Process

### Prerequisites

**On Build Machine (x86_64)**:
```bash
# Ubuntu 22.04 LTS
sudo apt install -y docker.io docker-buildx binfmt-support qemu-user-static

# Enable Docker BuildKit
docker buildx create --name mybuilder --use

# Verify ARM64 emulation
docker run --rm --platform linux/arm64 ubuntu uname -m  # Should print: aarch64
```

**On Target Machine (Orin Nano)**:
- JetPack 6.0+ installed (Ubuntu 22.04 + CUDA 12.x)
- 8GB RAM
- 128GB microSD (UHS-II) or NVMe SSD
- USB 3.0 hub for sensors

---

## Building from Source

### Step 1: Verify System

```bash
# On Orin Nano
cat /etc/os-release | grep VERSION

# Expected:
# VERSION="22.04.3 LTS (Jammy Jellyfish)"
# VARIANT_ID=ubuntu

# Check CUDA
nvcc --version

# Expected:
# nvcc: NVIDIA (R) Cuda compiler driver
# Cuda compilation tools, release 12.2, V12.2.13
```

### Step 2: Install Dependencies

```bash
# First, see docs/ORIN_NANO_SETUP.md Part 2.2 for complete list
# Quick summary:

sudo apt update
sudo apt install -y python3.10 python3-pip git build-essential cmake

# Python packages
pip install --upgrade pip setuptools wheel
pip install -r requirements.txt

# Open3D from source (important for ARM64)
git clone --depth 1 --branch v0.18.0 https://github.com/isl-org/Open3D.git
cd Open3D && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4  # Limited to 4 to avoid OOM
make install
pip install ..
```

### Step 3: Verify Installation

```bash
# Check Python imports
python3.10 -c "
import numpy as np
import scipy
import sklearn
import torch
import open3d
import flask
import pyserial
print('✓ All dependencies OK')
"

# Run lightweight tests
python3.10 -m pytest tests/test_driver_mock.py -v -q

# Expected output:
# tests/test_driver_mock.py::TestFrameStreamParsing::test_process_buffer_single_frame PASSED
# ... (all ~15 tests pass in ~5s)
```

### Step 4: Configure Environment

```bash
# Create .env file
cat > ~/.env << 'EOF'
# Hardware
LIDAR_PORT=/dev/ttyUSB0
LIDAR_BAUDRATE=115200
MAVLINK_PORT=/dev/ttyUSB1

# SLAM (tuned for Orin 8GB)
SLAM_MAX_POINTS=100
ICP_REGISTRATION_TIME_LIMIT_MS=50

# Data quality
DATA_QUALITY_BUFFER_SIZE=50
OUTLIER_STD_THRESHOLD=3.0

# Web API
LIDAR_API_TOKEN=your_secure_token_here
WEB_PORT=5000
REQUIRE_WS_AUTH=true

# Logging
DEBUG=false
LOG_LEVEL=INFO
DATA_DIR=/data/blsns

# Orin-specific
MAX_CPU_THREADS=4
EOF

source ~/.env
```

### Step 5: Run Application

```bash
# Development mode
python3.10 -m app.main

# Production mode (with Gunicorn)
pip install gunicorn
gunicorn --bind 0.0.0.0:5000 \
         --workers 2 \
         --worker-class sync \
         --timeout 120 \
         app.main:app

# Headless mode (no web UI)
python3.10 -m app.main --headless
```

---

## Docker Build Options

### Option A: Build & Push to Container Registry

```bash
# Build for specific platform
docker buildx build --platform linux/arm64 \
  --tag myregistry.azurecr.io/blueos-lidar:arm64-latest \
  --tag myregistry.azurecr.io/blueos-lidar:arm64-v1.0.0 \
  -f Dockerfile.arm64 \
  .

# Push to Azure Container Registry
az acr login --name myregistry
docker push myregistry.azurecr.io/blueos-lidar:arm64-latest

# Or push to Docker Hub
docker tag blueos-lidar:arm64 yourname/blueos-lidar:arm64
docker login
docker push yourname/blueos-lidar:arm64
```

### Option B: Build Locally on Orin Nano

```bash
# SSH into Orin
ssh ubuntu@<ORIN_IP>

# Build image
cd ~/tfmini_lidar
docker build -f Dockerfile.arm64 \
  --tag blueos-lidar:local \
  .

# Run locally (no push needed)
docker run -d \
  --name blsns \
  --device /dev/ttyUSB0:/dev/ttyUSB0 \
  -p 5000:5000 \
  blueos-lidar:local
```

### Option C: Multi-architecture Image (CI/CD)

```bash
# In CI pipeline (GitHub Actions, GitLab CI):
docker buildx build \
  --platform linux/amd64,linux/arm64 \
  --tag myregistry.azurecr.io/blueos-lidar:latest \
  --push \
  -f Dockerfile.arm64 \
  .

# Result: Single "latest" tag works on both x86_64 and ARM64
docker run blueos-lidar:latest  # Auto-selects correct arch
```

---

## Testing

### Unit Tests

```bash
# Fast tests (no numpy/Open3D)
python3.10 -m pytest tests/test_driver_mock.py -v
python3.10 -m pytest tests/test_security.py -v
python3.10 -m pytest tests/test_data_quality.py -v

# Full test suite (requires all dependencies)
python3.10 -m pytest tests/ -v --cov=app --cov-report=html

# Expected: 175/175 tests PASS
```

### Integration Tests with Emulation

```bash
# Terminal 1: Start emulation server
python3.10 -m tests.emulation_server --mode full

# Terminal 2: Run application
export LIDAR_PORT="127.0.0.1:5551"
python3.10 -m app.main --headless

# Terminal 3: Run integration tests
python3.10 -m pytest tests/test_lidar.py -v -s

# Verify
curl http://localhost:5000/api/status | jq .
```

### Performance Profiling

```bash
# CPU profiling
python3.10 -m cProfile -s cumulative -m pytest tests/test_slam_physics.py > profile.txt

# Memory profiling
pip install memory-profiler
python3.10 -m memory_profiler app/main.py

# Flamegraph (advanced)
pip install py-spy
sudo py-spy record -o profile.svg -- python3.10 -m app.main
```

---

## Deployment

### Systemd Service (Native)

```bash
# Create service file
sudo tee /etc/systemd/system/blsns.service > /dev/null << 'EOF'
[Unit]
Description=BlueOS LiDAR SLAM Navigation System
After=network.target

[Service]
Type=simple
User=ubuntu
WorkingDirectory=/home/ubuntu/tfmini_lidar
EnvironmentFile=/home/ubuntu/.env
ExecStart=/home/ubuntu/tfmini_lidar/venv/bin/python3 -m app.main
Restart=on-failure
RestartSec=5s
MemoryLimit=4G
CPUQuota=400%

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
EOF

# Enable and start
sudo systemctl daemon-reload
sudo systemctl enable blsns
sudo systemctl start blsns

# Monitor
sudo journalctl -u blsns -f
```

### Docker Compose (Orin Nano with Companion Services)

```bash
# Create docker-compose.yml
cat > docker-compose.yml << 'EOF'
version: '3.8'

services:
  blsns:
    image: blueos-lidar:arm64
    container_name: blsns
    restart: unless-stopped
    devices:
      - /dev/ttyUSB0:/dev/ttyUSB0  # LiDAR
      - /dev/ttyUSB1:/dev/ttyUSB1  # MAVLink
    volumes:
      - /data/blsns:/app/data
    ports:
      - "5000:5000"
    environment:
      LIDAR_PORT: /dev/ttyUSB0
      MAVLINK_PORT: /dev/ttyUSB1
      WEB_PORT: "5000"
      LOG_LEVEL: INFO
    healthcheck:
      test: ["CMD", "curl", "-f", "http://localhost:5000/api/health"]
      interval: 30s
      timeout: 10s
      retries: 3
      start_period: 40s
    logging:
      driver: "json-file"
      options:
        max-size: "10m"
        max-file: "3"

  # Optional: Prometheus monitoring
  prometheus:
    image: prom/prometheus:arm64v8
    container_name: prometheus
    restart: unless-stopped
    volumes:
      - ./prometheus.yml:/etc/prometheus/prometheus.yml
      - /data/prometheus:/prometheus
    ports:
      - "9090:9090"
    command:
      - '--config.file=/etc/prometheus/prometheus.yml'
      - '--storage.tsdb.path=/prometheus'

  # Optional: Grafana dashboards
  grafana:
    image: grafana/grafana:latest-arm64
    container_name: grafana
    restart: unless-stopped
    ports:
      - "3000:3000"
    environment:
      GF_SECURITY_ADMIN_PASSWORD: admin
    volumes:
      - /data/grafana:/var/lib/grafana

EOF

# Deploy
docker-compose up -d

# Check logs
docker-compose logs -f blsns
```

---

## Troubleshooting Build

### Issue: "No space left on device"

```bash
# Check disk usage
df -h

# If building Docker image, reduce intermediate layers
docker buildx prune --all  # Clear builder cache

# Build with smaller context
docker buildx build \
  --platform linux/arm64 \
  --cache-to type=registry,ref=myregistry.azurecr.io/blueos-lidar:buildcache \
  -f Dockerfile.arm64 \
  .
```

### Issue: "Out of memory during Open3D build"

```bash
# Reduce parallel compilation
cd Open3D/build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j2  # Use only 2 cores

# Or use pre-built binary (if available)
pip install open3d-python
```

### Issue: "QEMU timeout building ARM64 on x86_64"

```bash
# Increase qemu resource limits
docker buildx create --name mybuilder \
  --driver-opt network=host \
  --driver-opt image=moby/buildkit:master

# Or build natively on Orin Nano instead
```

### Issue: "ImportError: No module named 'app.main'"

```bash
# Verify Python path
export PYTHONPATH=/home/ubuntu/tfmini_lidar:$PYTHONPATH

# Or use absolute imports
python3.10 -m app.main
```

---

## Performance Tuning

### For Orin Nano 8GB

```bash
# Reduce SLAM point limit
export SLAM_MAX_POINTS=50

# Reduce particle filter size
export PARTICLE_FILTER_COUNT=500

# Reduce data quality buffer
export DATA_QUALITY_BUFFER_SIZE=20

# Limit threads
export MAX_CPU_THREADS=4

# Monitor during operation
watch -n 1 'nvidia-smi && free -h && top -bn1 | head -20'
```

### For x86_64 (Development)

```bash
# Higher quality settings
export SLAM_MAX_POINTS=500
export PARTICLE_FILTER_COUNT=2000
export DATA_QUALITY_BUFFER_SIZE=100
export MAX_CPU_THREADS=16

# Enable debug logging
export LOG_LEVEL=DEBUG
export DEBUG=true
```

---

## Cross-Compilation Reference

| Target | Build Machine | Command | Notes |
|--------|---------------|---------|-------|
| Orin Nano (ARM64) | x86_64 | `docker buildx build --platform linux/arm64` | Recommended |
| Orin Nano (ARM64) | ARM64 (native) | `docker build -f Dockerfile.arm64` | Slow (~1 hour) |
| x86_64 | x86_64 | `docker build -f Dockerfile` | Fast (~10 min) |
| Multi-arch | x86_64 + CI | `docker buildx build --platform linux/amd64,linux/arm64` | For registries |

---

## Next Steps

1. **Choose deployment method**: Docker (recommended) or native
2. **See docs/ORIN_NANO_SETUP.md** for hardware setup
3. **Run tests**: Verify installation with `pytest`
4. **Start application**: Use systemd or Docker
5. **Monitor**: Check logs and health checks
6. **Deploy to production**: Use docker-compose or BlueOS integration

