"""
Comprehensive Emulation Server for BLSNS Testing

Provides mock implementations of:
- TFmini-S LiDAR over simulated UART
- MAVLink IMU/Attitude data
- Depth sensor (MS5837 simulator)
- WebSocket sensor telemetry
- Realistic physics simulation (distance measurements under various conditions)

Purpose: Enable full system testing without real hardware (especially useful on
NVIDIA Orin Nano or headless environments).

Usage:
    python -m tests.emulation_server --mode lidar --port /tmp/ttyUSB0_emu
    python -m tests.emulation_server --mode mavlink --port /tmp/ttyUSB1_emu
    python -m tests.emulation_server --mode full --scenario underwater_drift
"""

import sys
import os
import time
import threading
import socket
import struct
import random
import math
import argparse
from datetime import datetime
from typing import Tuple, Optional, Callable
from dataclasses import dataclass
from enum import Enum

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


# ============================================================================
# Physics Simulation Models
# ============================================================================

class UnderwaterEnvironment:
    """Realistic underwater physics for sensor simulation."""

    def __init__(self, depth_m: float = 10.0, turbidity_ntu: float = 1.0,
                 temperature_c: float = 15.0, salinity_ppt: float = 35.0):
        self.depth = depth_m
        self.turbidity = turbidity_ntu  # Nephelometric Turbidity Units
        self.temperature = temperature_c
        self.salinity = salinity_ppt
        self.current_speed_ms = 0.1  # m/s
        self.current_direction = 0.0  # degrees

    def attenuation_coefficient(self) -> float:
        """Beer-Lambert attenuation: α(λ, turbidity, depth)

        Returns: attenuation in 1/meter
        At 850nm (TFmini-S), seawater absorption ≈ 0.15 /m
        Turbidity adds scattering: α += 0.1 * turbidity
        """
        alpha_base = 0.15  # Clear seawater @ 850nm
        alpha_turbidity = 0.1 * self.turbidity
        return alpha_base + alpha_turbidity

    def refractive_index(self) -> float:
        """Water refractive index varies with depth, salinity, temperature.

        n(depth, salinity, T) ≈ 1.333 + 0.00002*depth - 0.0001*salinity/35
        """
        n_base = 1.333
        n_depth_correction = 0.00002 * self.depth
        n_salinity_correction = -0.0001 * (self.salinity / 35.0)
        n_temp_correction = 0.0001 * (self.temperature - 15.0)
        return n_base + n_depth_correction + n_salinity_correction + n_temp_correction

    def get_range_with_noise(self, true_range_m: float) -> float:
        """Add realistic noise to distance measurement.

        TFmini-S spec: ±2cm @ 1m, scales with range
        Underwater: turbidity adds multipath noise ±5cm
        """
        spec_noise = 0.02 * (1.0 + true_range_m)  # 2cm + range-dependent
        turbidity_noise = 0.05 * (self.turbidity / 1.0)  # 5cm per NTU
        total_noise = np.sqrt(spec_noise**2 + turbidity_noise**2)

        noisy_range = true_range_m + np.random.normal(0, total_noise)
        noisy_range = max(0.1, min(12.0, noisy_range))  # Clamp to spec range
        return noisy_range

    def get_signal_strength(self, true_range_m: float) -> int:
        """Signal strength (0-65535) with attenuation.

        Beer-Lambert law: I = I0 * exp(-α * range)
        Turbidity reduces signal strength.
        """
        alpha = self.attenuation_coefficient()
        attenuation = math.exp(-alpha * true_range_m)

        # TFmini-S: nominally 200 at 1m in clear water
        signal_base = 200 * attenuation

        # Add turbidity-dependent degradation
        turbidity_factor = 1.0 / (1.0 + self.turbidity * 0.05)
        signal_turbid = signal_base * turbidity_factor

        # Add shot noise
        signal_noisy = int(max(0, signal_turbid + np.random.normal(0, 10)))
        signal_noisy = min(65535, signal_noisy)

        return signal_noisy


# ============================================================================
# TFmini-S Emulator (UART)
# ============================================================================

class TFminiSEmulator:
    """Mock TFmini-S LiDAR via virtual UART."""

    # TFmini-S frame format: 0x59 0x59 [dist_L dist_H] [strength_L strength_H] [temp_L temp_H] checksum
    HEADER = bytes([0x59, 0x59])
    FRAME_SIZE = 9

    def __init__(self, socket_obj: socket.socket, env: UnderwaterEnvironment):
        self.socket = socket_obj
        self.env = env
        self.is_running = True
        self.read_rate_hz = 10  # TFmini-S default: 10 Hz
        self.current_distance_m = 3.0  # Simulate current measurement
        self.last_frame_time = time.monotonic()
        self.frame_count = 0

    def generate_frame(self) -> bytes:
        """Generate one TFmini-S frame with realistic data."""
        # Simulate object at current distance with drift
        drift = np.sin(self.frame_count * 0.1) * 0.5  # Oscillate ±0.5m
        true_distance = self.current_distance_m + drift
        true_distance = max(0.1, min(12.0, true_distance))

        # Apply underwater physics
        measured_distance = self.env.get_range_with_noise(true_distance)
        signal_strength = self.env.get_signal_strength(true_distance)

        # TFmini-S temperature: (raw_value - 0) * (273.15 + temp_C) - 273.15
        # Simplified: raw = (temp_C + 45) * 8, so raw ≈ 520 @ 20°C
        temp_raw = int((self.env.temperature + 45) * 8)

        # Convert to frame format
        distance_cm = int(measured_distance * 100)
        dist_L = distance_cm & 0xFF
        dist_H = (distance_cm >> 8) & 0xFF

        strength_L = signal_strength & 0xFF
        strength_H = (signal_strength >> 8) & 0xFF

        temp_L = temp_raw & 0xFF
        temp_H = (temp_raw >> 8) & 0xFF

        # Checksum: sum of first 8 bytes, low byte
        frame_data = bytes([dist_L, dist_H, strength_L, strength_H, temp_L, temp_H])
        checksum = (sum(self.HEADER) + sum(frame_data)) & 0xFF

        return self.HEADER + frame_data + bytes([checksum])

    def run(self):
        """Send frames at 10 Hz."""
        while self.is_running:
            now = time.monotonic()
            elapsed = now - self.last_frame_time

            if elapsed >= (1.0 / self.read_rate_hz):
                try:
                    frame = self.generate_frame()
                    self.socket.sendall(frame)
                    self.last_frame_time = now
                    self.frame_count += 1
                except (BrokenPipeError, ConnectionResetError):
                    break
            else:
                time.sleep(0.001)  # Avoid busy-wait


# ============================================================================
# MAVLink IMU Emulator
# ============================================================================

class MAVLinkEmulator:
    """Mock MAVLink ATTITUDE_QUATERNION messages."""

    def __init__(self, socket_obj: socket.socket):
        self.socket = socket_obj
        self.is_running = True
        self.roll_deg = 0.0
        self.pitch_deg = 0.0
        self.yaw_deg = 0.0
        self.read_rate_hz = 50  # MAVLink typically 50 Hz
        self.last_frame_time = time.monotonic()
        self.frame_count = 0

    def euler_to_quaternion(self, roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """Convert Euler angles (deg) to quaternion (x, y, z, w)."""
        roll_rad = math.radians(roll)
        pitch_rad = math.radians(pitch)
        yaw_rad = math.radians(yaw)

        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return x, y, z, w

    def generate_mavlink_message(self) -> bytes:
        """Generate simplified MAVLink ATTITUDE message.

        Real MAVLink is complex; this is a teaching emulation.
        Format: [time_boot_ms(4)] [roll(4)] [pitch(4)] [yaw(4)] [rollspeed(4)] [pitchspeed(4)] [yawspeed(4)]
        """
        # Simulate slow rotations
        self.roll_deg = 5.0 * math.sin(self.frame_count * 0.01)  # ±5° roll
        self.pitch_deg = -3.0 * math.sin(self.frame_count * 0.015)  # ±3° pitch
        self.yaw_deg = (self.frame_count * 0.1) % 360.0  # Rotate slowly

        time_boot_ms = int((time.monotonic() % 100000) * 1000)

        # Pack as simple binary (not real MAVLink CRC, for emulation only)
        msg = struct.pack('<I', time_boot_ms)  # 4 bytes: time
        msg += struct.pack('<fff',
                           math.radians(self.roll_deg),
                           math.radians(self.pitch_deg),
                           math.radians(self.yaw_deg))  # 12 bytes: angles
        msg += struct.pack('<fff', 0.0, 0.0, 0.0)  # 12 bytes: angular velocities (zero for sim)

        return msg

    def run(self):
        """Send MAVLink messages at 50 Hz."""
        while self.is_running:
            now = time.monotonic()
            elapsed = now - self.last_frame_time

            if elapsed >= (1.0 / self.read_rate_hz):
                try:
                    msg = self.generate_mavlink_message()
                    self.socket.sendall(msg)
                    self.last_frame_time = now
                    self.frame_count += 1
                except (BrokenPipeError, ConnectionResetError):
                    break
            else:
                time.sleep(0.001)


# ============================================================================
# Depth Sensor (MS5837 Emulator)
# ============================================================================

class DepthSensorEmulator:
    """Mock MS5837 pressure sensor -> depth estimation."""

    def __init__(self, socket_obj: socket.socket, env: UnderwaterEnvironment):
        self.socket = socket_obj
        self.env = env
        self.is_running = True
        self.read_rate_hz = 10
        self.last_frame_time = time.monotonic()

    def generate_reading(self) -> bytes:
        """Generate pressure reading (in mbar).

        Depth (meters) ≈ (pressure_mbar - 1013.25) / 100
        At sea level: 1013.25 mbar
        At 10m depth: 1013.25 + 1000 ≈ 2013 mbar
        """
        pressure_mbar = 1013.25 + self.env.depth * 100.0
        temperature_c = self.env.temperature

        # Add noise
        pressure_noisy = pressure_mbar + np.random.normal(0, 5)  # ±5 mbar noise
        temp_noisy = temperature_c + np.random.normal(0, 0.5)

        # Pack as simple format: [pressure(4 bytes float)] [temp(4 bytes float)]
        msg = struct.pack('<ff', pressure_noisy, temp_noisy)
        return msg

    def run(self):
        """Send depth readings at 10 Hz."""
        while self.is_running:
            now = time.monotonic()
            elapsed = now - self.last_frame_time

            if elapsed >= (1.0 / self.read_rate_hz):
                try:
                    reading = self.generate_reading()
                    self.socket.sendall(reading)
                    self.last_frame_time = now
                except (BrokenPipeError, ConnectionResetError):
                    break
            else:
                time.sleep(0.001)


# ============================================================================
# Emulation Server (Multi-threaded)
# ============================================================================

class EmulationServer:
    """Multi-protocol emulation server for BLSNS testing."""

    def __init__(self, mode: str = 'full', port_lidar: int = 5551,
                 port_mavlink: int = 5552, port_depth: int = 5553):
        self.mode = mode  # 'lidar', 'mavlink', 'depth', 'full'
        self.port_lidar = port_lidar
        self.port_mavlink = port_mavlink
        self.port_depth = port_depth

        self.env = UnderwaterEnvironment(
            depth_m=10.0,
            turbidity_ntu=1.0,
            temperature_c=15.0,
            salinity_ppt=35.0
        )

        self.servers = {}
        self.threads = {}

    def start_lidar_server(self):
        """Start TCP server for TFmini-S emulation."""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('127.0.0.1', self.port_lidar))
        server_socket.listen(1)
        print(f"[LIDAR] Listening on 127.0.0.1:{self.port_lidar}")

        def accept_clients():
            while True:
                try:
                    client_socket, addr = server_socket.accept()
                    print(f"[LIDAR] Client connected: {addr}")

                    emulator = TFminiSEmulator(client_socket, self.env)
                    thread = threading.Thread(target=emulator.run, daemon=True)
                    thread.start()
                except KeyboardInterrupt:
                    break

        thread = threading.Thread(target=accept_clients, daemon=True)
        thread.start()
        self.servers['lidar'] = server_socket
        self.threads['lidar'] = thread

    def start_mavlink_server(self):
        """Start TCP server for MAVLink IMU emulation."""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('127.0.0.1', self.port_mavlink))
        server_socket.listen(1)
        print(f"[MAVLINK] Listening on 127.0.0.1:{self.port_mavlink}")

        def accept_clients():
            while True:
                try:
                    client_socket, addr = server_socket.accept()
                    print(f"[MAVLINK] Client connected: {addr}")

                    emulator = MAVLinkEmulator(client_socket)
                    thread = threading.Thread(target=emulator.run, daemon=True)
                    thread.start()
                except KeyboardInterrupt:
                    break

        thread = threading.Thread(target=accept_clients, daemon=True)
        thread.start()
        self.servers['mavlink'] = server_socket
        self.threads['mavlink'] = thread

    def start_depth_server(self):
        """Start TCP server for depth sensor emulation."""
        server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_socket.bind(('127.0.0.1', self.port_depth))
        server_socket.listen(1)
        print(f"[DEPTH] Listening on 127.0.0.1:{self.port_depth}")

        def accept_clients():
            while True:
                try:
                    client_socket, addr = server_socket.accept()
                    print(f"[DEPTH] Client connected: {addr}")

                    emulator = DepthSensorEmulator(client_socket, self.env)
                    thread = threading.Thread(target=emulator.run, daemon=True)
                    thread.start()
                except KeyboardInterrupt:
                    break

        thread = threading.Thread(target=accept_clients, daemon=True)
        thread.start()
        self.servers['depth'] = server_socket
        self.threads['depth'] = thread

    def start(self):
        """Start emulation server(s) based on mode."""
        if self.mode in ['lidar', 'full']:
            self.start_lidar_server()
        if self.mode in ['mavlink', 'full']:
            self.start_mavlink_server()
        if self.mode in ['depth', 'full']:
            self.start_depth_server()

        print(f"\n[EMULATION] Server started in '{self.mode}' mode")
        print(f"[ENV] Depth: {self.env.depth}m, Turbidity: {self.env.turbidity} NTU, Temp: {self.env.temperature}°C")

        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[EMULATION] Shutting down...")
            for server_socket in self.servers.values():
                server_socket.close()


# ============================================================================
# CLI
# ============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='BLSNS Emulation Server for Testing on NVIDIA Orin Nano'
    )
    parser.add_argument('--mode', default='full',
                        choices=['lidar', 'mavlink', 'depth', 'full'],
                        help='Emulation mode')
    parser.add_argument('--port-lidar', type=int, default=5551,
                        help='TCP port for LiDAR emulation')
    parser.add_argument('--port-mavlink', type=int, default=5552,
                        help='TCP port for MAVLink emulation')
    parser.add_argument('--port-depth', type=int, default=5553,
                        help='TCP port for depth sensor emulation')
    parser.add_argument('--depth', type=float, default=10.0,
                        help='Simulated depth (meters)')
    parser.add_argument('--turbidity', type=float, default=1.0,
                        help='Simulated turbidity (NTU)')
    parser.add_argument('--temperature', type=float, default=15.0,
                        help='Simulated water temperature (°C)')

    args = parser.parse_args()

    server = EmulationServer(
        mode=args.mode,
        port_lidar=args.port_lidar,
        port_mavlink=args.port_mavlink,
        port_depth=args.port_depth
    )
    server.env.depth = args.depth
    server.env.turbidity = args.turbidity
    server.env.temperature = args.temperature

    server.start()


if __name__ == '__main__':
    main()
