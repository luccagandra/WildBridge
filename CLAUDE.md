# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WildBridge is a ground station system for controlling DJI drones via an Android app (running on DJI RC Pro) that exposes HTTP/TCP/RTSP interfaces, consumed by a Python library and ROS 2 nodes.

## Architecture

Three-layer stack:

1. **Android App** (`WildBridgeApp/android-sdk-v5-as/`) — Runs on DJI RC Pro. Exposes:
   - HTTP server on port 8080 (POST commands)
   - TCP socket on port 8081 (JSON telemetry stream at 20Hz)
   - RTSP on port 8554 (live video)

2. **Python Interface** (`GroundStation/Python/djiInterface.py`) — `DJIInterface` class wraps HTTP commands and TCP telemetry. Direct usage without ROS.

3. **ROS 2 Nodes** (`GroundStation/ROS/`) — Three packages:
   - `dji_controller`: Bridges DJI interface to ROS topics/services (command subscribers + telemetry publishers)
   - `drone_videofeed`: RTSP → ROS Image topic publisher
   - `wildview_bringup`: Multi-drone launch orchestration with per-drone namespaces (`drone_1`, `drone_2`, ...)

## Build & Run

### ROS 2 (Humble)

```bash
cd GroundStation/ROS
pip install -r requirements.txt
colcon build
source install/setup.bash

# Launch multi-drone setup
ros2 launch wildview_bringup swarm_connection.launch.py

# Single drone nodes
ros2 run dji_controller dji_node --ros-args -p ip_rc:=192.168.1.100
ros2 run drone_videofeed rtsp_node --ros-args -p ip_rc:=192.168.1.100
```

### Tests (ROS packages)

```bash
cd GroundStation/ROS
colcon test
colcon test-result --verbose
```

Tests cover copyright headers (`test_copyright.py`), flake8 linting (`test_flake8.py`), and pep257 docstrings (`test_pep257.py`).

### Android App

Open `WildBridgeApp/android-sdk-v5-as` in Android Studio Koala 2024.1.1+. Requires a DJI developer API key in `local.properties`:

```
AIRCRAFT_API_KEY="your_api_key"
```

Build and deploy to DJI RC Pro via Android Studio. No command-line build is used.

### Python (standalone)

```bash
pip install -r GroundStation/Python/requirements.txt
python GroundStation/Python/djiInterface.py <RC_IP>
```

## Key Design Decisions

- **Multi-drone**: Each drone gets a unique ROS namespace. `swarm_connection.launch.py` maps MAC addresses to IPs and launches nodes per drone.
- **Telemetry transport**: TCP sockets (not HTTP polling) at 20Hz; the `DJIInterface` receiver thread buffers the latest JSON state.
- **ROS interface**: Commands arrive on `<namespace>/command/<action>` topics (String messages); telemetry is published on `<namespace>/<field>` topics.
- **HTTP port derivation**: `drone_videofeed` derives the HTTP port from the last octet of the RC IP address (dynamic assignment for multi-drone setups).
- **RTSP auth**: Hardcoded credentials `aaa:aaa` in the RTSP URL `rtsp://aaa:aaa@{ip}:8554/streaming/live/1`.

## Communication Ports

| Port | Protocol | Direction | Purpose |
|------|----------|-----------|---------|
| 8080 | HTTP POST | GS → Drone | Commands (takeoff, land, goto, gimbal, etc.) |
| 8081 | TCP | Drone → GS | Real-time telemetry (JSON lines) |
| 8554 | RTSP | Drone → GS | H.264 video stream |
