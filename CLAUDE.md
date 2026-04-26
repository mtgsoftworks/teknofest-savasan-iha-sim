# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

TEKNOFEST SIHA simulation stack — a PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy + MAVROS drone simulation environment running on WSL2 (Ubuntu-24.04). The host is Windows 11; all runtime commands execute inside WSL.

## Commands

All commands run through WSL. Prefix from PowerShell:

```
wsl -d Ubuntu-24.04 -- bash -lc "<command>"
```

### Build

```bash
cd /mnt/c/Users/mtg/Desktop/gazebo_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select offboard_takeoff
source install/setup.bash
```

### Run stack (PX4 + Gazebo + MAVROS)

```bash
/mnt/c/Users/mtg/Desktop/gazebo_project/scripts/start_px4_mavros.sh
```

Stop:

```bash
/mnt/c/Users/mtg/Desktop/gazebo_project/scripts/stop_px4_mavros.sh
```

### Run offboard mission

```bash
WAYPOINTS='0,0,3;6,0,3;6,6,3;0,0,3' /mnt/c/Users/mtg/Desktop/gazebo_project/scripts/run_offboard_mission.sh
```

Waypoint format: `x,y,z` pairs separated by semicolons.

### Full pipeline (single command)

```bash
/mnt/c/Users/mtg/Desktop/gazebo_project/scripts/run_savasan_iha_pipeline.sh
```

Runs all 5 steps sequentially: PX4/Gazebo/MAVROS → Rosbridge → Reset+Spawn+IMU/GPS → Camera/Lidar spawn → Offboard mission.

### Lint / Test

```bash
cd /mnt/c/Users/mtg/Desktop/gazebo_project/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon test --packages-select offboard_takeoff   # runs ament_flake8, ament_pep257, ament_copyright
```

### Check logs

```bash
tail -n 80 /tmp/px4_sitl.log
tail -n 80 /tmp/mavros.log
```

## Architecture

### Shell scripts (`scripts/`)

Orchestration layer — each script is standalone with `set -euo pipefail`:

| Script | Purpose |
|---|---|
| `start_px4_mavros.sh` | Launches PX4 SITL + Gazebo, applies SITL arming param patches (CBRK_SUPPLY_CHK, NAV_DLL_ACT), then starts MAVROS node. Waits for heartbeat before returning. |
| `stop_px4_mavros.sh` | Kills mavros_node, px4, and `gz sim` processes. |
| `start_rosbridge.sh` | Launches rosbridge websocket on :9090 (for MCP/Codex integration). |
| `stop_rosbridge.sh` | Kills rosbridge/rosapi processes. |
| `mcp_reset_spawn_sensor.sh` | Pauses+resets Gazebo world time, removes old model, spawns x500, starts ros_gz_bridge for IMU/NavSat, reads one sample each. |
| `mcp_spawn_lidar_camera_bridge.sh` | Spawns `x500_mono_cam` and `x500_lidar_front` models, bridges camera/Image + lidar/LaserScan topics to ROS. |
| `run_offboard_mission.sh` | Builds `offboard_takeoff` package then runs `offboard_mission` node with waypoint params. |
| `run_savasan_iha_pipeline.sh` | Full 5-step pipeline runner. Collects all runtime logs into `/tmp/savasan_iha_<RUN_ID>/`. |

Scripts use env vars for configuration (all have defaults): `PX4_DIR`, `PX4_SIM_MODEL`, `FCU_URL`, `GCS_URL`, `WORLD_NAME`, `MODEL_NAME`, `WAYPOINTS`, `HOLD_SEC_EACH_WP`, `POS_TOL`.

### ROS 2 package (`ros2_ws/src/offboard_takeoff/`)

Python ament package with two executable nodes:

- **`offboard_takeoff`** (`offboard_takeoff_node.py`): Arms vehicle, switches to OFFBOARD mode, publishes setpoint at `target_altitude`, waits until altitude reached. Uses force-arm fallback (MAV_CMD 400 with param2=21196) for SITL where pre-arm checks block.
- **`offboard_mission`** (`offboard_mission_node.py`): Same arm/offboard sequence, then visits waypoints sequentially. Each waypoint must be reached within `position_tolerance_m` and held for `hold_sec_each_wp`.

Both nodes share the same state machine: wait for MAVROS services → wait for FCU connection → publish setpoint burst (100-120 msgs) → set OFFBOARD mode → arm → execute. They use `_spin_sleep` instead of timers to interleave `spin_once` with sequential logic.

### Gazebo models

- `x500` — base quadrotor with IMU + NavSat (odometry plugin, no camera)
- `x500_mono_cam` — camera variant (`x500_cam_0`)
- `x500_lidar_front` — lidar variant (`x500_lidar_0`)
- `x500_vision` — odometry-only variant (mentioned in README, not used in current scripts)

### Topic bridges (ros_gz_bridge)

Core bridge: `/clock`, `/world/<name>/control`, `/world/<name>/create`, `/world/<name>/remove`, `/world/<name>/set_pose`.

Sensor bridges are started per-model and bridge Gazebo topics to ROS 2 equivalents (IMU, NavSat, Image, CameraInfo, LaserScan).

### MCP integration

Rosbridge on :9090 enables `ros-mcp` (via `uv/uvx`) for Codex/AI agent tool access to ROS topics and services.
