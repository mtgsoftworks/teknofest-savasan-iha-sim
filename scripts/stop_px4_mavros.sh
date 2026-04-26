#!/usr/bin/env bash
set -euo pipefail

echo "[info] stopping all TEKNOFEST pipeline processes"

# MAVROS
pkill -f 'mavros_nod[e]' || true
pkill -f 'ros2 run mavros' || true

# PX4 + Gazebo
pkill -x px4 || true
pkill -f 'gz sim' || true
sleep 1
# Force kill if still alive
pkill -9 -x px4 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true

# Clean PX4 lock/socket files (prevents "PX4 server already running")
rm -f /tmp/px4-sock-* 2>/dev/null || true

# ROS gz bridge
pkill -f 'parameter_bridge' || true

# Rosbridge
pkill -f 'rosbridge_websocket' || true
pkill -f 'rosapi_node' || true

# ROS 2 nodes (offboard_takeoff package)
pkill -f 'offboard_takeoff' || true

echo "[ok] all processes stopped"
