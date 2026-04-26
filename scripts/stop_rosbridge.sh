#!/usr/bin/env bash
set -euo pipefail

echo "[info] stopping rosbridge/rosapi"
pkill -f 'rosbridge_websocket' || true
pkill -f 'rosapi_node' || true
pkill -f 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml' || true
echo "[ok] stopped"

