#!/usr/bin/env bash
set -euo pipefail

LOG_FILE="${ROSBRIDGE_LOG:-/tmp/rosbridge.log}"
ROS_DISTRO_SETUP="${ROS_DISTRO_SETUP:-/opt/ros/jazzy/setup.bash}"

if [ ! -f "$ROS_DISTRO_SETUP" ]; then
  echo "[error] ROS setup not found: $ROS_DISTRO_SETUP"
  exit 1
fi

echo "[info] stopping old rosbridge/rosapi"
pkill -f 'rosbridge_websocket' || true
pkill -f 'rosapi_node' || true
pkill -f 'ros2 launch rosbridge_server rosbridge_websocket_launch.xml' || true

rm -f "$LOG_FILE"
echo "[info] starting rosbridge"
setsid -f bash -lc "
  set -euo pipefail
  set +u
  source \"$ROS_DISTRO_SETUP\"
  set -u
  exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
" >>"$LOG_FILE" 2>&1

sleep 5

if ss -ltn | grep -q ':9090'; then
  echo "[ok] rosbridge is listening on :9090"
  echo "[ok] log: $LOG_FILE"
else
  echo "[error] rosbridge failed to start"
  tail -n 80 "$LOG_FILE" || true
  exit 1
fi
