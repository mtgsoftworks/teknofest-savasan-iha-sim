#!/usr/bin/env bash
set -euo pipefail

# TEKNOFEST - Remove an HSS (no-fly) zone cylinder from Gazebo
# Usage: HSS_NAME=hss_zone_1 scripts/remove_hss_zone.sh

set +u
source /opt/ros/jazzy/setup.bash
set -u

WORLD_NAME="${WORLD_NAME:-teknofest_competition}"
HSS_NAME="${HSS_NAME:-hss_zone_1}"

# Ensure core bridge is running for /remove service
if ! pgrep -f "parameter_bridge.*/world/${WORLD_NAME}/remove" >/dev/null 2>&1; then
  echo "[info] starting ros_gz core bridge"
  nohup ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /world/"${WORLD_NAME}"/control@ros_gz_interfaces/srv/ControlWorld \
    /world/"${WORLD_NAME}"/create@ros_gz_interfaces/srv/SpawnEntity \
    /world/"${WORLD_NAME}"/remove@ros_gz_interfaces/srv/DeleteEntity \
    /world/"${WORLD_NAME}"/set_pose@ros_gz_interfaces/srv/SetEntityPose \
    >/tmp/ros_gz_bridge_core.log 2>&1 &
  sleep 2
fi

echo "[info] removing HSS zone: ${HSS_NAME}"
ros2 service call /world/"${WORLD_NAME}"/remove ros_gz_interfaces/srv/DeleteEntity \
  "{entity: {name: '${HSS_NAME}', type: 2}}" \
  >/tmp/remove_"${HSS_NAME}".log || true

echo "[ok] HSS zone ${HSS_NAME} removed"
