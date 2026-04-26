#!/usr/bin/env bash
set -euo pipefail

# TEKNOFEST - Spawn an HSS (no-fly) zone cylinder in Gazebo
# Usage: HSS_NAME=hss_zone_2 HSS_X=300 HSS_Y=100 HSS_RADIUS=30 scripts/spawn_hss_zone.sh

set +u
source /opt/ros/jazzy/setup.bash
set -u

WORLD_NAME="${WORLD_NAME:-teknofest_competition}"
HSS_NAME="${HSS_NAME:-hss_zone_1}"
HSS_X="${HSS_X:-300}"
HSS_Y="${HSS_Y:-0}"
HSS_Z="${HSS_Z:-0}"
HSS_RADIUS="${HSS_RADIUS:-50}"

MODEL_URI="model://teknofest_hss_zone"

# Ensure core bridge is running for /create service
if ! pgrep -f "parameter_bridge.*/world/${WORLD_NAME}/create" >/dev/null 2>&1; then
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

echo "[info] spawning HSS zone: ${HSS_NAME} at (${HSS_X}, ${HSS_Y}) radius=${HSS_RADIUS}m"
ros2 service call /world/"${WORLD_NAME}"/create ros_gz_interfaces/srv/SpawnEntity \
  "{entity_factory: {name: '${HSS_NAME}', allow_renaming: false, sdf_filename: '${MODEL_URI}', pose: {position: {x: ${HSS_X}, y: ${HSS_Y}, z: ${HSS_Z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, relative_to: 'world'}}" \
  >/tmp/spawn_"${HSS_NAME}".log

echo "[ok] HSS zone ${HSS_NAME} spawned at (${HSS_X}, ${HSS_Y})"
