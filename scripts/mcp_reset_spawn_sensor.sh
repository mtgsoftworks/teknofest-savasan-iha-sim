#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

WORLD_NAME="${WORLD_NAME:-default}"
MODEL_NAME="${MODEL_NAME:-x500_0}"
MODEL_URI="${MODEL_URI:-model://x500}"
MODEL_Z="${MODEL_Z:-0.2}"
BRIDGE_LOG="${BRIDGE_LOG:-/tmp/ros_gz_sensor_bridge_${MODEL_NAME}.log}"

ensure_core_bridge() {
  if pgrep -f "parameter_bridge /clock@rosgraph_msgs/msg/Clock\\[gz.msgs.Clock .* /world/${WORLD_NAME}/control@ros_gz_interfaces/srv/ControlWorld" >/dev/null 2>&1; then
    return
  fi

  nohup ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /world/"${WORLD_NAME}"/control@ros_gz_interfaces/srv/ControlWorld \
    /world/"${WORLD_NAME}"/create@ros_gz_interfaces/srv/SpawnEntity \
    /world/"${WORLD_NAME}"/remove@ros_gz_interfaces/srv/DeleteEntity \
    /world/"${WORLD_NAME}"/set_pose@ros_gz_interfaces/srv/SetEntityPose \
    >/tmp/ros_gz_bridge_core.log 2>&1 &
  sleep 1
}

spawn_model() {
  ros2 service call /world/"${WORLD_NAME}"/create ros_gz_interfaces/srv/SpawnEntity \
    "{entity_factory: {name: '${MODEL_NAME}', allow_renaming: false, sdf_filename: '${MODEL_URI}', pose: {position: {x: 0.0, y: 0.0, z: ${MODEL_Z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, relative_to: 'world'}}" \
    >/tmp/spawn_"${MODEL_NAME}".log
}

echo "[info] ensuring ros_gz core bridge"
ensure_core_bridge

echo "[info] pause + reset time"
ros2 service call /world/"${WORLD_NAME}"/control ros_gz_interfaces/srv/ControlWorld \
  "{world_control: {pause: true, reset: {time_only: true}}}" >/tmp/world_reset_time.log

echo "[info] removing old model if exists: ${MODEL_NAME}"
ros2 service call /world/"${WORLD_NAME}"/remove ros_gz_interfaces/srv/DeleteEntity \
  "{entity: {name: '${MODEL_NAME}', type: 2}}" >/tmp/remove_"${MODEL_NAME}".log || true

echo "[info] spawning model: ${MODEL_NAME} (${MODEL_URI})"
spawn_model

echo "[info] unpausing world"
ros2 service call /world/"${WORLD_NAME}"/control ros_gz_interfaces/srv/ControlWorld \
  "{world_control: {pause: false}}" >/tmp/world_unpause.log

IMU_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/base_link/sensor/imu_sensor/imu"
NAVSAT_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/base_link/sensor/navsat_sensor/navsat"

echo "[info] starting sensor bridge for imu/navsat"
pkill -f "parameter_bridge ${IMU_TOPIC}@sensor_msgs/msg/Imu\\[gz.msgs.IMU ${NAVSAT_TOPIC}@sensor_msgs/msg/NavSatFix\\[gz.msgs.NavSat" >/dev/null 2>&1 || true
nohup ros2 run ros_gz_bridge parameter_bridge \
  "${IMU_TOPIC}"@sensor_msgs/msg/Imu[gz.msgs.IMU \
  "${NAVSAT_TOPIC}"@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  >"${BRIDGE_LOG}" 2>&1 &
sleep 2

echo "[info] reading one imu sample"
timeout 8s ros2 topic echo "${IMU_TOPIC}" --once

echo "[info] reading one navsat sample"
timeout 8s ros2 topic echo "${NAVSAT_TOPIC}" --once

echo "[ok] reset + spawn + sensor read complete"
