#!/usr/bin/env bash
set -euo pipefail

set +u
source /opt/ros/jazzy/setup.bash
set -u

WORLD_NAME="${WORLD_NAME:-default}"
CAM_MODEL_NAME="${CAM_MODEL_NAME:-x500_cam_0}"
LIDAR_MODEL_NAME="${LIDAR_MODEL_NAME:-x500_lidar_0}"
CAM_MODEL_URI="${CAM_MODEL_URI:-model://x500_mono_cam}"
LIDAR_MODEL_URI="${LIDAR_MODEL_URI:-model://x500_lidar_front}"
BRIDGE_LOG="${BRIDGE_LOG:-/tmp/ros_gz_cam_lidar_bridge.log}"

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

spawn_at() {
  local name="$1"
  local uri="$2"
  local x="$3"
  local y="$4"
  local z="$5"

  ros2 service call /world/"${WORLD_NAME}"/create ros_gz_interfaces/srv/SpawnEntity \
    "{entity_factory: {name: '${name}', allow_renaming: false, sdf_filename: '${uri}', pose: {position: {x: ${x}, y: ${y}, z: ${z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, relative_to: 'world'}}" \
    >/tmp/spawn_"${name}".log
}

echo "[info] ensuring ros_gz core bridge"
ensure_core_bridge

echo "[info] removing old camera/lidar models"
ros2 service call /world/"${WORLD_NAME}"/remove ros_gz_interfaces/srv/DeleteEntity \
  "{entity: {name: '${CAM_MODEL_NAME}', type: 2}}" >/tmp/remove_"${CAM_MODEL_NAME}".log || true
ros2 service call /world/"${WORLD_NAME}"/remove ros_gz_interfaces/srv/DeleteEntity \
  "{entity: {name: '${LIDAR_MODEL_NAME}', type: 2}}" >/tmp/remove_"${LIDAR_MODEL_NAME}".log || true

echo "[info] spawning ${CAM_MODEL_NAME} and ${LIDAR_MODEL_NAME}"
spawn_at "${CAM_MODEL_NAME}" "${CAM_MODEL_URI}" 2.0 0.0 0.2
spawn_at "${LIDAR_MODEL_NAME}" "${LIDAR_MODEL_URI}" -2.0 0.0 0.2

CAM_TOPIC="/world/${WORLD_NAME}/model/${CAM_MODEL_NAME}/link/camera_link/sensor/camera/image"
CAM_INFO_TOPIC="/world/${WORLD_NAME}/model/${CAM_MODEL_NAME}/link/camera_link/sensor/camera/camera_info"
LIDAR_TOPIC="/world/${WORLD_NAME}/model/${LIDAR_MODEL_NAME}/link/lidar_sensor_link/sensor/lidar/scan"

echo "[info] starting camera/lidar bridge"
pkill -f "parameter_bridge ${CAM_TOPIC}@sensor_msgs/msg/Image\\[gz.msgs.Image" >/dev/null 2>&1 || true
nohup ros2 run ros_gz_bridge parameter_bridge \
  "${CAM_TOPIC}"@sensor_msgs/msg/Image[gz.msgs.Image \
  "${CAM_INFO_TOPIC}"@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
  "${LIDAR_TOPIC}"@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
  >"${BRIDGE_LOG}" 2>&1 &
sleep 2

echo "[info] reading one camera frame"
timeout 8s ros2 topic echo "${CAM_TOPIC}" --once

echo "[info] reading one lidar sample"
timeout 8s ros2 topic echo "${LIDAR_TOPIC}" --once

echo "[ok] camera/lidar spawn + bridge complete"
