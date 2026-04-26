#!/usr/bin/env bash
set -euo pipefail

# TEKNOFEST - Fixed-wing sensor bridge: IMU, NavSat, Airspeed, Camera
# Spawns rc_cessna (or variant) in Gazebo and bridges sensors to ROS 2

set +u
source /opt/ros/jazzy/setup.bash
set -u

WORLD_NAME="${WORLD_NAME:-default}"
MODEL_NAME="${MODEL_NAME:-rc_cessna_mono_cam_0}"
MODEL_URI="${MODEL_URI:-model://rc_cessna_mono_cam}"
MODEL_Z="${MODEL_Z:-0.3}"
BRIDGE_LOG="${BRIDGE_LOG:-/tmp/ros_gz_fw_sensor_bridge_${MODEL_NAME}.log}"
SERVICE_TIMEOUT_SEC="${SERVICE_TIMEOUT_SEC:-12}"
SERVICE_RETRY_COUNT="${SERVICE_RETRY_COUNT:-3}"
WAIT_MODEL_READY_SEC="${WAIT_MODEL_READY_SEC:-30}"
# PX4 already spawns the fixed-wing model in start_fw_px4_mavros.sh.
# Default to attach mode to avoid duplicate spawn side effects.
SKIP_MODEL_RESPAWN="${SKIP_MODEL_RESPAWN:-1}"

ensure_core_bridge() {
  if pgrep -f "parameter_bridge /clock@rosgraph_msgs/msg/Clock\[gz.msgs.Clock .* /world/${WORLD_NAME}/control@ros_gz_interfaces/srv/ControlWorld" >/dev/null 2>&1; then
    return
  fi

  nohup ros2 run ros_gz_bridge parameter_bridge \
    /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
    /world/"${WORLD_NAME}"/control@ros_gz_interfaces/srv/ControlWorld \
    /world/"${WORLD_NAME}"/create@ros_gz_interfaces/srv/SpawnEntity \
    /world/"${WORLD_NAME}"/remove@ros_gz_interfaces/srv/DeleteEntity \
    /world/"${WORLD_NAME}"/set_pose@ros_gz_interfaces/srv/SetEntityPose \
    >/tmp/ros_gz_bridge_core.log 2>&1 &
  sleep 2
}

list_world_services() {
  ros2 service list 2>/dev/null | grep "^/world/" || true
}

model_topics_exist() {
  if ! command -v gz >/dev/null 2>&1; then
    return 1
  fi

  gz topic -l 2>/dev/null | grep -q "^/world/${WORLD_NAME}/model/${MODEL_NAME}/"
}

wait_model_topics_present() {
  local timeout_sec="${1:-20}"
  local elapsed=0

  while [ "${elapsed}" -lt "${timeout_sec}" ]; do
    if model_topics_exist; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  return 1
}

camera_topic_available() {
  local attempts=8
  local i=0

  if ! command -v gz >/dev/null 2>&1; then
    return 1
  fi

  while [ "${i}" -lt "${attempts}" ]; do
    if gz topic -i -t "${CAM_TOPIC}" 2>/dev/null | grep -q "Publishers"; then
      return 0
    fi
    i=$((i + 1))
    sleep 1
  done

  return 1
}

wait_world_services() {
  local timeout_sec="${1:-20}"
  local elapsed=0

  while [ "${elapsed}" -lt "${timeout_sec}" ]; do
    if ros2 service list 2>/dev/null | grep -q "^/world/${WORLD_NAME}/control$"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  return 1
}

call_service_with_retry() {
  local service="$1"
  local srv_type="$2"
  local request="$3"
  local out_file="$4"
  local attempt=1

  while [ "${attempt}" -le "${SERVICE_RETRY_COUNT}" ]; do
    if timeout "${SERVICE_TIMEOUT_SEC}s" ros2 service call "${service}" "${srv_type}" "${request}" >"${out_file}" 2>&1; then
      return 0
    fi
    echo "[warn] service call failed (attempt ${attempt}/${SERVICE_RETRY_COUNT}): ${service}" >&2
    attempt=$((attempt + 1))
    sleep 1
  done

  return 1
}

spawn_model() {
  call_service_with_retry \
    "/world/${WORLD_NAME}/create" \
    "ros_gz_interfaces/srv/SpawnEntity" \
    "{entity_factory: {name: '${MODEL_NAME}', allow_renaming: false, sdf_filename: '${MODEL_URI}', pose: {position: {x: 0.0, y: 0.0, z: ${MODEL_Z}}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, relative_to: 'world'}}" \
    "/tmp/spawn_${MODEL_NAME}.log"
}

echo "[info] ensuring ros_gz core bridge"
ensure_core_bridge

if ! wait_world_services 20; then
  echo "[error] world services not available for /world/${WORLD_NAME}" >&2
  echo "[info] available world services:" >&2
  list_world_services >&2
  exit 1
fi

echo "[info] waiting for PX4 model topics: ${MODEL_NAME}"
if ! wait_model_topics_present "${WAIT_MODEL_READY_SEC}"; then
  echo "[error] model topics did not appear in ${WAIT_MODEL_READY_SEC}s: ${MODEL_NAME}" >&2
  echo "[hint] check /tmp/px4_fw_sitl.log for Gazebo model spawn errors" >&2
  exit 1
fi

if [ "${SKIP_MODEL_RESPAWN}" = "1" ]; then
  echo "[info] model already exists, skip remove/spawn: ${MODEL_NAME}"
else
  echo "[info] pausing world (no time reset)"
  if ! call_service_with_retry \
    "/world/${WORLD_NAME}/control" \
    "ros_gz_interfaces/srv/ControlWorld" \
    "{world_control: {pause: true}}" \
    "/tmp/world_pause.log"; then
    echo "[error] failed to pause world: ${WORLD_NAME}" >&2
    list_world_services >&2
    exit 1
  fi

  echo "[info] removing old model if exists: ${MODEL_NAME}"
  call_service_with_retry \
    "/world/${WORLD_NAME}/remove" \
    "ros_gz_interfaces/srv/DeleteEntity" \
    "{entity: {name: '${MODEL_NAME}', type: 2}}" \
    "/tmp/remove_${MODEL_NAME}.log" || true

  # Let Gazebo apply delete before re-spawning with the same name.
  sleep 2

  echo "[info] spawning model: ${MODEL_NAME} (${MODEL_URI})"
  if ! spawn_model; then
    echo "[error] failed to spawn model ${MODEL_NAME} in world ${WORLD_NAME}" >&2
    call_service_with_retry \
      "/world/${WORLD_NAME}/control" \
      "ros_gz_interfaces/srv/ControlWorld" \
      "{world_control: {pause: false}}" \
      "/tmp/world_unpause_after_spawn_fail.log" || true
    list_world_services >&2
    exit 1
  fi

  echo "[info] unpausing world"
  if ! call_service_with_retry \
    "/world/${WORLD_NAME}/control" \
    "ros_gz_interfaces/srv/ControlWorld" \
    "{world_control: {pause: false}}" \
    "/tmp/world_unpause.log"; then
    echo "[error] failed to unpause world: ${WORLD_NAME}" >&2
    list_world_services >&2
    exit 1
  fi
fi

# Fixed-wing sensor topics
IMU_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/base_link/sensor/imu_sensor/imu"
NAVSAT_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/base_link/sensor/navsat_sensor/navsat"
AIRPRESSURE_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/base_link/sensor/air_pressure_sensor/air_pressure"
AIRSPEED_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/airspeed_link/sensor/air_speed/air_speed"

# Camera topic (only if mono_cam variant is used)
CAM_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/camera_link/sensor/camera/image"
CAM_INFO_TOPIC="/world/${WORLD_NAME}/model/${MODEL_NAME}/link/camera_link/sensor/camera/camera_info"

echo "[info] starting sensor bridge for IMU/NavSat/AirPressure"
pkill -f "parameter_bridge ${IMU_TOPIC}@sensor_msgs/msg/Imu\[gz.msgs.IMU" >/dev/null 2>&1 || true
nohup ros2 run ros_gz_bridge parameter_bridge \
  "${IMU_TOPIC}"@sensor_msgs/msg/Imu[gz.msgs.IMU \
  "${NAVSAT_TOPIC}"@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat \
  "${AIRPRESSURE_TOPIC}"@sensor_msgs/msg/FluidPressure[gz.msgs.FluidPressure \
  >"${BRIDGE_LOG}" 2>&1 &
sleep 2

echo "[info] reading one IMU sample"
timeout 8s ros2 topic echo "${IMU_TOPIC}" --once || echo "[warn] IMU sample timeout"

echo "[info] reading one NavSat sample"
timeout 8s ros2 topic echo "${NAVSAT_TOPIC}" --once || echo "[warn] NavSat sample timeout"

echo "[info] checking Gazebo airspeed topic"
if command -v gz >/dev/null 2>&1; then
  if gz topic -i -t "${AIRSPEED_TOPIC}" >/tmp/airspeed_topic_info_"${MODEL_NAME}".log 2>&1; then
    echo "[info] airspeed topic found: ${AIRSPEED_TOPIC}"
  else
    echo "[warn] airspeed topic not found: ${AIRSPEED_TOPIC}"
    echo "[warn] ensure model includes model://airspeed and airspeed_link is fixed to base_link"
  fi
else
  echo "[warn] gz CLI not found, skipping airspeed topic check"
fi

# Try camera bridge (will silently skip if mono_cam model not used)
echo "[info] attempting camera bridge (mono_cam variant only)"
pkill -f "parameter_bridge ${CAM_TOPIC}@sensor_msgs/msg/Image" >/dev/null 2>&1 || true
if camera_topic_available; then
  nohup ros2 run ros_gz_bridge parameter_bridge \
    "${CAM_TOPIC}"@sensor_msgs/msg/Image[gz.msgs.Image \
    "${CAM_INFO_TOPIC}"@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo \
    >>"${BRIDGE_LOG}" 2>&1 &
  sleep 2
  echo "[info] camera bridge started"
  timeout 8s ros2 topic echo "${CAM_TOPIC}" --once || echo "[warn] camera sample timeout"
else
  echo "[info] no camera topic found (use rc_cessna_mono_cam for camera)"
fi

echo "[ok] fixed-wing reset + spawn + sensor bridge complete"
