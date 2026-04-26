#!/usr/bin/env bash
set -euo pipefail

# TEKNOFEST Savasan IHA - Fixed-Wing Full Pipeline
# Single command: PX4+Gazebo → MAVROS → Rosbridge → Spawn+Bridge → Offboard Mission

PROJECT_ROOT="${PROJECT_ROOT:-/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project}"
SCRIPTS_DIR="${PROJECT_ROOT}/scripts"
ROS2_WS="${ROS2_WS:-${PROJECT_ROOT}/ros2_ws}"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${OUT_DIR:-/tmp/savasan_iha_fw_${RUN_ID}}"

# Fixed-wing settings
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_rc_cessna_mono_cam}"
MODEL_NAME="${MODEL_NAME:-rc_cessna_mono_cam_0}"
MODEL_URI="${MODEL_URI:-model://rc_cessna_mono_cam}"
SIM_WORLD="${SIM_WORLD:-${WORLD_NAME:-${PX4_GZ_WORLD:-teknofest_competition}}}"
export WORLD_NAME="${SIM_WORLD}"
export PX4_GZ_WORLD="${SIM_WORLD}"
WAYPOINTS="${WAYPOINTS:-0,0,30;50,0,30;50,50,30;0,50,30;0,0,30}"
HOLD_SEC_EACH_WP="${HOLD_SEC_EACH_WP:-3.0}"
POS_TOL="${POS_TOL:-20.0}"
CRUISE_SPEED="${CRUISE_SPEED:-12.0}"

mkdir -p "${OUT_DIR}"

log_step() {
  echo "[$(date +%H:%M:%S)] $*"
}

detect_px4_world_from_log() {
  local px4_log="/tmp/px4_fw_sitl.log"
  local detected_world=""

  if [ -f "${px4_log}" ]; then
    detected_world="$(grep -Eo 'INFO  \[gz_bridge\] world: [^,]+' "${px4_log}" | tail -n 1 | sed -E 's/.*world: //')"
  fi

  if [ -n "${detected_world}" ]; then
    export WORLD_NAME="${detected_world}"
    export PX4_GZ_WORLD="${detected_world}"
    log_step "PX4 world algilandi: ${detected_world}"
  else
    log_step "PX4 world logdan algilanamadi, mevcut world kullanilacak: ${WORLD_NAME}"
  fi
}

copy_runtime_logs() {
  cp -f /tmp/px4_fw_sitl.log "${OUT_DIR}/px4_sitl.log" 2>/dev/null || true
  cp -f /tmp/mavros_fw.log "${OUT_DIR}/mavros.log" 2>/dev/null || true
  cp -f /tmp/rosbridge.log "${OUT_DIR}/rosbridge.log" 2>/dev/null || true
  cp -f /tmp/ros_gz_bridge_core.log "${OUT_DIR}/ros_gz_bridge_core.log" 2>/dev/null || true
  cp -f /tmp/ros_gz_fw_sensor_bridge_*.log "${OUT_DIR}/" 2>/dev/null || true
}

cleanup() {
  log_step "pipeline interrupted, cleaning up..."
  # Geofence ve telemetry arka plan processlerini durdur
  [ -n "${GEOFENCE_PID:-}" ] && kill "${GEOFENCE_PID}" 2>/dev/null || true
  [ -n "${TELEMETRY_PID:-}" ] && kill "${TELEMETRY_PID}" 2>/dev/null || true
  copy_runtime_logs
  "${SCRIPTS_DIR}/stop_px4_mavros.sh" || true
  log_step "cleanup done"
}
trap cleanup EXIT INT TERM

chmod +x "${SCRIPTS_DIR}"/*.sh

log_step "1/7 Sabit kanat stack baslatiliyor (PX4 + Gazebo + MAVROS)"
PX4_SIM_MODEL="${PX4_SIM_MODEL}" \
"${SCRIPTS_DIR}/start_fw_px4_mavros.sh" | tee "${OUT_DIR}/01_start_fw_stack.log"

detect_px4_world_from_log

log_step "2/7 Rosbridge baslatiliyor"
"${SCRIPTS_DIR}/start_rosbridge.sh" | tee "${OUT_DIR}/02_start_rosbridge.log"

log_step "3/7 Sabit kanat spawn + sensor bridge (IMU/NavSat/Airspeed)"
MODEL_NAME="${MODEL_NAME}" \
MODEL_URI="${MODEL_URI}" \
SKIP_MODEL_RESPAWN="${SKIP_MODEL_RESPAWN:-1}" \
"${SCRIPTS_DIR}/mcp_reset_spawn_fw_sensor.sh" | tee "${OUT_DIR}/03_fw_spawn_bridge.log"

log_step "4/7 ROS 2 paket derleniyor"
set +u
source /opt/ros/jazzy/setup.bash
set -u
cd "${ROS2_WS}"
colcon build --packages-select offboard_takeoff 2>&1 | tee "${OUT_DIR}/04_colcon_build.log"
set +u
source install/setup.bash
set -u

log_step "5/7 Geofence node baslatiliyor"
ros2 run offboard_takeoff geofence --ros-args \
  -p boundary_corners:="-5000,-5000;5000,-5000;5000,5000;-5000,5000" \
  -p hss_zones:="99999,99999,1" \
  -p auto_rtl_on_breach:=false \
  -p auto_rtl_on_hss:=false \
  -p boundary_log_interval_sec:=60.0 \
  -p hss_log_interval_sec:=60.0 \
  2>&1 | tee "${OUT_DIR}/05_geofence.log" &
GEOFENCE_PID=$!

log_step "6/7 Telemetry logger baslatiliyor"
ros2 run offboard_takeoff telemetry_logger --ros-args \
  -p log_dir:="${OUT_DIR}" \
  -p mission_name:="fw_mission" \
  2>&1 | tee "${OUT_DIR}/06_telemetry.log" &
TELEMETRY_PID=$!
sleep 1

log_step "7/7 Sabit kanat offboard mission baslatiliyor"
ros2 run offboard_takeoff fw_mission --ros-args \
  -p waypoints:="${WAYPOINTS}" \
  -p hold_sec_each_wp:="${HOLD_SEC_EACH_WP}" \
  -p position_tolerance_m:="${POS_TOL}" \
  -p mission_timeout_sec:=600.0 \
  -p cruise_speed_mps:="${CRUISE_SPEED}" \
  -p arm_settle_time_sec:=3.0 \
  -p enable_flyby_waypoint_acceptance:=true \
  -p flyby_cross_track_m:=45.0 \
  -p loiter_after_mission:=true \
  -p post_mission_loiter_sec:=5.0 \
  2>&1 | tee "${OUT_DIR}/07_fw_mission.log"

# Geofence ve telemetry arka plan processlerini durdur
kill "${GEOFENCE_PID}" 2>/dev/null || true
kill "${TELEMETRY_PID}" 2>/dev/null || true

copy_runtime_logs
log_step "Pipeline tamamlandi"
echo "[ok] cikti klasoru: ${OUT_DIR}"
