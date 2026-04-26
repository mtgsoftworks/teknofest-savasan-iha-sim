#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project}"
SCRIPTS_DIR="${PROJECT_ROOT}/scripts"
RUN_ID="${RUN_ID:-$(date +%Y%m%d_%H%M%S)}"
OUT_DIR="${OUT_DIR:-/tmp/savasan_iha_${RUN_ID}}"

WAYPOINTS="${WAYPOINTS:-0,0,3;6,0,3;6,6,3;0,6,3;0,0,3}"
HOLD_SEC_EACH_WP="${HOLD_SEC_EACH_WP:-2.0}"
POS_TOL="${POS_TOL:-0.7}"

mkdir -p "${OUT_DIR}"

log_step() {
  echo "[$(date +%H:%M:%S)] $*"
}

copy_runtime_logs() {
  cp -f /tmp/px4_sitl.log "${OUT_DIR}/px4_sitl.log" 2>/dev/null || true
  cp -f /tmp/mavros.log "${OUT_DIR}/mavros.log" 2>/dev/null || true
  cp -f /tmp/rosbridge.log "${OUT_DIR}/rosbridge.log" 2>/dev/null || true
  cp -f /tmp/ros_gz_bridge_core.log "${OUT_DIR}/ros_gz_bridge_core.log" 2>/dev/null || true
  cp -f /tmp/ros_gz_sensor_bridge_x500_0.log "${OUT_DIR}/ros_gz_sensor_bridge_x500_0.log" 2>/dev/null || true
  cp -f /tmp/ros_gz_cam_lidar_bridge.log "${OUT_DIR}/ros_gz_cam_lidar_bridge.log" 2>/dev/null || true
}

chmod +x "${SCRIPTS_DIR}"/*.sh

log_step "1/5 Stack baslatiliyor (PX4 + Gazebo + MAVROS)"
"${SCRIPTS_DIR}/start_px4_mavros.sh" | tee "${OUT_DIR}/01_start_px4_mavros.log"

log_step "2/5 Rosbridge baslatiliyor"
"${SCRIPTS_DIR}/start_rosbridge.sh" | tee "${OUT_DIR}/02_start_rosbridge.log"

log_step "3/5 Reset + x500 spawn + IMU/GPS sensor read"
"${SCRIPTS_DIR}/mcp_reset_spawn_sensor.sh" | tee "${OUT_DIR}/03_reset_spawn_sensor.log"

log_step "4/5 Kamera + Lidar model spawn + bridge"
"${SCRIPTS_DIR}/mcp_spawn_lidar_camera_bridge.sh" | tee "${OUT_DIR}/04_spawn_cam_lidar.log"

log_step "5/5 Offboard mission kosuluyor"
WAYPOINTS="${WAYPOINTS}" \
HOLD_SEC_EACH_WP="${HOLD_SEC_EACH_WP}" \
POS_TOL="${POS_TOL}" \
"${SCRIPTS_DIR}/run_offboard_mission.sh" | tee "${OUT_DIR}/05_offboard_mission.log"

copy_runtime_logs

log_step "Pipeline tamamlandi"
echo "[ok] cikti klasoru: ${OUT_DIR}"
