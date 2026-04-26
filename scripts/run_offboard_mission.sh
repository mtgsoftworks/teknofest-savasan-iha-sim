#!/usr/bin/env bash
set -euo pipefail

WS_DIR="${WS_DIR:-/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project/ros2_ws}"
WAYPOINTS="${WAYPOINTS:-0,0,3;4,0,3;4,4,3;0,0,3}"
HOLD_SEC_EACH_WP="${HOLD_SEC_EACH_WP:-2.0}"
POS_TOL="${POS_TOL:-0.6}"

set +u
source /opt/ros/jazzy/setup.bash
set -u

if [ ! -d "${WS_DIR}" ]; then
  echo "[error] workspace not found: ${WS_DIR}"
  exit 1
fi

echo "[info] building offboard_takeoff package"
cd "${WS_DIR}"
colcon build --packages-select offboard_takeoff >/tmp/offboard_build.log
set +u
source "${WS_DIR}/install/setup.bash"
set -u

echo "[info] running offboard mission"
echo "[info] waypoints=${WAYPOINTS}"
ros2 run offboard_takeoff offboard_mission --ros-args \
  -p waypoints:="${WAYPOINTS}" \
  -p hold_sec_each_wp:="${HOLD_SEC_EACH_WP}" \
  -p position_tolerance_m:="${POS_TOL}"
