#!/usr/bin/env bash
set -euo pipefail

# TEKNOFEST Savasan IHA - Fixed-Wing PX4 SITL + Gazebo + MAVROS launcher
# Uses rc_cessna_mono_cam fixed-wing model with built-in airspeed sensor

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD_DIR="${PX4_BUILD_DIR:-$PX4_DIR/build/px4_sitl_default}"
PX4_BIN="${PX4_BIN:-$PX4_BUILD_DIR/bin/px4}"
PX4_PARAM_BIN="${PX4_PARAM_BIN:-$PX4_BUILD_DIR/bin/px4-param}"

# Fixed-wing model autostart key (airframe suffix): gz_rc_cessna_mono_cam
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_rc_cessna_mono_cam}"
GZ_IP="${GZ_IP:-127.0.0.1}"
PX4_LOG="${PX4_LOG:-/tmp/px4_fw_sitl.log}"
MAVROS_LOG="${MAVROS_LOG:-/tmp/mavros_fw.log}"
FCU_URL="${FCU_URL:-udp://:14540@127.0.0.1:14580}"
GCS_URL="${GCS_URL:-udp://@127.0.0.1:14550}"
START_TIMEOUT_SEC="${START_TIMEOUT_SEC:-120}"

wait_for_log() {
  local file="$1"
  local pattern="$2"
  local timeout="$3"
  local i=0

  until [ "$i" -ge "$timeout" ]; do
    if [ -f "$file" ] && grep -q "$pattern" "$file"; then
      return 0
    fi
    sleep 1
    i=$((i + 1))
  done
  return 1
}

process_alive() {
  local pid="$1"
  kill -0 "$pid" 2>/dev/null
}

if [ ! -x "$PX4_BIN" ]; then
  echo "[error] PX4 binary not found: $PX4_BIN"
  echo "[hint] build once with: cd \"$PX4_DIR\" && make px4_sitl"
  exit 1
fi

if [ "$PX4_SIM_MODEL" = "gz_rc_cessna_mono_cam" ] && [ ! -f "${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_rc_cessna_mono_cam" ]; then
  echo "[error] missing airframe: 4022_gz_rc_cessna_mono_cam"
  echo "[hint] run: scripts/install_models_to_px4.sh"
  exit 1
fi

sync_fw_airframe_to_build() {
  local source_airframe="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/4022_gz_rc_cessna_mono_cam"
  local build_airframe_dir="${PX4_BUILD_DIR}/etc/init.d-posix/airframes"

  if [ -f "${source_airframe}" ] && [ -d "${build_airframe_dir}" ]; then
    cp -f "${source_airframe}" "${build_airframe_dir}/4022_gz_rc_cessna_mono_cam"
  fi
}

# Ensure PX4 boot script reads the latest project airframe values.
sync_fw_airframe_to_build

echo "[info] stopping old px4/gz/mavros processes"
pkill -f 'mavros_nod[e]' || true
pkill -x px4 || true
pkill -f 'gz sim' || true

# Remove BSON parameter files so airframe set-default values are applied cleanly.
# PX4 SITL can store these directly under rootfs/ (current) or rootfs/eeprom/ (legacy).
echo "[info] clearing BSON params so airframe defaults are reapplied"
rm -f "${PX4_BUILD_DIR}/rootfs/parameters.bson" 2>/dev/null || true
rm -f "${PX4_BUILD_DIR}/rootfs/parameters_backup.bson" 2>/dev/null || true
rm -f "${PX4_BUILD_DIR}/rootfs/eeprom/parameters.bson" 2>/dev/null || true
rm -f "${PX4_BUILD_DIR}/rootfs/eeprom/parameters_backup.bson" 2>/dev/null || true

rm -f "$PX4_LOG" "$MAVROS_LOG"
rm -f /tmp/px4-sock-* 2>/dev/null || true

echo "[info] starting PX4 SITL + Gazebo (model=$PX4_SIM_MODEL)"
export PX4_GZ_WORLD="${PX4_GZ_WORLD:-teknofest_competition}"
setsid -f bash -lc "
set -euo pipefail
cd \"$PX4_BUILD_DIR\"
export PX4_SIM_MODEL=\"$PX4_SIM_MODEL\"
export PX4_GZ_WORLD=\"${PX4_GZ_WORLD}\"
export GZ_IP=\"$GZ_IP\"
exec \"$PX4_BIN\" -d
" >>"$PX4_LOG" 2>&1

if ! wait_for_log "$PX4_LOG" "Startup script returned successfully" "$START_TIMEOUT_SEC"; then
  echo "[error] px4 startup timeout"
  tail -n 80 "$PX4_LOG" || true
  exit 1
fi

PX4_PID="$(pgrep -xo px4 || true)"
if [ -z "$PX4_PID" ]; then
  echo "[error] could not find running px4 process"
  tail -n 80 "$PX4_LOG" || true
  exit 1
fi

if ! process_alive "$PX4_PID"; then
  echo "[error] px4 process exited unexpectedly"
  tail -n 80 "$PX4_LOG" || true
  exit 1
fi

echo "[info] PX4 started (pid=$PX4_PID, model=$PX4_SIM_MODEL)"


echo "[info] starting MAVROS"
setsid -f bash -lc "
set -euo pipefail
set +u
source /opt/ros/jazzy/setup.bash
set -u
exec ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=$FCU_URL \
  -p gcs_url:=$GCS_URL
" >>"$MAVROS_LOG" 2>&1

if ! wait_for_log "$MAVROS_LOG" "Got HEARTBEAT, connected" "$START_TIMEOUT_SEC"; then
  echo "[error] mavros did not connect to PX4"
  echo "[hint] px4 log tail:"
  tail -n 60 "$PX4_LOG" || true
  echo "[hint] mavros log tail:"
  tail -n 60 "$MAVROS_LOG" || true
  exit 1
fi

MAVROS_PID="$(pgrep -f '/opt/ros/.*/lib/mavros/mavros_node' || true)"
if [ -z "$MAVROS_PID" ]; then
  echo "[error] could not find running mavros_node process"
  tail -n 60 "$MAVROS_LOG" || true
  exit 1
fi

echo "[ok] fixed-wing stack is ready"
echo "[ok] model: $PX4_SIM_MODEL"
echo "[ok] px4 pid: $PX4_PID"
echo "[ok] mavros pid: $MAVROS_PID"
echo "[ok] logs: $PX4_LOG , $MAVROS_LOG"
