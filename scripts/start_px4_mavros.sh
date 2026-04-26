#!/usr/bin/env bash
set -euo pipefail

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
PX4_BUILD_DIR="${PX4_BUILD_DIR:-$PX4_DIR/build/px4_sitl_default}"
PX4_BIN="${PX4_BIN:-$PX4_BUILD_DIR/bin/px4}"
PX4_PARAM_BIN="${PX4_PARAM_BIN:-$PX4_BUILD_DIR/bin/px4-param}"
PX4_SIM_MODEL="${PX4_SIM_MODEL:-gz_x500}"
GZ_IP="${GZ_IP:-127.0.0.1}"
PX4_LOG="${PX4_LOG:-/tmp/px4_sitl.log}"
MAVROS_LOG="${MAVROS_LOG:-/tmp/mavros.log}"
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

find_pid_by_pattern() {
  local pattern="$1"
  pgrep -f "$pattern" | head -n 1 || true
}

if [ ! -x "$PX4_BIN" ]; then
  echo "[error] PX4 binary not found: $PX4_BIN"
  echo "[hint] build once with: cd \"$PX4_DIR\" && make px4_sitl"
  exit 1
fi

echo "[info] stopping old px4/gz/mavros processes"
pkill -f 'mavros_nod[e]' || true
pkill -x px4 || true
pkill -f 'gz sim' || true

rm -f "$PX4_LOG" "$MAVROS_LOG"

echo "[info] starting PX4 SITL + Gazebo"
setsid -f bash -lc "
  set -euo pipefail
  cd \"$PX4_BUILD_DIR\"
  export PX4_SIM_MODEL=\"$PX4_SIM_MODEL\"
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

echo "[info] PX4 started (pid=$PX4_PID)"

if [ -x "$PX4_PARAM_BIN" ]; then
  echo "[info] applying SITL arming params"
  "$PX4_PARAM_BIN" set CBRK_SUPPLY_CHK 894281 >/dev/null 2>&1 || true
  "$PX4_PARAM_BIN" set NAV_DLL_ACT 0 >/dev/null 2>&1 || true
  "$PX4_PARAM_BIN" save >/dev/null 2>&1 || true
fi

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

MAVROS_PID="$(find_pid_by_pattern '/opt/ros/.*/lib/mavros/mavros_node')"
if [ -z "$MAVROS_PID" ]; then
  echo "[error] could not find running mavros_node process"
  tail -n 60 "$MAVROS_LOG" || true
  exit 1
fi

if ! process_alive "$MAVROS_PID"; then
  echo "[error] mavros process exited unexpectedly"
  tail -n 60 "$MAVROS_LOG" || true
  exit 1
fi

echo "[ok] stack is ready"
echo "[ok] px4 pid: $PX4_PID"
echo "[ok] mavros pid: $MAVROS_PID"
echo "[ok] logs: $PX4_LOG , $MAVROS_LOG"
