#!/usr/bin/env bash
set -euo pipefail

# TEKNOFEST - Copy custom models and world to PX4-Autopilot
# Run once after cloning or on a new machine

PROJECT_ROOT="${PROJECT_ROOT:-/mnt/c/Users/mtg/Desktop/My_Files/gazebo_project}"
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
GZ_MODELS_DIR="${PX4_DIR}/Tools/simulation/gz/models"
GZ_WORLDS_DIR="${PX4_DIR}/Tools/simulation/gz/worlds"
AIRFRAMES_DIR="${PX4_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes"
BUILD_AIRFRAMES_DIR="${PX4_DIR}/build/px4_sitl_default/etc/init.d-posix/airframes"

SRC_MODELS="${PROJECT_ROOT}/models"
SRC_WORLDS="${PROJECT_ROOT}/worlds"

echo "[info] PX4 dir: ${PX4_DIR}"
echo "[info] Source models: ${SRC_MODELS}"

if [ ! -d "${PX4_DIR}" ]; then
  echo "[error] PX4-Autopilot not found at ${PX4_DIR}"
  exit 1
fi

# Copy custom Gazebo models
for model in teknofest_runway teknofest_qr_target teknofest_hss_zone teknofest_boundary_marker rc_cessna_mono_cam; do
  if [ -d "${SRC_MODELS}/${model}" ]; then
    mkdir -p "${GZ_MODELS_DIR}/${model}"
    cp -v "${SRC_MODELS}/${model}/model.config" "${GZ_MODELS_DIR}/${model}/"
    cp -v "${SRC_MODELS}/${model}/model.sdf" "${GZ_MODELS_DIR}/${model}/"
    echo "[ok] ${model}"
  else
    echo "[skip] ${model} not found in source"
  fi
done

# Copy world files
mkdir -p "${GZ_WORLDS_DIR}"
for world in teknofest_competition; do
  if [ -f "${SRC_WORLDS}/${world}.sdf" ]; then
    cp -v "${SRC_WORLDS}/${world}.sdf" "${GZ_WORLDS_DIR}/"
    echo "[ok] ${world}.sdf"
  else
    echo "[skip] ${world}.sdf not found"
  fi
done

# Copy airframe file if it exists in project
SRC_AIRFRAME="${PROJECT_ROOT}/airframes/4022_gz_rc_cessna_mono_cam"
if [ -f "${SRC_AIRFRAME}" ]; then
  cp -v "${SRC_AIRFRAME}" "${AIRFRAMES_DIR}/"
  if [ -d "${BUILD_AIRFRAMES_DIR}" ]; then
    cp -v "${SRC_AIRFRAME}" "${BUILD_AIRFRAMES_DIR}/"
    echo "[ok] airframe synced to build/px4_sitl_default"
  fi
  echo "[ok] airframe 4022_gz_rc_cessna_mono_cam"
else
  echo "[info] Airframe file not in project dir (may already be in PX4)"
fi

echo ""
echo "[ok] All models and worlds copied to PX4"
echo "[hint] GZ_SIM_RESOURCE_PATH is set automatically by PX4's gz simulator plugin"
echo "[hint] To verify: gz sim -v4 -r ${GZ_WORLDS_DIR}/teknofest_competition.sdf"
