#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
FAST_LIO_DIR="${ROOT_DIR}/src/FAST_LIO"
THIRD_PARTY_DIR="${ROOT_DIR}/src/third_party"

mkdir -p "${THIRD_PARTY_DIR}"

echo "[1/3] Re-enabling FAST_LIO package"
rm -f "${FAST_LIO_DIR}/CATKIN_IGNORE"

echo "[2/3] Fetching livox_ros_driver into workspace"
if [ ! -d "${THIRD_PARTY_DIR}/livox_ros_driver" ]; then
  git clone --depth 1 https://github.com/Livox-SDK/livox_ros_driver.git "${THIRD_PARTY_DIR}/livox_ros_driver"
else
  echo "  livox_ros_driver already exists, skipping"
fi

echo "[3/3] Fetching ikd-Tree into FAST_LIO/include"
if [ ! -f "${FAST_LIO_DIR}/include/ikd-Tree/ikd_Tree.cpp" ]; then
  TMP_DIR="$(mktemp -d)"
  git clone --depth 1 https://github.com/hku-mars/ikd-Tree.git "${TMP_DIR}/ikd-Tree"
  rm -rf "${FAST_LIO_DIR}/include/ikd-Tree"
  mkdir -p "${FAST_LIO_DIR}/include/ikd-Tree"
  cp -r "${TMP_DIR}/ikd-Tree/"* "${FAST_LIO_DIR}/include/ikd-Tree/"
  rm -rf "${TMP_DIR}"
else
  echo "  ikd-Tree already exists, skipping"
fi

echo "FAST_LIO dependencies are now staged. Build with:"
echo "  source /opt/ros/noetic/setup.bash"
echo "  cd ${ROOT_DIR}"
echo "  catkin_make -DCMAKE_BUILD_TYPE=Release"
