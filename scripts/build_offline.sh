#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build/ges_voxel_mapping_cpp"

cmake -S "${ROOT_DIR}/src/ges_voxel_mapping" \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DGES_VOXEL_ENABLE_CATKIN=OFF

cmake --build "${BUILD_DIR}" -j"$(nproc)"

echo "Built C++ offline executable:"
echo "  ${BUILD_DIR}/voxel_morphology_analyzer"
echo "  ${BUILD_DIR}/voxel_primitive_comparison"
