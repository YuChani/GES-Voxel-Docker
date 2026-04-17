#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE_FILE="${ROOT_DIR}/docker-compose/fast-lio_compose.yml"

IMAGE_NAME="${IMAGE_NAME:-ges-voxel-noetic}"
CONTAINER_NAME="${CONTAINER_NAME:-ges-voxel-dev}"
BUILD_FAST_LIO="${BUILD_FAST_LIO:-0}"
export IMAGE_NAME CONTAINER_NAME BUILD_FAST_LIO

docker compose -f "${COMPOSE_FILE}" build
docker compose -f "${COMPOSE_FILE}" up -d
if [ "${BUILD_FAST_LIO}" = "1" ]; then
  docker compose -f "${COMPOSE_FILE}" exec ges-voxel-dev bash -lc "source /opt/ros/noetic/setup.bash && ./scripts/bootstrap_fastlio_deps.sh && catkin_make -DCMAKE_BUILD_TYPE=Release"
else
  docker compose -f "${COMPOSE_FILE}" exec ges-voxel-dev bash -lc "source /opt/ros/noetic/setup.bash && ./scripts/build_offline.sh"
fi
docker compose -f "${COMPOSE_FILE}" exec ges-voxel-dev bash
