#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

INPUT_PATH=""
OUTPUT_DIR=""
MODE="auto"
CONFIG_PATH=""
RUNNER="auto"
CXX_EXECUTABLE=""
OUTPUT_EXPLICIT=0

canonicalize_existing_path() {
  realpath "$1"
}

canonicalize_output_path() {
  realpath -m "$1"
}

infer_effective_mode() {
  local requested_mode="$1"
  local input_path="$2"
  if [ "${requested_mode}" = "auto" ]; then
    if [ -d "${input_path}" ]; then
      echo "aggregate"
    else
      echo "single"
    fi
    return 0
  fi
  echo "${requested_mode}"
}

default_output_dir() {
  local runner_label="$1"
  local effective_mode="$2"
  local input_path="$3"
  local base_dir="${ROOT_DIR}/results/voxel_morphology_${runner_label}"
  if [ "${effective_mode}" = "single" ]; then
    local input_name
    input_name="$(basename "${input_path}")"
    input_name="${input_name%.*}"
    echo "${base_dir}/${input_name}"
    return 0
  fi
  if [ "${effective_mode}" = "batch" ]; then
    echo "${base_dir}_batch"
    return 0
  fi
  echo "${base_dir}"
}

if [ $# -eq 0 ]; then
  echo "Usage: $0 --input <pcd|dir> [--output <dir>] [--mode auto|single|aggregate|batch] [--runner auto|cpp|python] [--config <yaml>]"
  echo "Legacy usage: $0 <input.pcd|input_dir> [output_dir]"
  exit 1
fi

if [[ "${1}" != --* ]]; then
  INPUT_PATH="$1"
  if [ $# -ge 2 ]; then
    OUTPUT_DIR="$2"
    OUTPUT_EXPLICIT=1
  fi
else
  while [ $# -gt 0 ]; do
    case "$1" in
      --input|-i)
        INPUT_PATH="$2"
        shift 2
        ;;
      --output|-o)
        OUTPUT_DIR="$2"
        OUTPUT_EXPLICIT=1
        shift 2
        ;;
      --mode|-m)
        MODE="$2"
        shift 2
        ;;
      --config|-c)
        CONFIG_PATH="$2"
        shift 2
        ;;
      --runner)
        RUNNER="$2"
        shift 2
        ;;
      --help|-h)
        echo "Usage: $0 --input <pcd|dir> [--output <dir>] [--mode auto|single|aggregate|batch] [--runner auto|cpp|python] [--config <yaml>]"
        exit 0
        ;;
      *)
        echo "Unknown argument: $1"
        exit 1
        ;;
    esac
  done
fi

if [ -z "${INPUT_PATH}" ]; then
  echo "--input is required"
  exit 1
fi

INPUT_PATH="$(canonicalize_existing_path "${INPUT_PATH}")"

if [ -z "${OUTPUT_DIR}" ]; then
  EFFECTIVE_MODE="$(infer_effective_mode "${MODE}" "${INPUT_PATH}")"
  if [ "${RUNNER}" = "python" ]; then
    OUTPUT_DIR="$(default_output_dir "python_fallback" "${EFFECTIVE_MODE}" "${INPUT_PATH}")"
  else
    OUTPUT_DIR="$(default_output_dir "cpp" "${EFFECTIVE_MODE}" "${INPUT_PATH}")"
  fi
fi
OUTPUT_DIR="$(canonicalize_output_path "${OUTPUT_DIR}")"

if [ -z "${CONFIG_PATH}" ]; then
  CONFIG_PATH="${ROOT_DIR}/src/ges_voxel_mapping/config/voxel_morphology.yaml"
fi
CONFIG_PATH="$(canonicalize_existing_path "${CONFIG_PATH}")"

CXX_EXECUTABLE="${ROOT_DIR}/build/ges_voxel_mapping_cpp/voxel_morphology_analyzer"

run_cpp() {
  local cmd=("${CXX_EXECUTABLE}" --input "${INPUT_PATH}" --output "${OUTPUT_DIR}" --mode "${MODE}" --config "${CONFIG_PATH}")
  echo "Running C++ executable: ${cmd[*]}"
  "${cmd[@]}"
}

run_python() {
  local py_cmd=(python3 "${ROOT_DIR}/scripts/voxel_morphology_fallback.py" --config "${CONFIG_PATH}" --input "${INPUT_PATH}" --output "${OUTPUT_DIR}" --mode "${MODE}" --output-is-final)
  echo "Running Python fallback: ${py_cmd[*]}"
  "${py_cmd[@]}"
}

ensure_cpp_built() {
  if [ -x "${CXX_EXECUTABLE}" ]; then
    return 0
  fi
  echo "C++ executable not found. Building it first..."
  bash "${ROOT_DIR}/scripts/build_offline.sh"
}

case "${RUNNER}" in
  cpp)
    ensure_cpp_built
    run_cpp
    ;;
  python)
    run_python
    ;;
  auto)
    if ensure_cpp_built; then
      if run_cpp; then
        exit 0
      fi
      echo "C++ path failed. Falling back to Python."
      if [ "${OUTPUT_EXPLICIT}" -eq 1 ]; then
        OUTPUT_DIR="$(canonicalize_output_path "${OUTPUT_DIR}_python_fallback")"
      else
        EFFECTIVE_MODE="$(infer_effective_mode "${MODE}" "${INPUT_PATH}")"
        OUTPUT_DIR="$(canonicalize_output_path "$(default_output_dir "python_fallback" "${EFFECTIVE_MODE}" "${INPUT_PATH}")")"
      fi
    fi
    run_python
    ;;
  *)
    echo "Unsupported --runner: ${RUNNER}"
    exit 1
    ;;
esac
