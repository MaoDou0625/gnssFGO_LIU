#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNNER_PATH="${ROOT_DIR}/build/offline_lc_runner"

if [[ ! -x "${RUNNER_PATH}" ]]; then
  echo "offline_lc_runner not found: ${RUNNER_PATH}" >&2
  echo "Build the project first with CMake." >&2
  exit 1
fi

LOCAL_GTSAM_LIB_DIRS=(
  "${ROOT_DIR}/third_party/gtsam-install/lib"
  "${HOME}/.local/offline_lc_minimal/gtsam/lib"
)

for lib_dir in "${LOCAL_GTSAM_LIB_DIRS[@]}"; do
  if [[ -d "${lib_dir}" ]]; then
    export LD_LIBRARY_PATH="${lib_dir}:${LD_LIBRARY_PATH:-}"
    break
  fi
done

exec "${RUNNER_PATH}" "$@"
