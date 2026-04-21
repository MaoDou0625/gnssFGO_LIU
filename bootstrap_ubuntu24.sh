#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
THIRD_PARTY_DIR="${ROOT_DIR}/third_party"
GTSAM_SRC_DIR="${THIRD_PARTY_DIR}/rwth_irt_gtsam"
GTSAM_INSTALL_DIR="${THIRD_PARTY_DIR}/gtsam-install"
GTSAM_BUILD_DIR="${GTSAM_SRC_DIR}/build"
PROJECT_BUILD_DIR="${ROOT_DIR}/build"

mkdir -p "${THIRD_PARTY_DIR}"

sudo apt-get update
sudo apt-get install -y \
  build-essential \
  cmake \
  ninja-build \
  git \
  pkg-config \
  python3 \
  libeigen3-dev \
  libboost-all-dev \
  libgeographiclib-dev \
  libtbb-dev

if [[ ! -d "${GTSAM_SRC_DIR}/.git" ]]; then
  git clone https://github.com/rwth-irt/gtsam.git "${GTSAM_SRC_DIR}"
else
  git -C "${GTSAM_SRC_DIR}" fetch --all --tags
fi

cmake -S "${GTSAM_SRC_DIR}" -B "${GTSAM_BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
  -DCMAKE_INSTALL_PREFIX="${GTSAM_INSTALL_DIR}"

cmake --build "${GTSAM_BUILD_DIR}"
cmake --install "${GTSAM_BUILD_DIR}"

cmake -S "${ROOT_DIR}" -B "${PROJECT_BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_DIR="${GTSAM_INSTALL_DIR}/lib/cmake/GTSAM"

cmake --build "${PROJECT_BUILD_DIR}"

cat <<EOF

offline_lc_minimal bootstrap completed.

Build directory:
  ${PROJECT_BUILD_DIR}

Example run:
  ${ROOT_DIR}/run_offline.sh --config ${ROOT_DIR}/config/default_offline.cfg

EOF
