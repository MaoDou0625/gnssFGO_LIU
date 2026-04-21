# Ubuntu 24.04 Setup

This project targets `Ubuntu 24.04` on WSL2.

## Option A: bootstrap script

From the project root:

```bash
chmod +x bootstrap_ubuntu24.sh
./bootstrap_ubuntu24.sh
```

The script installs the base toolchain, builds the preferred `rwth-irt/gtsam`, and then builds `offline_lc_minimal`.

It also prepares the default directory layout expected by `run_offline.sh`, so the runner can find the locally installed GTSAM shared libraries.

## Option B: manual setup

Install the minimal toolchain:

```bash
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
```

Clone and build the preferred GTSAM fork:

```bash
git clone https://github.com/rwth-irt/gtsam.git ~/third_party/rwth_irt_gtsam
cmake -S ~/third_party/rwth_irt_gtsam -B ~/third_party/rwth_irt_gtsam/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_BUILD_TESTS=OFF \
  -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_USE_SYSTEM_EIGEN=ON \
  -DCMAKE_INSTALL_PREFIX=$HOME/.local/offline_lc_minimal/gtsam
cmake --build ~/third_party/rwth_irt_gtsam/build
cmake --install ~/third_party/rwth_irt_gtsam/build
```

Build the runner:

```bash
cmake -S /mnt/d/Code/gnssFGO/offline_lc_minimal -B /mnt/d/Code/gnssFGO/offline_lc_minimal/build -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_DIR=$HOME/.local/offline_lc_minimal/gtsam/lib/cmake/GTSAM
cmake --build /mnt/d/Code/gnssFGO/offline_lc_minimal/build
```

## Quick smoke run

```bash
cd /mnt/d/Code/gnssFGO/offline_lc_minimal
chmod +x run_offline.sh
./run_offline.sh --config config/default_offline.cfg
```

## Notes on GTSAM on Ubuntu 24.04

- Prefer building `rwth-irt/gtsam` from source.
- The Ubuntu 24.04 `libgtsam-dev` package can be incomplete on some machines. In this workspace it installed a broken CMake export that referenced a missing `libCppUnitLite.a`, so the phase-1 path uses a source build by default.
- `run_offline.sh` checks both `${PROJECT_ROOT}/third_party/gtsam-install/lib` and `$HOME/.local/offline_lc_minimal/gtsam/lib` and prepends the first existing one to `LD_LIBRARY_PATH`.

## ROS note

Phase 1 intentionally does not install ROS.

If a future phase requires rosbag or ROS adapters, use `ROS 2 Jazzy` on Ubuntu 24.04 rather than reusing the old Humble-oriented docker path:

- [ROS 2 Jazzy Ubuntu install](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
- [REP 2000 target platform table](https://www.ros.org/reps/rep-2000.html)
