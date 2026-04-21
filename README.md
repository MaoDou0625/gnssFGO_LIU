# offline_lc_minimal

`offline_lc_minimal` is a new standalone first-phase implementation for offline IMU + solution-level GNSS fusion.

It is intentionally independent from `online_fgo` and does not depend on ROS, pluginlib, PCL, OpenCV, LiDAR, camera, or rosbag in phase 1.

In this repository line it lives under the dedicated branch worktree:

- Windows: `D:\Code\offline_lc_minimal`
- WSL: `/mnt/d/Code/offline_lc_minimal`

See [BRANCH_LAYOUT.md](BRANCH_LAYOUT.md) and [WORKFLOW_WINDOWS_WSL.md](WORKFLOW_WINDOWS_WSL.md) for branch-level workflow details.

## Architecture

- `fgo_core`
  - `include/offline_lc_minimal/core`
  - `src/core`
  - Batch IMU + GNSS solution fusion using GTSAM.
- `fgo_io_text`
  - `include/offline_lc_minimal/io`
  - `src/io`
  - Plain-text parsers for `imu_gnss_fgo.txt` and `gnss_solution_gnss_fgo.txt`.
- `offline_runner`
  - `src/offline_runner/main.cpp`
  - Thin CLI that loads config, runs the batch pipeline, and writes outputs.
- `fgo_io_ros2`
  - Placeholder only in phase 1.
  - Reserved for future Ubuntu 24.04 + ROS 2 Jazzy adapters.

## Phase 1 Scope

- Input:
  - `imu_gnss_fgo.txt`
  - `gnss_solution_gnss_fgo.txt`
- Factors:
  - IMU `CombinedImuFactor`
  - GNSS position factor only
- Initialization:
  - position from first valid GNSS solution
  - velocity default `0`
  - roll/pitch from gravity alignment
  - yaw from early displacement, then fallback yaw if displacement is too small
- IMU preintegration:
  - `Use2ndOrderCoriolis(true)` enabled by default

## Build

Preferred environment is documented in [README_ubuntu24.md](README_ubuntu24.md).

Quick example after dependencies are available:

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

## Run

```bash
chmod +x run_offline.sh
./run_offline.sh --config config/default_offline.cfg
```

CLI flags can override config values:

```bash
./run_offline.sh \
  --config config/default_offline.cfg \
  --imu /mnt/d/Code/dataset/.../imu_gnss_fgo.txt \
  --gnss /mnt/d/Code/dataset/.../gnss_solution_gnss_fgo.txt \
  --output-dir ./runs/demo
```

## Outputs

Each run writes:

- `summary.txt`
- `data_summary.txt`
- `config_snapshot.cfg`
- `trajectory.csv`
- `gnss_residuals.csv`

Optional comparison plot:

```bash
python scripts/plot_nav_vs_rtk.py \
  --trajectory runs/default_offline/trajectory.csv \
  --summary runs/default_offline/summary.txt \
  --gnss /path/to/gnss_solution_gnss_fgo.txt \
  --output runs/default_offline/nav_vs_rtk.png
```

## Notes

- Phase 1 does not require a high-precision GT/reference trajectory.
- Phase 1 targets an engineering baseline: deterministic runs, stable optimization, inspectable residuals, and a clean module boundary.
- TC is intentionally out of scope until per-satellite observations are available.
- `run_offline.sh` adds the local GTSAM library directory to `LD_LIBRARY_PATH` when the dependency is installed from source.
