# offline_lc_minimal

`offline_lc_minimal` is a new standalone first-phase implementation for offline IMU + solution-level GNSS fusion.

It is intentionally independent from `online_fgo` and does not depend on ROS, pluginlib, PCL, OpenCV, LiDAR, camera, or rosbag in phase 1.

In this repository line it lives under the dedicated branch worktree:

- Windows: `D:\Code\offline_lc_minimal`
- WSL: `/mnt/d/Code/offline_lc_minimal`

See [BRANCH_LAYOUT.md](BRANCH_LAYOUT.md) and [WORKFLOW_WINDOWS_WSL.md](WORKFLOW_WINDOWS_WSL.md) for branch-level workflow details.

Algorithm notes:

- [OFFLINE_IMU_GNSS_CURRENT_VS_ORIGINAL.md](OFFLINE_IMU_GNSS_CURRENT_VS_ORIGINAL.md)
- [OFFLINE_IMU_GNSS_PRECISION_ENHANCEMENT.md](OFFLINE_IMU_GNSS_PRECISION_ENHANCEMENT.md)
- [docs/shared_vertical_reference_workflow.md](docs/shared_vertical_reference_workflow.md)

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
  - GNSS position factor
  - GP-interpolated GNSS position factor when measurements fall between state timestamps
- Initialization:
  - optional initial static alignment phase before navigation states are created
  - position from the first quality-filtered GNSS solution after the static alignment phase
  - velocity default `0`
  - roll/pitch from gravity alignment by default
  - optional static dual-vector IMU alignment for initial attitude, yaw, and bias estimation
  - fallback yaw path remains early GNSS displacement, then configured yaw if displacement is too small
  - `static_alignment_duration_s` controls how long the initial IMU-only static alignment period lasts
- IMU preintegration:
  - `Use2ndOrderCoriolis(true)` enabled by default
- GNSS weighting:
  - robust loss is configurable
  - default uses `cauchy + 0.5`
  - `NO_SOLUTION` stays dropped by default
  - set `drop_non_rtkfix = true` to treat `RTKFLOAT` and `SINGLE` as deleted instead of merely down-weighted
- IMU-first yaw option:
  - set `prefer_imu_initial_yaw = true` to try static dual-vector alignment first
  - uses `imu_dual_vector_window_s`, `imu_dual_vector_min_sample_count`, and `imu_dual_vector_min_cross_norm`
  - falls back to GNSS displacement / configured yaw if the IMU alignment is not reliable

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
  - compatibility-friendly per-state GNSS summary
- `gnss_alignment.csv`
  - per-measurement sync/interpolation/drop diagnostics
  - written when `write_debug_csv = true`
- `trajectory_imu_rate.csv`
  - optional IMU-rate AVP export
  - written when `write_imu_rate_avp = true`
- `trajectory_imu_rate_diagnostics.csv`
  - per-interval IMU-rate reconstruction status
  - written when both `write_imu_rate_avp = true` and `write_debug_csv = true`

`trajectory.csv` keeps the low-rate optimized state outputs:

- pose and velocity
- IMU-driven orientation (`yaw/pitch/roll`)
- bias terms and GNSS usage/residual columns

`trajectory_imu_rate.csv` includes:

- IMU-rate pose / velocity / attitude
- linearly blended bias output between optimized node biases
- LLH converted from the fused ENU trajectory

`summary.txt` now reports:

- total GNSS factor count
- synchronized GNSS factor count
- interpolated GNSS factor count
- dropped / cached GNSS counts
- optional `dropped_non_rtkfix_count` when `drop_non_rtkfix = true`
- optional IMU-rate export count / interval count / skipped interval count

Optional comparison plot:

```bash
python scripts/plot_nav_vs_rtk.py \
  --trajectory runs/default_offline/trajectory.csv \
  --summary runs/default_offline/summary.txt \
  --gnss /path/to/gnss_solution_gnss_fgo.txt \
  --output runs/default_offline/nav_vs_rtk.png
```

Optional attitude plot:

```bash
python scripts/plot_attitude_over_time.py \
  --trajectory runs/default_offline/trajectory.csv \
  --output runs/default_offline/attitude_over_time.png
```

`attitude_over_time.png` uses three subplots for `yaw`, `pitch`, and `roll`.

## Shared Stage3 Height Workflow

For repeated runs of the same road segment, the v2.3 workflow can split the
pipeline into Stage2, shared distance-domain height reference generation, and
Stage3-only optimization. This keeps each member's Stage2 attitude, horizontal
position, horizontal velocity, and bias references, while using one common
`z_shared(s)` vertical target for all members.

See [docs/stage2_stage3_default_v2.3_workflow.md](docs/stage2_stage3_default_v2.3_workflow.md)
for the current default parameters, diagnostics, and IRI validation method.

See [docs/shared_vertical_reference_workflow.md](docs/shared_vertical_reference_workflow.md)
for the manifest format, commands, outputs, and plotting workflow.

## Notes

- Phase 1 does not require a high-precision GT/reference trajectory.
- Phase 1 targets an engineering baseline: deterministic runs, stable optimization, inspectable residuals, and a clean module boundary.
- TC is intentionally out of scope until per-satellite observations are available.
- `run_offline.sh` adds the local GTSAM library directory to `LD_LIBRARY_PATH` when the dependency is installed from source.
- The current offline path keeps IMU attitude in `Pose3.rotation()` and adds an `omega` state only to support finer GNSS time alignment.
- Set `write_imu_rate_avp = true` to enable the dual-end constrained scheme C IMU-rate AVP reconstruction.
