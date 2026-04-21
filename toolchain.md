# Toolchain Notes

## Preferred stack

- OS: Ubuntu 24.04.4 LTS on WSL2
- Compiler: GCC or Clang with C++20
- Build system: CMake + Ninja
- Linear algebra: Eigen3
- Geographic conversion: GeographicLib
- Graph optimization: `rwth-irt/gtsam`

## Why phase 1 avoids ROS

Phase 1 is deliberately scoped to offline text replay:

- plain-text inputs only
- no rosbag reader
- no `rclcpp`
- no `pluginlib`
- no message packages
- no vision/LiDAR dependencies

This keeps the baseline small enough to validate IMU + solution-level GNSS behavior independently from the legacy framework.

## GTSAM policy

Preferred:

- build `rwth-irt/gtsam` from source for repeatable behavior that stays close to the historical framework

Fallback for quick smoke builds on Ubuntu 24.04:

- `libgtsam-dev` from the Noble repository may work on some systems, but it is not the default recommendation
- in this workspace the packaged CMake export referenced a missing `libCppUnitLite.a`, so the documented path uses a local source build instead

Runtime note:

- when GTSAM is installed from source, use `run_offline.sh` so the local `libgtsam.so` and companion libraries are added to `LD_LIBRARY_PATH`

## Output philosophy

The runner always writes inspectable text outputs:

- data summary
- run summary
- trajectory
- GNSS residuals
- config snapshot

The first milestone is an engineering baseline, not a GT-backed benchmark.
