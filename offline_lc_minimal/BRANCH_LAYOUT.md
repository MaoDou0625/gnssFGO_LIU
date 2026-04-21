# Branch Layout

This branch is intentionally reduced to a single top-level project:

- `offline_lc_minimal/`

## Purpose

- keep the phase-1 offline IMU + solution-level GNSS work independent from the legacy framework
- avoid carrying the unrelated local edits from `tuiche_zero`
- make review and future extraction simpler

## Paths

- Windows worktree root: `D:\Code\gnssFGO_offline_lc_minimal`
- Windows project root: `D:\Code\gnssFGO_offline_lc_minimal\offline_lc_minimal`
- WSL project root: `/mnt/d/Code/gnssFGO_offline_lc_minimal/offline_lc_minimal`

## Branch identity

- local branch: `codex/offline-lc-minimal`
- remote branch: `origin/codex/offline-lc-minimal`
- base line used when creating the branch: `origin/main`

## Scope kept in this branch

- standalone C++ implementation for offline IMU + LC GNSS fusion
- Ubuntu 24.04 bootstrap and toolchain docs
- Windows edit + WSL build/run workflow
- optional RTKFIX comparison plotting script

## Scope intentionally removed from this branch

- `online_fgo`
- LiDAR, vision, rosbag, ROS message packages
- submodules and framework content that are not required by the new phase-1 baseline
