# Offline LC Minimal Branch

This branch is a clean integration path for the standalone phase-1 offline IMU + solution-level GNSS project.

## Purpose

- keep the new `offline_lc_minimal` work separate from the Windows debugging changes in `tuiche_zero`
- base the new line on `origin/main`
- make it easy to review, build, and push without mixing in the legacy framework edits

## Local layout

- Git worktree root: `D:\Code\gnssFGO_offline_lc_minimal`
- Main development project: `D:\Code\gnssFGO_offline_lc_minimal\offline_lc_minimal`
- Recommended WSL path: `/mnt/d/Code/gnssFGO_offline_lc_minimal/offline_lc_minimal`

## Branch

- local branch: `codex/offline-lc-minimal`
- remote target: `origin/codex/offline-lc-minimal`
- base branch: `origin/main`

## Scope in this branch

- standalone `offline_lc_minimal` C++ project
- Ubuntu 24.04 bootstrap and toolchain docs
- Windows edit + WSL run workflow docs
- no ROS dependency in phase 1
- no TC integration in phase 1

## Why this is a separate branch

The original repository currently contains multiple unrelated local changes in `online_fgo` and other submodules. This branch avoids carrying those edits into the new offline baseline and keeps review focused on the minimal IMU + GNSS implementation only.
