# Windows + WSL Workflow

This project is developed with a split workflow:

- edit source files on Windows
- build and run in WSL Ubuntu 24.04

## Paths

- Windows project root: `D:\Code\gnssFGO_offline_lc_minimal\offline_lc_minimal`
- WSL project root: `/mnt/d/Code/gnssFGO_offline_lc_minimal/offline_lc_minimal`

## Why this workflow

- the current phase-1 target is Linux-first
- GTSAM and the toolchain are easier to keep stable in WSL than in native Windows builds
- this avoids bringing the old Windows compatibility patches into the new offline baseline

## Typical loop

1. Edit code on Windows in the worktree under `D:\Code\gnssFGO_offline_lc_minimal`.
2. Open WSL Ubuntu 24.04.
3. Build:

```bash
cd /mnt/d/Code/gnssFGO_offline_lc_minimal/offline_lc_minimal
chmod +x bootstrap_ubuntu24.sh run_offline.sh
./bootstrap_ubuntu24.sh
```

4. Run:

```bash
cd /mnt/d/Code/gnssFGO_offline_lc_minimal/offline_lc_minimal
./run_offline.sh --config config/default_offline.cfg
```

5. Optional RTK comparison plot on Windows:

```powershell
python D:\Code\gnssFGO_offline_lc_minimal\offline_lc_minimal\scripts\plot_nav_vs_rtk.py `
  --trajectory D:\Code\gnssFGO_offline_lc_minimal\offline_lc_minimal\runs\default_offline\trajectory.csv `
  --summary D:\Code\gnssFGO_offline_lc_minimal\offline_lc_minimal\runs\default_offline\summary.txt `
  --gnss D:\Code\dataset\BeiJingGongLuTuiChe\gnssFGO_use\20260323_124742\transformed1cut1\gnss_solution_gnss_fgo.txt `
  --output D:\Code\gnssFGO_offline_lc_minimal\offline_lc_minimal\runs\default_offline\nav_vs_rtk.png
```

## Notes

- `run_offline.sh` adds the local GTSAM library directory to `LD_LIBRARY_PATH`.
- phase 1 only uses `imu_gnss_fgo.txt` and `gnss_solution_gnss_fgo.txt`.
- if a future phase needs rosbag support, add a separate ROS 2 Jazzy adapter layer instead of expanding the phase-1 core.
