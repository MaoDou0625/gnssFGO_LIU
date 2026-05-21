# Current vs release v1.4

## Scope

This document compares the current release candidate against `v1.4`
(`1bb139960af3812639f1cedbf4ca7358873c13fa`). The current candidate is based
on `54078209ee0df687a5fad64a8bc207176b27963d`.

## Commit Range

```text
5407820 Align late static height hold with initial static
9ea92fa Tighten late static height hold
72ce20f Add data-driven late static vertical anchoring
04875b4 Add Stage1 outage body-y envelope constraint
5ceef85 Implement RTK outage boundary handoff
```

## Main Changes

- RTK outage segmented Stage2 now runs `PRE_RTK_VALID`, `POST_RTK_VALID`, and
  `RTK_OUTAGE` with the outage segment solved last from pre/post vertical
  boundary references.
- The recovery-side reference is derived from post-outage RTKFIX samples instead
  of inheriting outage-end height.
- Boundary constraints are limited to `up`, `vz`, and `ba_z` policy decisions;
  attitude remains governed by the Stage1 global reference.
- Stage1 can estimate a pre-outage body-y velocity envelope and apply a
  fixed-axis robust velocity envelope inside RTK outages.
- Late-static detection is data driven from RTK and IMU features, excluding the
  initial static alignment span and RTK outage spans.
- Late-static Stage2 vertical constraints now match the initial static vertical
  behavior: `vz=0`, RTK median height reference, and 1 mm height hold from the
  window reference state to later static states.

## New Diagnostics

- `rtk_outage_boundary_diagnostics.csv`
- `rtk_outage_recovery_reference.csv`
- `rtk_outage_bias_continuity_policy.csv`
- `stage1_outage_body_y_envelope.csv`
- `stage1_outage_body_y_state_diagnostics.csv`
- `late_static_feature_diagnostics.csv`
- `late_static_threshold_diagnostics.csv`
- `late_static_windows.csv`

## Validation Snapshot

The release candidate was validated with the default config on
`transformed1rtkjumpcut1`:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ctest --test-dir build --output-on-failure'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && rm -rf runs/transformed1rtkjumpcut1_late_static_initial_consistent_smoke && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ./build/offline_lc_runner --config config/default_offline.cfg --output-dir runs/transformed1rtkjumpcut1_late_static_initial_consistent_smoke'
```

Key smoke anchors:

```text
rtk_outage_segmented_batch_enabled=true
rtk_outage_segmented_batch_run_count=3
rtk_outage_boundary_constraints_enabled=true
rtk_outage_boundary_reference_count=3
rtk_outage_boundary_up_factor_count=3
rtk_outage_boundary_vz_factor_count=3
stage1_outage_body_y_envelope_enabled=true
stage1_outage_body_y_envelope_valid_count=1
stage1_outage_body_y_velocity_factor_count=1588
stage1_outage_body_y_rmse_mps=0.0159714
stage1_outage_body_y_deadband_mps=0.0319427
late_static_window_count=1
late_static_vz_factor_count=1621
late_static_up_factor_count=1621
late_static_height_hold_factor_count=1620
late_static_rtk_speed_threshold_mps=0.0629657
late_static_gyro_rms_threshold_radps=0.00228279
vertical_position_velocity_consistency_max_abs_mismatch_m=6.96387e-05
```

Late-static vertical stability in the `400-480 s` relative-time window:

```text
nav_up_range_m=0.000657588
nav_up_minus_median_rms_m=0.000061120
nav_up_slope_mps=-1.50842068e-07
rtk_up_range_m=0.034399234
```
