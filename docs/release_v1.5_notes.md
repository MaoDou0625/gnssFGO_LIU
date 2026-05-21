# offline_lc_minimal v1.5

## Summary

`v1.5` promotes the current RTK outage recovery path as the default release. It
keeps the `v1.4` segmented Stage2 structure, then tightens the outage handoff,
adds a Stage1 body-y outage velocity envelope, and adds data-driven late-static
vertical stabilization for the post-outage stop period.

## Highlights

- Added RTK outage boundary recovery references:
  - post-outage RTKFIX samples provide the recovery-side `up/vz` anchor;
  - `PRE` and `POST` are solved as trusted segments before solving `OUTAGE`;
  - outage assembly keeps the pre side at outage start and the post side at
    outage end.
- Added boundary constraints for vertical handoff:
  - `up` and `vz` constraints are always kept when a valid boundary reference is
    available;
  - `ba_z` continuity is controlled by the new bias-continuity policy and can be
    broken for large bias changes or reestimate windows;
  - attitude is not constrained by the outage boundary layer.
- Added Stage1 RTK-outage body-y velocity envelope:
  - estimates the pre-outage body-y normal envelope from the baseline Stage1
    trajectory;
  - applies fixed-axis robust body-y velocity factors inside the outage only;
  - avoids adding horizontal line or displacement constraints.
- Added data-driven late-static detection from RTK and IMU features:
  - thresholds are estimated from the dataset distribution;
  - initial static and RTK outage spans are excluded;
  - accepted late-static windows add Stage2-only vertical constraints.
- Aligned late-static vertical behavior with the initial static alignment:
  - `vz=0` uses the same `0.0005 m/s` scale as the initial static ZUPT;
  - RTK median height reference uses the same `0.02 m` scale as the initial
    static RTK height reference;
  - static height hold remains `1 mm` and uses a reference-state-to-window-state
    topology like the initial static hold.
- Added diagnostics for outage recovery, body-y envelope behavior, and
  late-static detection/constraints.

## Validation

Build and full test suite:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ctest --test-dir build --output-on-failure'
```

Default smoke:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && rm -rf runs/transformed1rtkjumpcut1_late_static_initial_consistent_smoke && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ./build/offline_lc_runner --config config/default_offline.cfg --output-dir runs/transformed1rtkjumpcut1_late_static_initial_consistent_smoke'
```

Smoke anchors:

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
vertical_position_velocity_consistency_max_abs_mismatch_m=6.96387e-05
```

Late-static height check for the `400-480 s` relative-time window:

```text
nav_up_range_m=0.000657588
nav_up_minus_median_rms_m=0.000061120
nav_up_slope_mps=-1.50842068e-07
rtk_up_range_m=0.034399234
```

## Notes

- `v1.5` intentionally does not add attitude boundary constraints. Stage1
  remains the source of attitude and horizontal reference for segmented Stage2.
- If late-static detection cannot separate dynamic and static distributions, the
  detector records the skip reason and the run falls back without late-static
  constraints.
