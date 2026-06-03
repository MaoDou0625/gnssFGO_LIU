# offline_lc_minimal v1.3

## Summary

`v1.3` promotes the Phase 32 RTK outage smoother path as the default offline
configuration. The release focuses on keeping attitude stable when RTKFIX
support disappears, while letting velocity absorb the outage recovery through
loose IMU-propagated delta-velocity constraints.

## Highlights

- Promoted the current Phase 32 configuration to `config/default_offline.cfg`.
- Added RTK outage attitude recovery constraints:
  - guarded 3D absolute attitude hold around each RTK outage window;
  - relative yaw continuity over the same guarded span;
  - configurable `rtk_outage_attitude_guard_duration_s` to prevent boundary
    attitude flips.
- Added `VelocityDeltaFactor(V_i, V_j)` for 3D outage velocity continuity.
  This factor only connects velocity variables and does not feed velocity
  residuals back into pose attitude.
- Kept non-fixed RTK samples out of GNSS factors through `drop_non_rtkfix=true`.
- Kept stage1 free of NHC and stage2 vehicle constraints while allowing RTK
  outage smoothing, attitude hold, and loose 3D velocity-delta recovery.
- Added diagnostics:
  - `rtk_outage_attitude_hold_diagnostics.csv`
  - `rtk_outage_velocity_delta_3d_diagnostics.csv`
  - summary counters and residual statistics for outage attitude and velocity
    recovery factors.

## Validation

Validated on the Phase 32 RTK outage smoother path:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && ctest --test-dir build --output-on-failure'
```

The promoted default configuration was also smoke-tested:

```text
./build/offline_lc_runner --config config/default_offline.cfg --output-dir runs/default_offline_v1_3_release_check
```

Default-run anchors:

```text
vertical_motion_adaptive_converged=true
body_z_nhc_strict_effective_weighting_enabled=true
body_z_nhc_velocity_duplicate_state_count=0
body_z_nhc_interval_overlap_count=0
stage2_velocity_optimization_enabled=false
```

Smoke validation used `transformed1rtkjumpcut1` with the RTK loss point moved
20 s earlier by pseudo-float labels and `drop_non_rtkfix=true`:

```text
rtk_outage_attitude_hold_factor_count=1628
rtk_outage_relative_attitude_factor_count=1627
rtk_outage_velocity_delta_3d_factor_count=1587
body_z_nhc_velocity_factor_count=0
stage2_velocity_optimization_enabled=false
```

Within the outage window `5986.992-6066.389 s`, the optimized attitude remained
bounded:

```text
pitch range ~= [-0.99, 0.44] deg
roll range  ~= [-0.12, 0.98] deg
```

No 100 deg class yaw/roll/pitch flip was observed after adding the guarded
outage attitude hold.
