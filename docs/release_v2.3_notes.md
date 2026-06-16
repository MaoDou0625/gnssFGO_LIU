# offline_lc_minimal v2.3

## Summary

`v2.3` promotes the currently validated Stage2/Stage3 height workflow to the
system default. The default behavior now keeps Stage2 as the source of attitude,
horizontal position, horizontal velocity, and bias continuity, while Stage3 only
performs low-frequency absolute-height correction.

The current v2.3 line also includes the Stage3 height-reference envelope tuning
that landed after the `v2.3` tag. For the final Stage3 pass, the authoritative
policy is `MakeStage3HeightOptimizationConfig()`, not a direct replay of every
field in `config/default_offline.cfg`.

Formal release runs should load `config/default_offline.cfg`. The C++ defaults
in `include/offline_lc_minimal/common/Config.h` are fallback values for tests and
ad-hoc construction paths, and they are not a complete copy of the release cfg.
Stage-specific policy helpers, especially `MakeStage3HeightOptimizationConfig()`,
are the final authority for child Stage2/Stage3 passes.

## Default Stage2/Stage3 Policy

Stage2 is enabled by default:

```text
enable_stage2_velocity_optimization=true
enable_stage2_vehicle_nhc_constraint=true
```

Stage3 is enabled by default and uses the low-frequency shared-height strategy:

```text
enable_stage3_vertical_reference_optimization=true
stage3_vertical_reference_smoothing_method=spline_baseline
stage3_vertical_reference_lowpass_cutoff_hz=0.01
stage3_vertical_reference_constraint_mode=gaussian
stage3_vertical_anchor_sigma_m=0.001
stage3_disable_stage2_vehicle_nhc_constraint=true
```

The final Stage3 pass then forces the tuned low-frequency delta policy:

```text
stage3_vertical_reference_constraint_mode=envelope
stage3_vertical_envelope_half_width_m=0.005
stage3_vertical_envelope_sigma_m=0.003
enable_stage3_vertical_envelope_center_pull=false
enable_stage3_stage2_vertical_increment_hold=true
stage3_stage2_vertical_increment_sigma_m=0.0002
stage3_stage2_vertical_increment_jump_sigma_m=0.0005
enable_stage3_stage2_jump_shape_hold=true
stage3_stage2_jump_shape_sigma_m=0.0005
```

`stage3_vertical_anchor_sigma_m=0.001` remains the fallback sigma for mapped
reference rows and Gaussian compatibility paths, but the current final pass uses
the 5 mm envelope gate with the 3 mm envelope sigma.

The older Stage3 jump regularizers remain disabled by default:

```text
enable_stage3_jump_velocity_smoothness_regularizer=false
enable_stage3_jump_height_highfreq_deadband=false
enable_stage3_jump_adaptive_context_envelope=false
```

## Detailed Workflow Document

The detailed operating guide is:

```text
docs/stage2_stage3_default_v2.3_workflow.md
```

It documents:

- the Stage2-only flow;
- shared distance-domain height-reference generation;
- Stage3-only consumption of the shared reference;
- default parameters and enabled modules;
- diagnostic files to inspect;
- the recommended `IRI(stage2 - stage3)` validation method.

## Validation

Build and full test suite:

```bash
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ctest --test-dir build --output-on-failure'
```

The historical rtk_err_11 validation outputs used to establish the
`IRI(stage2 - stage3)` measurement method are under:

```text
runs/road_noise_state_verify_20260609/132613_stage3_lowfreq_delta_policy_default
runs/road_noise_state_verify_20260609/181436_stage3_lowfreq_delta_policy_default
runs/road_noise_state_verify_20260609/stage2_stage3_iri_lowfreq_delta_policy_default
runs/road_noise_state_verify_20260609/stage2_stage3_height_lowfreq_delta_policy_default
```

Historical measured `IRI(stage2 - stage3)` for the low-frequency delta policy:

```text
dataset  20 m mm/m  50 m mm/m
132613   0.2123     0.2026
181436   0.2025     0.1967
```

For comparison, the previous non-low-frequency Stage3 delta strategy produced:

```text
dataset  20 m mm/m  50 m mm/m
132613   1.8756     1.9397
181436   1.2108     1.2533
```

## Notes

- Stage3 still keeps the vertical jump/bias framework because IMU vertical
  artifacts are still present.
- Stage3 should not reinterpret attitude, horizontal position, or horizontal
  velocity. Those remain inherited from Stage2.
- The shared reference is a low-frequency absolute-height target, not raw RTK
  scatter copied into the optimizer.
