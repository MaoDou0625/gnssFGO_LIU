# offline_lc_minimal v1.4

## Summary

`v1.4` promotes the RTK outage segmented Stage2 pipeline as the default release
configuration. The main behavior change is that the pre-outage vertical result is
now solved as a standalone cutoff-equivalent segment, so outage and post-outage
information no longer rewrites the pre-outage `z/vz/ba_z` solution.

## Highlights

- Added RTK vertical drift gate-aware weighting to reduce nav-reference driven
  RTK center drift outside the raw RTK envelope.
- Added outage-aware RTK drift/reference segmentation so post-outage drift
  observations do not smooth backward into pre-outage references.
- Added body-z `ba_z` reestimate segments with hard `ba_z` continuity breaks:
  GM transitions are skipped across reestimate boundaries and the corresponding
  IMU intervals use `BazContinuityBreakCombinedImuFactor`.
- Added RTK outage `ba_z` reestimate planning, including RTK-only outage
  segments with weak scalar `B.z` priors.
- Added causal pre-outage reference/fence support and then replaced the release
  path with segmented batch execution across `PRE_RTK_VALID`, `RTK_OUTAGE`, and
  `POST_RTK_VALID` sections.
- Added global Stage1 reference support for segmented Stage2, while preserving
  the segment-local vertical state so global Stage1 `z/vz/ba_z` is not injected
  into Stage2 vertical solves.
- Changed the pre-outage segment runner to use the original base config and a
  standalone `[0, outage_start]` cutoff solve. This makes the full segmented run
  match the standalone cutoff result before the first RTK outage.
- Promoted the validated segmented Stage2 pseudo-float outage configuration to
  `config/default_offline.cfg`.

## Validation

Validated build and unit/integration tests:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && ctest --test-dir build --output-on-failure'
```

Validated run used to select and inspect the promoted default:

```text
runs/transformed1rtkjumpcut1_stage2_segmented_batch_standalone_pre_v2
```

The promoted `config/default_offline.cfg` was smoke-tested with:

```text
./run_offline.sh --config config/default_offline.cfg --output-dir runs/default_offline_v1_4_release_check
```

Default smoke anchors:

```text
stage2_velocity_optimization_enabled = true
rtk_outage_segmented_batch_enabled = true
rtk_outage_batch_segment_count = 3
rtk_outage_segmented_batch_run_count = 3
rtk_vertical_lowpass_reference_enabled = false
stage1_yaw_refinement_enabled = true
stage1_yaw_refinement_final_yaw_rad = -1.81019
static_baz_ug = -529.465
feedback_forward_up_slope_10s = -8.23736e-05 m/s
```

Key validation anchors from the current segmented run:

```text
stage1_yaw_refinement_final_yaw_rad = -1.81019
static_baz_ug = -529.465
optimized_first_dynamic_baz_ug = -529.465
feedback_forward_up_slope_10s = -8.23736e-05 m/s
rtk_vertical_drift_first20_mean_correction_m = 0.00352883
```

Pre-outage alignment against the standalone cutoff run
`runs/transformed1rtkjumpcut1_stage2_latest_cut_before_outage_horizontal_ref_v1`:

```text
first dynamic 10 s slope:
  segmented full = -0.0914 mm/s
  cutoff pre     = -0.0914 mm/s

pre-outage up RMS difference  ~= 2.4e-7 mm
pre-outage ba_z RMS difference ~= 3.1e-8 ug
```

The full segmented result therefore preserves the cutoff-equivalent pre-outage
vertical solution while still solving outage and post-outage sections as separate
batch segments.
