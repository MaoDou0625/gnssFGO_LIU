# Current vs Release v1.5

## Summary

This comparison documents the v2.0 release candidate against `v1.5`.

`v1.5` formalized the RTK outage recovery path with segmented Stage2, Stage1
body-y outage envelope constraints, and late-static vertical stabilization.
The current release keeps that Stage2 path as the first pass, then adds an
experimental-to-default Stage3 pass that uses the Stage2 lowpass height as the
global vertical reference.

## Main Behavioral Change

The v2.0 default configuration now enables Stage3 vertical-reference
optimization:

```text
enable_stage3_vertical_reference_optimization=true
stage3_vertical_reference_lowpass_cutoff_hz=0.01
stage3_vertical_anchor_sigma_m=0.005
stage3_disable_rtk_outage_segmented_batch=true
stage3_disable_stage2_vehicle_nhc_constraint=true
```

Stage3 no longer treats RTK outage segmentation as a special vertical path.
Instead, it first runs the v1.5 Stage2 workflow, builds a low-frequency
vertical reference from the final Stage2 trajectory, and then solves one global
Stage3 pass against that lowpass reference.

## Jump-Window Stabilization

The v2.0 default also promotes the validated jump-window local envelope:

```text
enable_stage3_jump_velocity_smoothness_regularizer=true
stage3_jump_velocity_smoothness_deadband_mps=0.008
stage3_jump_velocity_smoothness_sigma_mps=0.005
enable_stage3_jump_height_highfreq_deadband=true
stage3_jump_height_highfreq_deadband_m=0.00085
stage3_jump_height_highfreq_sigma_m=0.0012
enable_stage3_jump_adaptive_context_envelope=true
stage3_jump_context_velocity_cap_mps=0.008
stage3_jump_context_height_cap_m=0.00085
```

These values mean:

- jump-window `vz` residuals within `+/-8 mm/s` are not penalized;
- jump-window height high-frequency residuals within `+/-0.85 mm` are not
  penalized;
- overflow beyond those deadbands is penalized by the configured sigma values.

The selected setting was chosen because it removes the previous jump-window
spikes without making the jump windows significantly smoother than nearby
non-jump context.

## DVZ Context Scaling

The v2.0 default enables context-aware DVZ output sigma scaling:

```text
vertical_velocity_delta_sigma_scale=100
enable_vertical_velocity_delta_context_sigma_scale=true
vertical_velocity_delta_context_normal_sigma_scale=100
vertical_velocity_delta_context_rough_sigma_scale=1000
vertical_velocity_delta_context_outage_sigma_scale=100
vertical_velocity_delta_context_jump_sigma_scale=1000
vertical_velocity_delta_context_jump_extra_padding_s=0.6
```

This keeps the normal-road DVZ constraint tighter than jump/rough-road spans,
where IMU preintegration is less reliable.

## IRI Result

The selected v2.0 default parameter set was validated with the documented
50 m IRI workflow:

```text
run=transformed1rtkjumpcut1_stage3_jump_scan_v008_h00085
segment_count=5
mean=1.457452789 mm/m
median=1.403372961 mm/m
max=1.832818026 mm/m
```

Per-segment IRI:

```text
0-50 m     1.225889 mm/m
50-100 m   1.832818 mm/m
100-150 m  1.554379 mm/m
150-200 m  1.270805 mm/m
200-250 m  1.403373 mm/m
```

Against the earlier Stage3 jump-context setting (`20 mm/s`, `2.0 mm`), the
selected v2.0 setting reduces mean IRI from `1.624023243 mm/m` to
`1.457452789 mm/m`.

## Code Scope

Major code additions since `v1.5`:

- Stage3 vertical-reference runner, profile planner, timeline aligner, and
  constraint builder;
- Stage3 jump regularizer and adaptive context-envelope planner;
- fixed-forward low-frequency reference experiments for Stage2;
- initial dynamic static detection, lowpass protection, and Vz constraint;
- context-aware vertical velocity delta sigma scaling;
- RTK vertical reference selector and related diagnostics;
- expanded config serialization/validation and regression tests.
