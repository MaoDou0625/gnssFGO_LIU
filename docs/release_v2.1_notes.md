# offline_lc_minimal v2.1

## Summary

`v2.1` promotes the Stage3 spline-baseline vertical reference as the formal
default. It replaces the v2.0 lowpass-only Stage3 reference with a
time-aligned spatial spline baseline built from the Stage2 trajectory, then
uses that baseline as the Stage3 vertical reference for Stage3 and downstream
Stage3-derived road-profile work.

The release target is a smoother navigation-derived vertical baseline with low
50 m IRI, without losing the RTK-consistent static and terminal-static height
anchors.

## Current Default Parameters

The formal v2.1 Stage3 vertical reference parameters in
`config/default_offline.cfg` are:

```text
enable_stage3_vertical_reference_optimization=true
stage3_vertical_reference_smoothing_method=spline_baseline
stage3_vertical_reference_lowpass_cutoff_hz=0.01
stage3_vertical_reference_spline_knot_spacing_m=1
stage3_vertical_reference_spline_smooth_lambda=10000
stage3_vertical_reference_spline_anchor_weight=100000
stage3_vertical_reference_spline_slope_weight=1000
stage3_vertical_anchor_sigma_m=0.005
stage3_vertical_reference_constraint_mode=gaussian
```

The v2.0 jump-window local regularizers remain enabled:

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

Initial dynamic static protection remains part of the default Stage2/Stage3
setup:

```text
enable_initial_dynamic_static_detection=true
initial_dynamic_static_search_duration_s=20
initial_dynamic_static_threshold_multiplier=3
initial_dynamic_static_min_duration_s=8
initial_dynamic_static_merge_gap_s=2
enable_initial_dynamic_static_lowpass_protection=true
initial_dynamic_static_lowpass_blend_s=2
enable_initial_dynamic_static_vz_constraint=true
initial_dynamic_static_vz_sigma_mps=0.0005
```

## Highlights

- Added a modular Stage3 vertical reference smoother:
  - supports the formal `spline_baseline` method;
  - keeps the existing lowpass method available for comparison;
  - constructs the spline in station space and maps the result back to the
    Stage3 time-domain reference used by the optimizer.
- Anchored terminal static Stage3 reference rows:
  - terminal static samples now receive Stage3 vertical reference factors;
  - this prevents the final static span from drifting above the RTK-consistent
    Stage3 reference.
- Added the MATLAB Stage2 smoothing comparison workflow:
  - interactive display toggles for RTK scatter, Stage2 curve, smoothed curve,
    and static-alignment RTK points;
  - adjustable smoothing-frequency range for lowpass experiments;
  - interactive comparison for spatial PLS and spline baseline variants;
  - 50 m IRI calculation button for the optimized smoothing curve.
- Added a reusable `stage2_iri50_calculator.m` workflow for the same 0.25 m
  spatial resampling and 50 m IRI calculation used in release validation.

## IRI Validation

The formal v2.1 setting was validated on the corrected Stage3 run:

```text
run=runs/stage3_spline_terminal_anchor_20260531
source_trajectory=trajectory.csv
moving_window=5823.747800000056 s -> 6104.047800001075 s
trimmed_distance=299.25 m
sampling_interval=0.25 m
segment_length=50 m
segment_count=5
mean=1.456547084 mm/m
median=1.396578124 mm/m
min=1.222060807 mm/m
max=1.832826149 mm/m
std=0.220677727 mm/m
```

Per-segment IRI:

```text
0-50 m      1.222061 mm/m
50-100 m    1.832826 mm/m
100-150 m   1.557021 mm/m
150-200 m   1.274250 mm/m
200-250 m   1.396578 mm/m
```

The remaining `49.25 m` at the end of the moving window is shorter than one
50 m segment and is not included in the segment table.

## Terminal Static Validation

The corrected Stage3 terminal-static behavior was validated on the same run:

```text
stage3_vertical_reference_factor_count=7617
stage3_vertical_reference_skipped=201
terminal_static_start_time=6107.1478000010875 s
terminal_nav_minus_rtk_up_rms=0.007264 m
```

The final static span no longer drifts upward after the dynamic segment; the
Stage3 reference remains anchored through the terminal static interval.

## Validation

Build and full test suite:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ctest --test-dir build --output-on-failure'
```

Default run used for the release validation:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ./build/offline_lc_runner --config config/default_offline.cfg --output-dir runs/stage3_spline_terminal_anchor_20260531'
```

## Notes

- `v2.1` keeps the v2.0 Stage3 anchor sigma and jump-window envelope values.
- The main default behavior change is the Stage3 reference source:
  `spline_baseline` is now formalized as the default Stage3 smoothing method.
- The spline baseline is still exported as time-domain Stage3 reference data,
  so downstream Stage3 processing does not need a separate spatial-domain
  optimizer interface.
