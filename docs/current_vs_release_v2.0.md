# Current vs Release v2.0

## Summary

This comparison documents `v2.1` against the `v2.0` release.

`v2.0` formalized the Stage3 lowpass vertical-reference workflow. The current
release keeps that Stage3 architecture, but changes the default reference
smoothing method from lowpass-only smoothing to a spatial spline baseline.

## Main Behavioral Change

The v2.1 default configuration now uses:

```text
enable_stage3_vertical_reference_optimization=true
stage3_vertical_reference_smoothing_method=spline_baseline
stage3_vertical_reference_spline_knot_spacing_m=1
stage3_vertical_reference_spline_smooth_lambda=10000
stage3_vertical_reference_spline_anchor_weight=100000
stage3_vertical_reference_spline_slope_weight=1000
stage3_vertical_anchor_sigma_m=0.005
```

The lowpass setting remains available:

```text
stage3_vertical_reference_lowpass_cutoff_hz=0.01
```

but it is no longer the formal default reference source when
`stage3_vertical_reference_smoothing_method=spline_baseline`.

## Why It Changed

The lowpass-only Stage3 reference reduced high-frequency vertical roughness,
but it could still produce visible endpoint/transition behavior around static
to dynamic changes. The spline baseline smooths in station space and maps the
result back to the Stage3 time-domain reference, which better matches the
downstream Stage3 workflow while keeping a low IRI baseline.

## Terminal Static Fix

`v2.1` also fixes the terminal static Stage3 reference behavior. Terminal
static rows are now constrained by the Stage3 vertical reference instead of
being skipped, so the final static span remains anchored to the RTK-consistent
height reference.

## IRI Comparison

The v2.0 release note recorded:

```text
run=transformed1rtkjumpcut1_stage3_jump_scan_v008_h00085
mean=1.457452789 mm/m
median=1.403372961 mm/m
max=1.832818026 mm/m
```

The v2.1 corrected spline-baseline run records:

```text
run=runs/stage3_spline_terminal_anchor_20260531
mean=1.456547084 mm/m
median=1.396578124 mm/m
max=1.832826149 mm/m
```

Per-segment v2.1 IRI:

```text
0-50 m      1.222061 mm/m
50-100 m    1.832826 mm/m
100-150 m   1.557021 mm/m
150-200 m   1.274250 mm/m
200-250 m   1.396578 mm/m
```

## Code Scope

Major additions since `v2.0`:

- `Stage3VerticalReferenceSmoother` module for Stage3 reference smoothing;
- spline-baseline config serialization, defaults, and tests;
- Stage3 terminal-static reference anchoring;
- MATLAB smoothing tuner controls for lowpass, spatial PLS, and spline
  baseline comparisons;
- 50 m IRI calculation support from the smoothing workflow.
