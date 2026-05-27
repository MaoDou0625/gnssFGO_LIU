# offline_lc_minimal v2.0

## Summary

`v2.0` promotes the Stage3 lowpass vertical-reference workflow as the formal
default. The release keeps the `v1.5` Stage2 RTK outage recovery path as the
first pass, then runs a global Stage3 pass using the Stage2 lowpass height as
the vertical reference.

The release target is lower IRI from the navigation-derived road profile while
avoiding raw RTK high-frequency vertical noise and jump-window spikes.

## Highlights

- Added Stage3 vertical-reference optimization:
  - builds a zero-phase lowpass height reference from the final Stage2
    trajectory;
  - adds a global Stage3 vertical anchor on dynamic states;
  - disables Stage3 RTK outage segmentation so the lowpass reference is applied
    consistently across valid, outage, and recovery spans;
  - disables Stage3 vehicle NHC in the final pass so the lowpass vertical
    anchor is the dominant vertical reference.
- Promoted the validated Stage3 default settings:
  - `stage3_vertical_reference_lowpass_cutoff_hz=0.01`;
  - `stage3_vertical_anchor_sigma_m=0.005`;
  - `stage3_jump_velocity_smoothness_deadband_mps=0.008`;
  - `stage3_jump_height_highfreq_deadband_m=0.00085`.
- Added jump-window local regularization:
  - velocity deadband around the lowpass-derived reference;
  - height high-frequency deadband around the Stage2 lowpass height;
  - context-derived diagnostics for inside/outside jump-window comparisons.
- Added initial dynamic static protection:
  - detects early dynamic-start static spans from IMU/RTK features;
  - protects lowpass reference construction around those static spans;
  - optionally constrains vertical velocity during the detected initial static
    interval.
- Added context-aware DVZ sigma scaling:
  - normal-road DVZ remains comparatively tighter;
  - rough-road and jump contexts get relaxed DVZ output sigma;
  - jump spans retain the jump-local velocity and height regularizers instead
    of relying on DVZ alone.
- Added diagnostics for:
  - Stage3 vertical reference residuals;
  - Stage3 jump regularizer residuals;
  - Stage3 adaptive context-envelope profiles;
  - RTK vertical reference selection and Stage2 low-frequency experiments.

## IRI Validation

The formal v2.0 setting is the `8 mm/s + 0.85 mm` jump-envelope scan:

```text
run=transformed1rtkjumpcut1_stage3_jump_scan_v008_h00085
segment_count=5
mean=1.457452789 mm/m
median=1.403372961 mm/m
max=1.832818026 mm/m
```

Per-segment IRI:

```text
0-50 m     1.225889
50-100 m   1.832818
100-150 m  1.554379
150-200 m  1.270805
200-250 m  1.403373
```

Jump-window comparison for the selected setting:

```text
max inside/outside p95 ratio, vz      = 0.7469
median inside/outside p95 ratio, vz   = 0.4310
max inside/outside p95 ratio, height  = 1.1295
median inside/outside p95 ratio, height = 0.5994
```

This removes the previous jump-window high-frequency spikes; tighter scans
reduced IRI slightly more but made jump-window motion noticeably smoother than
nearby non-jump context.

## Validation

Build and full test suite:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ctest --test-dir build --output-on-failure'
```

Default smoke:

```text
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ./build/offline_lc_runner --config config/default_offline.cfg --output-dir runs/v2_0_default_smoke'
```

## Notes

- The v2.0 default intentionally uses Stage2 as the low-frequency source and
  does not feed raw RTK vertical high-frequency residuals directly into the
  final Stage3 pass.
- The selected jump envelope is a practical release setting, not a claim that
  lower deadbands are always invalid. Use the scan configs when evaluating a
  new dataset.
