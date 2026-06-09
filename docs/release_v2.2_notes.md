# offline_lc_minimal v2.2

## Summary

`v2.2` formalizes the shared vertical reference workflow for repeated runs of
the same road segment. The pipeline is now split into:

1. per-member Stage2 runs;
2. one group-level distance-domain height reference, `z_shared(s)`;
3. per-member Stage3-only runs using the same shared reference.

This release is intended for grouped road-profile processing where each member
should keep its own Stage2 attitude, horizontal position, horizontal velocity,
and bias behavior, while Stage3 uses a common absolute height target.

## Highlights

- Added `TrajectoryCsvReader` so saved Stage2 `trajectory.csv` files can be
  loaded back into the optimizer.
- Added `offline_lc_shared_vertical_reference_builder`.
  - Builds a shared reference line from the longest projected Stage2 trajectory.
  - Projects usable RTKFIX and Stage2 nav-bridge samples into the same distance
    coordinate.
  - Uses stable RTK sections for absolute height and a de-biased Stage2 bridge
    in outage, recovery, and sparse sections.
  - Exports reference, reference-line, projection diagnostics, and summary CSVs.
- Added `offline_lc_stage3_runner`.
  - Consumes saved Stage2 trajectory plus `z_shared(s)`.
  - Holds Stage2 attitude, horizontal position, horizontal velocity, and bias.
  - Optimizes height with the shared target while retaining the vertical
    jump/bias and IMU-related vertical framework.
- Simplified the Stage3 height optimization policy so the Stage3-only path is
  a pure shared-height refinement instead of a nested Stage1/Stage2 rerun.
- Added plotting and validation support for shared-reference profiles and IRI
  comparison.
- Added a detailed Stage1-Stage3 pipeline and module catalog:
  `docs/stage1_to_stage3_pipeline_and_modules.md`.

## Usage

The full usage guide is in:

```text
docs/shared_vertical_reference_workflow.md
```

The detailed Stage1-Stage3 flow and module/default-parameter reference is in:

```text
docs/stage1_to_stage3_pipeline_and_modules.md
```

## Promoted Default Parameters

The default configuration now matches the final shared-reference validation
settings:

```text
enable_stage_attitude_debug_export=true
enable_base_graph_tilt_reference_constraint=true
base_graph_tilt_reference_sigma_rad=0.003
stage3_vertical_anchor_sigma_m=0.001
enable_stage3_jump_velocity_smoothness_regularizer=false
enable_stage3_jump_height_highfreq_deadband=false
enable_stage3_jump_adaptive_context_envelope=false
enable_stage3_stage2_vertical_increment_hold=true
stage3_stage2_vertical_increment_sigma_m=0.0002
stage3_stage2_vertical_increment_jump_sigma_m=0.0005
enable_stage3_stage2_jump_shape_hold=true
stage3_stage2_jump_shape_sigma_m=0.0005
vertical_envelope_gate_sigma_multiple=1
vertical_envelope_factor_sigma_m=0.01
vertical_velocity_delta_context_jump_sigma_scale=100
```

Minimal command sequence:

```bash
./build/offline_lc_runner \
  --config config/member_a.cfg \
  --output-dir runs/member_a_stage2 \
  --set enable_stage3_vertical_reference_optimization=false

./build/offline_lc_runner \
  --config config/member_b.cfg \
  --output-dir runs/member_b_stage2 \
  --set enable_stage3_vertical_reference_optimization=false

./build/offline_lc_shared_vertical_reference_builder \
  --manifest runs/shared_manifest.csv \
  --output-dir runs/shared_reference \
  --grid-spacing-m 1 \
  --sigma-m 0.015

./build/offline_lc_stage3_runner \
  --config config/member_a.cfg \
  --stage2-trajectory runs/member_a_stage2/trajectory.csv \
  --shared-reference runs/shared_reference/shared_vertical_reference.csv \
  --shared-reference-line runs/shared_reference/shared_reference_line.csv \
  --output-dir runs/member_a_stage3_shared

./build/offline_lc_stage3_runner \
  --config config/member_b.cfg \
  --stage2-trajectory runs/member_b_stage2/trajectory.csv \
  --shared-reference runs/shared_reference/shared_vertical_reference.csv \
  --shared-reference-line runs/shared_reference/shared_reference_line.csv \
  --output-dir runs/member_b_stage3_shared
```

Manifest format:

```csv
member_id,config_path,stage2_trajectory_path,gnss_path
member_a,config/member_a.cfg,runs/member_a_stage2/trajectory.csv,/mnt/d/Code/dataset/.../member_a/gnss_solution_gnss_fgo.txt
member_b,config/member_b.cfg,runs/member_b_stage2/trajectory.csv,/mnt/d/Code/dataset/.../member_b/gnss_solution_gnss_fgo.txt
```

Plot command:

```bash
python scripts/plot_shared_vertical_reference_profiles.py \
  --shared-reference runs/shared_reference/shared_vertical_reference.csv \
  --shared-reference-line runs/shared_reference/shared_reference_line.csv \
  --projection-diagnostics runs/shared_reference/shared_vertical_reference_projection_diagnostics.csv \
  --trajectory member_a=runs/member_a_stage3_shared/trajectory.csv \
  --trajectory member_b=runs/member_b_stage3_shared/trajectory.csv \
  --output runs/shared_reference/stage3_shared_height_profiles.png
```

## Validation

Build and full native test suite:

```bash
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && cmake --build build -j4'
wsl bash -lc 'cd /mnt/d/Code/offline_lc_minimal && LD_LIBRARY_PATH=/home/xunyi/.local/offline_lc_minimal/gtsam/lib ctest --test-dir build --output-on-failure'
```

The current rtk_err_11 shared-reference validation used:

```text
runs/rtk_err_11_shared_vertical_20260607
```

Final Stage3 IRI on the common distance interval
`0.274888860 m -> 300.274888860 m`:

```text
label   segment  windows  mean_m_per_km  min_m_per_km  max_m_per_km
132613  20 m     15       2.346565869    1.382487616   3.163548628
181436  20 m     15       2.389214916    1.658299105   3.359658408
132613  50 m     6        2.346565869    1.853878606   2.909512560
181436  50 m     6        2.389214916    1.882627342   2.830074117
```

IRI outputs from that validation are under:

```text
runs/rtk_err_11_shared_vertical_20260607/stage3_final_iri_segments
```

## Notes

- This release does not remove the vertical jump/bias framework. Stage3-only
  still uses it because IMU vertical artifacts remain part of the problem.
- The shared reference is low-frequency and group-level. It should not be
  interpreted as raw RTK height scatter copied into the optimizer.
- Stage3-only maps the shared height to `Stage2 + lowfreq(z_shared - Stage2)`.
  The final pass then uses a tight Gaussian height anchor plus Stage2 increment
  and jump-shape inheritance so `Stage3 - Stage2` remains low-frequency for IRI.
- Stage3-only is intentionally independent from the Stage1/Stage2 execution
  path; rerun Stage2 first whenever the Stage2 configuration changes.
