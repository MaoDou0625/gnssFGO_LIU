# Shared Vertical Reference Workflow

This workflow separates group-consistent height estimation into three explicit steps.

1. Run each member only through Stage2 and keep its `trajectory.csv`.
2. Build one group-level `z_shared(s)` from the Stage2 trajectories and usable RTKFIX samples.
3. Run `offline_lc_stage3_runner` for each member with the same shared reference files.

Example Stage2 command:

```bash
offline_lc_runner --config member.cfg --output-dir member_stage2 \
  --set enable_stage3_vertical_reference_optimization=false
```

Build the shared reference from a manifest:

```bash
offline_lc_shared_vertical_reference_builder --manifest shared_manifest.csv \
  --output-dir shared_reference --grid-spacing-m 1 --sigma-m 0.015
```

The manifest columns are:

```csv
member_id,config_path,stage2_trajectory_path,gnss_path
```

Run Stage3-only for each member:

```bash
offline_lc_stage3_runner --config member.cfg \
  --stage2-trajectory member_stage2/trajectory.csv \
  --shared-reference shared_reference/shared_vertical_reference.csv \
  --shared-reference-line shared_reference/shared_reference_line.csv \
  --output-dir member_stage3_shared
```

Plot both Stage3 results on the shared distance axis:

```bash
python scripts/plot_shared_vertical_reference_profiles.py \
  --shared-reference shared_reference/shared_vertical_reference.csv \
  --shared-reference-line shared_reference/shared_reference_line.csv \
  --projection-diagnostics shared_reference/shared_vertical_reference_projection_diagnostics.csv \
  --trajectory run_a=run_a_stage3/trajectory.csv \
  --trajectory run_b=run_b_stage3/trajectory.csv \
  --jump-windows run_a=run_a_stage3/body_z_seed_jump_windows.csv \
  --jump-windows run_b=run_b_stage3/body_z_seed_jump_windows.csv \
  --output shared_reference/stage3_shared_height_profiles.png
```

The shared reference stores group-level absolute height from `trajectory.csv` `h_m` and usable RTKFIX `h_m`. It uses the de-biased Stage2 nav bridge as the short-wave shape, applies only a smoothed low-frequency RTK offset, and finishes with a local-linear distance-domain lowpass so 1 m RTK scatter is not copied directly into `z_shared(s)`. The Stage3-only runner converts that height back into each member's local ENU `up_m` before adding vertical factors. It still uses the normal IMU, vertical jump, and vertical bias factors. It consumes Stage2 as the hold reference for attitude, horizontal position, horizontal velocity, and bias; only the vertical target comes from `z_shared(s)`.
