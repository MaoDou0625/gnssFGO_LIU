# Shared Vertical Reference Workflow

This workflow separates group-consistent height estimation into three explicit
steps:

1. Run each member only through Stage2 and keep its `trajectory.csv`.
2. Build one group-level `z_shared(s)` from all Stage2 trajectories and usable
   RTKFIX samples.
3. Run Stage3-only for each member with the same shared reference files.

The goal is to keep the reliable Stage2 state components from each member
while making the Stage3 absolute height target common across the group.
Stage3-only inherits Stage2 attitude, horizontal position, horizontal velocity,
and bias references. Only the vertical target is replaced by `z_shared(s)`.

## Build

Build the runner, shared-reference builder, Stage3-only runner, and tests:

```bash
cmake --build build -j4
```

The expected executable names are:

```text
build/offline_lc_runner
build/offline_lc_shared_vertical_reference_builder
build/offline_lc_stage3_runner
```

## Step 1: Run Stage2

Run each member to Stage2 and disable the embedded Stage3 pass:

```bash
./build/offline_lc_runner \
  --config config/member_a.cfg \
  --output-dir runs/member_a_stage2 \
  --set enable_stage3_vertical_reference_optimization=false

./build/offline_lc_runner \
  --config config/member_b.cfg \
  --output-dir runs/member_b_stage2 \
  --set enable_stage3_vertical_reference_optimization=false
```

Each Stage2 output directory must contain `trajectory.csv`. The original GNSS
file is still required by the shared-reference builder so it can apply the same
RTKFIX usability policy while building the group reference.

## Step 2: Create A Manifest

Create a manifest CSV. Relative paths are resolved relative to the manifest
file.

```csv
member_id,config_path,stage2_trajectory_path,gnss_path
member_a,config/member_a.cfg,runs/member_a_stage2/trajectory.csv,/mnt/d/Code/dataset/.../member_a/gnss_solution_gnss_fgo.txt
member_b,config/member_b.cfg,runs/member_b_stage2/trajectory.csv,/mnt/d/Code/dataset/.../member_b/gnss_solution_gnss_fgo.txt
```

Required columns:

- `member_id`: unique member name used in diagnostics.
- `config_path`: member config containing `imu_path` and `gnss_path`.
- `stage2_trajectory_path`: Stage2 `trajectory.csv`.

Optional column:

- `gnss_path`: overrides `gnss_path` from the config.

At least two members are required.

## Step 3: Build `z_shared(s)`

Build the shared reference:

```bash
./build/offline_lc_shared_vertical_reference_builder \
  --manifest runs/shared_manifest.csv \
  --output-dir runs/shared_reference \
  --grid-spacing-m 1 \
  --sigma-m 0.015
```

Outputs:

- `shared_vertical_reference.csv`: distance-domain height reference with
  `s_m`, `reference_up_m`, `sigma_m`, source labels, and weights.
- `shared_reference_line.csv`: common reference line used to project every
  member into the same distance coordinate.
- `shared_vertical_reference_projection_diagnostics.csv`: RTK and Stage2 bridge
  projection diagnostics.
- `shared_vertical_reference_summary.txt`: reference member and row counts.

Reference construction:

- The longest valid Stage2 trajectory defines the shared reference line.
- Usable RTKFIX samples are projected to the reference line and used in stable
  RTK sections.
- RTK outage, recovery guard, and sparse sections use a de-biased Stage2 nav
  bridge.
- The final `z_shared(s)` applies a low-frequency offset smoother and a
  distance-domain local-linear lowpass, so 1 m RTK scatter is not copied
  directly into Stage3.

## Step 4: Run Stage3-Only

Run each member with the same shared reference:

```bash
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

Optional overrides:

```bash
./build/offline_lc_stage3_runner \
  --config config/member_a.cfg \
  --imu /mnt/d/Code/dataset/.../imu_gnss_fgo.txt \
  --gnss /mnt/d/Code/dataset/.../gnss_solution_gnss_fgo.txt \
  --stage2-trajectory runs/member_a_stage2/trajectory.csv \
  --shared-reference runs/shared_reference/shared_vertical_reference.csv \
  --shared-reference-line runs/shared_reference/shared_reference_line.csv \
  --output-dir runs/member_a_stage3_shared \
  --set stage3_vertical_anchor_sigma_m=0.002 \
  --verbose
```

Stage3-only behavior:

- raw GNSS factors are disabled by the Stage3 height optimization policy;
- Stage2 attitude, horizontal position, horizontal velocity, and bias are held
  as references;
- the vertical target comes from `z_shared(s)` after projecting each Stage2
  state to the shared reference line;
- IMU, vertical jump, vertical velocity, and vertical bias factors remain in the
  optimization.

## Plot And Inspect

Plot Stage3 results on the shared distance axis:

```bash
python scripts/plot_shared_vertical_reference_profiles.py \
  --shared-reference runs/shared_reference/shared_vertical_reference.csv \
  --shared-reference-line runs/shared_reference/shared_reference_line.csv \
  --projection-diagnostics runs/shared_reference/shared_vertical_reference_projection_diagnostics.csv \
  --trajectory member_a=runs/member_a_stage3_shared/trajectory.csv \
  --trajectory member_b=runs/member_b_stage3_shared/trajectory.csv \
  --jump-windows member_a=runs/member_a_stage3_shared/body_z_seed_jump_windows.csv \
  --jump-windows member_b=runs/member_b_stage3_shared/body_z_seed_jump_windows.csv \
  --output runs/shared_reference/stage3_shared_height_profiles.png
```

The plot overlays:

- `z_shared(s)`;
- each projected Stage3 height trajectory;
- used RTKFIX samples from the projection diagnostics;
- vertical jump windows.

For IRI validation, resample the projected Stage3 height to the desired spatial
spacing and compute non-overlapping 20 m or 50 m IRI windows with the same
third-party Sroubek/Sorel IRI implementation used by the chapter 5 MATLAB
workflow.

## Notes

- `trajectory.csv` yaw, pitch, and roll fields remain output and diagnostics;
  internal cross-stage attitude references are rotation-native.
- The shared reference is a group-level absolute height target, not a direct
  raw RTK trace.
- Stage3-only is intentionally independent from Stage1/Stage2 execution. This
  makes the workflow reproducible: rerun Stage2 first, rebuild `z_shared(s)`,
  then rerun Stage3-only for every member with the same shared reference files.
