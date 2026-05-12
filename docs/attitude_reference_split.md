# Attitude Reference Split

This document records the current attitude-reference constraint boundary after the RTK outage work.

## Constraint Semantics

`enable_attitude_reference_constraint` now enables two non-overlapping factor types through `AttitudeReferenceConstraintBuilder`:

- `RollPitchReferenceFactor(X_i)` keeps the optimized roll and pitch close to the seed/reference roll and pitch at every dynamic node.
- `RelativeYawReferenceFactor(X_i, X_j)` keeps only the adjacent dynamic-node yaw increment close to the seed/reference yaw increment.

The builder adds roll/pitch factors for `dynamic_start_index ... end`, and relative-yaw factors for `dynamic_start_index + 1 ... end`. It intentionally does not add a static-to-dynamic relative-yaw edge, so the static/initial absolute yaw cannot be reintroduced through this path.

## Yaw Offset

The attitude reference path no longer locks each node to an absolute reference yaw. A common yaw offset across the dynamic segment is therefore not corrected by these factors. That offset must be estimated by a later yaw-sensitive constraint, such as RTK course or a body-y/NHC relationship against RTK velocity.

## Configuration

- `attitude_reference_sigma_rad` controls the roll/pitch absolute reference factor.
- `attitude_reference_relative_yaw_sigma_rad` controls the adjacent relative-yaw reference factor. The default is `0.01`.

## Diagnostics

- `attitude_reference_diagnostics.csv` is now a per-state roll/pitch diagnostic. It still records reference and optimized yaw for inspection, but its yaw residual column is `NaN` because no absolute yaw factor is present.
- `relative_yaw_reference_diagnostics.csv` records each relative-yaw edge, including reference delta yaw, optimized delta yaw, and the yaw residual.
