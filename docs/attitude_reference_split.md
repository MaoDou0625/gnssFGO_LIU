# Attitude Reference Split

This document records the current attitude-reference constraint boundary after the RTK outage work.

## Constraint Semantics

`enable_attitude_reference_constraint` now enables two non-overlapping factor types through `AttitudeReferenceConstraintBuilder`:

- `RollPitchReferenceFactor(X_i)` keeps the optimized roll and pitch close to the seed/reference roll and pitch at every dynamic node.
- `RelativeYawReferenceFactor(X_i, X_j)` keeps only the adjacent yaw increment close to the base/IMU reference yaw increment.

The builder adds roll/pitch factors for `dynamic_start_index ... end`, using the RTK seed/reference attitude only for roll and pitch. It adds relative-yaw factors for every adjacent graph state from `X(0)->X(1)` through the final state, including the initial static segment and the static-to-dynamic boundary. Those yaw increments come from the base IMU forward/reference states, not from the RTK seed optimized attitude, so low-motion dynamic intervals are not pulled by RTK-seed yaw drift.

## Yaw Offset

The attitude reference path no longer locks each node to an absolute reference yaw. A common yaw offset across the full static/dynamic chain is therefore not corrected by these factors. If a later yaw-sensitive constraint is added, the relative yaw chain lets the initial/static and dynamic attitudes move together rather than bending only the dynamic segment.

This path does not add RTK course, RTK velocity, body-y velocity, or other yaw observation factors. RTK-derived heading checks are diagnostics or future work only.

## Configuration

- `attitude_reference_sigma_rad` controls the roll/pitch absolute reference factor.
- `attitude_reference_relative_yaw_sigma_rad` controls the adjacent relative-yaw reference factor. The default is `0.01`.

## Diagnostics

- `attitude_reference_diagnostics.csv` is now a per-state roll/pitch diagnostic. It still records reference and optimized yaw for inspection, but its yaw residual column is `NaN` because no absolute yaw factor is present.
- `relative_yaw_reference_diagnostics.csv` records each full-chain relative-yaw edge, including reference delta yaw, optimized delta yaw, and the yaw residual.
