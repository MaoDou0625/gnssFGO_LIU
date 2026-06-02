# Attitude Reference Split

This document records the current attitude-reference constraint boundary after the RTK outage work.

## Constraint Semantics

`enable_attitude_reference_constraint` now enables only the yaw-increment reference path through
`AttitudeReferenceConstraintBuilder`:

- `RelativeYawReferenceFactor(X_i, X_j)` keeps only the adjacent yaw increment close to the base/IMU reference yaw increment.

The builder no longer adds per-node roll/pitch absolute factors from the RTK/body-z
seed reference. Roll and pitch continuity are left to the IMU preintegration chain
and any explicit physical factors in the main graph. This avoids feeding a
seed-optimized attitude branch back into the main optimization as a pseudo
observation.

The builder adds relative-yaw factors for every adjacent graph state from
`X(0)->X(1)` through the final state, including the initial static segment and the
static-to-dynamic boundary. Those yaw increments come from the base IMU
forward/reference states, not from the RTK seed optimized attitude, so low-motion
dynamic intervals are not pulled by RTK-seed yaw drift.

## Yaw Offset

The attitude reference path no longer locks each node to an absolute reference yaw. A common yaw offset across the full static/dynamic chain is therefore not corrected by these factors. If a later yaw-sensitive constraint is added, the relative yaw chain lets the initial/static and dynamic attitudes move together rather than bending only the dynamic segment.

This path does not add RTK course, RTK velocity, body-y velocity, or other yaw observation factors. RTK-derived heading checks are diagnostics or future work only.

## Configuration

- `attitude_reference_sigma_rad` is retained for backward config compatibility but
  is not used by the ordinary attitude-reference builder.
- `attitude_reference_relative_yaw_sigma_rad` controls the adjacent relative-yaw reference factor. The default is `0.01`.

## Diagnostics

- `attitude_reference_diagnostics.csv` is empty for the ordinary builder because no
  per-state roll/pitch reference is added.
- `relative_yaw_reference_diagnostics.csv` records each full-chain relative-yaw edge, including reference delta yaw, optimized delta yaw, and the yaw residual.
