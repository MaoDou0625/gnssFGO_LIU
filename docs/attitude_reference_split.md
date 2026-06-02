# Attitude Reference Split

This document records the current attitude-reference constraint boundary after the RTK outage work.

## Constraint Semantics

`enable_attitude_reference_constraint` is retained for configuration compatibility,
but `AttitudeReferenceConstraintBuilder` no longer adds ordinary yaw, roll, or
pitch reference factors.

The builder no longer adds per-node roll/pitch absolute factors from the RTK/body-z
seed reference and no longer adds adjacent yaw-increment factors from the base/IMU
reference states. Yaw, roll, and pitch continuity are left to the IMU
preintegration chain and any explicit physical factors in the main graph. This
avoids feeding a seed-optimized attitude branch, or a separate precomputed yaw
reference chain, back into the main optimization as a pseudo observation.

## Yaw Offset

The attitude reference path no longer locks each node to an absolute reference yaw
and no longer constrains adjacent yaw increments. A common yaw offset across the
full static/dynamic chain is therefore not corrected by this path. If a later
yaw-sensitive constraint is added, it should be introduced explicitly instead of
reusing seed-derived attitude references.

This path does not add RTK course, RTK velocity, body-y velocity, IMU-derived
relative yaw, or other yaw observation factors. RTK-derived heading checks are
diagnostics or future work only.

RTK outage recovery is a separate path. When `enable_rtk_outage_attitude_hold`
is enabled, it no longer adds per-state absolute attitude hold factors inside
the guarded outage span. It adds adjacent `Rot3` relative-rotation factors whose
residual is `Logmap(reference_delta.between(optimized_delta))`, matching the
relative-change semantics of the velocity-delta outage recovery path.

## Configuration

- `attitude_reference_sigma_rad` is retained for backward config compatibility but
  is not used by the ordinary attitude-reference builder.
- `attitude_reference_relative_yaw_sigma_rad` is retained for backward config
  compatibility but is not used by the ordinary attitude-reference builder.
- `rtk_outage_relative_attitude_sigma_rad` is the active outage attitude sigma
  for adjacent `Rot3` relative-rotation factors.
- `rtk_outage_absolute_attitude_sigma_rad` is retained for compatibility and for
  other legacy attitude-hold paths, but outage recovery no longer uses it for
  per-state attitude hold.

## Diagnostics

- `attitude_reference_diagnostics.csv` is empty for the ordinary builder because no
  per-state roll/pitch reference is added.
- `relative_yaw_reference_diagnostics.csv` is empty for the ordinary builder
  because no relative-yaw reference is added.
- `rtk_outage_attitude_hold_diagnostics.csv` uses `constraint_type=relative_rotation`
  for outage recovery rows. `residual_x/y/z_rad` are the full relative `Rot3`
  residual vector, while `reference_relative_rotvec_*` and
  `optimized_relative_rotvec_*` record the target and optimized adjacent-state
  rotation vectors.
