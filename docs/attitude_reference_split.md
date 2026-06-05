# Attitude Reference Split

This document records the current attitude-reference constraint boundary after the RTK outage work.

## Constraint Semantics

`enable_attitude_reference_constraint` now enables separate tilt and yaw factor types through `AttitudeReferenceConstraintBuilder`:

- `RollPitchReferenceFactor(X_i)` keeps the optimized roll and pitch close to the seed/reference roll and pitch at every dynamic node.
- `TiltReferenceFactor(X_i)` keeps the optimized body tilt close to `base_graph_optimized` when `enable_base_graph_tilt_reference_constraint=true`.
- `YawReferenceFactor(X_i)` keeps the optimized yaw close to the Stage1/Stage2 yaw reference when base-tilt mode is active. That yaw reference still comes from the existing RTKFIX heading refinement loop through `initial_yaw_override_rad`; it is not a new RTK course factor in the final graph.
- `RelativeYawReferenceFactor(X_i, X_j)` keeps only the adjacent yaw increment close to the base/IMU reference yaw increment.

Without base-tilt mode, the builder adds roll/pitch factors for `dynamic_start_index ... end`, using the RTK seed/reference attitude only for roll and pitch. It adds relative-yaw factors for every adjacent graph state from `X(0)->X(1)` through the final state, including the initial static segment and the static-to-dynamic boundary.

With base-tilt mode, the builder adds a `TiltReferenceFactor` from `base_graph_optimized` and a `YawReferenceFactor` from the yaw-refined reference states at each dynamic node. Outage recovery and outage boundary attitude constraints use the same split: base graph for tilt, and the original outage/boundary reference for yaw. They no longer use full three-axis `AttitudeHoldFactor` in base-tilt mode.

## Yaw Offset

The base-tilt path now has an explicit yaw-only anchor, so the final `graph_with_gnss` cannot drift by a common yaw offset while roll/pitch remain tied to `base_graph_optimized`. The yaw anchor source is still the previously refined initial yaw path, not a full attitude copy from body-z seed or stage2.

This path does not add a new RTK course factor to the final graph. RTK-derived heading is still consumed by Stage1 yaw refinement before the final graph is built.

## Configuration

- `attitude_reference_sigma_rad` controls the roll/pitch absolute reference factor.
- `attitude_reference_relative_yaw_sigma_rad` controls the adjacent relative-yaw reference factor and the yaw-only reference factor in base-tilt mode. The default is `0.01`.
- `base_graph_tilt_reference_sigma_rad` controls the base graph tilt reference factor.

## Diagnostics

- `attitude_reference_diagnostics.csv` is a per-state tilt/yaw diagnostic. In base-tilt mode, reference yaw comes from the yaw-refined reference states and reference pitch/roll come from `base_graph_optimized`.
- `relative_yaw_reference_diagnostics.csv` records each full-chain relative-yaw edge, including reference delta yaw, optimized delta yaw, and the yaw residual.
- `rtk_outage_attitude_hold_diagnostics.csv` labels split base-tilt outage rows as `tilt` and `yaw`; `rtk_outage_boundary_diagnostics.csv` labels split boundary rows as `tilt_yaw`.
