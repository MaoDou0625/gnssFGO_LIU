# Stage 2 Vertical Optimization

Stage 2 is entered through `enable_stage2_velocity_optimization=true`. The runner first executes the Stage 1 RTK heading refinement pass, then uses the final Stage 1 trajectory as the initial values and fixed attitude-axis reference for the second pass.

The current Stage 2 scope is vertical recovery only. Stage 1 already provides the horizontal pose and heading baseline, so Stage 2 must not add horizontal NHC or RTK velocity constraints that can degrade the horizontal solution.

In Stage 2, NHC constraints do not update the body axes. `body_x`, `body_y`, and `body_z` are projections onto the Stage 1 fixed IMU/body axes. The graph keeps the legacy three-element mount leakage variable for compatibility, but the active Stage 2 model estimates only one coefficient:

- `k_zx`: forward `body_x` velocity leaking into vehicle vertical velocity.
- `k_zy`: fixed to zero in the active Stage 2 builder.
- `k_yx`: fixed to zero in the active Stage 2 builder.

The vehicle-frame constraint speeds are:

```text
v_vehicle_x = v_body_x
v_vehicle_y = v_body_y
v_vehicle_z = v_body_z_reference_horizontal + body_z_axis_up * v_nav_up - k_zx * v_body_x_reference
```

`v_body_x_reference` and the horizontal part of `v_body_z_reference_horizontal` come from the fixed Stage 1 reference trajectory. The vehicle-z residual Jacobian with respect to the optimized velocity uses only the navigation up component scaled by the fixed body-z up projection; it does not feed the NHC residual back into current horizontal velocity or use the residual to reshape forward speed. The `k_zx` Jacobian still uses the fixed reference forward speed, so the graph can estimate the vertical leakage coefficient without making forward velocity the correction target.

The Stage 2 vehicle NHC factors connect only navigation velocity `V_i` and the global mount leakage variable `m0`. They do not connect pose `X_i`, so they cannot rotate the attitude to satisfy NHC. A strong per-node attitude hold factor anchors each pose attitude to the Stage 1 final attitude.

Stage 2 also adds horizontal hold factors from the Stage 1 final trajectory:

- horizontal position hold on `X_i.translation().x/y`, default `stage2_horizontal_position_hold_sigma_m = 1e-4`;
- horizontal velocity hold on `V_i.x/y`, default `stage2_horizontal_velocity_hold_sigma_mps = 1e-4`.

These holds keep the second pass focused on height. RTK horizontal velocity and outage 3D velocity-delta constraints are disabled in Stage 2.

Outputs:

- `stage2_mount_leakage_diagnostics.csv`: initial/optimized `k`, prior residual, and raw/corrected `vehicle_y/z` RMS/max. In the active vertical-only path, only `k_zx` should move.
- `stage2_vehicle_nhc_state_diagnostics.csv`: per-state `v_body_x/y/z`, `v_vehicle_y/z`, and the correction terms used by Stage 2. `vehicle_y` is diagnostic only in the active vertical-only path.
