# Stage 2 Velocity Optimization

Stage 2 is entered through `enable_stage2_velocity_optimization=true`. The runner first executes the Stage 1 RTK heading refinement pass, then uses the final Stage 1 trajectory as the initial values and fixed attitude-axis reference for the second pass.

In Stage 2, speed/NHC constraints do not update the body axes. `body_x`, `body_y`, and `body_z` are projections onto the Stage 1 fixed IMU/body axes. The graph instead estimates one global mount leakage vector:

- `k_zx`: forward `body_x` velocity leaking into vehicle vertical velocity.
- `k_zy`: lateral `body_y` velocity leaking into vehicle vertical velocity.
- `k_yx`: forward `body_x` velocity leaking into vehicle lateral velocity.

The vehicle-frame constraint speeds are:

```text
v_vehicle_x = v_body_x
v_vehicle_y = v_body_y - k_yx * v_body_x
v_vehicle_z = v_body_z - k_zx * v_body_x - k_zy * v_body_y
```

The Stage 2 vehicle NHC factors connect only navigation velocity `V_i` and the global mount leakage variable `m0`. They do not connect pose `X_i`, so they cannot rotate the attitude to satisfy NHC. A strong per-node attitude hold factor anchors each pose attitude to the Stage 1 final attitude, while RTK horizontal velocity remains a navigation-frame velocity constraint.

Outputs:

- `stage2_mount_leakage_diagnostics.csv`: initial/optimized `k`, prior residual, and raw/corrected `vehicle_y/z` RMS/max.
- `stage2_vehicle_nhc_state_diagnostics.csv`: per-state `v_body_x/y/z`, `v_vehicle_y/z`, and the correction terms used by Stage 2.

