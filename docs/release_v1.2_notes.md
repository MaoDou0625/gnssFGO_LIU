# offline_lc_minimal v1.2

## Summary

`v1.2` makes the Phase 31 vertical recovery setup the default offline configuration.
The main change is strict, explainable Body-Z NHC weighting: the configured NHC
sigma now matches the effective graph weight instead of being strengthened
implicitly by overlapping windows or duplicate per-state velocity factors.

## Highlights

- Promoted the current Phase 31 configuration to `config/default_offline.cfg`.
- Added strict Body-Z NHC effective weighting:
  - every dynamic state receives at most one Body-Z velocity factor;
  - global Body-Z NHC windows are non-overlapping;
  - jump and non-jump NHC use the same effective strength, `0.005 m/s` and `0.005 m`.
- Preserved the Phase 30 RTK drift-corrected center reference path:
  - raw RTK envelope gate remains unchanged;
  - drift-corrected RTK is used only for center-pull.
- Added diagnostics for applied NHC sigma, duplicate velocity states, interval overlap,
  and global/jump NHC factor counts.
- Kept the existing adaptive vertical motion reweighting, PV consistency,
  jump-bias, and leakage-corrected Body-Z residual structure.

## Validation

Validated on the Phase 31 smoke run:

```text
cmake --build build -j4
ctest --test-dir build --output-on-failure
./run_offline.sh --config config/transformed1cut1_vertical_envelope_phase31_strict_nhc_weight.cfg --output-dir build/rtk_gate_eval/phase31_strict_nhc_weight
```

Key Phase 31 NHC diagnostics:

```text
body_z_nhc_velocity_factor_count=3800
body_z_nhc_unique_velocity_factor_count=3800
body_z_nhc_velocity_duplicate_state_count=0
body_z_nhc_interval_overlap_count=0
body_z_nhc_jump_velocity_factor_count=864
body_z_nhc_global_velocity_factor_count=2936
```

50 m IRI comparison from the same smoke run:

| Source | Mean IRI `mm/m` |
|---|---:|
| optimized_nav | 5.225 |
| raw_rtk | 15.596 |
| drift_corrected_rtk | 5.573 |
| optimized_nav_on_rtk_station | 4.891 |
