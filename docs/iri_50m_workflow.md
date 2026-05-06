# IRI 50m Computation Workflow

This workflow computes 50 m International Roughness Index (IRI) values from an optimized navigation trajectory by first converting the time-domain trajectory to a spatial road profile. The third-party repository `D:\Code\michalsorel_iri_repo` is used only as a read-only IRI implementation dependency; generated profiles, CSV files, and plots should be written under this repository's `runs` directory.

## Inputs

- Navigation trajectory:
  `D:\Code\offline_lc_minimal\runs\transformed1cut1_vertical_envelope_phase7_center_pull\trajectory.csv`
- IRI implementation:
  `D:\Code\michalsorel_iri_repo\python\iri.py`
- Output directory:
  `D:\Code\offline_lc_minimal\runs\transformed1cut1_vertical_envelope_phase7_center_pull\iri_50m`

## Processing Steps

1. Read the optimized trajectory columns `time_s`, `east_m`, `north_m`, `up_m`, `vx_mps`, and `vy_mps`.
2. Convert from time domain to spatial domain by accumulating horizontal distance:
   `s_i = sum(hypot(delta_east, delta_north))`.
3. Remove the low-speed beginning and ending portions using a horizontal speed threshold. The current run uses:
   `speed_threshold_mps = 0.5`.
4. Reset the retained station and elevation to start from zero:
   `s_trim = s - s_start`, `up_trim = up - up_start`.
5. Remove repeated station samples, then resample `up_trim(s_trim)` to a regular 0.25 m spatial grid.
6. Write the two-column spatial profile `[distance_m, elevation_m]` to:
   `space_profile_0p25m.txt`.
7. Call the Michal Sorel IRI implementation with:
   `segment_length = 50 m`, `start_pos = 0`, `method = 2`.
8. Save the IRI table and plots under the output directory.

## Python Invocation Pattern

The current local NumPy version does not expose `np.in1d`, while `iri.py` still calls it. Use a runtime wrapper that maps `np.in1d` to `np.isin` without modifying the third-party source:

```powershell
python -c "import runpy, sys, numpy as np; script=sys.argv[1]; np.in1d=getattr(np,'in1d',np.isin); sys.argv=[script]+sys.argv[2:]; runpy.run_path(script, run_name='__main__')" `
  D:\Code\michalsorel_iri_repo\python\iri.py `
  D:\Code\offline_lc_minimal\runs\transformed1cut1_vertical_envelope_phase7_center_pull\iri_50m\space_profile_0p25m.txt `
  D:\Code\offline_lc_minimal\runs\transformed1cut1_vertical_envelope_phase7_center_pull\iri_50m\iri_50m.txt `
  -segment_length 50 `
  -start_pos 0 `
  -method 2 `
  -plot_file D:\Code\offline_lc_minimal\runs\transformed1cut1_vertical_envelope_phase7_center_pull\iri_50m\iri_50m_michalsorel_plot.png
```

## Current Run Outputs

- `iri_50m.csv`: normalized CSV table with `start_m`, `end_m`, `iri_mm_per_m`, and `iri_std_mm_per_m`.
- `iri_50m.txt`: raw output from `iri.py`.
- `space_profile_0p25m.txt`: time-to-space converted profile used by the IRI calculator.
- `iri_50m_profile_plot.png`: combined spatial profile and 50 m IRI plot.
- `speed_trim_diagnostic.png`: speed threshold and retained interval diagnostic.
- `processing_summary.txt`: parameters, trim interval, retained distance, and IRI summary.

For the current Phase 7 run, `0.5 m/s` trimming keeps about `187.40 m`, producing three complete 50 m segments. The remaining tail shorter than 50 m is intentionally not included in the non-overlapping segment IRI table.
