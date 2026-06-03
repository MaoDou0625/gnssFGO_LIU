# Stage1 Yaw Refinement

This stage is an outer-loop initialization refinement, not a new graph factor.
It runs the existing offline optimizer with RTK/GNSS position, IMU, initial static
constraints, RTK vertical envelope/drift paths, and the current attitude
reference path. It disables NHC, RTK velocity, vertical motion window
consistency, and vertical jump/window constraints. RTK outage smoothing remains
available in stage1 so fixed-solution gaps can keep attitude locked while
velocity is recovered with loose IMU-propagated delta-velocity constraints.

After each optimization run, it computes RTK course from RTKFIX position
differences using a centered heading window, matches those course samples to the
optimized navigation yaw, and estimates the robust median `nav_yaw - rtk_course`
error. If the median is larger than the noise threshold, the next run updates
`initial_yaw_override_rad` by the opposite signed error, capped by
`stage1_yaw_update_max_rad`. The next run then uses the existing
`TrajectoryInitializer` override path, so the static IMU window recomputes the
initial bias for the adjusted yaw.

The stage does not add RTK course, RTK velocity, body-y velocity, or other
yaw-observation factors to the graph. Those remain separate second-stage or
future constraints.

Key configuration:

- `enable_stage1_yaw_refinement`
- `stage1_yaw_refinement_max_iterations`
- `stage1_heading_window_s`
- `stage1_heading_time_tolerance_s`
- `stage1_heading_min_displacement_m`
- `stage1_heading_noise_floor_rad`
- `stage1_yaw_update_max_rad`
- `enable_rtk_outage_attitude_hold`
- `rtk_outage_attitude_guard_duration_s`
- `rtk_outage_absolute_attitude_sigma_rad`
- `rtk_outage_relative_attitude_sigma_rad`
- `enable_rtk_outage_velocity_delta_3d`
- `rtk_outage_velocity_delta_3d_sigma_mps`

The output `trajectory.csv` is the final stage1 trajectory. The file
`stage1_yaw_refinement_diagnostics.csv` records each optimization run, including
the input yaw, median heading error, estimated heading noise, applied update,
next yaw, matched sample count, optimizer final error, and GNSS NIS mean.

When outage recovery is enabled, `rtk_outage_attitude_hold_diagnostics.csv`
records the guarded absolute attitude hold rows and relative-yaw rows, and
`rtk_outage_velocity_delta_3d_diagnostics.csv` records the loose 3D
delta-velocity residuals inside each outage window.
