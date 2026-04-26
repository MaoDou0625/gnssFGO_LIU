#pragma once

#include <string>
#include <string_view>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct OfflineRunnerConfig {
  std::string imu_path;
  std::string gnss_path;
  std::string output_dir = "./runs/default_offline";

  bool enable_gnss = true;
  bool enable_gp_interpolated_gnss = true;
  bool enable_error_state_feedback = false;
  bool enable_segment_error_feedback = false;
  bool enable_segment_local_error_feedback = false;
  bool verbose = false;
  bool write_debug_csv = true;
  bool write_error_diagnostics = false;
  bool write_segment_error_diagnostics = false;
  bool write_imu_rate_avp = false;
  double state_frequency_hz = 20.0;
  double error_state_frequency_hz = 1.0;
  double gnss_time_offset_s = 0.0;
  double state_meas_sync_lower_bound_s = -0.02;
  double state_meas_sync_upper_bound_s = 0.02;

  double gravity_mps2 = 9.81;
  double imu_sigma_acc = 0.08;
  double imu_sigma_gyro = 0.002;
  double integration_sigma = 1e-5;
  double bias_acc_sigma = 1e-4;
  double bias_gyro_sigma = 1e-5;
  double bias_acc_prior_sigma = 0.10;
  double bias_gyro_prior_sigma = 0.01;
  bool enable_global_acc_bias = false;
  double global_acc_bias_tie_sigma_mps2 = 1e-5;
  double global_acc_bias_tie_sigma_xy_mps2 = 1e-5;
  bool enable_global_gyro_bias = false;
  double global_gyro_bias_tie_sigma_radps = 1e-7;
  bool enable_vertical_acc_bias_gm_process = false;
  double vertical_acc_bias_tau_s = 100.0;
  double vertical_acc_bias_process_noise_scale = 1.0;
  bool enable_vertical_rtk_preintegration_feedback = false;
  double vertical_rtk_gate_sigma_multiple = 2.0;
  double vertical_rtk_inside_gate_sigma_scale = 2.0;
  double vertical_rtk_outside_gate_sigma_scale = 1.0;
  double vertical_rtk_inside_feedback_gain_scale = 1.0;
  double vertical_rtk_outside_feedback_gain_scale = 1.0;
  double vertical_rtk_feedback_bias_gain = 1e-4;
  double vertical_rtk_feedback_attitude_gain = 1e-3;
  double vertical_rtk_feedback_sigma_dp_m = 5.0;
  double vertical_rtk_feedback_sigma_baz_mps2 = 10.0;
  double vertical_rtk_feedback_sigma_attitude_rad = 1.0;
  double vertical_rtk_feedback_min_interval_s = 1.0;
  int vertical_local_recovery_max_iterations = 4;
  double vertical_local_recovery_max_attitude_delta_rad = 1e-2;
  double vertical_local_recovery_max_baz_delta_mps2 = 1e-2;
  double vertical_global_vz_window_s = 1.0;
  double vertical_global_vz_smooth_window_s = 1.0;
  double vertical_jump_candidate_min_separation_s = 1.0;
  int vertical_jump_max_candidates_per_segment = 5;
  int vertical_jump_max_selected_points_per_segment = 3;
  double vertical_jump_hold_window_s = 2.0;
  double vertical_jump_step_min_threshold_mps = 0.05;
  double vertical_jump_vz_prior_sigma_mps = 0.03;
  int vertical_jump_window_default_padding_states = 2;
  double vertical_jump_window_support_ratio = 0.25;
  double vertical_jump_window_max_duration_s = 0.75;
  int vertical_jump_window_max_points = 15;
  double vertical_jump_window_tail_target_s = 1.0;
  double vertical_jump_window_velocity_smoothness_weight = 20.0;
  double vertical_jump_window_height_integral_weight = 5.0;
  double vertical_jump_window_ref_weight = 1.0;
  int vertical_jump_window_max_correction_attempts = 240;
  double vertical_jump_future_trend_window_s = 0.0;
  int vertical_jump_future_trend_min_fix_count = 5;
  double vertical_jump_future_trend_mean_weight = 0.0;
  double vertical_jump_future_trend_slope_weight = 0.0;
  bool enable_vertical_local_up_anchor_fallback = false;
  bool enable_vertical_inside_bias_adaptation = false;
  double vertical_inside_bias_window_s = 8.0;
  double vertical_inside_bias_min_window_s = 2.0;
  int vertical_inside_bias_min_observations = 8;
  double vertical_inside_bias_update_interval_s = 0.2;
  double vertical_inside_bias_gain = 0.002;
  double vertical_inside_attitude_gain = 0.002;
  double vertical_inside_max_delta_attitude_rad = 2e-6;
  double vertical_inside_bias_max_delta_mps2 = 5e-6;
  double vertical_inside_bias_min_abs_residual_m = 0.04;
  double vertical_inside_bias_min_residual_delta_m = 0.005;
  double vertical_inside_bias_gate_fraction = 0.95;
  bool enable_vertical_rtk_seed_pass = false;
  bool enable_body_z_seed_jump_windows = false;
  bool body_z_seed_jump_use_fix_only = true;
  double body_z_jump_pre_post_window_s = 0.25;
  double body_z_jump_center_gap_s = 0.03;
  double body_z_jump_velocity_smooth_s = 0.20;
  double body_z_jump_threshold_ratio = 0.35;
  double body_z_jump_support_ratio = 0.25;
  double body_z_jump_redundant_padding_s = 0.30;
  double body_z_jump_min_score_mps = 0.008;
  double body_z_jump_min_separation_s = 0.50;
  double body_z_jump_max_window_duration_s = 1.50;
  int body_z_jump_max_levels = 12;
  double body_z_jump_dense_gap_s = 0.80;
  int body_z_jump_dense_peak_count = 20;
  double body_z_jump_dense_peak_floor_ratio = 4.0;
  bool enable_nhc_jump_reference = false;
  double nhc_history_half_life_s = 10.0;
  double nhc_history_max_age_s = 60.0;
  double nhc_body_vy_min_threshold_mps = 0.03;
  double nhc_body_vz_min_threshold_mps = 0.003;
  double nhc_body_vz_max_threshold_mps = 0.0;
  double nhc_body_vy_percentile_scale = 1.3;
  double nhc_body_vz_percentile_scale = 3.0;
  double nhc_jump_min_separation_s = 0.5;
  double nhc_jump_recovery_lookback_s = 5.0;
  bool reserve_vertical_velocity_feedback_interface = false;
  bool enable_reweighted_combined_imu_factor = false;
  double reweighted_combined_imu_attitude_sigma_rad = 1e-3;
  double reweighted_combined_imu_specific_force_sigma_mps2 = 0.0;
  double reweighted_combined_imu_specific_force_sigma_x_mps2 = 0.0;
  double reweighted_combined_imu_specific_force_sigma_y_mps2 = 0.0;
  double reweighted_combined_imu_specific_force_sigma_z_mps2 = 0.0;
  double reweighted_combined_imu_position_sigma_m = 0.0;
  double reweighted_combined_imu_velocity_sigma_mps = 0.0;
  bool reweighted_combined_imu_specific_force_sigma_x_specified = false;
  bool reweighted_combined_imu_specific_force_sigma_y_specified = false;
  bool reweighted_combined_imu_specific_force_sigma_z_specified = false;
  double error_process_noise_scale = 1.0;
  double tau_acc_bias_s = 100.0;
  double tau_gyro_bias_s = 36000.0;
  double bias_process_noise_acc_scale = 1.0;
  double bias_process_noise_gyro_scale = 1.0;
  double segment_feedback_attitude_gain = 1.0;
  double segment_feedback_velocity_gain = 0.05;
  double segment_feedback_position_gain = 0.01;
  double segment_feedback_acc_sigma_mps2 = 5e-5;
  double segment_feedback_gyro_sigma_radps = 5e-7;
  double error_state_rotation_sigma_rad = 1e-4;
  double error_state_position_sigma_m = 1.0;
  double error_state_velocity_sigma_mps = 0.05;
  double error_state_acc_bias_sigma_mps2 = 1e-3;
  double error_state_gyro_bias_sigma_radps = 1e-5;

  double stationary_window_s = 1.0;
  double stationary_acc_tolerance_mps2 = 0.8;
  double stationary_gyro_threshold_radps = 0.02;
  bool prefer_imu_initial_yaw = false;
  double static_alignment_duration_s = 0.0;
  double imu_dual_vector_window_s = 100.0;
  int imu_dual_vector_min_sample_count = 1000;
  double imu_dual_vector_min_cross_norm = 1e-3;
  bool enable_initial_static_zupt_zaru = false;
  double initial_static_zupt_velocity_sigma_mps = 0.05;
  double initial_static_zaru_sigma_radps = 1e-4;
  bool enable_initial_static_zero_specific_force = false;
  bool enable_initial_static_vertical_specific_force = false;
  double initial_static_specific_force_sigma_mps2 = 1e-2;
  double initial_static_vertical_specific_force_sigma_mps2 = 1e-2;
  bool enable_initial_static_subgraph = false;
  double initial_static_state_frequency_hz = 1.0;
  double initial_static_attitude_drift_sigma_rad = 1e-3;
  double yaw_min_distance_m = 1.0;
  double fallback_initial_yaw_rad = 0.0;

  double early_gnss_relaxation_duration_s = 5.0;
  double early_gnss_relaxation_scale = 3.0;

  double position_sigma_floor_m = 0.0;
  double position_sigma_floor_horizontal_m = 0.0;
  double position_sigma_floor_up_m = 0.0;
  double position_sigma_ceiling_m = 50.0;
  GnssVerticalSigmaMode gnss_vertical_sigma_mode = GnssVerticalSigmaMode::kFixed;
  double gnss_vertical_fixed_sigma_m = 0.05;
  double gnss_sigma_scale_horizontal = 1.0;
  double gnss_sigma_scale_up = 1.0;
  bool enable_gnss_vertical_drift_model = false;
  GnssVerticalDriftReferenceMode gnss_vertical_drift_reference_mode =
    GnssVerticalDriftReferenceMode::kMovingAverage;
  double gnss_vertical_drift_window_s = 10.0;
  GnssNoiseModel gnss_position_noise_model = GnssNoiseModel::kCauchy;
  double gnss_position_robust_param = 0.5;
  double rtkfix_scale = 1.0;
  double rtkfloat_scale = 2.0;
  double single_scale = 5.0;
  bool drop_non_rtkfix = false;
  int required_best_sol_status_code = 1;
  bool drop_no_solution = true;
  bool drop_nonfinite_sigma = true;
  GnssConsistencyGateMode gnss_consistency_gate_mode = GnssConsistencyGateMode::kNone;
  double gnss_nis_confidence = 0.95;
  double gnss_axis_sigma_multiple = 2.0;
  double gnss_consistency_relaxed_threshold_ratio = 0.25;
  double gnss_consistency_max_scale_horizontal = 4.0;
  double gnss_consistency_max_scale_up = 4.0;

  double initial_position_sigma_m = 3.0;
  double initial_roll_pitch_sigma_rad = 0.35;
  double initial_yaw_sigma_rad = 0.70;
  double initial_velocity_sigma_mps = 1.0;

  double lm_lambda_initial = 1e-3;
  int lm_max_iterations = 80;
};

OfflineRunnerConfig DefaultConfig();
OfflineRunnerConfig LoadConfigFile(std::string_view config_path, const OfflineRunnerConfig &base_config);
std::string ConfigToString(const OfflineRunnerConfig &config);
void OverrideConfigField(OfflineRunnerConfig &config, std::string_view key, std::string_view value);

}  // namespace offline_lc_minimal
