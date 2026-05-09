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
  double vertical_acc_bias_sigma_mps2 = 0.0;
  double vertical_acc_bias_process_noise_scale = 1.0;

  bool enable_body_z_jump_detection = false;
  bool body_z_seed_jump_use_fix_only = true;
  double body_z_jump_pre_post_window_s = 0.25;
  double body_z_jump_center_gap_s = 0.03;
  double body_z_jump_velocity_smooth_s = 0.20;
  double body_z_jump_threshold_ratio = 0.35;
  double body_z_jump_support_ratio = 0.25;
  double body_z_jump_redundant_padding_s = 0.30;
  double body_z_jump_merge_gap_s = 0.0;
  double body_z_jump_merge_max_duration_s = 0.0;
  double body_z_jump_min_score_mps = 0.008;
  double body_z_jump_min_separation_s = 0.50;
  double body_z_jump_max_window_duration_s = 1.50;
  int body_z_jump_max_levels = 12;
  double body_z_jump_dense_gap_s = 0.80;
  int body_z_jump_dense_peak_count = 20;
  double body_z_jump_dense_peak_floor_ratio = 4.0;

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
  bool enable_initial_static_vertical_bias_soft_prior = false;
  double initial_static_vertical_bias_global_tie_sigma_mps2 = 5e-5;
  bool enable_initial_static_vertical_bias_gm_tightening = false;
  double initial_static_vertical_bias_gm_sigma_mps2 = 1.96133e-7;
  bool enable_initial_static_vertical_position_hold = false;
  double initial_static_vertical_position_hold_sigma_m = 0.005;
  bool enable_initial_static_rtk_height_reference = false;
  double initial_static_rtk_height_reference_sigma_m = 0.02;
  int initial_static_rtk_height_reference_min_sample_count = 5;
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
  VerticalConstraintMode vertical_constraint_mode = VerticalConstraintMode::kDirectZ;
  double vertical_envelope_gate_sigma_multiple = 2.0;
  double vertical_envelope_min_half_width_m = 0.10;
  double vertical_envelope_factor_sigma_m = 0.20;
  bool enable_vertical_envelope_center_pull = false;
  double vertical_envelope_center_sigma_m = 0.60;
  VerticalEnvelopeCenterSigmaMode vertical_envelope_center_sigma_mode = VerticalEnvelopeCenterSigmaMode::kFixed;
  double vertical_envelope_center_deadband_m = 0.01;
  bool enable_vertical_velocity_delta_constraint = false;
  double vertical_velocity_delta_acc_sigma_mps2 = 0.50;
  double vertical_velocity_delta_min_sigma_mps = 0.02;
  double vertical_velocity_delta_jump_padding_s = 0.25;
  double vertical_velocity_delta_target_acc_limit_mps2 = 0.85;
  bool enable_vertical_velocity_delta_initial_static_constraint = false;
  bool enable_vertical_velocity_delta_bias_consistent_sigma = false;
  bool enable_vertical_velocity_delta_bias_aware_target = false;
  double vertical_velocity_delta_bias_sigma_mps2 = 9.80665e-5;
  double vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  double vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  double vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  bool enable_attitude_reference_constraint = false;
  double attitude_reference_sigma_rad = 0.01;
  bool enable_body_z_nhc_constraint = false;
  bool enable_body_z_nhc_global_weak_constraint = false;
  double body_z_nhc_jump_padding_s = 0.30;
  double body_z_nhc_merge_gap_s = 0.20;
  double body_z_nhc_min_window_s = 0.50;
  double body_z_nhc_jump_velocity_sigma_mps = 0.02;
  double body_z_nhc_jump_displacement_sigma_m = 0.02;
  double body_z_nhc_global_window_s = 3.0;
  double body_z_nhc_global_stride_s = 1.0;
  double body_z_nhc_global_velocity_sigma_mps = 0.05;
  double body_z_nhc_global_displacement_sigma_m = 0.05;
  bool enable_vertical_jump_masked_imu = false;
  double vertical_jump_masked_imu_padding_s = 0.25;
  bool enable_vertical_jump_impulse = false;
  double vertical_jump_impulse_prior_sigma_mps = 0.30;
  double vertical_jump_impulse_velocity_sigma_mps = 0.03;
  double vertical_jump_impulse_position_velocity_sigma_m = 0.02;
  bool enable_vertical_jump_bias = false;
  double vertical_jump_bias_padding_s = 0.0;
  double vertical_jump_bias_prior_sigma_mps2 = 0.05;
  double vertical_jump_bias_velocity_sigma_mps = 0.01;
  double vertical_jump_bias_position_velocity_sigma_m = 0.02;
  bool enable_vertical_jump_segmented_bias = false;
  double vertical_jump_segmented_bias_min_segment_s = 0.30;
  int vertical_jump_segmented_bias_max_segments = 5;
  double vertical_jump_segmented_bias_slope_merge_threshold_mps2 = 0.015;
  double vertical_jump_bias_highfreq_sigma_scale = 0.02;
  double vertical_jump_bias_highfreq_sigma_max_mps = 0.08;
  bool enable_vertical_jump_velocity_ramp_smoothing = false;
  double vertical_jump_velocity_ramp_sigma_mps = 0.08;
  bool enable_vertical_jump_position_ramp_smoothing = false;
  double vertical_jump_position_ramp_sigma_m = 0.10;
  bool enable_vertical_jump_velocity_continuity = false;
  double vertical_jump_velocity_continuity_sigma_mps = 0.02;
  bool enable_vertical_jump_velocity_context_mean = false;
  double vertical_jump_velocity_context_window_s = 1.0;
  double vertical_jump_velocity_context_mean_sigma_mps = 0.03;
  bool enable_vertical_jump_context_mean_continuity = false;
  double vertical_jump_context_mean_continuity_sigma_mps = 0.02;
  bool enable_vertical_jump_position_velocity_consistency = false;
  double vertical_jump_position_velocity_consistency_sigma_m = 0.08;
  double vertical_jump_boundary_position_velocity_consistency_sigma_m = 0.01;
  bool enable_vertical_jump_velocity_height_slope_constraint = false;
  double vertical_jump_velocity_height_slope_sigma_mps = 0.50;
  double gnss_sigma_scale_horizontal = 1.0;
  double gnss_sigma_scale_up = 1.0;
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
void ValidateConfig(const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
