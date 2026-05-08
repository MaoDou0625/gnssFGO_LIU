#pragma once

#include <cstddef>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Units.h"

namespace offline_lc_minimal {

struct RunSummary {
  bool gnss_enabled = true;
  bool initial_static_constraints_enabled = false;
  bool initial_static_subgraph_enabled = false;
  std::size_t state_count = 0;
  std::size_t initial_static_state_count = 0;
  std::size_t initial_static_trajectory_count = 0;
  std::size_t gnss_factor_count = 0;
  std::size_t imu_rate_avp_count = 0;
  std::size_t imu_rate_interval_count = 0;
  std::size_t imu_rate_skipped_interval_count = 0;
  std::size_t gnss_synced_factor_count = 0;
  std::size_t gnss_interpolated_factor_count = 0;
  std::size_t gnss_dropped_count = 0;
  std::size_t gnss_cached_count = 0;
  std::size_t dropped_non_rtkfix_count = 0;
  std::size_t dropped_no_solution_count = 0;
  std::size_t dropped_nonfinite_sigma_count = 0;
  std::size_t dropped_bad_status_count = 0;
  std::size_t dropped_out_of_imu_coverage_count = 0;
  std::size_t initial_static_constraint_sample_count = 0;
  std::size_t initial_static_vertical_bias_prior_factor_count = 0;
  std::size_t initial_static_vertical_bias_gm_tightened_factor_count = 0;
  std::size_t initial_static_vertical_position_hold_factor_count = 0;
  std::size_t initial_static_rtk_height_reference_sample_count = 0;
  std::size_t initial_static_rtk_height_reference_factor_count = 0;
  std::size_t error_state_count = 0;
  std::size_t segment_error_count = 0;
  std::size_t vertical_velocity_delta_factor_count = 0;
  std::size_t vertical_velocity_delta_skipped_disabled_count = 0;
  std::size_t vertical_velocity_delta_skipped_static_count = 0;
  std::size_t vertical_velocity_delta_skipped_jump_count = 0;
  std::size_t vertical_velocity_delta_skipped_gnss_support_count = 0;
  std::size_t vertical_velocity_delta_skipped_invalid_count = 0;
  std::size_t vertical_velocity_delta_target_clamped_count = 0;
  bool vertical_velocity_delta_bias_consistent_sigma_enabled = false;
  bool vertical_velocity_delta_bias_aware_target_enabled = false;
  std::size_t vertical_velocity_delta_bias_aware_factor_count = 0;
  double vertical_velocity_delta_sigma_mean_mps = std::numeric_limits<double>::quiet_NaN();
  double vertical_velocity_delta_sigma_max_mps = std::numeric_limits<double>::quiet_NaN();
  std::size_t vertical_velocity_delta_sigma_clamped_floor_count = 0;
  std::size_t vertical_velocity_delta_sigma_clamped_ceiling_count = 0;
  std::size_t attitude_reference_factor_count = 0;
  std::size_t body_z_nhc_velocity_factor_count = 0;
  std::size_t body_z_nhc_displacement_factor_count = 0;
  std::size_t body_z_nhc_window_count = 0;
  std::size_t body_z_nhc_skipped_short_window_count = 0;
  std::size_t body_z_nhc_skipped_invalid_count = 0;
  std::size_t vertical_jump_combined_imu_factor_count = 0;
  std::size_t vertical_jump_masked_imu_factor_count = 0;
  std::size_t vertical_jump_impulse_factor_count = 0;
  std::size_t vertical_jump_impulse_prior_factor_count = 0;
  std::size_t vertical_jump_impulse_replaced_imu_factor_count = 0;
  std::size_t vertical_jump_impulse_skipped_count = 0;
  std::size_t vertical_jump_bias_velocity_factor_count = 0;
  std::size_t vertical_jump_bias_prior_factor_count = 0;
  std::size_t vertical_jump_bias_replaced_imu_factor_count = 0;
  std::size_t vertical_jump_bias_position_velocity_factor_count = 0;
  std::size_t vertical_jump_bias_skipped_count = 0;
  std::size_t vertical_jump_bias_segment_count = 0;
  std::size_t vertical_jump_bias_highfreq_inflated_factor_count = 0;
  std::size_t vertical_jump_velocity_ramp_factor_count = 0;
  std::size_t vertical_jump_position_ramp_factor_count = 0;
  std::size_t vertical_jump_velocity_height_slope_factor_count = 0;
  std::size_t vertical_jump_velocity_continuity_factor_count = 0;
  std::size_t vertical_jump_velocity_context_factor_count = 0;
  std::size_t vertical_jump_context_mean_continuity_factor_count = 0;
  std::size_t vertical_jump_position_velocity_consistency_factor_count = 0;
  std::size_t vertical_jump_continuity_skipped_count = 0;
  std::size_t vertical_jump_velocity_context_skipped_count = 0;
  std::size_t vertical_jump_context_mean_continuity_skipped_count = 0;
  std::size_t vertical_jump_velocity_ramp_skipped_count = 0;
  double initial_static_velocity_norm_mean_mps = 0.0;
  double initial_static_velocity_norm_std_mps = 0.0;
  double initial_static_velocity_norm_max_mps = 0.0;
  double static_specific_force_window_std_x_mps2 = 0.0;
  double static_specific_force_window_std_y_mps2 = 0.0;
  double static_specific_force_window_std_z_mps2 = 0.0;
  double static_specific_force_window_rms_xyz_mps2 = 0.0;
  double initial_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double initial_bgz_radps = std::numeric_limits<double>::quiet_NaN();
  double static_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double static_bgz_radps = std::numeric_limits<double>::quiet_NaN();
  double optimized_last_static_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double optimized_last_static_bgz_radps = std::numeric_limits<double>::quiet_NaN();
  double optimized_first_static_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double optimized_first_static_bgz_radps = std::numeric_limits<double>::quiet_NaN();
  double optimized_first_dynamic_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double optimized_first_dynamic_bgz_radps = std::numeric_limits<double>::quiet_NaN();
  double bootstrap_to_optimized_first_dynamic_baz_delta_mps2 =
    std::numeric_limits<double>::quiet_NaN();
  double static_to_dynamic_baz_delta_mps2 = std::numeric_limits<double>::quiet_NaN();
  double initial_static_horizontal_drift_max_m = 0.0;
  double initial_static_up_drift_max_m = 0.0;
  double initial_static_3d_drift_max_m = 0.0;
  double static_alignment_up_drift_m = std::numeric_limits<double>::quiet_NaN();
  double static_alignment_up_range_m = std::numeric_limits<double>::quiet_NaN();
  double static_alignment_vz_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double static_alignment_baz_range_ug = std::numeric_limits<double>::quiet_NaN();
  double static_alignment_baz_minus_global_max_abs_ug = std::numeric_limits<double>::quiet_NaN();
  double initial_static_rtk_height_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double static_alignment_rtk_reference_residual_mean_m = std::numeric_limits<double>::quiet_NaN();
  double static_alignment_rtk_reference_residual_max_abs_m = std::numeric_limits<double>::quiet_NaN();
  double gnss_nis_mean = std::numeric_limits<double>::quiet_NaN();
  double gnss_nis_median = std::numeric_limits<double>::quiet_NaN();
  double gnss_nis_p95 = std::numeric_limits<double>::quiet_NaN();
  double axis_2sigma_pass_rate = std::numeric_limits<double>::quiet_NaN();
  double feedback_forward_up_slope_10s = std::numeric_limits<double>::quiet_NaN();
  double feedback_forward_up_slope_30s = std::numeric_limits<double>::quiet_NaN();
  double feedback_forward_horizontal_slope_10s = std::numeric_limits<double>::quiet_NaN();
  double feedback_forward_horizontal_slope_30s = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_mean_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_mean_bgz_radps = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_mean_roll_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_mean_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_mean_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_std_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_std_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_std_roll_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_up_total_variation_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_first30s_vz_total_variation_mps = std::numeric_limits<double>::quiet_NaN();
  double forward_first30s_up_total_variation_m = std::numeric_limits<double>::quiet_NaN();
  double forward_first30s_vz_total_variation_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_static_terminal_forward20s_up_total_variation_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_static_terminal_forward20s_vz_total_variation_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_error = 0.0;
  double final_error = 0.0;
  double origin_lat_rad = 0.0;
  double origin_lon_rad = 0.0;
  double origin_h_m = 0.0;
  double alignment_start_time_s = 0.0;
  double navigation_start_time_s = 0.0;
  double dynamic_start_time_s = 0.0;
  double static_alignment_duration_s = 0.0;
  std::string yaw_source = "fallback";

  [[nodiscard]] std::string ToMultilineString() const {
    std::ostringstream oss;
    oss << "gnss_enabled=" << (gnss_enabled ? "true" : "false") << '\n'
        << "initial_static_constraints_enabled=" << (initial_static_constraints_enabled ? "true" : "false") << '\n'
        << "initial_static_subgraph_enabled=" << (initial_static_subgraph_enabled ? "true" : "false") << '\n'
        << "state_count=" << state_count << '\n'
        << "initial_static_state_count=" << initial_static_state_count << '\n'
        << "initial_static_trajectory_count=" << initial_static_trajectory_count << '\n'
        << "gnss_factor_count=" << gnss_factor_count << '\n'
        << "imu_rate_avp_count=" << imu_rate_avp_count << '\n'
        << "imu_rate_interval_count=" << imu_rate_interval_count << '\n'
        << "imu_rate_skipped_interval_count=" << imu_rate_skipped_interval_count << '\n'
        << "gnss_synced_factor_count=" << gnss_synced_factor_count << '\n'
        << "gnss_interpolated_factor_count=" << gnss_interpolated_factor_count << '\n'
        << "gnss_dropped_count=" << gnss_dropped_count << '\n'
        << "gnss_cached_count=" << gnss_cached_count << '\n'
        << "dropped_non_rtkfix_count=" << dropped_non_rtkfix_count << '\n'
        << "dropped_no_solution_count=" << dropped_no_solution_count << '\n'
        << "dropped_nonfinite_sigma_count=" << dropped_nonfinite_sigma_count << '\n'
        << "dropped_bad_status_count=" << dropped_bad_status_count << '\n'
        << "dropped_out_of_imu_coverage_count=" << dropped_out_of_imu_coverage_count << '\n'
        << "initial_static_constraint_sample_count=" << initial_static_constraint_sample_count << '\n'
        << "initial_static_vertical_bias_prior_factor_count="
        << initial_static_vertical_bias_prior_factor_count << '\n'
        << "initial_static_vertical_bias_gm_tightened_factor_count="
        << initial_static_vertical_bias_gm_tightened_factor_count << '\n'
        << "initial_static_vertical_position_hold_factor_count="
        << initial_static_vertical_position_hold_factor_count << '\n'
        << "initial_static_rtk_height_reference_sample_count="
        << initial_static_rtk_height_reference_sample_count << '\n'
        << "initial_static_rtk_height_reference_factor_count="
        << initial_static_rtk_height_reference_factor_count << '\n'
        << "error_state_count=" << error_state_count << '\n'
        << "segment_error_count=" << segment_error_count << '\n'
        << "vertical_velocity_delta_factor_count=" << vertical_velocity_delta_factor_count << '\n'
        << "vertical_velocity_delta_skipped_disabled_count=" << vertical_velocity_delta_skipped_disabled_count << '\n'
        << "vertical_velocity_delta_skipped_static_count=" << vertical_velocity_delta_skipped_static_count << '\n'
        << "vertical_velocity_delta_skipped_jump_count=" << vertical_velocity_delta_skipped_jump_count << '\n'
        << "vertical_velocity_delta_skipped_gnss_support_count="
        << vertical_velocity_delta_skipped_gnss_support_count << '\n'
        << "vertical_velocity_delta_skipped_invalid_count=" << vertical_velocity_delta_skipped_invalid_count << '\n'
        << "vertical_velocity_delta_target_clamped_count="
        << vertical_velocity_delta_target_clamped_count << '\n'
        << "vertical_velocity_delta_bias_consistent_sigma_enabled="
        << (vertical_velocity_delta_bias_consistent_sigma_enabled ? "true" : "false") << '\n'
        << "vertical_velocity_delta_bias_aware_target_enabled="
        << (vertical_velocity_delta_bias_aware_target_enabled ? "true" : "false") << '\n'
        << "vertical_velocity_delta_bias_aware_factor_count="
        << vertical_velocity_delta_bias_aware_factor_count << '\n'
        << "vertical_velocity_delta_sigma_mean_mps=" << vertical_velocity_delta_sigma_mean_mps << '\n'
        << "vertical_velocity_delta_sigma_max_mps=" << vertical_velocity_delta_sigma_max_mps << '\n'
        << "vertical_velocity_delta_sigma_clamped_floor_count="
        << vertical_velocity_delta_sigma_clamped_floor_count << '\n'
        << "vertical_velocity_delta_sigma_clamped_ceiling_count="
        << vertical_velocity_delta_sigma_clamped_ceiling_count << '\n'
        << "attitude_reference_factor_count=" << attitude_reference_factor_count << '\n'
        << "body_z_nhc_velocity_factor_count=" << body_z_nhc_velocity_factor_count << '\n'
        << "body_z_nhc_displacement_factor_count=" << body_z_nhc_displacement_factor_count << '\n'
        << "body_z_nhc_window_count=" << body_z_nhc_window_count << '\n'
        << "body_z_nhc_skipped_short_window_count=" << body_z_nhc_skipped_short_window_count << '\n'
        << "body_z_nhc_skipped_invalid_count=" << body_z_nhc_skipped_invalid_count << '\n'
        << "vertical_jump_combined_imu_factor_count=" << vertical_jump_combined_imu_factor_count << '\n'
        << "vertical_jump_masked_imu_factor_count=" << vertical_jump_masked_imu_factor_count << '\n'
        << "vertical_jump_impulse_factor_count=" << vertical_jump_impulse_factor_count << '\n'
        << "vertical_jump_impulse_prior_factor_count=" << vertical_jump_impulse_prior_factor_count << '\n'
        << "vertical_jump_impulse_replaced_imu_factor_count="
        << vertical_jump_impulse_replaced_imu_factor_count << '\n'
        << "vertical_jump_impulse_skipped_count=" << vertical_jump_impulse_skipped_count << '\n'
        << "vertical_jump_bias_velocity_factor_count=" << vertical_jump_bias_velocity_factor_count << '\n'
        << "vertical_jump_bias_prior_factor_count=" << vertical_jump_bias_prior_factor_count << '\n'
        << "vertical_jump_bias_replaced_imu_factor_count="
        << vertical_jump_bias_replaced_imu_factor_count << '\n'
        << "vertical_jump_bias_position_velocity_factor_count="
        << vertical_jump_bias_position_velocity_factor_count << '\n'
        << "vertical_jump_bias_skipped_count=" << vertical_jump_bias_skipped_count << '\n'
        << "vertical_jump_bias_segment_count=" << vertical_jump_bias_segment_count << '\n'
        << "vertical_jump_bias_highfreq_inflated_factor_count="
        << vertical_jump_bias_highfreq_inflated_factor_count << '\n'
        << "vertical_jump_velocity_ramp_factor_count=" << vertical_jump_velocity_ramp_factor_count << '\n'
        << "vertical_jump_position_ramp_factor_count=" << vertical_jump_position_ramp_factor_count << '\n'
        << "vertical_jump_velocity_height_slope_factor_count="
        << vertical_jump_velocity_height_slope_factor_count << '\n'
        << "vertical_jump_velocity_continuity_factor_count="
        << vertical_jump_velocity_continuity_factor_count << '\n'
        << "vertical_jump_velocity_context_factor_count="
        << vertical_jump_velocity_context_factor_count << '\n'
        << "vertical_jump_context_mean_continuity_factor_count="
        << vertical_jump_context_mean_continuity_factor_count << '\n'
        << "vertical_jump_position_velocity_consistency_factor_count="
        << vertical_jump_position_velocity_consistency_factor_count << '\n'
        << "vertical_jump_continuity_skipped_count=" << vertical_jump_continuity_skipped_count << '\n'
        << "vertical_jump_velocity_context_skipped_count="
        << vertical_jump_velocity_context_skipped_count << '\n'
        << "vertical_jump_context_mean_continuity_skipped_count="
        << vertical_jump_context_mean_continuity_skipped_count << '\n'
        << "vertical_jump_velocity_ramp_skipped_count=" << vertical_jump_velocity_ramp_skipped_count << '\n'
        << "initial_static_velocity_norm_mean_mps=" << initial_static_velocity_norm_mean_mps << '\n'
        << "initial_static_velocity_norm_std_mps=" << initial_static_velocity_norm_std_mps << '\n'
        << "initial_static_velocity_norm_max_mps=" << initial_static_velocity_norm_max_mps << '\n'
        << "static_specific_force_window_std_x_mps2=" << static_specific_force_window_std_x_mps2 << '\n'
        << "static_specific_force_window_std_y_mps2=" << static_specific_force_window_std_y_mps2 << '\n'
        << "static_specific_force_window_std_z_mps2=" << static_specific_force_window_std_z_mps2 << '\n'
        << "static_specific_force_window_rms_xyz_mps2=" << static_specific_force_window_rms_xyz_mps2 << '\n'
        << "initial_baz_ug=" << Mps2ToMicroG(initial_baz_mps2) << '\n'
        << "initial_bgz_radps=" << initial_bgz_radps << '\n'
        << "static_baz_ug=" << Mps2ToMicroG(static_baz_mps2) << '\n'
        << "static_bgz_radps=" << static_bgz_radps << '\n'
        << "optimized_last_static_baz_ug=" << Mps2ToMicroG(optimized_last_static_baz_mps2) << '\n'
        << "optimized_last_static_bgz_radps=" << optimized_last_static_bgz_radps << '\n'
        << "optimized_first_static_baz_ug=" << Mps2ToMicroG(optimized_first_static_baz_mps2) << '\n'
        << "optimized_first_static_bgz_radps=" << optimized_first_static_bgz_radps << '\n'
        << "optimized_first_dynamic_baz_ug=" << Mps2ToMicroG(optimized_first_dynamic_baz_mps2) << '\n'
        << "optimized_first_dynamic_bgz_radps=" << optimized_first_dynamic_bgz_radps << '\n'
        << "bootstrap_to_optimized_first_dynamic_baz_delta_ug="
        << Mps2ToMicroG(bootstrap_to_optimized_first_dynamic_baz_delta_mps2) << '\n'
        << "static_to_dynamic_baz_delta_ug=" << Mps2ToMicroG(static_to_dynamic_baz_delta_mps2) << '\n'
        << "initial_static_horizontal_drift_max_m=" << initial_static_horizontal_drift_max_m << '\n'
        << "initial_static_up_drift_max_m=" << initial_static_up_drift_max_m << '\n'
        << "initial_static_3d_drift_max_m=" << initial_static_3d_drift_max_m << '\n'
        << "static_alignment_up_drift_m=" << static_alignment_up_drift_m << '\n'
        << "static_alignment_up_range_m=" << static_alignment_up_range_m << '\n'
        << "static_alignment_vz_max_abs_mps=" << static_alignment_vz_max_abs_mps << '\n'
        << "static_alignment_baz_range_ug=" << static_alignment_baz_range_ug << '\n'
        << "static_alignment_baz_minus_global_max_abs_ug="
        << static_alignment_baz_minus_global_max_abs_ug << '\n'
        << "initial_static_rtk_height_reference_up_m="
        << initial_static_rtk_height_reference_up_m << '\n'
        << "static_alignment_rtk_reference_residual_mean_m="
        << static_alignment_rtk_reference_residual_mean_m << '\n'
        << "static_alignment_rtk_reference_residual_max_abs_m="
        << static_alignment_rtk_reference_residual_max_abs_m << '\n'
        << "gnss_nis_mean=" << gnss_nis_mean << '\n'
        << "gnss_nis_median=" << gnss_nis_median << '\n'
        << "gnss_nis_p95=" << gnss_nis_p95 << '\n'
        << "axis_2sigma_pass_rate=" << axis_2sigma_pass_rate << '\n'
        << "feedback_forward_up_slope_10s=" << feedback_forward_up_slope_10s << '\n'
        << "feedback_forward_up_slope_30s=" << feedback_forward_up_slope_30s << '\n'
        << "feedback_forward_horizontal_slope_10s=" << feedback_forward_horizontal_slope_10s << '\n'
        << "feedback_forward_horizontal_slope_30s=" << feedback_forward_horizontal_slope_30s << '\n'
        << "optimized_first30s_mean_baz_ug=" << Mps2ToMicroG(optimized_first30s_mean_baz_mps2) << '\n'
        << "optimized_first30s_mean_bgz_radps=" << optimized_first30s_mean_bgz_radps << '\n'
        << "optimized_first30s_mean_roll_rad=" << optimized_first30s_mean_roll_rad << '\n'
        << "optimized_first30s_mean_pitch_rad=" << optimized_first30s_mean_pitch_rad << '\n'
        << "optimized_first30s_mean_yaw_rad=" << optimized_first30s_mean_yaw_rad << '\n'
        << "optimized_first30s_std_baz_ug=" << Mps2ToMicroG(optimized_first30s_std_baz_mps2) << '\n'
        << "optimized_first30s_std_pitch_rad=" << optimized_first30s_std_pitch_rad << '\n'
        << "optimized_first30s_std_roll_rad=" << optimized_first30s_std_roll_rad << '\n'
        << "optimized_first30s_up_total_variation_m=" << optimized_first30s_up_total_variation_m << '\n'
        << "optimized_first30s_vz_total_variation_mps=" << optimized_first30s_vz_total_variation_mps << '\n'
        << "forward_first30s_up_total_variation_m=" << forward_first30s_up_total_variation_m << '\n'
        << "forward_first30s_vz_total_variation_mps=" << forward_first30s_vz_total_variation_mps << '\n'
        << "optimized_static_terminal_forward20s_up_total_variation_m="
        << optimized_static_terminal_forward20s_up_total_variation_m << '\n'
        << "optimized_static_terminal_forward20s_vz_total_variation_mps="
        << optimized_static_terminal_forward20s_vz_total_variation_mps << '\n'
        << "initial_error=" << initial_error << '\n'
        << "final_error=" << final_error << '\n'
        << "origin_lat_rad=" << origin_lat_rad << '\n'
        << "origin_lon_rad=" << origin_lon_rad << '\n'
        << "origin_h_m=" << origin_h_m << '\n'
        << "alignment_start_time_s=" << alignment_start_time_s << '\n'
        << "navigation_start_time_s=" << navigation_start_time_s << '\n'
        << "dynamic_start_time_s=" << dynamic_start_time_s << '\n'
        << "static_alignment_duration_s=" << static_alignment_duration_s << '\n'
        << "yaw_source=" << yaw_source << '\n';
    return oss.str();
  }
};

struct OfflineRunResult {
  DataSummary data_summary;
  RunSummary run_summary;
  std::vector<TrajectoryRow> initial_static_trajectory;
  std::vector<TrajectoryRow> optimized_static_terminal_forward_trajectory;
  std::vector<ReferenceNodeRow> reference_node_trajectory;
  std::vector<ReferenceNodeState> attitude_reference_states;
  std::vector<BodyZSeedImuDiagnosticRow> seed_body_z_acc_diagnostics;
  std::vector<BodyZSeedJumpWindowRow> body_z_seed_jump_windows;
  std::vector<ErrorStateRow> error_state_trajectory;
  std::vector<SegmentErrorDiagnostic> segment_error_diagnostics;
  std::vector<TrajectoryRow> trajectory;
  std::vector<ImuRateAvpRow> imu_rate_avp;
  std::vector<ImuRateIntervalDiagnostic> imu_rate_interval_diagnostics;
  std::vector<GnssFactorRecord> gnss_factor_records;
  std::vector<GnssConsistencyRecord> gnss_consistency_records;
  std::vector<VerticalEnvelopeDiagnosticRow> vertical_envelope_diagnostics;
  std::vector<StaticAlignmentValidationRow> static_alignment_validation;
  std::vector<VerticalVelocityDeltaDiagnosticRow> vertical_velocity_delta_diagnostics;
  std::vector<AttitudeReferenceDiagnosticRow> attitude_reference_diagnostics;
  std::vector<BodyZNHCDiagnosticRow> body_z_nhc_diagnostics;
  std::vector<VerticalJumpMaskedImuDiagnosticRow> vertical_jump_masked_imu_diagnostics;
  std::vector<VerticalJumpImpulseDiagnosticRow> vertical_jump_impulse_diagnostics;
  std::vector<VerticalJumpBiasDiagnosticRow> vertical_jump_bias_diagnostics;
  std::vector<VerticalJumpVelocityRampDiagnosticRow> vertical_jump_velocity_ramp_diagnostics;
  std::vector<VerticalJumpContinuityDiagnosticRow> vertical_jump_continuity_diagnostics;
  std::vector<VerticalStateCorrectionRow> vertical_state_corrections;
};

class OfflineRunFailure : public std::runtime_error {
 public:
  OfflineRunFailure(std::string message, OfflineRunResult partial_result)
      : std::runtime_error(std::move(message)),
        partial_result_(std::move(partial_result)) {}

  [[nodiscard]] const OfflineRunResult &partial_result() const { return partial_result_; }

 private:
  OfflineRunResult partial_result_;
};

}  // namespace offline_lc_minimal
