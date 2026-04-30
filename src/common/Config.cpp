#include "offline_lc_minimal/common/Config.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

namespace offline_lc_minimal {

namespace {

constexpr double kNumericalSigmaFloorM = 1e-4;

std::string Trim(std::string value) {
  auto not_space = [](const unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string StripQuotes(std::string value) {
  value = Trim(std::move(value));
  if (value.size() >= 2 && ((value.front() == '"' && value.back() == '"') ||
                            (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1, value.size() - 2);
  }
  return value;
}

bool ParseBool(const std::string &value) {
  const auto lowered = [&]() {
    std::string result = value;
    std::transform(result.begin(), result.end(), result.begin(), [](const unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
    return result;
  }();

  if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
    return true;
  }
  if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
    return false;
  }
  throw std::runtime_error("invalid boolean value: " + value);
}

double ParseDouble(const std::string &value) {
  return std::stod(value);
}

int ParseInt(const std::string &value) {
  return std::stoi(value);
}

GnssNoiseModel ParseNoiseModel(const std::string &value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (lowered == "gaussian") {
    return GnssNoiseModel::kGaussian;
  }
  if (lowered == "cauchy") {
    return GnssNoiseModel::kCauchy;
  }
  if (lowered == "huber") {
    return GnssNoiseModel::kHuber;
  }
  if (lowered == "dcs") {
    return GnssNoiseModel::kDcs;
  }
  if (lowered == "tukey") {
    return GnssNoiseModel::kTukey;
  }
  if (lowered == "geman_mcclure") {
    return GnssNoiseModel::kGemanMcClure;
  }
  if (lowered == "welsch") {
    return GnssNoiseModel::kWelsch;
  }
  throw std::runtime_error("invalid GNSS noise model: " + value);
}

GnssConsistencyGateMode ParseConsistencyGateMode(const std::string &value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (lowered == "none") {
    return GnssConsistencyGateMode::kNone;
  }
  if (lowered == "nis") {
    return GnssConsistencyGateMode::kNis;
  }
  throw std::runtime_error("invalid GNSS consistency gate mode: " + value);
}

GnssVerticalSigmaMode ParseVerticalSigmaMode(const std::string &value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (lowered == "from_file") {
    return GnssVerticalSigmaMode::kFromFile;
  }
  if (lowered == "fixed") {
    return GnssVerticalSigmaMode::kFixed;
  }
  throw std::runtime_error("invalid GNSS vertical sigma mode: " + value);
}

GnssVerticalDriftReferenceMode ParseVerticalDriftReferenceMode(const std::string &value) {
  std::string lowered = value;
  std::transform(lowered.begin(), lowered.end(), lowered.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  if (lowered == "moving_average") {
    return GnssVerticalDriftReferenceMode::kMovingAverage;
  }
  throw std::runtime_error("invalid GNSS vertical drift reference mode: " + value);
}

void ResolveReweightedSpecificForceSigmaAxes(OfflineRunnerConfig &config) {
  const bool any_axis_specified =
    config.reweighted_combined_imu_specific_force_sigma_x_specified ||
    config.reweighted_combined_imu_specific_force_sigma_y_specified ||
    config.reweighted_combined_imu_specific_force_sigma_z_specified;
  if (!any_axis_specified) {
    config.reweighted_combined_imu_specific_force_sigma_x_mps2 =
      config.reweighted_combined_imu_specific_force_sigma_mps2;
    config.reweighted_combined_imu_specific_force_sigma_y_mps2 =
      config.reweighted_combined_imu_specific_force_sigma_mps2;
    config.reweighted_combined_imu_specific_force_sigma_z_mps2 =
      config.reweighted_combined_imu_specific_force_sigma_mps2;
  }
}

}  // namespace

OfflineRunnerConfig DefaultConfig() {
  return OfflineRunnerConfig{};
}

void OverrideConfigField(OfflineRunnerConfig &config, const std::string_view key, const std::string_view value) {
  const std::string normalized_key = Trim(std::string(key));
  const std::string normalized_value = StripQuotes(std::string(value));

  if (normalized_key == "imu_path") {
    config.imu_path = normalized_value;
  } else if (normalized_key == "gnss_path") {
    config.gnss_path = normalized_value;
  } else if (normalized_key == "output_dir") {
    config.output_dir = normalized_value;
  } else if (normalized_key == "enable_gnss") {
    config.enable_gnss = ParseBool(normalized_value);
  } else if (normalized_key == "enable_gp_interpolated_gnss") {
    config.enable_gp_interpolated_gnss = ParseBool(normalized_value);
  } else if (normalized_key == "enable_error_state_feedback") {
    config.enable_error_state_feedback = ParseBool(normalized_value);
    config.enable_segment_error_feedback = config.enable_error_state_feedback;
  } else if (normalized_key == "enable_segment_error_feedback") {
    config.enable_segment_error_feedback = ParseBool(normalized_value);
  } else if (normalized_key == "enable_segment_local_error_feedback") {
    config.enable_segment_local_error_feedback = ParseBool(normalized_value);
  } else if (normalized_key == "verbose") {
    config.verbose = ParseBool(normalized_value);
  } else if (normalized_key == "write_debug_csv") {
    config.write_debug_csv = ParseBool(normalized_value);
  } else if (normalized_key == "write_error_diagnostics") {
    config.write_error_diagnostics = ParseBool(normalized_value);
  } else if (normalized_key == "write_segment_error_diagnostics") {
    config.write_segment_error_diagnostics = ParseBool(normalized_value);
  } else if (normalized_key == "write_imu_rate_avp") {
    config.write_imu_rate_avp = ParseBool(normalized_value);
  } else if (normalized_key == "state_frequency_hz") {
    config.state_frequency_hz = ParseDouble(normalized_value);
  } else if (normalized_key == "error_state_frequency_hz") {
    config.error_state_frequency_hz = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_time_offset_s") {
    config.gnss_time_offset_s = ParseDouble(normalized_value);
  } else if (normalized_key == "state_meas_sync_lower_bound_s") {
    config.state_meas_sync_lower_bound_s = ParseDouble(normalized_value);
  } else if (normalized_key == "state_meas_sync_upper_bound_s") {
    config.state_meas_sync_upper_bound_s = ParseDouble(normalized_value);
  } else if (normalized_key == "gravity_mps2") {
    config.gravity_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "imu_sigma_acc") {
    config.imu_sigma_acc = ParseDouble(normalized_value);
  } else if (normalized_key == "imu_sigma_gyro") {
    config.imu_sigma_gyro = ParseDouble(normalized_value);
  } else if (normalized_key == "integration_sigma") {
    config.integration_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_acc_sigma") {
    config.bias_acc_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_gyro_sigma") {
    config.bias_gyro_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_acc_prior_sigma") {
    config.bias_acc_prior_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_gyro_prior_sigma") {
    config.bias_gyro_prior_sigma = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_global_acc_bias") {
    config.enable_global_acc_bias = ParseBool(normalized_value);
  } else if (normalized_key == "global_acc_bias_tie_sigma_mps2") {
    config.global_acc_bias_tie_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "global_acc_bias_tie_sigma_xy_mps2") {
    config.global_acc_bias_tie_sigma_xy_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_global_gyro_bias") {
    config.enable_global_gyro_bias = ParseBool(normalized_value);
  } else if (normalized_key == "global_gyro_bias_tie_sigma_radps") {
    config.global_gyro_bias_tie_sigma_radps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_acc_bias_gm_process") {
    config.enable_vertical_acc_bias_gm_process = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_acc_bias_tau_s") {
    config.vertical_acc_bias_tau_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_acc_bias_sigma_mps2") {
    config.vertical_acc_bias_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_acc_bias_process_noise_scale") {
    config.vertical_acc_bias_process_noise_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_rtk_preintegration_feedback") {
    config.enable_vertical_rtk_preintegration_feedback = ParseBool(normalized_value);
  } else if (normalized_key == "enable_vertical_rtk_global_position_factor") {
    config.enable_vertical_rtk_global_position_factor = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_rtk_gate_sigma_multiple") {
    config.vertical_rtk_gate_sigma_multiple = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_inside_gate_sigma_scale") {
    config.vertical_rtk_inside_gate_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_outside_gate_sigma_scale") {
    config.vertical_rtk_outside_gate_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_inside_feedback_gain_scale") {
    config.vertical_rtk_inside_feedback_gain_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_outside_feedback_gain_scale") {
    config.vertical_rtk_outside_feedback_gain_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_feedback_bias_gain") {
    config.vertical_rtk_feedback_bias_gain = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_feedback_attitude_gain") {
    config.vertical_rtk_feedback_attitude_gain = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_feedback_sigma_dp_m") {
    config.vertical_rtk_feedback_sigma_dp_m = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_feedback_sigma_baz_mps2") {
    config.vertical_rtk_feedback_sigma_baz_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_feedback_sigma_attitude_rad") {
    config.vertical_rtk_feedback_sigma_attitude_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_feedback_sigma_vz_mps") {
    config.vertical_rtk_feedback_sigma_vz_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_feedback_min_interval_s") {
    config.vertical_rtk_feedback_min_interval_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_rtk_jump_inside_sigma_scale") {
    config.vertical_rtk_jump_inside_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_local_recovery_max_iterations") {
    config.vertical_local_recovery_max_iterations = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_local_recovery_max_attitude_delta_rad") {
    config.vertical_local_recovery_max_attitude_delta_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_local_recovery_max_baz_delta_mps2") {
    config.vertical_local_recovery_max_baz_delta_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_global_vz_window_s") {
    config.vertical_global_vz_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_global_vz_smooth_window_s") {
    config.vertical_global_vz_smooth_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_candidate_min_separation_s") {
    config.vertical_jump_candidate_min_separation_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_max_candidates_per_segment") {
    config.vertical_jump_max_candidates_per_segment = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_max_selected_points_per_segment") {
    config.vertical_jump_max_selected_points_per_segment = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_hold_window_s") {
    config.vertical_jump_hold_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_step_min_threshold_mps") {
    config.vertical_jump_step_min_threshold_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_vz_prior_sigma_mps") {
    config.vertical_jump_vz_prior_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_window_default_padding_states") {
    config.vertical_jump_window_default_padding_states = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_window_support_ratio") {
    config.vertical_jump_window_support_ratio = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_window_max_duration_s") {
    config.vertical_jump_window_max_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_window_max_points") {
    config.vertical_jump_window_max_points = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_window_tail_target_s") {
    config.vertical_jump_window_tail_target_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_window_velocity_smoothness_weight") {
    config.vertical_jump_window_velocity_smoothness_weight = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_window_height_integral_weight") {
    config.vertical_jump_window_height_integral_weight = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_window_ref_weight") {
    config.vertical_jump_window_ref_weight = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_window_max_correction_attempts") {
    config.vertical_jump_window_max_correction_attempts = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_future_trend_window_s") {
    config.vertical_jump_future_trend_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_future_trend_min_fix_count") {
    config.vertical_jump_future_trend_min_fix_count = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_future_trend_mean_weight") {
    config.vertical_jump_future_trend_mean_weight = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_future_trend_slope_weight") {
    config.vertical_jump_future_trend_slope_weight = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_min_duration_s") {
    config.vertical_interval_feedback_min_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_min_slope_mps") {
    config.vertical_interval_feedback_min_slope_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_min_drift_m") {
    config.vertical_interval_feedback_min_drift_m = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_min_residual_m") {
    config.vertical_interval_feedback_min_residual_m = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_snr_threshold") {
    config.vertical_interval_feedback_snr_threshold = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_noise_floor_m") {
    config.vertical_interval_feedback_noise_floor_m = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_gain") {
    config.vertical_interval_feedback_gain = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_interval_feedback_max_delta_vz_mps") {
    config.vertical_interval_feedback_max_delta_vz_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_local_up_anchor_fallback") {
    config.enable_vertical_local_up_anchor_fallback = ParseBool(normalized_value);
  } else if (normalized_key == "enable_vertical_inside_bias_adaptation") {
    config.enable_vertical_inside_bias_adaptation = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_inside_attitude_gain") {
    config.vertical_inside_attitude_gain = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_inside_max_delta_attitude_rad") {
    config.vertical_inside_max_delta_attitude_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_rtk_seed_pass") {
    config.enable_vertical_rtk_seed_pass = ParseBool(normalized_value);
  } else if (normalized_key == "enable_body_z_seed_jump_windows") {
    config.enable_body_z_seed_jump_windows = ParseBool(normalized_value);
  } else if (normalized_key == "body_z_seed_jump_use_fix_only") {
    config.body_z_seed_jump_use_fix_only = ParseBool(normalized_value);
  } else if (normalized_key == "body_z_jump_pre_post_window_s") {
    config.body_z_jump_pre_post_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_center_gap_s") {
    config.body_z_jump_center_gap_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_velocity_smooth_s") {
    config.body_z_jump_velocity_smooth_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_threshold_ratio") {
    config.body_z_jump_threshold_ratio = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_support_ratio") {
    config.body_z_jump_support_ratio = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_redundant_padding_s") {
    config.body_z_jump_redundant_padding_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_merge_gap_s") {
    config.body_z_jump_merge_gap_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_merge_max_duration_s") {
    config.body_z_jump_merge_max_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_min_score_mps") {
    config.body_z_jump_min_score_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_min_separation_s") {
    config.body_z_jump_min_separation_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_max_window_duration_s") {
    config.body_z_jump_max_window_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_max_levels") {
    config.body_z_jump_max_levels = ParseInt(normalized_value);
  } else if (normalized_key == "body_z_jump_dense_gap_s") {
    config.body_z_jump_dense_gap_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_jump_dense_peak_count") {
    config.body_z_jump_dense_peak_count = ParseInt(normalized_value);
  } else if (normalized_key == "body_z_jump_dense_peak_floor_ratio") {
    config.body_z_jump_dense_peak_floor_ratio = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_nhc_jump_reference") {
    config.enable_nhc_jump_reference = ParseBool(normalized_value);
  } else if (normalized_key == "nhc_history_half_life_s") {
    config.nhc_history_half_life_s = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_history_max_age_s") {
    config.nhc_history_max_age_s = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_body_vy_min_threshold_mps") {
    config.nhc_body_vy_min_threshold_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_body_vz_min_threshold_mps") {
    config.nhc_body_vz_min_threshold_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_body_vz_max_threshold_mps") {
    config.nhc_body_vz_max_threshold_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_body_vy_percentile_scale") {
    config.nhc_body_vy_percentile_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_body_vz_percentile_scale") {
    config.nhc_body_vz_percentile_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_jump_min_separation_s") {
    config.nhc_jump_min_separation_s = ParseDouble(normalized_value);
  } else if (normalized_key == "nhc_jump_recovery_lookback_s") {
    config.nhc_jump_recovery_lookback_s = ParseDouble(normalized_value);
  } else if (normalized_key == "reserve_vertical_velocity_feedback_interface") {
    config.reserve_vertical_velocity_feedback_interface = ParseBool(normalized_value);
  } else if (
    normalized_key == "enable_reweighted_combined_imu_factor" ||
    normalized_key == "enable_imu_relative_attitude_factor") {
    config.enable_reweighted_combined_imu_factor = ParseBool(normalized_value);
  } else if (
    normalized_key == "reweighted_combined_imu_attitude_sigma_rad" ||
    normalized_key == "imu_relative_attitude_sigma_rad") {
    config.reweighted_combined_imu_attitude_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "reweighted_combined_imu_specific_force_sigma_mps2") {
    config.reweighted_combined_imu_specific_force_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "reweighted_combined_imu_specific_force_sigma_x_mps2") {
    config.reweighted_combined_imu_specific_force_sigma_x_mps2 = ParseDouble(normalized_value);
    config.reweighted_combined_imu_specific_force_sigma_x_specified = true;
  } else if (normalized_key == "reweighted_combined_imu_specific_force_sigma_y_mps2") {
    config.reweighted_combined_imu_specific_force_sigma_y_mps2 = ParseDouble(normalized_value);
    config.reweighted_combined_imu_specific_force_sigma_y_specified = true;
  } else if (normalized_key == "reweighted_combined_imu_specific_force_sigma_z_mps2") {
    config.reweighted_combined_imu_specific_force_sigma_z_mps2 = ParseDouble(normalized_value);
    config.reweighted_combined_imu_specific_force_sigma_z_specified = true;
  } else if (normalized_key == "reweighted_combined_imu_position_sigma_m") {
    config.reweighted_combined_imu_position_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "reweighted_combined_imu_velocity_sigma_mps") {
    config.reweighted_combined_imu_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "error_process_noise_scale") {
    config.error_process_noise_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "tau_acc_bias_s") {
    config.tau_acc_bias_s = ParseDouble(normalized_value);
  } else if (normalized_key == "tau_gyro_bias_s") {
    config.tau_gyro_bias_s = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_process_noise_acc_scale") {
    config.bias_process_noise_acc_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "bias_process_noise_gyro_scale") {
    config.bias_process_noise_gyro_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "segment_feedback_attitude_gain") {
    config.segment_feedback_attitude_gain = ParseDouble(normalized_value);
  } else if (normalized_key == "segment_feedback_velocity_gain") {
    config.segment_feedback_velocity_gain = ParseDouble(normalized_value);
  } else if (normalized_key == "segment_feedback_position_gain") {
    config.segment_feedback_position_gain = ParseDouble(normalized_value);
  } else if (normalized_key == "segment_feedback_acc_sigma_mps2") {
    config.segment_feedback_acc_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "segment_feedback_gyro_sigma_radps") {
    config.segment_feedback_gyro_sigma_radps = ParseDouble(normalized_value);
  } else if (normalized_key == "error_state_rotation_sigma_rad") {
    config.error_state_rotation_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "error_state_position_sigma_m") {
    config.error_state_position_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "error_state_velocity_sigma_mps") {
    config.error_state_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "error_state_acc_bias_sigma_mps2") {
    config.error_state_acc_bias_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "error_state_gyro_bias_sigma_radps") {
    config.error_state_gyro_bias_sigma_radps = ParseDouble(normalized_value);
  } else if (normalized_key == "stationary_window_s") {
    config.stationary_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "stationary_acc_tolerance_mps2") {
    config.stationary_acc_tolerance_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "stationary_gyro_threshold_radps") {
    config.stationary_gyro_threshold_radps = ParseDouble(normalized_value);
  } else if (normalized_key == "prefer_imu_initial_yaw") {
    config.prefer_imu_initial_yaw = ParseBool(normalized_value);
  } else if (normalized_key == "static_alignment_duration_s") {
    config.static_alignment_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "imu_dual_vector_window_s") {
    config.imu_dual_vector_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "imu_dual_vector_min_sample_count") {
    config.imu_dual_vector_min_sample_count = ParseInt(normalized_value);
  } else if (normalized_key == "imu_dual_vector_min_cross_norm") {
    config.imu_dual_vector_min_cross_norm = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_static_zupt_zaru") {
    config.enable_initial_static_zupt_zaru = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_zupt_velocity_sigma_mps") {
    config.initial_static_zupt_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_static_zaru_sigma_radps") {
    config.initial_static_zaru_sigma_radps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_static_zero_specific_force") {
    config.enable_initial_static_zero_specific_force = ParseBool(normalized_value);
  } else if (normalized_key == "enable_initial_static_vertical_specific_force") {
    config.enable_initial_static_vertical_specific_force = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_specific_force_sigma_mps2") {
    config.initial_static_specific_force_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_static_vertical_specific_force_sigma_mps2") {
    config.initial_static_vertical_specific_force_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_static_subgraph") {
    config.enable_initial_static_subgraph = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_state_frequency_hz") {
    config.initial_static_state_frequency_hz = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_static_attitude_drift_sigma_rad") {
    config.initial_static_attitude_drift_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "yaw_min_distance_m") {
    config.yaw_min_distance_m = ParseDouble(normalized_value);
  } else if (normalized_key == "fallback_initial_yaw_rad") {
    config.fallback_initial_yaw_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "early_gnss_relaxation_duration_s") {
    config.early_gnss_relaxation_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "early_gnss_relaxation_scale") {
    config.early_gnss_relaxation_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "position_sigma_floor_m") {
    config.position_sigma_floor_m = ParseDouble(normalized_value);
    config.position_sigma_floor_horizontal_m = config.position_sigma_floor_m;
    config.position_sigma_floor_up_m = config.position_sigma_floor_m;
  } else if (normalized_key == "position_sigma_floor_horizontal_m") {
    config.position_sigma_floor_horizontal_m = ParseDouble(normalized_value);
  } else if (normalized_key == "position_sigma_floor_up_m") {
    config.position_sigma_floor_up_m = ParseDouble(normalized_value);
  } else if (normalized_key == "position_sigma_ceiling_m") {
    config.position_sigma_ceiling_m = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_vertical_sigma_mode") {
    config.gnss_vertical_sigma_mode = ParseVerticalSigmaMode(normalized_value);
  } else if (normalized_key == "gnss_vertical_fixed_sigma_m") {
    config.gnss_vertical_fixed_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_sigma_scale_horizontal") {
    config.gnss_sigma_scale_horizontal = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_sigma_scale_up") {
    config.gnss_sigma_scale_up = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_gnss_vertical_drift_model") {
    config.enable_gnss_vertical_drift_model = ParseBool(normalized_value);
  } else if (normalized_key == "gnss_vertical_drift_reference_mode") {
    config.gnss_vertical_drift_reference_mode = ParseVerticalDriftReferenceMode(normalized_value);
  } else if (normalized_key == "gnss_vertical_drift_window_s") {
    config.gnss_vertical_drift_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_position_noise_model") {
    config.gnss_position_noise_model = ParseNoiseModel(normalized_value);
  } else if (normalized_key == "gnss_position_robust_param") {
    config.gnss_position_robust_param = ParseDouble(normalized_value);
  } else if (normalized_key == "rtkfix_scale") {
    config.rtkfix_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "rtkfloat_scale") {
    config.rtkfloat_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "single_scale") {
    config.single_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "drop_non_rtkfix") {
    config.drop_non_rtkfix = ParseBool(normalized_value);
  } else if (normalized_key == "required_best_sol_status_code") {
    config.required_best_sol_status_code = ParseInt(normalized_value);
  } else if (normalized_key == "drop_no_solution") {
    config.drop_no_solution = ParseBool(normalized_value);
  } else if (normalized_key == "drop_nonfinite_sigma") {
    config.drop_nonfinite_sigma = ParseBool(normalized_value);
  } else if (normalized_key == "gnss_consistency_gate_mode") {
    config.gnss_consistency_gate_mode = ParseConsistencyGateMode(normalized_value);
  } else if (normalized_key == "gnss_nis_confidence") {
    config.gnss_nis_confidence = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_axis_sigma_multiple") {
    config.gnss_axis_sigma_multiple = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_consistency_relaxed_threshold_ratio") {
    config.gnss_consistency_relaxed_threshold_ratio = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_consistency_max_scale_horizontal") {
    config.gnss_consistency_max_scale_horizontal = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_consistency_max_scale_up") {
    config.gnss_consistency_max_scale_up = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_position_sigma_m") {
    config.initial_position_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_roll_pitch_sigma_rad") {
    config.initial_roll_pitch_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_yaw_sigma_rad") {
    config.initial_yaw_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_velocity_sigma_mps") {
    config.initial_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "lm_lambda_initial") {
    config.lm_lambda_initial = ParseDouble(normalized_value);
  } else if (normalized_key == "lm_max_iterations") {
    config.lm_max_iterations = ParseInt(normalized_value);
  } else {
    throw std::runtime_error("unknown config key: " + normalized_key);
  }
}

OfflineRunnerConfig LoadConfigFile(const std::string_view config_path, const OfflineRunnerConfig &base_config) {
  OfflineRunnerConfig config = base_config;
  if (config_path.empty()) {
    ResolveReweightedSpecificForceSigmaAxes(config);
    if (config.state_meas_sync_lower_bound_s > config.state_meas_sync_upper_bound_s) {
      throw std::runtime_error("state_meas_sync_lower_bound_s must be <= state_meas_sync_upper_bound_s");
    }
    if (config.state_meas_sync_lower_bound_s > 0.0 || config.state_meas_sync_upper_bound_s < 0.0) {
      throw std::runtime_error("state_meas_sync bounds must satisfy lower <= 0 <= upper");
    }
    if (config.state_frequency_hz <= 0.0) {
      throw std::runtime_error("state_frequency_hz must be positive");
    }
    if (config.gnss_position_robust_param <= 0.0) {
      throw std::runtime_error("gnss_position_robust_param must be positive");
    }
    if (config.position_sigma_floor_horizontal_m < 0.0 || config.position_sigma_floor_up_m < 0.0) {
      throw std::runtime_error("position sigma floors must be non-negative");
    }
    if (config.position_sigma_floor_horizontal_m > config.position_sigma_ceiling_m ||
        config.position_sigma_floor_up_m > config.position_sigma_ceiling_m) {
      throw std::runtime_error("position sigma floors must not exceed position_sigma_ceiling_m");
    }
    if (config.gnss_vertical_fixed_sigma_m <= 0.0) {
      throw std::runtime_error("gnss_vertical_fixed_sigma_m must be positive");
    }
    if (config.gnss_sigma_scale_horizontal <= 0.0 || config.gnss_sigma_scale_up <= 0.0) {
      throw std::runtime_error("gnss sigma scales must be positive");
    }
    if (config.enable_gnss_vertical_drift_model && config.gnss_vertical_drift_window_s <= 0.0) {
      throw std::runtime_error("gnss_vertical_drift_window_s must be positive when drift model is enabled");
    }
    if (config.stationary_window_s <= 0.0) {
      throw std::runtime_error("stationary_window_s must be positive");
    }
    if (config.global_acc_bias_tie_sigma_mps2 <= 0.0) {
      throw std::runtime_error("global_acc_bias_tie_sigma_mps2 must be positive");
    }
    if (config.global_acc_bias_tie_sigma_xy_mps2 <= 0.0) {
      throw std::runtime_error("global_acc_bias_tie_sigma_xy_mps2 must be positive");
    }
    if (config.global_gyro_bias_tie_sigma_radps <= 0.0) {
      throw std::runtime_error("global_gyro_bias_tie_sigma_radps must be positive");
    }
    if (config.vertical_acc_bias_tau_s <= 0.0) {
      throw std::runtime_error("vertical_acc_bias_tau_s must be positive");
    }
    if (config.vertical_acc_bias_sigma_mps2 < 0.0) {
      throw std::runtime_error("vertical_acc_bias_sigma_mps2 must be non-negative");
    }
    if (config.vertical_acc_bias_process_noise_scale <= 0.0) {
      throw std::runtime_error("vertical_acc_bias_process_noise_scale must be positive");
    }
    if (config.reweighted_combined_imu_attitude_sigma_rad <= 0.0) {
      throw std::runtime_error("reweighted_combined_imu_attitude_sigma_rad must be positive");
    }
    if (config.reweighted_combined_imu_specific_force_sigma_mps2 < 0.0) {
      throw std::runtime_error("reweighted_combined_imu_specific_force_sigma_mps2 must be non-negative");
    }
    if (config.reweighted_combined_imu_specific_force_sigma_x_mps2 < 0.0 ||
        config.reweighted_combined_imu_specific_force_sigma_y_mps2 < 0.0 ||
        config.reweighted_combined_imu_specific_force_sigma_z_mps2 < 0.0) {
      throw std::runtime_error("reweighted_combined_imu_specific_force_sigma_{x,y,z}_mps2 must be non-negative");
    }
    if (config.reweighted_combined_imu_position_sigma_m < 0.0) {
      throw std::runtime_error("reweighted_combined_imu_position_sigma_m must be non-negative");
    }
    if (config.reweighted_combined_imu_velocity_sigma_mps < 0.0) {
      throw std::runtime_error("reweighted_combined_imu_velocity_sigma_mps must be non-negative");
    }
    if (config.error_process_noise_scale <= 0.0) {
      throw std::runtime_error("error_process_noise_scale must be positive");
    }
    if (config.tau_acc_bias_s <= 0.0 || config.tau_gyro_bias_s <= 0.0) {
      throw std::runtime_error("tau_acc_bias_s and tau_gyro_bias_s must be positive");
    }
    if (config.bias_process_noise_acc_scale <= 0.0 || config.bias_process_noise_gyro_scale <= 0.0) {
      throw std::runtime_error("bias_process_noise_acc_scale and bias_process_noise_gyro_scale must be positive");
    }
    if (config.gnss_nis_confidence <= 0.0 || config.gnss_nis_confidence >= 1.0) {
      throw std::runtime_error("gnss_nis_confidence must be in (0, 1)");
    }
    if (config.gnss_axis_sigma_multiple <= 0.0) {
      throw std::runtime_error("gnss_axis_sigma_multiple must be positive");
    }
    if (config.gnss_consistency_relaxed_threshold_ratio <= 0.0 ||
        config.gnss_consistency_relaxed_threshold_ratio >= 1.0) {
      throw std::runtime_error("gnss_consistency_relaxed_threshold_ratio must be in (0, 1)");
    }
    if (config.gnss_consistency_max_scale_horizontal < 1.0 || config.gnss_consistency_max_scale_up < 1.0) {
      throw std::runtime_error("gnss consistency max scales must be >= 1");
    }
    if (config.enable_segment_error_feedback && (config.enable_global_acc_bias || config.enable_global_gyro_bias)) {
      throw std::runtime_error(
        "enable_segment_error_feedback is incompatible with enable_global_acc_bias/enable_global_gyro_bias");
    }
    if (config.enable_vertical_acc_bias_gm_process && !config.enable_global_acc_bias) {
      throw std::runtime_error("enable_vertical_acc_bias_gm_process requires enable_global_acc_bias");
    }
    if (config.enable_vertical_acc_bias_gm_process && config.enable_segment_error_feedback) {
      throw std::runtime_error(
        "enable_vertical_acc_bias_gm_process is incompatible with enable_segment_error_feedback");
    }
    if (config.enable_vertical_acc_bias_gm_process && config.enable_segment_local_error_feedback) {
      throw std::runtime_error(
        "enable_vertical_acc_bias_gm_process is incompatible with enable_segment_local_error_feedback");
    }
    if (config.enable_vertical_rtk_preintegration_feedback && !config.enable_gnss) {
      throw std::runtime_error("enable_vertical_rtk_preintegration_feedback requires enable_gnss");
    }
    if (config.enable_vertical_rtk_preintegration_feedback && !config.enable_global_acc_bias) {
      throw std::runtime_error("enable_vertical_rtk_preintegration_feedback requires enable_global_acc_bias");
    }
    if (config.enable_vertical_rtk_preintegration_feedback && !config.enable_vertical_acc_bias_gm_process) {
      throw std::runtime_error(
        "enable_vertical_rtk_preintegration_feedback requires enable_vertical_acc_bias_gm_process");
    }
    if (config.enable_vertical_rtk_preintegration_feedback && config.enable_segment_error_feedback) {
      throw std::runtime_error(
        "enable_vertical_rtk_preintegration_feedback is incompatible with enable_segment_error_feedback");
    }
    if (config.enable_vertical_rtk_preintegration_feedback && config.enable_segment_local_error_feedback) {
      throw std::runtime_error(
        "enable_vertical_rtk_preintegration_feedback is incompatible with enable_segment_local_error_feedback");
    }
    if (config.enable_segment_local_error_feedback && !config.enable_segment_error_feedback) {
      throw std::runtime_error(
        "enable_segment_local_error_feedback requires enable_segment_error_feedback");
    }
    if (config.segment_feedback_attitude_gain < 0.0 || config.segment_feedback_velocity_gain < 0.0 ||
        config.segment_feedback_position_gain < 0.0) {
      throw std::runtime_error("segment_feedback gains must be non-negative");
    }
    if (config.segment_feedback_acc_sigma_mps2 <= 0.0 || config.segment_feedback_gyro_sigma_radps <= 0.0) {
      throw std::runtime_error("segment_feedback sigmas must be positive");
    }
    if (config.vertical_rtk_gate_sigma_multiple <= 0.0) {
      throw std::runtime_error("vertical_rtk_gate_sigma_multiple must be positive");
    }
    if (config.vertical_rtk_inside_gate_sigma_scale <= 0.0 || config.vertical_rtk_outside_gate_sigma_scale <= 0.0) {
      throw std::runtime_error("vertical RTK gate sigma scales must be positive");
    }
    if (config.vertical_rtk_inside_feedback_gain_scale < 0.0 ||
        config.vertical_rtk_outside_feedback_gain_scale < 0.0) {
      throw std::runtime_error("vertical RTK feedback gain scales must be non-negative");
    }
    if (config.vertical_rtk_feedback_bias_gain < 0.0 || config.vertical_rtk_feedback_attitude_gain < 0.0) {
      throw std::runtime_error("vertical RTK feedback gains must be non-negative");
    }
    if (config.vertical_rtk_feedback_sigma_dp_m <= 0.0 ||
        config.vertical_rtk_feedback_sigma_baz_mps2 <= 0.0 ||
        config.vertical_rtk_feedback_sigma_attitude_rad <= 0.0 ||
        config.vertical_rtk_feedback_sigma_vz_mps <= 0.0) {
      throw std::runtime_error("vertical RTK feedback sigmas must be positive");
    }
    if (config.vertical_rtk_feedback_min_interval_s < 0.0) {
      throw std::runtime_error("vertical_rtk_feedback_min_interval_s must be non-negative");
    }
    if (config.vertical_rtk_jump_inside_sigma_scale < 1.0) {
      throw std::runtime_error("vertical_rtk_jump_inside_sigma_scale must be >= 1");
    }
    if (config.vertical_local_recovery_max_iterations <= 0) {
      throw std::runtime_error("vertical_local_recovery_max_iterations must be positive");
    }
    if (config.vertical_local_recovery_max_attitude_delta_rad <= 0.0 ||
        config.vertical_local_recovery_max_baz_delta_mps2 <= 0.0) {
      throw std::runtime_error("vertical local recovery slow-variable limits must be positive");
    }
    if (config.vertical_global_vz_window_s <= 0.0 || config.vertical_global_vz_smooth_window_s <= 0.0) {
      throw std::runtime_error("vertical global vz windows must be positive");
    }
    if (config.vertical_jump_candidate_min_separation_s < 0.0) {
      throw std::runtime_error("vertical_jump_candidate_min_separation_s must be non-negative");
    }
    if (config.vertical_jump_max_candidates_per_segment <= 0 ||
        config.vertical_jump_max_selected_points_per_segment <= 0) {
      throw std::runtime_error("vertical jump candidate and selection limits must be positive");
    }
    if (config.vertical_jump_max_selected_points_per_segment > config.vertical_jump_max_candidates_per_segment) {
      throw std::runtime_error(
        "vertical_jump_max_selected_points_per_segment must be <= vertical_jump_max_candidates_per_segment");
    }
    if (config.vertical_jump_hold_window_s <= 0.0) {
      throw std::runtime_error("vertical_jump_hold_window_s must be positive");
    }
    if (config.vertical_jump_step_min_threshold_mps <= 0.0 || config.vertical_jump_vz_prior_sigma_mps <= 0.0) {
      throw std::runtime_error("vertical jump threshold and prior sigma must be positive");
    }
    if (config.vertical_jump_window_default_padding_states < 0 ||
        config.vertical_jump_window_max_points <= 0) {
      throw std::runtime_error("vertical jump window padding must be non-negative and max points must be positive");
    }
    if (config.vertical_jump_window_support_ratio < 0.0 || config.vertical_jump_window_support_ratio > 1.0) {
      throw std::runtime_error("vertical_jump_window_support_ratio must be in [0, 1]");
    }
    if (config.vertical_jump_window_max_duration_s <= 0.0 ||
        config.vertical_jump_window_tail_target_s <= 0.0) {
      throw std::runtime_error("vertical jump window duration and tail target must be positive");
    }
    if (config.vertical_jump_window_velocity_smoothness_weight < 0.0 ||
        config.vertical_jump_window_height_integral_weight < 0.0 ||
        config.vertical_jump_window_ref_weight < 0.0) {
      throw std::runtime_error("vertical jump window objective weights must be non-negative");
    }
    if (config.vertical_jump_window_max_correction_attempts <= 0) {
      throw std::runtime_error("vertical_jump_window_max_correction_attempts must be positive");
    }
    if (config.vertical_jump_future_trend_window_s < 0.0 ||
        config.vertical_jump_future_trend_min_fix_count < 0 ||
        config.vertical_jump_future_trend_mean_weight < 0.0 ||
        config.vertical_jump_future_trend_slope_weight < 0.0) {
      throw std::runtime_error("vertical jump future trend settings must be non-negative");
    }
    if (config.vertical_interval_feedback_min_duration_s <= 0.0 ||
        config.vertical_interval_feedback_min_slope_mps < 0.0 ||
        config.vertical_interval_feedback_min_drift_m < 0.0 ||
        config.vertical_interval_feedback_min_residual_m < 0.0 ||
        config.vertical_interval_feedback_snr_threshold <= 0.0 ||
        config.vertical_interval_feedback_noise_floor_m < 0.0 ||
        config.vertical_interval_feedback_gain < 0.0 ||
        config.vertical_interval_feedback_max_delta_vz_mps <= 0.0) {
      throw std::runtime_error("vertical interval feedback settings must be valid");
    }
    if (config.vertical_inside_attitude_gain < 0.0 ||
        config.vertical_inside_max_delta_attitude_rad <= 0.0) {
      throw std::runtime_error("vertical inside attitude gain and limit must be valid");
    }
    if (config.enable_body_z_seed_jump_windows && !config.enable_vertical_rtk_seed_pass) {
      throw std::runtime_error("enable_body_z_seed_jump_windows requires enable_vertical_rtk_seed_pass");
    }
    if (config.enable_vertical_rtk_seed_pass && !config.enable_gnss) {
      throw std::runtime_error("enable_vertical_rtk_seed_pass requires enable_gnss");
    }
    if (config.body_z_jump_pre_post_window_s <= 0.0 ||
        config.body_z_jump_velocity_smooth_s <= 0.0 ||
        config.body_z_jump_min_score_mps <= 0.0 ||
        config.body_z_jump_min_separation_s < 0.0 ||
        config.body_z_jump_max_window_duration_s <= 0.0 ||
        config.body_z_jump_dense_gap_s <= 0.0 ||
        config.body_z_jump_dense_peak_floor_ratio <= 0.0) {
      throw std::runtime_error("body-z jump detector timing and threshold settings are invalid");
    }
    if (config.body_z_jump_center_gap_s < 0.0 ||
        config.body_z_jump_redundant_padding_s < 0.0 ||
        config.body_z_jump_merge_gap_s < 0.0 ||
        config.body_z_jump_merge_max_duration_s < 0.0) {
      throw std::runtime_error("body-z jump detector gap and padding settings must be non-negative");
    }
    if (config.body_z_jump_threshold_ratio <= 0.0 ||
        config.body_z_jump_support_ratio < 0.0 ||
        config.body_z_jump_support_ratio > 1.0) {
      throw std::runtime_error("body-z jump detector ratios are invalid");
    }
    if (config.body_z_jump_max_levels <= 0 || config.body_z_jump_dense_peak_count <= 0) {
      throw std::runtime_error("body-z jump detector level and density limits must be positive");
    }
    if (config.nhc_history_half_life_s <= 0.0 || config.nhc_history_max_age_s <= 0.0) {
      throw std::runtime_error("NHC history windows must be positive");
    }
    if (config.nhc_history_max_age_s < config.nhc_history_half_life_s) {
      throw std::runtime_error("nhc_history_max_age_s must be >= nhc_history_half_life_s");
    }
    if (config.nhc_body_vy_min_threshold_mps <= 0.0 || config.nhc_body_vz_min_threshold_mps <= 0.0) {
      throw std::runtime_error("NHC minimum body-velocity thresholds must be positive");
    }
    if (config.nhc_body_vz_max_threshold_mps > 0.0 &&
        config.nhc_body_vz_max_threshold_mps < config.nhc_body_vz_min_threshold_mps) {
      throw std::runtime_error("nhc_body_vz_max_threshold_mps must be >= nhc_body_vz_min_threshold_mps when enabled");
    }
    if (config.nhc_body_vy_percentile_scale < 1.0 || config.nhc_body_vz_percentile_scale < 1.0) {
      throw std::runtime_error("NHC percentile scales must be >= 1");
    }
    if (config.nhc_jump_min_separation_s < 0.0) {
      throw std::runtime_error("nhc_jump_min_separation_s must be non-negative");
    }
    if (config.nhc_jump_recovery_lookback_s <= 0.0) {
      throw std::runtime_error("nhc_jump_recovery_lookback_s must be positive");
    }
    if (config.static_alignment_duration_s < 0.0) {
      throw std::runtime_error("static_alignment_duration_s must be non-negative");
    }
    if (config.imu_dual_vector_window_s <= 0.0) {
      throw std::runtime_error("imu_dual_vector_window_s must be positive");
    }
    if (config.imu_dual_vector_min_sample_count <= 0) {
      throw std::runtime_error("imu_dual_vector_min_sample_count must be positive");
    }
    if (config.imu_dual_vector_min_cross_norm <= 0.0) {
      throw std::runtime_error("imu_dual_vector_min_cross_norm must be positive");
    }
    if (config.initial_static_zupt_velocity_sigma_mps <= 0.0) {
      throw std::runtime_error("initial_static_zupt_velocity_sigma_mps must be positive");
    }
    if (config.initial_static_zaru_sigma_radps <= 0.0) {
      throw std::runtime_error("initial_static_zaru_sigma_radps must be positive");
    }
    if (config.initial_static_specific_force_sigma_mps2 <= 0.0) {
      throw std::runtime_error("initial_static_specific_force_sigma_mps2 must be positive");
    }
    if (config.initial_static_vertical_specific_force_sigma_mps2 <= 0.0) {
      throw std::runtime_error("initial_static_vertical_specific_force_sigma_mps2 must be positive");
    }
    if (config.initial_static_state_frequency_hz <= 0.0) {
      throw std::runtime_error("initial_static_state_frequency_hz must be positive");
    }
    if (config.initial_static_attitude_drift_sigma_rad <= 0.0) {
      throw std::runtime_error("initial_static_attitude_drift_sigma_rad must be positive");
    }
    if (config.enable_initial_static_zupt_zaru && config.static_alignment_duration_s <= 0.0) {
      throw std::runtime_error("enable_initial_static_zupt_zaru requires static_alignment_duration_s > 0");
    }
    if (config.enable_initial_static_zero_specific_force && config.static_alignment_duration_s <= 0.0) {
      throw std::runtime_error("enable_initial_static_zero_specific_force requires static_alignment_duration_s > 0");
    }
    if (config.enable_initial_static_vertical_specific_force && config.static_alignment_duration_s <= 0.0) {
      throw std::runtime_error(
        "enable_initial_static_vertical_specific_force requires static_alignment_duration_s > 0");
    }
    if (config.enable_initial_static_subgraph && config.static_alignment_duration_s <= 0.0) {
      throw std::runtime_error("enable_initial_static_subgraph requires static_alignment_duration_s > 0");
    }
    return config;
  }

  std::ifstream input_stream{std::string(config_path)};
  if (!input_stream.is_open()) {
    throw std::runtime_error("failed to open config file: " + std::string(config_path));
  }

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(input_stream, line)) {
    ++line_number;
    const auto comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }
    line = Trim(std::move(line));
    if (line.empty()) {
      continue;
    }

    const auto delimiter_pos = line.find('=');
    if (delimiter_pos == std::string::npos) {
      throw std::runtime_error("invalid config line " + std::to_string(line_number) + ": missing '='");
    }

    const auto key = Trim(line.substr(0, delimiter_pos));
    const auto value = Trim(line.substr(delimiter_pos + 1));
    OverrideConfigField(config, key, value);
  }
  ResolveReweightedSpecificForceSigmaAxes(config);

  if (config.state_meas_sync_lower_bound_s > config.state_meas_sync_upper_bound_s) {
    throw std::runtime_error("state_meas_sync_lower_bound_s must be <= state_meas_sync_upper_bound_s");
  }
  if (config.state_meas_sync_lower_bound_s > 0.0 || config.state_meas_sync_upper_bound_s < 0.0) {
    throw std::runtime_error("state_meas_sync bounds must satisfy lower <= 0 <= upper");
  }
  if (config.state_frequency_hz <= 0.0) {
    throw std::runtime_error("state_frequency_hz must be positive");
  }
  if (config.gnss_position_robust_param <= 0.0) {
    throw std::runtime_error("gnss_position_robust_param must be positive");
  }
  if (config.position_sigma_floor_horizontal_m < 0.0 || config.position_sigma_floor_up_m < 0.0) {
    throw std::runtime_error("position sigma floors must be non-negative");
  }
  if (config.position_sigma_floor_horizontal_m > config.position_sigma_ceiling_m ||
      config.position_sigma_floor_up_m > config.position_sigma_ceiling_m) {
    throw std::runtime_error("position sigma floors must not exceed position_sigma_ceiling_m");
  }
  if (config.gnss_vertical_fixed_sigma_m <= 0.0) {
    throw std::runtime_error("gnss_vertical_fixed_sigma_m must be positive");
  }
  if (config.gnss_sigma_scale_horizontal <= 0.0 || config.gnss_sigma_scale_up <= 0.0) {
    throw std::runtime_error("gnss sigma scales must be positive");
  }
  if (config.enable_gnss_vertical_drift_model && config.gnss_vertical_drift_window_s <= 0.0) {
    throw std::runtime_error("gnss_vertical_drift_window_s must be positive when drift model is enabled");
  }
  if (config.stationary_window_s <= 0.0) {
    throw std::runtime_error("stationary_window_s must be positive");
  }
  if (config.global_acc_bias_tie_sigma_mps2 <= 0.0) {
    throw std::runtime_error("global_acc_bias_tie_sigma_mps2 must be positive");
  }
  if (config.global_acc_bias_tie_sigma_xy_mps2 <= 0.0) {
    throw std::runtime_error("global_acc_bias_tie_sigma_xy_mps2 must be positive");
  }
  if (config.global_gyro_bias_tie_sigma_radps <= 0.0) {
    throw std::runtime_error("global_gyro_bias_tie_sigma_radps must be positive");
  }
  if (config.vertical_acc_bias_tau_s <= 0.0) {
    throw std::runtime_error("vertical_acc_bias_tau_s must be positive");
  }
  if (config.vertical_acc_bias_sigma_mps2 < 0.0) {
    throw std::runtime_error("vertical_acc_bias_sigma_mps2 must be non-negative");
  }
  if (config.vertical_acc_bias_process_noise_scale <= 0.0) {
    throw std::runtime_error("vertical_acc_bias_process_noise_scale must be positive");
  }
  if (config.reweighted_combined_imu_attitude_sigma_rad <= 0.0) {
    throw std::runtime_error("reweighted_combined_imu_attitude_sigma_rad must be positive");
  }
  if (config.reweighted_combined_imu_specific_force_sigma_mps2 < 0.0) {
    throw std::runtime_error("reweighted_combined_imu_specific_force_sigma_mps2 must be non-negative");
  }
  if (config.reweighted_combined_imu_specific_force_sigma_x_mps2 < 0.0 ||
      config.reweighted_combined_imu_specific_force_sigma_y_mps2 < 0.0 ||
      config.reweighted_combined_imu_specific_force_sigma_z_mps2 < 0.0) {
    throw std::runtime_error("reweighted_combined_imu_specific_force_sigma_{x,y,z}_mps2 must be non-negative");
  }
  if (config.reweighted_combined_imu_position_sigma_m < 0.0) {
    throw std::runtime_error("reweighted_combined_imu_position_sigma_m must be non-negative");
  }
  if (config.reweighted_combined_imu_velocity_sigma_mps < 0.0) {
    throw std::runtime_error("reweighted_combined_imu_velocity_sigma_mps must be non-negative");
  }
  if (config.error_process_noise_scale <= 0.0) {
    throw std::runtime_error("error_process_noise_scale must be positive");
  }
  if (config.tau_acc_bias_s <= 0.0 || config.tau_gyro_bias_s <= 0.0) {
    throw std::runtime_error("tau_acc_bias_s and tau_gyro_bias_s must be positive");
  }
  if (config.bias_process_noise_acc_scale <= 0.0 || config.bias_process_noise_gyro_scale <= 0.0) {
    throw std::runtime_error("bias_process_noise_acc_scale and bias_process_noise_gyro_scale must be positive");
  }
  if (config.gnss_nis_confidence <= 0.0 || config.gnss_nis_confidence >= 1.0) {
    throw std::runtime_error("gnss_nis_confidence must be in (0, 1)");
  }
  if (config.gnss_axis_sigma_multiple <= 0.0) {
    throw std::runtime_error("gnss_axis_sigma_multiple must be positive");
  }
  if (config.gnss_consistency_relaxed_threshold_ratio <= 0.0 ||
      config.gnss_consistency_relaxed_threshold_ratio >= 1.0) {
    throw std::runtime_error("gnss_consistency_relaxed_threshold_ratio must be in (0, 1)");
  }
  if (config.gnss_consistency_max_scale_horizontal < 1.0 || config.gnss_consistency_max_scale_up < 1.0) {
    throw std::runtime_error("gnss consistency max scales must be >= 1");
  }
  if (config.enable_segment_error_feedback && (config.enable_global_acc_bias || config.enable_global_gyro_bias)) {
    throw std::runtime_error(
      "enable_segment_error_feedback is incompatible with enable_global_acc_bias/enable_global_gyro_bias");
  }
  if (config.enable_vertical_acc_bias_gm_process && !config.enable_global_acc_bias) {
    throw std::runtime_error("enable_vertical_acc_bias_gm_process requires enable_global_acc_bias");
  }
  if (config.enable_vertical_acc_bias_gm_process && config.enable_segment_error_feedback) {
    throw std::runtime_error(
      "enable_vertical_acc_bias_gm_process is incompatible with enable_segment_error_feedback");
  }
  if (config.enable_vertical_acc_bias_gm_process && config.enable_segment_local_error_feedback) {
    throw std::runtime_error(
      "enable_vertical_acc_bias_gm_process is incompatible with enable_segment_local_error_feedback");
  }
  if (config.enable_vertical_rtk_preintegration_feedback && !config.enable_gnss) {
    throw std::runtime_error("enable_vertical_rtk_preintegration_feedback requires enable_gnss");
  }
  if (config.enable_vertical_rtk_preintegration_feedback && !config.enable_global_acc_bias) {
    throw std::runtime_error("enable_vertical_rtk_preintegration_feedback requires enable_global_acc_bias");
  }
  if (config.enable_vertical_rtk_preintegration_feedback && !config.enable_vertical_acc_bias_gm_process) {
    throw std::runtime_error(
      "enable_vertical_rtk_preintegration_feedback requires enable_vertical_acc_bias_gm_process");
  }
  if (config.enable_vertical_rtk_preintegration_feedback && config.enable_segment_error_feedback) {
    throw std::runtime_error(
      "enable_vertical_rtk_preintegration_feedback is incompatible with enable_segment_error_feedback");
  }
  if (config.enable_vertical_rtk_preintegration_feedback && config.enable_segment_local_error_feedback) {
    throw std::runtime_error(
      "enable_vertical_rtk_preintegration_feedback is incompatible with enable_segment_local_error_feedback");
  }
  if (config.enable_segment_local_error_feedback && !config.enable_segment_error_feedback) {
    throw std::runtime_error(
      "enable_segment_local_error_feedback requires enable_segment_error_feedback");
  }
  if (config.segment_feedback_attitude_gain < 0.0 || config.segment_feedback_velocity_gain < 0.0 ||
      config.segment_feedback_position_gain < 0.0) {
    throw std::runtime_error("segment_feedback gains must be non-negative");
  }
  if (config.segment_feedback_acc_sigma_mps2 <= 0.0 || config.segment_feedback_gyro_sigma_radps <= 0.0) {
    throw std::runtime_error("segment_feedback sigmas must be positive");
  }
  if (config.vertical_rtk_gate_sigma_multiple <= 0.0) {
    throw std::runtime_error("vertical_rtk_gate_sigma_multiple must be positive");
  }
  if (config.vertical_rtk_inside_gate_sigma_scale <= 0.0 || config.vertical_rtk_outside_gate_sigma_scale <= 0.0) {
    throw std::runtime_error("vertical RTK gate sigma scales must be positive");
  }
  if (config.vertical_rtk_inside_feedback_gain_scale < 0.0 ||
      config.vertical_rtk_outside_feedback_gain_scale < 0.0) {
    throw std::runtime_error("vertical RTK feedback gain scales must be non-negative");
  }
  if (config.vertical_rtk_feedback_bias_gain < 0.0 || config.vertical_rtk_feedback_attitude_gain < 0.0) {
    throw std::runtime_error("vertical RTK feedback gains must be non-negative");
  }
  if (config.vertical_rtk_feedback_sigma_dp_m <= 0.0 ||
      config.vertical_rtk_feedback_sigma_baz_mps2 <= 0.0 ||
      config.vertical_rtk_feedback_sigma_attitude_rad <= 0.0 ||
      config.vertical_rtk_feedback_sigma_vz_mps <= 0.0) {
    throw std::runtime_error("vertical RTK feedback sigmas must be positive");
  }
  if (config.vertical_rtk_feedback_min_interval_s < 0.0) {
    throw std::runtime_error("vertical_rtk_feedback_min_interval_s must be non-negative");
  }
  if (config.vertical_rtk_jump_inside_sigma_scale < 1.0) {
    throw std::runtime_error("vertical_rtk_jump_inside_sigma_scale must be >= 1");
  }
  if (config.vertical_local_recovery_max_iterations <= 0) {
    throw std::runtime_error("vertical_local_recovery_max_iterations must be positive");
  }
  if (config.vertical_local_recovery_max_attitude_delta_rad <= 0.0 ||
      config.vertical_local_recovery_max_baz_delta_mps2 <= 0.0) {
    throw std::runtime_error("vertical local recovery slow-variable limits must be positive");
  }
  if (config.vertical_global_vz_window_s <= 0.0 || config.vertical_global_vz_smooth_window_s <= 0.0) {
    throw std::runtime_error("vertical global vz windows must be positive");
  }
  if (config.vertical_jump_candidate_min_separation_s < 0.0) {
    throw std::runtime_error("vertical_jump_candidate_min_separation_s must be non-negative");
  }
  if (config.vertical_jump_max_candidates_per_segment <= 0 ||
      config.vertical_jump_max_selected_points_per_segment <= 0) {
    throw std::runtime_error("vertical jump candidate and selection limits must be positive");
  }
  if (config.vertical_jump_max_selected_points_per_segment > config.vertical_jump_max_candidates_per_segment) {
    throw std::runtime_error(
      "vertical_jump_max_selected_points_per_segment must be <= vertical_jump_max_candidates_per_segment");
  }
  if (config.vertical_jump_hold_window_s <= 0.0) {
    throw std::runtime_error("vertical_jump_hold_window_s must be positive");
  }
  if (config.vertical_jump_step_min_threshold_mps <= 0.0 || config.vertical_jump_vz_prior_sigma_mps <= 0.0) {
    throw std::runtime_error("vertical jump threshold and prior sigma must be positive");
  }
  if (config.vertical_jump_window_default_padding_states < 0 ||
      config.vertical_jump_window_max_points <= 0) {
    throw std::runtime_error("vertical jump window padding must be non-negative and max points must be positive");
  }
  if (config.vertical_jump_window_support_ratio < 0.0 || config.vertical_jump_window_support_ratio > 1.0) {
    throw std::runtime_error("vertical_jump_window_support_ratio must be in [0, 1]");
  }
  if (config.vertical_jump_window_max_duration_s <= 0.0 ||
      config.vertical_jump_window_tail_target_s <= 0.0) {
    throw std::runtime_error("vertical jump window duration and tail target must be positive");
  }
  if (config.vertical_jump_window_velocity_smoothness_weight < 0.0 ||
      config.vertical_jump_window_height_integral_weight < 0.0 ||
      config.vertical_jump_window_ref_weight < 0.0) {
    throw std::runtime_error("vertical jump window objective weights must be non-negative");
  }
  if (config.vertical_jump_window_max_correction_attempts <= 0) {
    throw std::runtime_error("vertical_jump_window_max_correction_attempts must be positive");
  }
  if (config.vertical_jump_future_trend_window_s < 0.0 ||
      config.vertical_jump_future_trend_min_fix_count < 0 ||
      config.vertical_jump_future_trend_mean_weight < 0.0 ||
      config.vertical_jump_future_trend_slope_weight < 0.0) {
    throw std::runtime_error("vertical jump future trend settings must be non-negative");
  }
  if (config.vertical_interval_feedback_min_duration_s <= 0.0 ||
      config.vertical_interval_feedback_min_slope_mps < 0.0 ||
      config.vertical_interval_feedback_min_drift_m < 0.0 ||
      config.vertical_interval_feedback_min_residual_m < 0.0 ||
      config.vertical_interval_feedback_snr_threshold <= 0.0 ||
      config.vertical_interval_feedback_noise_floor_m < 0.0 ||
      config.vertical_interval_feedback_gain < 0.0 ||
      config.vertical_interval_feedback_max_delta_vz_mps <= 0.0) {
    throw std::runtime_error("vertical interval feedback settings must be valid");
  }
  if (config.vertical_inside_attitude_gain < 0.0 ||
      config.vertical_inside_max_delta_attitude_rad <= 0.0) {
    throw std::runtime_error("vertical inside attitude gain and limit must be valid");
  }
  if (config.enable_body_z_seed_jump_windows && !config.enable_vertical_rtk_seed_pass) {
    throw std::runtime_error("enable_body_z_seed_jump_windows requires enable_vertical_rtk_seed_pass");
  }
  if (config.enable_vertical_rtk_seed_pass && !config.enable_gnss) {
    throw std::runtime_error("enable_vertical_rtk_seed_pass requires enable_gnss");
  }
  if (config.body_z_jump_pre_post_window_s <= 0.0 ||
      config.body_z_jump_velocity_smooth_s <= 0.0 ||
      config.body_z_jump_min_score_mps <= 0.0 ||
      config.body_z_jump_min_separation_s < 0.0 ||
      config.body_z_jump_max_window_duration_s <= 0.0 ||
      config.body_z_jump_dense_gap_s <= 0.0 ||
      config.body_z_jump_dense_peak_floor_ratio <= 0.0) {
    throw std::runtime_error("body-z jump detector timing and threshold settings are invalid");
  }
  if (config.body_z_jump_center_gap_s < 0.0 ||
      config.body_z_jump_redundant_padding_s < 0.0 ||
      config.body_z_jump_merge_gap_s < 0.0 ||
      config.body_z_jump_merge_max_duration_s < 0.0) {
    throw std::runtime_error("body-z jump detector gap and padding settings must be non-negative");
  }
  if (config.body_z_jump_threshold_ratio <= 0.0 ||
      config.body_z_jump_support_ratio < 0.0 ||
      config.body_z_jump_support_ratio > 1.0) {
    throw std::runtime_error("body-z jump detector ratios are invalid");
  }
  if (config.body_z_jump_max_levels <= 0 || config.body_z_jump_dense_peak_count <= 0) {
    throw std::runtime_error("body-z jump detector level and density limits must be positive");
  }
  if (config.nhc_history_half_life_s <= 0.0 || config.nhc_history_max_age_s <= 0.0) {
    throw std::runtime_error("NHC history windows must be positive");
  }
  if (config.nhc_history_max_age_s < config.nhc_history_half_life_s) {
    throw std::runtime_error("nhc_history_max_age_s must be >= nhc_history_half_life_s");
  }
  if (config.nhc_body_vy_min_threshold_mps <= 0.0 || config.nhc_body_vz_min_threshold_mps <= 0.0) {
    throw std::runtime_error("NHC minimum body-velocity thresholds must be positive");
  }
  if (config.nhc_body_vz_max_threshold_mps > 0.0 &&
      config.nhc_body_vz_max_threshold_mps < config.nhc_body_vz_min_threshold_mps) {
    throw std::runtime_error("nhc_body_vz_max_threshold_mps must be >= nhc_body_vz_min_threshold_mps when enabled");
  }
  if (config.nhc_body_vy_percentile_scale < 1.0 || config.nhc_body_vz_percentile_scale < 1.0) {
    throw std::runtime_error("NHC percentile scales must be >= 1");
  }
  if (config.nhc_jump_min_separation_s < 0.0) {
    throw std::runtime_error("nhc_jump_min_separation_s must be non-negative");
  }
  if (config.nhc_jump_recovery_lookback_s <= 0.0) {
    throw std::runtime_error("nhc_jump_recovery_lookback_s must be positive");
  }
  if (config.static_alignment_duration_s < 0.0) {
    throw std::runtime_error("static_alignment_duration_s must be non-negative");
  }
  if (config.imu_dual_vector_window_s <= 0.0) {
    throw std::runtime_error("imu_dual_vector_window_s must be positive");
  }
  if (config.imu_dual_vector_min_sample_count <= 0) {
    throw std::runtime_error("imu_dual_vector_min_sample_count must be positive");
  }
  if (config.imu_dual_vector_min_cross_norm <= 0.0) {
    throw std::runtime_error("imu_dual_vector_min_cross_norm must be positive");
  }
  if (config.initial_static_zupt_velocity_sigma_mps <= 0.0) {
    throw std::runtime_error("initial_static_zupt_velocity_sigma_mps must be positive");
  }
  if (config.initial_static_zaru_sigma_radps <= 0.0) {
    throw std::runtime_error("initial_static_zaru_sigma_radps must be positive");
  }
  if (config.initial_static_specific_force_sigma_mps2 <= 0.0) {
    throw std::runtime_error("initial_static_specific_force_sigma_mps2 must be positive");
  }
  if (config.initial_static_vertical_specific_force_sigma_mps2 <= 0.0) {
    throw std::runtime_error("initial_static_vertical_specific_force_sigma_mps2 must be positive");
  }
  if (config.initial_static_state_frequency_hz <= 0.0) {
    throw std::runtime_error("initial_static_state_frequency_hz must be positive");
  }
  if (config.initial_static_attitude_drift_sigma_rad <= 0.0) {
    throw std::runtime_error("initial_static_attitude_drift_sigma_rad must be positive");
  }
  if (config.enable_initial_static_zupt_zaru && config.static_alignment_duration_s <= 0.0) {
    throw std::runtime_error("enable_initial_static_zupt_zaru requires static_alignment_duration_s > 0");
  }
  if (config.enable_initial_static_zero_specific_force && config.static_alignment_duration_s <= 0.0) {
    throw std::runtime_error("enable_initial_static_zero_specific_force requires static_alignment_duration_s > 0");
  }
  if (config.enable_initial_static_vertical_specific_force && config.static_alignment_duration_s <= 0.0) {
    throw std::runtime_error("enable_initial_static_vertical_specific_force requires static_alignment_duration_s > 0");
  }
  if (config.enable_initial_static_subgraph && config.static_alignment_duration_s <= 0.0) {
    throw std::runtime_error("enable_initial_static_subgraph requires static_alignment_duration_s > 0");
  }

  return config;
}

std::string ConfigToString(const OfflineRunnerConfig &config) {
  std::ostringstream oss;
  oss << "imu_path=" << config.imu_path << '\n'
      << "gnss_path=" << config.gnss_path << '\n'
      << "output_dir=" << config.output_dir << '\n'
      << "enable_gnss=" << (config.enable_gnss ? "true" : "false") << '\n'
      << "enable_gp_interpolated_gnss=" << (config.enable_gp_interpolated_gnss ? "true" : "false") << '\n'
      << "enable_segment_error_feedback=" << (config.enable_segment_error_feedback ? "true" : "false") << '\n'
      << "enable_segment_local_error_feedback="
      << (config.enable_segment_local_error_feedback ? "true" : "false") << '\n'
      << "verbose=" << (config.verbose ? "true" : "false") << '\n'
      << "write_debug_csv=" << (config.write_debug_csv ? "true" : "false") << '\n'
      << "write_error_diagnostics=" << (config.write_error_diagnostics ? "true" : "false") << '\n'
      << "write_segment_error_diagnostics=" << (config.write_segment_error_diagnostics ? "true" : "false") << '\n'
      << "write_imu_rate_avp=" << (config.write_imu_rate_avp ? "true" : "false") << '\n'
      << "state_frequency_hz=" << config.state_frequency_hz << '\n'
      << "gnss_time_offset_s=" << config.gnss_time_offset_s << '\n'
      << "state_meas_sync_lower_bound_s=" << config.state_meas_sync_lower_bound_s << '\n'
      << "state_meas_sync_upper_bound_s=" << config.state_meas_sync_upper_bound_s << '\n'
      << "gravity_mps2=" << config.gravity_mps2 << '\n'
      << "imu_sigma_acc=" << config.imu_sigma_acc << '\n'
      << "imu_sigma_gyro=" << config.imu_sigma_gyro << '\n'
      << "integration_sigma=" << config.integration_sigma << '\n'
      << "bias_acc_sigma=" << config.bias_acc_sigma << '\n'
      << "bias_gyro_sigma=" << config.bias_gyro_sigma << '\n'
      << "bias_acc_prior_sigma=" << config.bias_acc_prior_sigma << '\n'
      << "bias_gyro_prior_sigma=" << config.bias_gyro_prior_sigma << '\n'
      << "enable_global_acc_bias=" << (config.enable_global_acc_bias ? "true" : "false") << '\n'
      << "global_acc_bias_tie_sigma_mps2=" << config.global_acc_bias_tie_sigma_mps2 << '\n'
      << "global_acc_bias_tie_sigma_xy_mps2=" << config.global_acc_bias_tie_sigma_xy_mps2 << '\n'
      << "enable_global_gyro_bias=" << (config.enable_global_gyro_bias ? "true" : "false") << '\n'
      << "global_gyro_bias_tie_sigma_radps=" << config.global_gyro_bias_tie_sigma_radps << '\n'
      << "enable_vertical_acc_bias_gm_process=" << (config.enable_vertical_acc_bias_gm_process ? "true" : "false")
      << '\n'
      << "vertical_acc_bias_tau_s=" << config.vertical_acc_bias_tau_s << '\n'
      << "vertical_acc_bias_sigma_mps2=" << config.vertical_acc_bias_sigma_mps2 << '\n'
      << "vertical_acc_bias_process_noise_scale=" << config.vertical_acc_bias_process_noise_scale << '\n'
      << "enable_vertical_rtk_preintegration_feedback="
      << (config.enable_vertical_rtk_preintegration_feedback ? "true" : "false") << '\n'
      << "enable_vertical_rtk_global_position_factor="
      << (config.enable_vertical_rtk_global_position_factor ? "true" : "false") << '\n'
      << "vertical_rtk_gate_sigma_multiple=" << config.vertical_rtk_gate_sigma_multiple << '\n'
      << "vertical_rtk_inside_gate_sigma_scale=" << config.vertical_rtk_inside_gate_sigma_scale << '\n'
      << "vertical_rtk_outside_gate_sigma_scale=" << config.vertical_rtk_outside_gate_sigma_scale << '\n'
      << "vertical_rtk_inside_feedback_gain_scale=" << config.vertical_rtk_inside_feedback_gain_scale << '\n'
      << "vertical_rtk_outside_feedback_gain_scale=" << config.vertical_rtk_outside_feedback_gain_scale << '\n'
      << "vertical_rtk_feedback_bias_gain=" << config.vertical_rtk_feedback_bias_gain << '\n'
      << "vertical_rtk_feedback_attitude_gain=" << config.vertical_rtk_feedback_attitude_gain << '\n'
      << "vertical_rtk_feedback_sigma_dp_m=" << config.vertical_rtk_feedback_sigma_dp_m << '\n'
      << "vertical_rtk_feedback_sigma_baz_mps2=" << config.vertical_rtk_feedback_sigma_baz_mps2 << '\n'
      << "vertical_rtk_feedback_sigma_attitude_rad=" << config.vertical_rtk_feedback_sigma_attitude_rad << '\n'
      << "vertical_rtk_feedback_sigma_vz_mps=" << config.vertical_rtk_feedback_sigma_vz_mps << '\n'
      << "vertical_rtk_feedback_min_interval_s=" << config.vertical_rtk_feedback_min_interval_s << '\n'
      << "vertical_rtk_jump_inside_sigma_scale=" << config.vertical_rtk_jump_inside_sigma_scale << '\n'
      << "vertical_local_recovery_max_iterations=" << config.vertical_local_recovery_max_iterations << '\n'
      << "vertical_local_recovery_max_attitude_delta_rad="
      << config.vertical_local_recovery_max_attitude_delta_rad << '\n'
      << "vertical_local_recovery_max_baz_delta_mps2="
      << config.vertical_local_recovery_max_baz_delta_mps2 << '\n'
      << "vertical_global_vz_window_s=" << config.vertical_global_vz_window_s << '\n'
      << "vertical_global_vz_smooth_window_s=" << config.vertical_global_vz_smooth_window_s << '\n'
      << "vertical_jump_candidate_min_separation_s=" << config.vertical_jump_candidate_min_separation_s << '\n'
      << "vertical_jump_max_candidates_per_segment=" << config.vertical_jump_max_candidates_per_segment << '\n'
      << "vertical_jump_max_selected_points_per_segment="
      << config.vertical_jump_max_selected_points_per_segment << '\n'
      << "vertical_jump_hold_window_s=" << config.vertical_jump_hold_window_s << '\n'
      << "vertical_jump_step_min_threshold_mps=" << config.vertical_jump_step_min_threshold_mps << '\n'
      << "vertical_jump_vz_prior_sigma_mps=" << config.vertical_jump_vz_prior_sigma_mps << '\n'
      << "vertical_jump_window_default_padding_states="
      << config.vertical_jump_window_default_padding_states << '\n'
      << "vertical_jump_window_support_ratio=" << config.vertical_jump_window_support_ratio << '\n'
      << "vertical_jump_window_max_duration_s=" << config.vertical_jump_window_max_duration_s << '\n'
      << "vertical_jump_window_max_points=" << config.vertical_jump_window_max_points << '\n'
      << "vertical_jump_window_tail_target_s=" << config.vertical_jump_window_tail_target_s << '\n'
      << "vertical_jump_window_velocity_smoothness_weight="
      << config.vertical_jump_window_velocity_smoothness_weight << '\n'
      << "vertical_jump_window_height_integral_weight="
      << config.vertical_jump_window_height_integral_weight << '\n'
      << "vertical_jump_window_ref_weight=" << config.vertical_jump_window_ref_weight << '\n'
      << "vertical_jump_window_max_correction_attempts="
      << config.vertical_jump_window_max_correction_attempts << '\n'
      << "vertical_jump_future_trend_window_s=" << config.vertical_jump_future_trend_window_s << '\n'
      << "vertical_jump_future_trend_min_fix_count="
      << config.vertical_jump_future_trend_min_fix_count << '\n'
      << "vertical_jump_future_trend_mean_weight=" << config.vertical_jump_future_trend_mean_weight << '\n'
      << "vertical_jump_future_trend_slope_weight=" << config.vertical_jump_future_trend_slope_weight << '\n'
      << "vertical_interval_feedback_min_duration_s="
      << config.vertical_interval_feedback_min_duration_s << '\n'
      << "vertical_interval_feedback_min_slope_mps="
      << config.vertical_interval_feedback_min_slope_mps << '\n'
      << "vertical_interval_feedback_min_drift_m="
      << config.vertical_interval_feedback_min_drift_m << '\n'
      << "vertical_interval_feedback_min_residual_m="
      << config.vertical_interval_feedback_min_residual_m << '\n'
      << "vertical_interval_feedback_snr_threshold="
      << config.vertical_interval_feedback_snr_threshold << '\n'
      << "vertical_interval_feedback_noise_floor_m="
      << config.vertical_interval_feedback_noise_floor_m << '\n'
      << "vertical_interval_feedback_gain=" << config.vertical_interval_feedback_gain << '\n'
      << "vertical_interval_feedback_max_delta_vz_mps="
      << config.vertical_interval_feedback_max_delta_vz_mps << '\n'
      << "enable_vertical_local_up_anchor_fallback="
      << (config.enable_vertical_local_up_anchor_fallback ? "true" : "false") << '\n'
      << "enable_vertical_inside_bias_adaptation="
      << (config.enable_vertical_inside_bias_adaptation ? "true" : "false") << '\n'
      << "vertical_inside_attitude_gain=" << config.vertical_inside_attitude_gain << '\n'
      << "vertical_inside_max_delta_attitude_rad=" << config.vertical_inside_max_delta_attitude_rad << '\n'
      << "enable_vertical_rtk_seed_pass=" << (config.enable_vertical_rtk_seed_pass ? "true" : "false") << '\n'
      << "enable_body_z_seed_jump_windows=" << (config.enable_body_z_seed_jump_windows ? "true" : "false") << '\n'
      << "body_z_seed_jump_use_fix_only=" << (config.body_z_seed_jump_use_fix_only ? "true" : "false") << '\n'
      << "body_z_jump_pre_post_window_s=" << config.body_z_jump_pre_post_window_s << '\n'
      << "body_z_jump_center_gap_s=" << config.body_z_jump_center_gap_s << '\n'
      << "body_z_jump_velocity_smooth_s=" << config.body_z_jump_velocity_smooth_s << '\n'
      << "body_z_jump_threshold_ratio=" << config.body_z_jump_threshold_ratio << '\n'
      << "body_z_jump_support_ratio=" << config.body_z_jump_support_ratio << '\n'
      << "body_z_jump_redundant_padding_s=" << config.body_z_jump_redundant_padding_s << '\n'
      << "body_z_jump_merge_gap_s=" << config.body_z_jump_merge_gap_s << '\n'
      << "body_z_jump_merge_max_duration_s=" << config.body_z_jump_merge_max_duration_s << '\n'
      << "body_z_jump_min_score_mps=" << config.body_z_jump_min_score_mps << '\n'
      << "body_z_jump_min_separation_s=" << config.body_z_jump_min_separation_s << '\n'
      << "body_z_jump_max_window_duration_s=" << config.body_z_jump_max_window_duration_s << '\n'
      << "body_z_jump_max_levels=" << config.body_z_jump_max_levels << '\n'
      << "body_z_jump_dense_gap_s=" << config.body_z_jump_dense_gap_s << '\n'
      << "body_z_jump_dense_peak_count=" << config.body_z_jump_dense_peak_count << '\n'
      << "body_z_jump_dense_peak_floor_ratio=" << config.body_z_jump_dense_peak_floor_ratio << '\n'
      << "enable_nhc_jump_reference=" << (config.enable_nhc_jump_reference ? "true" : "false") << '\n'
      << "nhc_history_half_life_s=" << config.nhc_history_half_life_s << '\n'
      << "nhc_history_max_age_s=" << config.nhc_history_max_age_s << '\n'
      << "nhc_body_vy_min_threshold_mps=" << config.nhc_body_vy_min_threshold_mps << '\n'
      << "nhc_body_vz_min_threshold_mps=" << config.nhc_body_vz_min_threshold_mps << '\n'
      << "nhc_body_vz_max_threshold_mps=" << config.nhc_body_vz_max_threshold_mps << '\n'
      << "nhc_body_vy_percentile_scale=" << config.nhc_body_vy_percentile_scale << '\n'
      << "nhc_body_vz_percentile_scale=" << config.nhc_body_vz_percentile_scale << '\n'
      << "nhc_jump_min_separation_s=" << config.nhc_jump_min_separation_s << '\n'
      << "nhc_jump_recovery_lookback_s=" << config.nhc_jump_recovery_lookback_s << '\n'
      << "reserve_vertical_velocity_feedback_interface="
      << (config.reserve_vertical_velocity_feedback_interface ? "true" : "false") << '\n'
      << "enable_reweighted_combined_imu_factor="
      << (config.enable_reweighted_combined_imu_factor ? "true" : "false") << '\n'
      << "reweighted_combined_imu_attitude_sigma_rad=" << config.reweighted_combined_imu_attitude_sigma_rad << '\n'
      << "reweighted_combined_imu_specific_force_sigma_mps2="
      << config.reweighted_combined_imu_specific_force_sigma_mps2 << '\n'
      << "reweighted_combined_imu_specific_force_sigma_x_mps2="
      << config.reweighted_combined_imu_specific_force_sigma_x_mps2 << '\n'
      << "reweighted_combined_imu_specific_force_sigma_y_mps2="
      << config.reweighted_combined_imu_specific_force_sigma_y_mps2 << '\n'
      << "reweighted_combined_imu_specific_force_sigma_z_mps2="
      << config.reweighted_combined_imu_specific_force_sigma_z_mps2 << '\n'
      << "reweighted_combined_imu_position_sigma_m=" << config.reweighted_combined_imu_position_sigma_m << '\n'
      << "reweighted_combined_imu_velocity_sigma_mps=" << config.reweighted_combined_imu_velocity_sigma_mps << '\n'
      << "error_process_noise_scale=" << config.error_process_noise_scale << '\n'
      << "tau_acc_bias_s=" << config.tau_acc_bias_s << '\n'
      << "tau_gyro_bias_s=" << config.tau_gyro_bias_s << '\n'
      << "bias_process_noise_acc_scale=" << config.bias_process_noise_acc_scale << '\n'
      << "bias_process_noise_gyro_scale=" << config.bias_process_noise_gyro_scale << '\n'
      << "segment_feedback_attitude_gain=" << config.segment_feedback_attitude_gain << '\n'
      << "segment_feedback_velocity_gain=" << config.segment_feedback_velocity_gain << '\n'
      << "segment_feedback_position_gain=" << config.segment_feedback_position_gain << '\n'
      << "segment_feedback_acc_sigma_mps2=" << config.segment_feedback_acc_sigma_mps2 << '\n'
      << "segment_feedback_gyro_sigma_radps=" << config.segment_feedback_gyro_sigma_radps << '\n'
      << "error_state_frequency_hz=" << config.error_state_frequency_hz << '\n'
      << "error_state_rotation_sigma_rad=" << config.error_state_rotation_sigma_rad << '\n'
      << "error_state_position_sigma_m=" << config.error_state_position_sigma_m << '\n'
      << "error_state_velocity_sigma_mps=" << config.error_state_velocity_sigma_mps << '\n'
      << "error_state_acc_bias_sigma_mps2=" << config.error_state_acc_bias_sigma_mps2 << '\n'
      << "error_state_gyro_bias_sigma_radps=" << config.error_state_gyro_bias_sigma_radps << '\n'
      << "stationary_window_s=" << config.stationary_window_s << '\n'
      << "stationary_acc_tolerance_mps2=" << config.stationary_acc_tolerance_mps2 << '\n'
      << "stationary_gyro_threshold_radps=" << config.stationary_gyro_threshold_radps << '\n'
      << "prefer_imu_initial_yaw=" << (config.prefer_imu_initial_yaw ? "true" : "false") << '\n'
      << "static_alignment_duration_s=" << config.static_alignment_duration_s << '\n'
      << "imu_dual_vector_window_s=" << config.imu_dual_vector_window_s << '\n'
      << "imu_dual_vector_min_sample_count=" << config.imu_dual_vector_min_sample_count << '\n'
      << "imu_dual_vector_min_cross_norm=" << config.imu_dual_vector_min_cross_norm << '\n'
      << "enable_initial_static_zupt_zaru=" << (config.enable_initial_static_zupt_zaru ? "true" : "false") << '\n'
      << "initial_static_zupt_velocity_sigma_mps=" << config.initial_static_zupt_velocity_sigma_mps << '\n'
      << "initial_static_zaru_sigma_radps=" << config.initial_static_zaru_sigma_radps << '\n'
      << "enable_initial_static_zero_specific_force="
      << (config.enable_initial_static_zero_specific_force ? "true" : "false") << '\n'
      << "enable_initial_static_vertical_specific_force="
      << (config.enable_initial_static_vertical_specific_force ? "true" : "false") << '\n'
      << "initial_static_specific_force_sigma_mps2=" << config.initial_static_specific_force_sigma_mps2 << '\n'
      << "initial_static_vertical_specific_force_sigma_mps2="
      << config.initial_static_vertical_specific_force_sigma_mps2 << '\n'
      << "enable_initial_static_subgraph=" << (config.enable_initial_static_subgraph ? "true" : "false") << '\n'
      << "initial_static_state_frequency_hz=" << config.initial_static_state_frequency_hz << '\n'
      << "initial_static_attitude_drift_sigma_rad=" << config.initial_static_attitude_drift_sigma_rad << '\n'
      << "yaw_min_distance_m=" << config.yaw_min_distance_m << '\n'
      << "fallback_initial_yaw_rad=" << config.fallback_initial_yaw_rad << '\n'
    << "early_gnss_relaxation_duration_s=" << config.early_gnss_relaxation_duration_s << '\n'
    << "early_gnss_relaxation_scale=" << config.early_gnss_relaxation_scale << '\n'
    << "position_sigma_floor_m=" << config.position_sigma_floor_m << '\n'
    << "position_sigma_floor_horizontal_m=" << config.position_sigma_floor_horizontal_m << '\n'
    << "position_sigma_floor_up_m=" << config.position_sigma_floor_up_m << '\n'
    << "position_sigma_ceiling_m=" << config.position_sigma_ceiling_m << '\n'
    << "gnss_vertical_sigma_mode=" << ToString(config.gnss_vertical_sigma_mode) << '\n'
    << "gnss_vertical_fixed_sigma_m=" << config.gnss_vertical_fixed_sigma_m << '\n'
    << "gnss_sigma_scale_horizontal=" << config.gnss_sigma_scale_horizontal << '\n'
    << "gnss_sigma_scale_up=" << config.gnss_sigma_scale_up << '\n'
    << "enable_gnss_vertical_drift_model=" << (config.enable_gnss_vertical_drift_model ? "true" : "false") << '\n'
    << "gnss_vertical_drift_reference_mode=" << ToString(config.gnss_vertical_drift_reference_mode) << '\n'
    << "gnss_vertical_drift_window_s=" << config.gnss_vertical_drift_window_s << '\n'
    << "gnss_position_noise_model=" << ToString(config.gnss_position_noise_model) << '\n'
    << "gnss_position_robust_param=" << config.gnss_position_robust_param << '\n'
    << "rtkfix_scale=" << config.rtkfix_scale << '\n'
      << "rtkfloat_scale=" << config.rtkfloat_scale << '\n'
      << "single_scale=" << config.single_scale << '\n'
      << "drop_non_rtkfix=" << (config.drop_non_rtkfix ? "true" : "false") << '\n'
      << "required_best_sol_status_code=" << config.required_best_sol_status_code << '\n'
      << "drop_no_solution=" << (config.drop_no_solution ? "true" : "false") << '\n'
      << "drop_nonfinite_sigma=" << (config.drop_nonfinite_sigma ? "true" : "false") << '\n'
    << "gnss_consistency_gate_mode=" << ToString(config.gnss_consistency_gate_mode) << '\n'
    << "gnss_nis_confidence=" << config.gnss_nis_confidence << '\n'
    << "gnss_axis_sigma_multiple=" << config.gnss_axis_sigma_multiple << '\n'
    << "gnss_consistency_relaxed_threshold_ratio=" << config.gnss_consistency_relaxed_threshold_ratio << '\n'
    << "gnss_consistency_max_scale_horizontal=" << config.gnss_consistency_max_scale_horizontal << '\n'
    << "gnss_consistency_max_scale_up=" << config.gnss_consistency_max_scale_up << '\n'
      << "initial_position_sigma_m=" << config.initial_position_sigma_m << '\n'
      << "initial_roll_pitch_sigma_rad=" << config.initial_roll_pitch_sigma_rad << '\n'
      << "initial_yaw_sigma_rad=" << config.initial_yaw_sigma_rad << '\n'
      << "initial_velocity_sigma_mps=" << config.initial_velocity_sigma_mps << '\n'
      << "lm_lambda_initial=" << config.lm_lambda_initial << '\n'
      << "lm_max_iterations=" << config.lm_max_iterations << '\n';
  return oss.str();
}

}  // namespace offline_lc_minimal
