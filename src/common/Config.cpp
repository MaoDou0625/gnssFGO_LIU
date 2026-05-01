#include "offline_lc_minimal/common/Config.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

std::string Trim(std::string value) {
  auto not_space = [](const unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::string StripQuotes(std::string value) {
  value = Trim(std::move(value));
  if (value.size() >= 2U && ((value.front() == '"' && value.back() == '"') ||
                             (value.front() == '\'' && value.back() == '\''))) {
    return value.substr(1U, value.size() - 2U);
  }
  return value;
}

std::string Lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool ParseBool(const std::string &value) {
  const std::string lowered = Lowercase(value);
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
  const std::string lowered = Lowercase(value);
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
  const std::string lowered = Lowercase(value);
  if (lowered == "none") {
    return GnssConsistencyGateMode::kNone;
  }
  if (lowered == "nis") {
    return GnssConsistencyGateMode::kNis;
  }
  throw std::runtime_error("invalid GNSS consistency gate mode: " + value);
}

GnssVerticalSigmaMode ParseVerticalSigmaMode(const std::string &value) {
  const std::string lowered = Lowercase(value);
  if (lowered == "from_file") {
    return GnssVerticalSigmaMode::kFromFile;
  }
  if (lowered == "fixed") {
    return GnssVerticalSigmaMode::kFixed;
  }
  throw std::runtime_error("invalid GNSS vertical sigma mode: " + value);
}

VerticalConstraintMode ParseVerticalConstraintMode(const std::string &value) {
  const std::string lowered = Lowercase(value);
  if (lowered == "direct_z") {
    return VerticalConstraintMode::kDirectZ;
  }
  if (lowered == "envelope") {
    return VerticalConstraintMode::kEnvelope;
  }
  throw std::runtime_error("invalid vertical constraint mode: " + value);
}

}  // namespace

void ValidateConfig(const OfflineRunnerConfig &config) {
  if (config.state_meas_sync_lower_bound_s > config.state_meas_sync_upper_bound_s) {
    throw std::runtime_error("state_meas_sync_lower_bound_s must be <= state_meas_sync_upper_bound_s");
  }
  if (config.state_meas_sync_lower_bound_s > 0.0 || config.state_meas_sync_upper_bound_s < 0.0) {
    throw std::runtime_error("state_meas_sync bounds must satisfy lower <= 0 <= upper");
  }
  if (config.state_frequency_hz <= 0.0 || config.error_state_frequency_hz <= 0.0) {
    throw std::runtime_error("state and error-state frequencies must be positive");
  }
  if (config.gravity_mps2 <= 0.0 || config.imu_sigma_acc <= 0.0 ||
      config.imu_sigma_gyro <= 0.0 || config.integration_sigma <= 0.0) {
    throw std::runtime_error("IMU noise and gravity settings must be positive");
  }
  if (config.bias_acc_sigma <= 0.0 || config.bias_gyro_sigma <= 0.0 ||
      config.bias_acc_prior_sigma <= 0.0 || config.bias_gyro_prior_sigma <= 0.0) {
    throw std::runtime_error("bias sigmas must be positive");
  }
  if (config.global_acc_bias_tie_sigma_mps2 <= 0.0 ||
      config.global_acc_bias_tie_sigma_xy_mps2 <= 0.0 ||
      config.global_gyro_bias_tie_sigma_radps <= 0.0) {
    throw std::runtime_error("global bias tie sigmas must be positive");
  }
  if (config.vertical_acc_bias_tau_s <= 0.0 || config.vertical_acc_bias_sigma_mps2 < 0.0 ||
      config.vertical_acc_bias_process_noise_scale <= 0.0) {
    throw std::runtime_error("vertical accelerometer bias GM settings are invalid");
  }
  if (config.enable_vertical_acc_bias_gm_process && !config.enable_global_acc_bias) {
    throw std::runtime_error("enable_vertical_acc_bias_gm_process requires enable_global_acc_bias");
  }
  if (config.enable_vertical_acc_bias_gm_process &&
      (config.enable_segment_error_feedback || config.enable_segment_local_error_feedback)) {
    throw std::runtime_error(
      "enable_vertical_acc_bias_gm_process is incompatible with segment error feedback");
  }
  if (config.enable_body_z_jump_detection && !config.enable_gnss) {
    throw std::runtime_error("enable_body_z_jump_detection requires enable_gnss");
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
  if (config.error_process_noise_scale <= 0.0 || config.tau_acc_bias_s <= 0.0 ||
      config.tau_gyro_bias_s <= 0.0 || config.bias_process_noise_acc_scale <= 0.0 ||
      config.bias_process_noise_gyro_scale <= 0.0) {
    throw std::runtime_error("error-state process noise settings must be positive");
  }
  if (config.enable_segment_error_feedback && (config.enable_global_acc_bias || config.enable_global_gyro_bias)) {
    throw std::runtime_error(
      "enable_segment_error_feedback is incompatible with enable_global_acc_bias/enable_global_gyro_bias");
  }
  if (config.enable_segment_local_error_feedback && !config.enable_segment_error_feedback) {
    throw std::runtime_error("enable_segment_local_error_feedback requires enable_segment_error_feedback");
  }
  if (config.segment_feedback_attitude_gain < 0.0 || config.segment_feedback_velocity_gain < 0.0 ||
      config.segment_feedback_position_gain < 0.0 ||
      config.segment_feedback_acc_sigma_mps2 <= 0.0 ||
      config.segment_feedback_gyro_sigma_radps <= 0.0) {
    throw std::runtime_error("segment feedback gains and sigmas are invalid");
  }
  if (config.error_state_rotation_sigma_rad <= 0.0 || config.error_state_position_sigma_m <= 0.0 ||
      config.error_state_velocity_sigma_mps <= 0.0 || config.error_state_acc_bias_sigma_mps2 <= 0.0 ||
      config.error_state_gyro_bias_sigma_radps <= 0.0) {
    throw std::runtime_error("error-state prior sigmas must be positive");
  }
  if (config.stationary_window_s <= 0.0 || config.stationary_acc_tolerance_mps2 <= 0.0 ||
      config.stationary_gyro_threshold_radps <= 0.0) {
    throw std::runtime_error("stationary detector settings must be positive");
  }
  if (config.imu_dual_vector_window_s <= 0.0 || config.imu_dual_vector_min_sample_count <= 0 ||
      config.imu_dual_vector_min_cross_norm <= 0.0) {
    throw std::runtime_error("IMU dual-vector alignment settings must be positive");
  }
  if (config.initial_static_zupt_velocity_sigma_mps <= 0.0 ||
      config.initial_static_zaru_sigma_radps <= 0.0 ||
      config.initial_static_specific_force_sigma_mps2 <= 0.0 ||
      config.initial_static_vertical_specific_force_sigma_mps2 <= 0.0 ||
      config.initial_static_state_frequency_hz <= 0.0 ||
      config.initial_static_attitude_drift_sigma_rad <= 0.0) {
    throw std::runtime_error("initial static constraint settings must be positive");
  }
  if ((config.enable_initial_static_zupt_zaru ||
       config.enable_initial_static_zero_specific_force ||
       config.enable_initial_static_vertical_specific_force ||
       config.enable_initial_static_subgraph) &&
      config.static_alignment_duration_s <= 0.0) {
    throw std::runtime_error("initial static constraints require static_alignment_duration_s > 0");
  }
  if (config.early_gnss_relaxation_duration_s < 0.0 || config.early_gnss_relaxation_scale <= 0.0) {
    throw std::runtime_error("early GNSS relaxation settings are invalid");
  }
  if (config.position_sigma_floor_m < 0.0 ||
      config.position_sigma_floor_horizontal_m < 0.0 ||
      config.position_sigma_floor_up_m < 0.0 ||
      config.position_sigma_ceiling_m <= 0.0 ||
      config.gnss_vertical_fixed_sigma_m <= 0.0 ||
      config.gnss_sigma_scale_horizontal <= 0.0 ||
      config.gnss_sigma_scale_up <= 0.0) {
    throw std::runtime_error("GNSS sigma settings are invalid");
  }
  if (config.vertical_envelope_gate_sigma_multiple <= 0.0 ||
      config.vertical_envelope_min_half_width_m <= 0.0 ||
      config.vertical_envelope_factor_sigma_m <= 0.0) {
    throw std::runtime_error("vertical envelope settings must be positive");
  }
  if (config.enable_vertical_velocity_delta_constraint && !config.enable_body_z_jump_detection) {
    throw std::runtime_error("enable_vertical_velocity_delta_constraint requires enable_body_z_jump_detection");
  }
  if (config.vertical_velocity_delta_acc_sigma_mps2 <= 0.0 ||
      config.vertical_velocity_delta_min_sigma_mps <= 0.0 ||
      config.vertical_velocity_delta_jump_padding_s <= 0.0 ||
      config.vertical_velocity_delta_target_acc_limit_mps2 <= 0.0) {
    throw std::runtime_error("vertical velocity delta settings must be positive");
  }
  if ((config.enable_vertical_jump_masked_imu ||
       config.enable_vertical_jump_velocity_ramp_smoothing ||
       config.enable_vertical_jump_position_ramp_smoothing ||
       config.enable_vertical_jump_velocity_continuity ||
       config.enable_vertical_jump_velocity_context_mean ||
       config.enable_vertical_jump_context_mean_continuity ||
       config.enable_vertical_jump_position_velocity_consistency ||
       config.enable_vertical_jump_velocity_height_slope_constraint) &&
      !config.enable_body_z_jump_detection) {
    throw std::runtime_error("vertical jump constraints require enable_body_z_jump_detection");
  }
  if (config.enable_vertical_jump_masked_imu && config.enable_segment_error_feedback) {
    throw std::runtime_error("enable_vertical_jump_masked_imu requires CombinedImuFactor mode");
  }
  if (config.vertical_jump_masked_imu_padding_s <= 0.0 ||
      config.vertical_jump_velocity_ramp_sigma_mps <= 0.0 ||
      config.vertical_jump_position_ramp_sigma_m <= 0.0 ||
      config.vertical_jump_velocity_continuity_sigma_mps <= 0.0 ||
      !std::isfinite(config.vertical_jump_velocity_context_window_s) ||
      config.vertical_jump_velocity_context_window_s <= 0.0 ||
      !std::isfinite(config.vertical_jump_velocity_context_mean_sigma_mps) ||
      config.vertical_jump_velocity_context_mean_sigma_mps <= 0.0 ||
      !std::isfinite(config.vertical_jump_context_mean_continuity_sigma_mps) ||
      config.vertical_jump_context_mean_continuity_sigma_mps <= 0.0 ||
      config.vertical_jump_position_velocity_consistency_sigma_m <= 0.0 ||
      config.vertical_jump_boundary_position_velocity_consistency_sigma_m <= 0.0 ||
      config.vertical_jump_velocity_height_slope_sigma_mps <= 0.0) {
    throw std::runtime_error("vertical jump settings must be positive");
  }
  if (config.gnss_position_robust_param <= 0.0) {
    throw std::runtime_error("gnss_position_robust_param must be positive");
  }
  if (config.rtkfix_scale <= 0.0 || config.rtkfloat_scale <= 0.0 || config.single_scale <= 0.0) {
    throw std::runtime_error("GNSS fix type scales must be positive");
  }
  if (config.gnss_nis_confidence <= 0.0 || config.gnss_nis_confidence >= 1.0 ||
      config.gnss_axis_sigma_multiple <= 0.0 ||
      config.gnss_consistency_relaxed_threshold_ratio < 0.0 ||
      config.gnss_consistency_relaxed_threshold_ratio > 1.0 ||
      config.gnss_consistency_max_scale_horizontal < 1.0 ||
      config.gnss_consistency_max_scale_up < 1.0) {
    throw std::runtime_error("GNSS consistency gate settings are invalid");
  }
  if (config.initial_position_sigma_m <= 0.0 || config.initial_roll_pitch_sigma_rad <= 0.0 ||
      config.initial_yaw_sigma_rad <= 0.0 || config.initial_velocity_sigma_mps <= 0.0 ||
      config.lm_lambda_initial <= 0.0 || config.lm_max_iterations <= 0) {
    throw std::runtime_error("initial prior and optimizer settings must be positive");
  }
}

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
  } else if (normalized_key == "enable_body_z_jump_detection") {
    config.enable_body_z_jump_detection = ParseBool(normalized_value);
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
  } else if (normalized_key == "vertical_constraint_mode") {
    config.vertical_constraint_mode = ParseVerticalConstraintMode(normalized_value);
  } else if (normalized_key == "vertical_envelope_gate_sigma_multiple") {
    config.vertical_envelope_gate_sigma_multiple = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_envelope_min_half_width_m") {
    config.vertical_envelope_min_half_width_m = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_envelope_factor_sigma_m") {
    config.vertical_envelope_factor_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_velocity_delta_constraint") {
    config.enable_vertical_velocity_delta_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_acc_sigma_mps2") {
    config.vertical_velocity_delta_acc_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_min_sigma_mps") {
    config.vertical_velocity_delta_min_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_jump_padding_s") {
    config.vertical_velocity_delta_jump_padding_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_target_acc_limit_mps2") {
    config.vertical_velocity_delta_target_acc_limit_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_masked_imu") {
    config.enable_vertical_jump_masked_imu = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_masked_imu_padding_s") {
    config.vertical_jump_masked_imu_padding_s = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_velocity_ramp_smoothing") {
    config.enable_vertical_jump_velocity_ramp_smoothing = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_velocity_ramp_sigma_mps") {
    config.vertical_jump_velocity_ramp_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_position_ramp_smoothing") {
    config.enable_vertical_jump_position_ramp_smoothing = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_position_ramp_sigma_m") {
    config.vertical_jump_position_ramp_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_velocity_continuity") {
    config.enable_vertical_jump_velocity_continuity = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_velocity_continuity_sigma_mps") {
    config.vertical_jump_velocity_continuity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_velocity_context_mean") {
    config.enable_vertical_jump_velocity_context_mean = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_velocity_context_window_s") {
    config.vertical_jump_velocity_context_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_velocity_context_mean_sigma_mps") {
    config.vertical_jump_velocity_context_mean_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_context_mean_continuity") {
    config.enable_vertical_jump_context_mean_continuity = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_context_mean_continuity_sigma_mps") {
    config.vertical_jump_context_mean_continuity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_position_velocity_consistency") {
    config.enable_vertical_jump_position_velocity_consistency = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_position_velocity_consistency_sigma_m") {
    config.vertical_jump_position_velocity_consistency_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_boundary_position_velocity_consistency_sigma_m") {
    config.vertical_jump_boundary_position_velocity_consistency_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_velocity_height_slope_constraint") {
    config.enable_vertical_jump_velocity_height_slope_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_velocity_height_slope_sigma_mps") {
    config.vertical_jump_velocity_height_slope_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_sigma_scale_horizontal") {
    config.gnss_sigma_scale_horizontal = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_sigma_scale_up") {
    config.gnss_sigma_scale_up = ParseDouble(normalized_value);
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

OfflineRunnerConfig LoadConfigFile(
    const std::string_view config_path,
    const OfflineRunnerConfig &base_config) {
  OfflineRunnerConfig config = base_config;
  if (config_path.empty()) {
    ValidateConfig(config);
    return config;
  }

  std::ifstream stream{std::string(config_path)};
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open config file: " + std::string(config_path));
  }

  std::string line;
  std::size_t line_number = 0;
  while (std::getline(stream, line)) {
    ++line_number;
    const std::size_t comment_pos = line.find('#');
    if (comment_pos != std::string::npos) {
      line = line.substr(0, comment_pos);
    }
    line = Trim(std::move(line));
    if (line.empty()) {
      continue;
    }

    const std::size_t delimiter_pos = line.find('=');
    if (delimiter_pos == std::string::npos) {
      throw std::runtime_error(
        "invalid config line " + std::to_string(line_number) + ": missing '='");
    }
    OverrideConfigField(
      config,
      Trim(line.substr(0, delimiter_pos)),
      Trim(line.substr(delimiter_pos + 1U)));
  }

  ValidateConfig(config);
  return config;
}

std::string ConfigToString(const OfflineRunnerConfig &config) {
  std::ostringstream oss;
  oss
    << "imu_path=" << config.imu_path << '\n'
    << "gnss_path=" << config.gnss_path << '\n'
    << "output_dir=" << config.output_dir << '\n'
    << "enable_gnss=" << (config.enable_gnss ? "true" : "false") << '\n'
    << "enable_gp_interpolated_gnss=" << (config.enable_gp_interpolated_gnss ? "true" : "false") << '\n'
    << "enable_error_state_feedback=" << (config.enable_error_state_feedback ? "true" : "false") << '\n'
    << "enable_segment_error_feedback=" << (config.enable_segment_error_feedback ? "true" : "false") << '\n'
    << "enable_segment_local_error_feedback=" << (config.enable_segment_local_error_feedback ? "true" : "false") << '\n'
    << "verbose=" << (config.verbose ? "true" : "false") << '\n'
    << "write_debug_csv=" << (config.write_debug_csv ? "true" : "false") << '\n'
    << "write_error_diagnostics=" << (config.write_error_diagnostics ? "true" : "false") << '\n'
    << "write_segment_error_diagnostics=" << (config.write_segment_error_diagnostics ? "true" : "false") << '\n'
    << "write_imu_rate_avp=" << (config.write_imu_rate_avp ? "true" : "false") << '\n'
    << "state_frequency_hz=" << config.state_frequency_hz << '\n'
    << "error_state_frequency_hz=" << config.error_state_frequency_hz << '\n'
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
    << "enable_vertical_acc_bias_gm_process=" << (config.enable_vertical_acc_bias_gm_process ? "true" : "false") << '\n'
    << "vertical_acc_bias_tau_s=" << config.vertical_acc_bias_tau_s << '\n'
    << "vertical_acc_bias_sigma_mps2=" << config.vertical_acc_bias_sigma_mps2 << '\n'
    << "vertical_acc_bias_process_noise_scale=" << config.vertical_acc_bias_process_noise_scale << '\n'
    << "enable_body_z_jump_detection=" << (config.enable_body_z_jump_detection ? "true" : "false") << '\n'
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
    << "enable_initial_static_zero_specific_force=" << (config.enable_initial_static_zero_specific_force ? "true" : "false") << '\n'
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
    << "vertical_constraint_mode=" << ToString(config.vertical_constraint_mode) << '\n'
    << "vertical_envelope_gate_sigma_multiple=" << config.vertical_envelope_gate_sigma_multiple << '\n'
    << "vertical_envelope_min_half_width_m=" << config.vertical_envelope_min_half_width_m << '\n'
    << "vertical_envelope_factor_sigma_m=" << config.vertical_envelope_factor_sigma_m << '\n'
    << "enable_vertical_velocity_delta_constraint="
    << (config.enable_vertical_velocity_delta_constraint ? "true" : "false") << '\n'
    << "vertical_velocity_delta_acc_sigma_mps2=" << config.vertical_velocity_delta_acc_sigma_mps2 << '\n'
    << "vertical_velocity_delta_min_sigma_mps=" << config.vertical_velocity_delta_min_sigma_mps << '\n'
    << "vertical_velocity_delta_jump_padding_s=" << config.vertical_velocity_delta_jump_padding_s << '\n'
    << "vertical_velocity_delta_target_acc_limit_mps2="
    << config.vertical_velocity_delta_target_acc_limit_mps2 << '\n'
    << "enable_vertical_jump_masked_imu=" << (config.enable_vertical_jump_masked_imu ? "true" : "false") << '\n'
    << "vertical_jump_masked_imu_padding_s=" << config.vertical_jump_masked_imu_padding_s << '\n'
    << "enable_vertical_jump_velocity_ramp_smoothing="
    << (config.enable_vertical_jump_velocity_ramp_smoothing ? "true" : "false") << '\n'
    << "vertical_jump_velocity_ramp_sigma_mps=" << config.vertical_jump_velocity_ramp_sigma_mps << '\n'
    << "enable_vertical_jump_position_ramp_smoothing="
    << (config.enable_vertical_jump_position_ramp_smoothing ? "true" : "false") << '\n'
    << "vertical_jump_position_ramp_sigma_m=" << config.vertical_jump_position_ramp_sigma_m << '\n'
    << "enable_vertical_jump_velocity_continuity="
    << (config.enable_vertical_jump_velocity_continuity ? "true" : "false") << '\n'
    << "vertical_jump_velocity_continuity_sigma_mps="
    << config.vertical_jump_velocity_continuity_sigma_mps << '\n'
    << "enable_vertical_jump_velocity_context_mean="
    << (config.enable_vertical_jump_velocity_context_mean ? "true" : "false") << '\n'
    << "vertical_jump_velocity_context_window_s=" << config.vertical_jump_velocity_context_window_s << '\n'
    << "vertical_jump_velocity_context_mean_sigma_mps="
    << config.vertical_jump_velocity_context_mean_sigma_mps << '\n'
    << "enable_vertical_jump_context_mean_continuity="
    << (config.enable_vertical_jump_context_mean_continuity ? "true" : "false") << '\n'
    << "vertical_jump_context_mean_continuity_sigma_mps="
    << config.vertical_jump_context_mean_continuity_sigma_mps << '\n'
    << "enable_vertical_jump_position_velocity_consistency="
    << (config.enable_vertical_jump_position_velocity_consistency ? "true" : "false") << '\n'
    << "vertical_jump_position_velocity_consistency_sigma_m="
    << config.vertical_jump_position_velocity_consistency_sigma_m << '\n'
    << "vertical_jump_boundary_position_velocity_consistency_sigma_m="
    << config.vertical_jump_boundary_position_velocity_consistency_sigma_m << '\n'
    << "enable_vertical_jump_velocity_height_slope_constraint="
    << (config.enable_vertical_jump_velocity_height_slope_constraint ? "true" : "false") << '\n'
    << "vertical_jump_velocity_height_slope_sigma_mps="
    << config.vertical_jump_velocity_height_slope_sigma_mps << '\n'
    << "gnss_sigma_scale_horizontal=" << config.gnss_sigma_scale_horizontal << '\n'
    << "gnss_sigma_scale_up=" << config.gnss_sigma_scale_up << '\n'
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
