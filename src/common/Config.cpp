#include "offline_lc_minimal/common/Config.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/common/Units.h"

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

VerticalEnvelopeCenterSigmaMode ParseVerticalEnvelopeCenterSigmaMode(const std::string &value) {
  const std::string lowered = Lowercase(value);
  if (lowered == "fixed") {
    return VerticalEnvelopeCenterSigmaMode::kFixed;
  }
  if (lowered == "gate_sigma") {
    return VerticalEnvelopeCenterSigmaMode::kGateSigma;
  }
  throw std::runtime_error("invalid vertical envelope center sigma mode: " + value);
}

GnssVerticalReferenceSource ParseGnssVerticalReferenceSource(const std::string &value) {
  const std::string lowered = Lowercase(value);
  if (lowered == "raw_rtk") {
    return GnssVerticalReferenceSource::kRawRtk;
  }
  if (lowered == "stage2_lowpass") {
    return GnssVerticalReferenceSource::kStage2Lowpass;
  }
  if (lowered == "rtk_drift_lowpass") {
    return GnssVerticalReferenceSource::kRtkDriftLowpass;
  }
  throw std::runtime_error("invalid GNSS vertical reference source: " + value);
}

Stage3VerticalReferenceConstraintMode ParseStage3VerticalReferenceConstraintMode(
  const std::string &value) {
  const std::string lowered = Lowercase(value);
  if (lowered == "gaussian") {
    return Stage3VerticalReferenceConstraintMode::kGaussian;
  }
  if (lowered == "envelope") {
    return Stage3VerticalReferenceConstraintMode::kEnvelope;
  }
  throw std::runtime_error("invalid Stage3 vertical reference constraint mode: " + value);
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
  if (!std::isfinite(config.processing_start_time_s) || config.processing_start_time_s < 0.0) {
    throw std::runtime_error("processing_start_time_s must be finite and non-negative");
  }
  if (!std::isfinite(config.processing_end_time_s) || config.processing_end_time_s < 0.0) {
    throw std::runtime_error("processing_end_time_s must be finite and non-negative");
  }
  if (config.processing_start_time_s > 0.0 && config.processing_end_time_s > 0.0 &&
      config.processing_end_time_s <= config.processing_start_time_s) {
    throw std::runtime_error("processing_end_time_s must be after processing_start_time_s");
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
  if (config.initial_static_vertical_bias_global_tie_sigma_mps2 <= 0.0 ||
      config.initial_static_vertical_bias_gm_sigma_mps2 <= 0.0 ||
      config.initial_static_vertical_position_hold_sigma_m <= 0.0 ||
      config.initial_static_position_hold_sigma_m <= 0.0) {
    throw std::runtime_error("initial static vertical bias and position settings must be positive");
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
      config.body_z_jump_merge_max_duration_s < 0.0 ||
      config.body_z_long_bias_min_duration_s < 0.0) {
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
  if (config.enable_initial_yaw_override && !std::isfinite(config.initial_yaw_override_rad)) {
    throw std::runtime_error("initial yaw override must be finite when enabled");
  }
  if (config.stage1_yaw_refinement_max_iterations <= 0 ||
      !std::isfinite(config.stage1_heading_window_s) ||
      !std::isfinite(config.stage1_heading_time_tolerance_s) ||
      !std::isfinite(config.stage1_heading_min_displacement_m) ||
      !std::isfinite(config.stage1_heading_noise_floor_rad) ||
      !std::isfinite(config.stage1_yaw_update_max_rad) ||
      config.stage1_heading_window_s <= 0.0 ||
      config.stage1_heading_time_tolerance_s <= 0.0 ||
      config.stage1_heading_min_displacement_m <= 0.0 ||
      config.stage1_heading_noise_floor_rad <= 0.0 ||
      config.stage1_yaw_update_max_rad <= 0.0) {
    throw std::runtime_error("stage1 yaw refinement settings must be positive and finite");
  }
  if (!std::isfinite(config.stage1_outage_body_y_pre_window_s) ||
      !std::isfinite(config.stage1_outage_body_y_deadband_rmse_multiplier) ||
      !std::isfinite(config.stage1_outage_body_y_min_speed_mps) ||
      !std::isfinite(config.stage1_outage_body_y_min_sigma_mps) ||
      !std::isfinite(config.stage1_outage_body_y_max_sigma_mps) ||
      !std::isfinite(config.stage1_outage_body_y_huber_k) ||
      config.stage1_outage_body_y_pre_window_s <= 0.0 ||
      config.stage1_outage_body_y_deadband_rmse_multiplier <= 0.0 ||
      config.stage1_outage_body_y_min_sample_count <= 0 ||
      config.stage1_outage_body_y_min_speed_mps < 0.0 ||
      config.stage1_outage_body_y_min_sigma_mps <= 0.0 ||
      config.stage1_outage_body_y_max_sigma_mps < config.stage1_outage_body_y_min_sigma_mps ||
      config.stage1_outage_body_y_huber_k <= 0.0) {
    throw std::runtime_error("stage1 outage body-y envelope settings must be positive and finite");
  }
  if (!std::isfinite(config.late_static_window_s) ||
      !std::isfinite(config.late_static_stride_s) ||
      !std::isfinite(config.late_static_min_duration_s) ||
      !std::isfinite(config.late_static_merge_gap_s) ||
      !std::isfinite(config.late_static_vz_sigma_mps) ||
      !std::isfinite(config.late_static_up_sigma_m) ||
      !std::isfinite(config.late_static_height_hold_sigma_m) ||
      config.late_static_window_s <= 0.0 ||
      config.late_static_stride_s <= 0.0 ||
      config.late_static_min_duration_s <= 0.0 ||
      config.late_static_min_rtkfix_samples <= 1 ||
      config.late_static_merge_gap_s < 0.0 ||
      config.late_static_vz_sigma_mps <= 0.0 ||
      config.late_static_up_sigma_m <= 0.0 ||
      config.late_static_height_hold_sigma_m <= 0.0) {
    throw std::runtime_error("late-static detection settings must be positive and finite");
  }
  if (Lowercase(config.late_static_threshold_method) != "log_otsu") {
    throw std::runtime_error("late_static_threshold_method must be log_otsu");
  }
  if (!std::isfinite(config.initial_dynamic_static_search_duration_s) ||
      !std::isfinite(config.initial_dynamic_static_threshold_multiplier) ||
      !std::isfinite(config.initial_dynamic_static_min_duration_s) ||
      !std::isfinite(config.initial_dynamic_static_merge_gap_s) ||
      !std::isfinite(config.initial_dynamic_static_lowpass_blend_s) ||
      !std::isfinite(config.initial_dynamic_static_vz_sigma_mps) ||
      config.initial_dynamic_static_search_duration_s <= 0.0 ||
      config.initial_dynamic_static_threshold_multiplier <= 0.0 ||
      config.initial_dynamic_static_min_duration_s <= 0.0 ||
      config.initial_dynamic_static_merge_gap_s < 0.0 ||
      config.initial_dynamic_static_lowpass_blend_s < 0.0 ||
      config.initial_dynamic_static_vz_sigma_mps <= 0.0) {
    throw std::runtime_error("initial dynamic static detector settings are invalid");
  }
  if ((config.enable_initial_dynamic_static_lowpass_protection ||
       config.enable_initial_dynamic_static_vz_constraint) &&
      !config.enable_initial_dynamic_static_detection) {
    throw std::runtime_error(
      "initial dynamic static lowpass/constraint requires detection to be enabled");
  }
  if (!std::isfinite(config.stage2_attitude_hold_sigma_rad) ||
      !std::isfinite(config.stage2_horizontal_position_hold_sigma_m) ||
      !std::isfinite(config.stage2_horizontal_velocity_hold_sigma_mps) ||
      !std::isfinite(config.stage2_mount_leakage_prior_sigma_rad) ||
      !std::isfinite(config.stage2_vehicle_y_nhc_velocity_sigma_mps) ||
      !std::isfinite(config.stage2_vehicle_y_nhc_displacement_sigma_m) ||
      config.stage2_attitude_hold_sigma_rad <= 0.0 ||
      config.stage2_horizontal_position_hold_sigma_m <= 0.0 ||
      config.stage2_horizontal_velocity_hold_sigma_mps <= 0.0 ||
      config.stage2_mount_leakage_prior_sigma_rad <= 0.0 ||
      config.stage2_vehicle_y_nhc_velocity_sigma_mps <= 0.0 ||
      config.stage2_vehicle_y_nhc_displacement_sigma_m <= 0.0) {
    throw std::runtime_error("stage2 velocity optimization settings must be positive and finite");
  }
  if (!std::isfinite(config.stage2_lowfreq_vertical_reference_cutoff_hz) ||
      config.stage2_lowfreq_vertical_reference_cutoff_hz <= 0.0) {
    throw std::runtime_error(
      "stage2 lowfreq vertical reference settings must be finite and positive");
  }
  if (config.enable_stage2_lowfreq_vertical_reference_optimization &&
      !config.enable_stage2_velocity_optimization) {
    throw std::runtime_error(
      "enable_stage2_lowfreq_vertical_reference_optimization requires enable_stage2_velocity_optimization");
  }
  if (config.enable_stage2_lowfreq_vertical_reference_optimization &&
      config.stage2_lowfreq_vertical_reference_source == GnssVerticalReferenceSource::kRawRtk) {
    throw std::runtime_error(
      "stage2 lowfreq vertical reference optimization requires a non-raw final reference source");
  }
  if (!config.enable_stage2_lowfreq_vertical_reference_optimization &&
      config.gnss_vertical_reference_source == GnssVerticalReferenceSource::kStage2Lowpass) {
    throw std::runtime_error(
      "stage2_lowpass GNSS vertical reference requires stage2 lowfreq vertical reference optimization");
  }
  if (config.enable_stage2_lowfreq_vertical_reference_optimization &&
      config.enable_stage3_vertical_reference_optimization) {
    throw std::runtime_error(
      "stage2 lowfreq vertical reference optimization is incompatible with Stage3 vertical reference optimization");
  }
  if (!std::isfinite(config.stage2_lowfreq_final_dvz_sigma_scale) ||
      config.stage2_lowfreq_final_dvz_sigma_scale <= 0.0) {
    throw std::runtime_error(
      "stage2 lowfreq final DVZ relaxation scale must be finite and positive");
  }
  if (!std::isfinite(config.stage2_lowfreq_final_attitude_hold_sigma_scale) ||
      !std::isfinite(config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale) ||
      !std::isfinite(config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale) ||
      config.stage2_lowfreq_final_attitude_hold_sigma_scale <= 0.0 ||
      config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale <= 0.0 ||
      config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale <= 0.0) {
    throw std::runtime_error(
      "stage2 lowfreq final hold relaxation scales must be finite and positive");
  }
  if (config.enable_stage2_lowfreq_final_dvz_relaxation &&
      !config.enable_stage2_lowfreq_vertical_reference_optimization) {
    throw std::runtime_error(
      "stage2 lowfreq final DVZ relaxation requires stage2 lowfreq vertical reference optimization");
  }
  if (config.enable_stage2_lowfreq_final_dvz_relaxation &&
      config.stage2_lowfreq_final_dvz_sigma_scale < 1.0) {
    throw std::runtime_error(
      "stage2 lowfreq final DVZ relaxation requires a sigma scale >= 1");
  }
  if (config.enable_stage2_lowfreq_final_hold_relaxation &&
      !config.enable_stage2_lowfreq_vertical_reference_optimization) {
    throw std::runtime_error(
      "stage2 lowfreq final hold relaxation requires stage2 lowfreq vertical reference optimization");
  }
  if (config.enable_stage2_lowfreq_final_hold_relaxation &&
      (config.stage2_lowfreq_final_attitude_hold_sigma_scale < 1.0 ||
       config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale < 1.0 ||
       config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale < 1.0)) {
    throw std::runtime_error(
      "stage2 lowfreq final hold relaxation requires sigma scales >= 1");
  }
  if (!std::isfinite(config.stage3_vertical_reference_lowpass_cutoff_hz) ||
      !std::isfinite(config.stage3_vertical_anchor_sigma_m) ||
      config.stage3_vertical_reference_lowpass_cutoff_hz <= 0.0 ||
      config.stage3_vertical_anchor_sigma_m <= 0.0) {
    throw std::runtime_error(
      "stage3 vertical reference settings must be finite and positive");
  }
  if (!std::isfinite(config.stage3_initial_dynamic_static_reference_hold_duration_s) ||
      !std::isfinite(config.stage3_initial_dynamic_static_reference_hold_blend_s) ||
      config.stage3_initial_dynamic_static_reference_hold_duration_s < 0.0 ||
      config.stage3_initial_dynamic_static_reference_hold_blend_s < 0.0 ||
      (config.enable_stage3_initial_dynamic_static_reference_hold &&
       config.stage3_initial_dynamic_static_reference_hold_duration_s <= 0.0)) {
    throw std::runtime_error(
      "stage3 initial dynamic static reference hold settings are invalid");
  }
  if (config.stage3_vertical_reference_constraint_mode ==
      Stage3VerticalReferenceConstraintMode::kEnvelope) {
    if (!std::isfinite(config.stage3_vertical_envelope_half_width_m) ||
        !std::isfinite(config.stage3_vertical_envelope_sigma_m) ||
        config.stage3_vertical_envelope_half_width_m <= 0.0 ||
        config.stage3_vertical_envelope_sigma_m <= 0.0) {
      throw std::runtime_error(
        "stage3 vertical envelope settings must be finite with positive sigmas and half-widths");
    }
    if (config.enable_stage3_vertical_envelope_center_pull) {
      if (!std::isfinite(config.stage3_vertical_envelope_center_sigma_m) ||
          !std::isfinite(config.stage3_vertical_envelope_center_deadband_m) ||
          config.stage3_vertical_envelope_center_sigma_m <= 0.0 ||
          config.stage3_vertical_envelope_center_deadband_m < 0.0) {
        throw std::runtime_error(
          "stage3 vertical envelope center-pull settings must be finite with positive sigma");
      }
      if (config.stage3_vertical_envelope_center_deadband_m >=
          config.stage3_vertical_envelope_half_width_m) {
        throw std::runtime_error(
          "stage3 vertical envelope center deadband must be smaller than the half-width");
      }
    }
  }
  if (config.enable_stage3_vertical_reference_optimization &&
      !config.enable_stage2_velocity_optimization) {
    throw std::runtime_error(
      "enable_stage3_vertical_reference_optimization requires enable_stage2_velocity_optimization");
  }
  if (config.stage3_disable_stage2_vehicle_nhc_constraint &&
      !config.enable_stage3_vertical_reference_optimization) {
    throw std::runtime_error(
      "stage3_disable_stage2_vehicle_nhc_constraint requires Stage3 vertical reference optimization");
  }
  if (config.initial_static_zupt_velocity_sigma_mps <= 0.0 ||
      config.initial_static_zaru_sigma_radps <= 0.0 ||
      config.initial_static_specific_force_sigma_mps2 <= 0.0 ||
      config.initial_static_vertical_specific_force_sigma_mps2 <= 0.0 ||
      config.initial_static_rtk_height_reference_sigma_m <= 0.0 ||
      config.initial_static_rtk_height_reference_min_sample_count <= 0 ||
      config.initial_static_state_frequency_hz <= 0.0 ||
      config.initial_static_attitude_drift_sigma_rad <= 0.0) {
    throw std::runtime_error("initial static constraint settings must be positive");
  }
  if ((config.enable_initial_static_zupt_zaru ||
       config.enable_initial_static_zero_specific_force ||
       config.enable_initial_static_vertical_specific_force ||
       config.enable_initial_static_vertical_bias_soft_prior ||
       config.enable_initial_static_vertical_bias_gm_tightening ||
       config.enable_initial_static_vertical_position_hold ||
       config.enable_initial_static_position_hold ||
       config.enable_initial_static_rtk_height_reference ||
       config.enable_initial_static_subgraph) &&
      config.static_alignment_duration_s <= 0.0) {
    throw std::runtime_error("initial static constraints require static_alignment_duration_s > 0");
  }
  if (config.enable_initial_static_vertical_bias_soft_prior &&
      (!config.enable_global_acc_bias || !config.enable_vertical_acc_bias_gm_process)) {
    throw std::runtime_error(
      "initial static vertical bias soft prior requires global accelerometer bias and vertical GM bias process");
  }
  if (config.enable_initial_static_vertical_bias_soft_prior && !config.enable_initial_static_subgraph) {
    throw std::runtime_error("initial static vertical bias soft prior requires initial static subgraph");
  }
  if (config.enable_initial_static_vertical_bias_gm_tightening &&
      (!config.enable_initial_static_subgraph || !config.enable_vertical_acc_bias_gm_process)) {
    throw std::runtime_error(
      "initial static vertical bias GM tightening requires initial static subgraph and vertical GM bias process");
  }
  if (config.enable_initial_static_vertical_position_hold && !config.enable_initial_static_subgraph) {
    throw std::runtime_error("initial static vertical position hold requires initial static subgraph");
  }
  if (config.enable_initial_static_position_hold && !config.enable_initial_static_subgraph) {
    throw std::runtime_error("initial static position hold requires initial static subgraph");
  }
  if (config.enable_initial_static_rtk_height_reference &&
      (!config.enable_initial_static_subgraph || !config.enable_gnss)) {
    throw std::runtime_error("initial static RTK height reference requires initial static subgraph and GNSS");
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
      config.vertical_envelope_factor_sigma_m <= 0.0 ||
      config.vertical_envelope_center_sigma_m <= 0.0 ||
      config.vertical_envelope_center_deadband_m < 0.0) {
    throw std::runtime_error("vertical envelope settings are invalid");
  }
  if (config.vertical_envelope_center_deadband_m >= config.vertical_envelope_min_half_width_m) {
    throw std::runtime_error("vertical envelope center deadband must be smaller than the minimum half-width");
  }
  if (!std::isfinite(config.rtk_vertical_drift_correlation_time_s) ||
      !std::isfinite(config.rtk_vertical_drift_sigma_m) ||
      !std::isfinite(config.rtk_vertical_white_noise_sigma_m) ||
      !std::isfinite(config.rtk_vertical_drift_huber_sigma_m) ||
      !std::isfinite(config.rtk_vertical_drift_gate_weight_floor) ||
      !std::isfinite(config.rtk_outage_preoutage_fence_stride_s) ||
      !std::isfinite(config.rtk_outage_preoutage_fence_up_sigma_m) ||
      !std::isfinite(config.rtk_outage_preoutage_fence_vz_sigma_mps) ||
      !std::isfinite(config.rtk_vertical_drift_max_abs_correction_m) ||
      !std::isfinite(config.rtk_vertical_drift_convergence_threshold_m) ||
      !std::isfinite(config.rtk_vertical_lowpass_reference_cutoff_hz) ||
      config.rtk_vertical_drift_correlation_time_s <= 0.0 ||
      config.rtk_vertical_drift_sigma_m <= 0.0 ||
      config.rtk_vertical_white_noise_sigma_m <= 0.0 ||
      config.rtk_vertical_drift_huber_sigma_m <= 0.0 ||
      config.rtk_vertical_drift_gate_weight_floor <= 0.0 ||
      config.rtk_vertical_drift_gate_weight_floor > 1.0 ||
      config.rtk_outage_causal_reference_max_prefix_runs < 0 ||
      config.rtk_outage_preoutage_fence_stride_s <= 0.0 ||
      config.rtk_outage_preoutage_fence_up_sigma_m <= 0.0 ||
      config.rtk_outage_preoutage_fence_vz_sigma_mps <= 0.0 ||
      config.rtk_vertical_drift_max_abs_correction_m <= 0.0 ||
      config.rtk_vertical_drift_convergence_threshold_m <= 0.0 ||
      config.rtk_vertical_lowpass_reference_cutoff_hz <= 0.0 ||
      config.rtk_vertical_drift_outer_iterations < 0) {
    throw std::runtime_error("RTK vertical drift reference settings must be positive");
  }
  if (config.rtk_outage_segmented_batch_max_outages < 0) {
    throw std::runtime_error("RTK outage segmented batch settings must be non-negative");
  }
  if (config.rtk_outage_recovery_reference_min_fix_samples <= 0 ||
      !std::isfinite(config.rtk_outage_recovery_reference_max_duration_s) ||
      !std::isfinite(config.rtk_outage_boundary_up_sigma_m) ||
      !std::isfinite(config.rtk_outage_boundary_vz_sigma_mps) ||
      !std::isfinite(config.rtk_outage_boundary_baz_sigma_mps2) ||
      !std::isfinite(config.rtk_outage_baz_continuity_break_delta_threshold_mps2) ||
      config.rtk_outage_recovery_reference_max_duration_s <= 0.0 ||
      config.rtk_outage_boundary_up_sigma_m <= 0.0 ||
      config.rtk_outage_boundary_vz_sigma_mps <= 0.0 ||
      config.rtk_outage_boundary_baz_sigma_mps2 <= 0.0 ||
      config.rtk_outage_baz_continuity_break_delta_threshold_mps2 <= 0.0) {
    throw std::runtime_error("RTK outage boundary constraint settings must be positive");
  }
  if (config.enable_rtk_vertical_drift_reference) {
    if (!config.enable_gnss ||
        config.vertical_constraint_mode != VerticalConstraintMode::kEnvelope ||
        !config.enable_vertical_envelope_center_pull ||
        !config.rtk_vertical_drift_use_for_center_pull) {
      throw std::runtime_error(
        "RTK vertical drift reference requires GNSS envelope center-pull constraints");
    }
    if (config.rtk_vertical_drift_use_for_envelope_gate) {
      throw std::runtime_error(
        "rtk_vertical_drift_use_for_envelope_gate is not supported; raw RTK gate is preserved");
    }
    if (!config.enable_initial_static_rtk_height_reference ||
        config.static_alignment_duration_s <= 0.0) {
      throw std::runtime_error(
        "RTK vertical drift reference requires the initial static RTK height reference");
    }
  }
  if (config.enable_rtk_vertical_lowpass_reference &&
      (!config.enable_rtk_vertical_drift_reference ||
       !config.rtk_vertical_drift_use_for_center_pull)) {
    throw std::runtime_error(
      "RTK vertical lowpass reference requires RTK vertical drift center-pull reference");
  }
  if ((config.gnss_vertical_reference_source == GnssVerticalReferenceSource::kRtkDriftLowpass ||
       config.stage2_lowfreq_vertical_reference_source == GnssVerticalReferenceSource::kRtkDriftLowpass) &&
      (!config.enable_rtk_vertical_drift_reference ||
       !config.enable_rtk_vertical_lowpass_reference)) {
    throw std::runtime_error(
      "RTK drift lowpass GNSS vertical reference requires RTK vertical drift lowpass reference");
  }
  if (config.enable_rtk_outage_smoothing && !config.enable_gnss) {
    throw std::runtime_error("RTK outage smoothing requires GNSS input");
  }
  if (!std::isfinite(config.rtk_outage_min_gap_s) ||
      !std::isfinite(config.rtk_outage_position_ramp_sigma_m) ||
      !std::isfinite(config.rtk_outage_velocity_delta_sigma_mps) ||
      !std::isfinite(config.rtk_outage_velocity_delta_target_acc_limit_mps2) ||
      !std::isfinite(config.rtk_outage_attitude_guard_duration_s) ||
      !std::isfinite(config.rtk_outage_absolute_attitude_sigma_rad) ||
      !std::isfinite(config.rtk_outage_relative_attitude_sigma_rad) ||
      !std::isfinite(config.rtk_outage_velocity_delta_3d_sigma_mps) ||
      config.rtk_outage_min_gap_s <= 0.0 ||
      config.rtk_outage_position_ramp_sigma_m <= 0.0 ||
      config.rtk_outage_velocity_delta_sigma_mps <= 0.0 ||
      config.rtk_outage_velocity_delta_target_acc_limit_mps2 <= 0.0 ||
      config.rtk_outage_attitude_guard_duration_s < 0.0 ||
      config.rtk_outage_absolute_attitude_sigma_rad <= 0.0 ||
      config.rtk_outage_relative_attitude_sigma_rad <= 0.0 ||
      config.rtk_outage_velocity_delta_3d_sigma_mps <= 0.0 ||
      config.rtk_outage_position_ramp_stride <= 0) {
    throw std::runtime_error("RTK outage smoothing settings must be positive");
  }
  if (config.enable_rtk_velocity_constraint && !config.enable_gnss) {
    throw std::runtime_error("RTK velocity constraints require GNSS input");
  }
  if (!std::isfinite(config.rtk_velocity_window_s) ||
      !std::isfinite(config.rtk_velocity_horizontal_sigma_mps) ||
      config.rtk_velocity_window_s <= 0.0 ||
      config.rtk_velocity_horizontal_sigma_mps <= 0.0) {
    throw std::runtime_error("RTK velocity constraint settings must be positive");
  }
  if (config.enable_vertical_velocity_delta_constraint && !config.enable_body_z_jump_detection) {
    throw std::runtime_error("enable_vertical_velocity_delta_constraint requires enable_body_z_jump_detection");
  }
  if (config.vertical_velocity_delta_acc_sigma_mps2 <= 0.0 ||
      config.vertical_velocity_delta_min_sigma_mps <= 0.0 ||
      config.vertical_velocity_delta_jump_padding_s <= 0.0 ||
      config.vertical_velocity_delta_target_acc_limit_mps2 <= 0.0 ||
      config.vertical_velocity_delta_bias_sigma_mps2 <= 0.0 ||
      config.vertical_velocity_delta_attitude_sigma_rad <= 0.0 ||
      config.vertical_velocity_delta_sigma_floor_mps <= 0.0 ||
      config.vertical_velocity_delta_sigma_ceiling_mps <= 0.0 ||
      !std::isfinite(config.vertical_velocity_delta_sigma_scale) ||
      config.vertical_velocity_delta_sigma_scale <= 0.0 ||
      !std::isfinite(config.vertical_velocity_delta_context_normal_sigma_scale) ||
      !std::isfinite(config.vertical_velocity_delta_context_rough_sigma_scale) ||
      !std::isfinite(config.vertical_velocity_delta_context_outage_sigma_scale) ||
      !std::isfinite(config.vertical_velocity_delta_context_jump_sigma_scale) ||
      !std::isfinite(config.vertical_velocity_delta_context_jump_extra_padding_s) ||
      config.vertical_velocity_delta_context_normal_sigma_scale <= 0.0 ||
      config.vertical_velocity_delta_context_rough_sigma_scale <= 0.0 ||
      config.vertical_velocity_delta_context_outage_sigma_scale <= 0.0 ||
      config.vertical_velocity_delta_context_jump_sigma_scale <= 0.0 ||
      config.vertical_velocity_delta_context_jump_extra_padding_s < 0.0 ||
      config.vertical_motion_adaptive_outer_iterations < 0 ||
      config.vertical_motion_adaptive_convergence_score_epsilon <= 0.0 ||
      config.vertical_motion_adaptive_stability_window_s <= 0.0 ||
      config.vertical_motion_adaptive_static_horizontal_speed_rms_mps <= 0.0 ||
      config.vertical_motion_adaptive_static_vz_rms_mps <= 0.0 ||
      config.vertical_motion_adaptive_static_target_acc_rms_mps2 <= 0.0 ||
      config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 <= 0.0 ||
      config.vertical_motion_adaptive_static_attitude_sigma_rad <= 0.0 ||
      config.vertical_motion_adaptive_static_sigma_floor_mps <= 0.0 ||
      config.vertical_motion_adaptive_static_sigma_ceiling_mps <= 0.0 ||
      config.vertical_motion_adaptive_static_baz_gm_sigma_mps2 <= 0.0) {
    throw std::runtime_error("vertical velocity delta settings must be positive");
  }
  if (config.vertical_velocity_delta_sigma_ceiling_mps < config.vertical_velocity_delta_sigma_floor_mps) {
    throw std::runtime_error("vertical velocity delta sigma ceiling must be >= floor");
  }
  if (config.vertical_motion_adaptive_static_sigma_ceiling_mps <
      config.vertical_motion_adaptive_static_sigma_floor_mps) {
    throw std::runtime_error("adaptive vertical velocity delta sigma ceiling must be >= floor");
  }
  if (!std::isfinite(config.vertical_position_velocity_consistency_sigma_m) ||
      config.vertical_position_velocity_consistency_sigma_m <= 0.0) {
    throw std::runtime_error("vertical position-velocity consistency settings must be positive");
  }
  if (!std::isfinite(config.vertical_position_velocity_window_s) ||
      !std::isfinite(config.vertical_position_velocity_window_stride_s) ||
      !std::isfinite(config.vertical_position_velocity_window_sigma_m) ||
      config.vertical_position_velocity_window_s <= 0.0 ||
      config.vertical_position_velocity_window_stride_s <= 0.0 ||
      config.vertical_position_velocity_window_sigma_m <= 0.0) {
    throw std::runtime_error("vertical position-velocity window consistency settings must be positive");
  }
  if (!std::isfinite(config.attitude_reference_sigma_rad) ||
      !std::isfinite(config.attitude_reference_relative_yaw_sigma_rad) ||
      config.attitude_reference_sigma_rad <= 0.0 ||
      config.attitude_reference_relative_yaw_sigma_rad <= 0.0) {
    throw std::runtime_error("attitude reference settings must be positive");
  }
  if (config.enable_attitude_reference_constraint && !config.enable_body_z_jump_detection) {
    throw std::runtime_error("attitude reference constraint requires body-z jump detection seed optimization");
  }
  if (config.enable_body_z_nhc_global_weak_constraint &&
      !config.enable_body_z_nhc_constraint) {
    throw std::runtime_error("enable_body_z_nhc_global_weak_constraint requires enable_body_z_nhc_constraint");
  }
  if (config.enable_body_z_nhc_constraint &&
      !config.enable_body_z_jump_detection &&
      !config.enable_body_z_nhc_global_weak_constraint) {
    throw std::runtime_error("jump-only body-z NHC requires enable_body_z_jump_detection");
  }
  if (!std::isfinite(config.body_z_nhc_jump_padding_s) ||
      config.body_z_nhc_jump_padding_s < 0.0 ||
      !std::isfinite(config.body_z_nhc_merge_gap_s) ||
      config.body_z_nhc_merge_gap_s < 0.0 ||
      !std::isfinite(config.body_z_nhc_min_window_s) ||
      config.body_z_nhc_min_window_s <= 0.0 ||
      !std::isfinite(config.body_z_nhc_jump_velocity_sigma_mps) ||
      config.body_z_nhc_jump_velocity_sigma_mps <= 0.0 ||
      !std::isfinite(config.body_z_nhc_jump_displacement_sigma_m) ||
      config.body_z_nhc_jump_displacement_sigma_m <= 0.0 ||
      !std::isfinite(config.body_z_nhc_global_window_s) ||
      config.body_z_nhc_global_window_s <= 0.0 ||
      !std::isfinite(config.body_z_nhc_global_stride_s) ||
      config.body_z_nhc_global_stride_s <= 0.0 ||
      !std::isfinite(config.body_z_nhc_global_velocity_sigma_mps) ||
      config.body_z_nhc_global_velocity_sigma_mps <= 0.0 ||
      !std::isfinite(config.body_z_nhc_global_displacement_sigma_m) ||
      config.body_z_nhc_global_displacement_sigma_m <= 0.0) {
    throw std::runtime_error("body-z NHC settings are invalid");
  }
  if (!std::isfinite(config.body_z_nhc_horizontal_leakage_min_speed_mps) ||
      config.body_z_nhc_horizontal_leakage_min_speed_mps < 0.0 ||
      config.body_z_nhc_horizontal_leakage_min_sample_count <= 0 ||
      !std::isfinite(config.body_z_nhc_horizontal_leakage_huber_sigma_mps) ||
      config.body_z_nhc_horizontal_leakage_huber_sigma_mps <= 0.0 ||
      !std::isfinite(config.body_z_nhc_horizontal_leakage_max_abs_coeff_rad) ||
      config.body_z_nhc_horizontal_leakage_max_abs_coeff_rad <= 0.0 ||
      !std::isfinite(config.body_z_nhc_horizontal_leakage_guard_s) ||
      config.body_z_nhc_horizontal_leakage_guard_s < 0.0) {
    throw std::runtime_error("body-z NHC horizontal leakage settings are invalid");
  }
  if (config.enable_body_z_nhc_horizontal_leakage_correction &&
      !config.enable_body_z_nhc_constraint) {
    throw std::runtime_error(
      "enable_body_z_nhc_horizontal_leakage_correction requires enable_body_z_nhc_constraint");
  }
  if (config.body_z_nhc_global_window_s < config.body_z_nhc_min_window_s) {
    throw std::runtime_error("body-z NHC global window must be at least the minimum window duration");
  }
  if (config.enable_body_z_nhc_strict_effective_weighting &&
      config.enable_body_z_nhc_global_weak_constraint &&
      config.body_z_nhc_global_stride_s + 1.0e-12 < config.body_z_nhc_global_window_s) {
    throw std::runtime_error(
      "strict body-z NHC effective weighting requires global stride to be at least global window");
  }
  if ((config.enable_vertical_jump_masked_imu ||
       config.enable_vertical_jump_impulse ||
       config.enable_vertical_jump_bias ||
       config.enable_vertical_jump_segmented_bias ||
       config.enable_vertical_jump_spectral_bias_relaxation ||
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
  if (config.enable_vertical_jump_impulse && config.enable_segment_error_feedback) {
    throw std::runtime_error("enable_vertical_jump_impulse requires CombinedImuFactor mode");
  }
  if (config.enable_vertical_jump_bias && config.enable_segment_error_feedback) {
    throw std::runtime_error("enable_vertical_jump_bias requires CombinedImuFactor mode");
  }
  if (config.enable_vertical_jump_impulse && config.enable_vertical_jump_masked_imu) {
    throw std::runtime_error("enable_vertical_jump_impulse is incompatible with enable_vertical_jump_masked_imu");
  }
  if (config.enable_vertical_jump_bias &&
      (config.enable_vertical_jump_impulse || config.enable_vertical_jump_masked_imu)) {
    throw std::runtime_error("enable_vertical_jump_bias is incompatible with impulse or masked IMU modes");
  }
  if (config.enable_vertical_jump_segmented_bias && !config.enable_vertical_jump_bias) {
    throw std::runtime_error("enable_vertical_jump_segmented_bias requires enable_vertical_jump_bias");
  }
  if (config.vertical_jump_masked_imu_padding_s <= 0.0 ||
      !std::isfinite(config.vertical_jump_impulse_prior_sigma_mps) ||
      config.vertical_jump_impulse_prior_sigma_mps <= 0.0 ||
      !std::isfinite(config.vertical_jump_impulse_velocity_sigma_mps) ||
      config.vertical_jump_impulse_velocity_sigma_mps <= 0.0 ||
      !std::isfinite(config.vertical_jump_impulse_position_velocity_sigma_m) ||
      config.vertical_jump_impulse_position_velocity_sigma_m <= 0.0 ||
      !std::isfinite(config.vertical_jump_bias_padding_s) ||
      config.vertical_jump_bias_padding_s < 0.0 ||
      !std::isfinite(config.vertical_jump_bias_prior_sigma_mps2) ||
      config.vertical_jump_bias_prior_sigma_mps2 <= 0.0 ||
      !std::isfinite(config.vertical_jump_bias_velocity_sigma_mps) ||
      config.vertical_jump_bias_velocity_sigma_mps <= 0.0 ||
      !std::isfinite(config.vertical_jump_bias_position_velocity_sigma_m) ||
      config.vertical_jump_bias_position_velocity_sigma_m <= 0.0 ||
      !std::isfinite(config.vertical_jump_segmented_bias_min_segment_s) ||
      config.vertical_jump_segmented_bias_min_segment_s <= 0.0 ||
      config.vertical_jump_segmented_bias_max_segments <= 0 ||
      !std::isfinite(config.vertical_jump_segmented_bias_slope_merge_threshold_mps2) ||
      config.vertical_jump_segmented_bias_slope_merge_threshold_mps2 < 0.0 ||
      !std::isfinite(config.vertical_jump_bias_highfreq_sigma_scale) ||
      config.vertical_jump_bias_highfreq_sigma_scale < 0.0 ||
      !std::isfinite(config.vertical_jump_bias_highfreq_sigma_max_mps) ||
      config.vertical_jump_bias_highfreq_sigma_max_mps <= 0.0 ||
      !std::isfinite(config.vertical_jump_spectral_window_s) ||
      config.vertical_jump_spectral_window_s <= 0.0 ||
      !std::isfinite(config.vertical_jump_spectral_stride_s) ||
      config.vertical_jump_spectral_stride_s <= 0.0 ||
      !std::isfinite(config.vertical_jump_spectral_reference_margin_s) ||
      config.vertical_jump_spectral_reference_margin_s < 0.0 ||
      config.vertical_jump_spectral_min_reference_window_count <= 0 ||
      !std::isfinite(config.vertical_jump_spectral_response_trigger_ratio) ||
      config.vertical_jump_spectral_response_trigger_ratio < 1.0 ||
      !std::isfinite(config.vertical_jump_spectral_response_full_ratio) ||
      config.vertical_jump_spectral_response_full_ratio <=
        config.vertical_jump_spectral_response_trigger_ratio ||
      !std::isfinite(config.vertical_jump_spectral_bias_prior_max_sigma_mps2) ||
      config.vertical_jump_spectral_bias_prior_max_sigma_mps2 <= 0.0 ||
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
  if (config.enable_vertical_jump_spectral_bias_relaxation) {
    if (!config.enable_vertical_jump_bias) {
      throw std::runtime_error(
        "enable_vertical_jump_spectral_bias_relaxation requires enable_vertical_jump_bias");
    }
    if (config.vertical_jump_spectral_bias_prior_max_sigma_mps2 <
        config.vertical_jump_bias_prior_sigma_mps2) {
      throw std::runtime_error(
        "vertical jump spectral bias prior max sigma must be >= vertical_jump_bias_prior_sigma_mps2");
    }
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
  } else if (normalized_key == "processing_start_time_s") {
    config.processing_start_time_s = ParseDouble(normalized_value);
  } else if (normalized_key == "processing_end_time_s") {
    config.processing_end_time_s = ParseDouble(normalized_value);
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
  } else if (normalized_key == "global_acc_bias_tie_sigma_ug") {
    config.global_acc_bias_tie_sigma_mps2 = MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "global_acc_bias_tie_sigma_mps2") {
    config.global_acc_bias_tie_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "global_acc_bias_tie_sigma_xy_ug") {
    config.global_acc_bias_tie_sigma_xy_mps2 = MicroGToMps2(ParseDouble(normalized_value));
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
  } else if (normalized_key == "vertical_acc_bias_sigma_ug") {
    config.vertical_acc_bias_sigma_mps2 = MicroGToMps2(ParseDouble(normalized_value));
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
    config.body_z_long_bias_min_duration_s = config.body_z_jump_merge_max_duration_s;
  } else if (normalized_key == "body_z_long_bias_min_duration_s") {
    config.body_z_long_bias_min_duration_s = ParseDouble(normalized_value);
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
  } else if (normalized_key == "enable_initial_yaw_override") {
    config.enable_initial_yaw_override = ParseBool(normalized_value);
  } else if (normalized_key == "initial_yaw_override_rad") {
    config.initial_yaw_override_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage1_yaw_refinement") {
    config.enable_stage1_yaw_refinement = ParseBool(normalized_value);
  } else if (normalized_key == "stage1_yaw_refinement_max_iterations") {
    config.stage1_yaw_refinement_max_iterations = ParseInt(normalized_value);
  } else if (normalized_key == "stage1_heading_window_s") {
    config.stage1_heading_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_heading_time_tolerance_s") {
    config.stage1_heading_time_tolerance_s = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_heading_min_displacement_m") {
    config.stage1_heading_min_displacement_m = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_heading_noise_floor_rad") {
    config.stage1_heading_noise_floor_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_yaw_update_max_rad") {
    config.stage1_yaw_update_max_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage1_outage_body_y_envelope") {
    config.enable_stage1_outage_body_y_envelope = ParseBool(normalized_value);
  } else if (normalized_key == "stage1_outage_body_y_pre_window_s") {
    config.stage1_outage_body_y_pre_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_outage_body_y_deadband_rmse_multiplier") {
    config.stage1_outage_body_y_deadband_rmse_multiplier = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_outage_body_y_min_sample_count") {
    config.stage1_outage_body_y_min_sample_count = ParseInt(normalized_value);
  } else if (normalized_key == "stage1_outage_body_y_min_speed_mps") {
    config.stage1_outage_body_y_min_speed_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_outage_body_y_min_sigma_mps") {
    config.stage1_outage_body_y_min_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_outage_body_y_max_sigma_mps") {
    config.stage1_outage_body_y_max_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "stage1_outage_body_y_huber_k") {
    config.stage1_outage_body_y_huber_k = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_late_static_detection") {
    config.enable_late_static_detection = ParseBool(normalized_value);
  } else if (normalized_key == "late_static_window_s") {
    config.late_static_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "late_static_stride_s") {
    config.late_static_stride_s = ParseDouble(normalized_value);
  } else if (normalized_key == "late_static_min_duration_s") {
    config.late_static_min_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "late_static_threshold_method") {
    config.late_static_threshold_method = StripQuotes(std::string(normalized_value));
  } else if (normalized_key == "late_static_min_rtkfix_samples") {
    config.late_static_min_rtkfix_samples = ParseInt(normalized_value);
  } else if (normalized_key == "late_static_merge_gap_s") {
    config.late_static_merge_gap_s = ParseDouble(normalized_value);
  } else if (normalized_key == "late_static_exclude_initial_static") {
    config.late_static_exclude_initial_static = ParseBool(normalized_value);
  } else if (normalized_key == "late_static_exclude_rtk_outage") {
    config.late_static_exclude_rtk_outage = ParseBool(normalized_value);
  } else if (normalized_key == "late_static_vz_sigma_mps") {
    config.late_static_vz_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "late_static_up_sigma_m") {
    config.late_static_up_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "late_static_height_hold_sigma_m") {
    config.late_static_height_hold_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_dynamic_static_detection") {
    config.enable_initial_dynamic_static_detection = ParseBool(normalized_value);
  } else if (normalized_key == "initial_dynamic_static_search_duration_s") {
    config.initial_dynamic_static_search_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_dynamic_static_threshold_multiplier") {
    config.initial_dynamic_static_threshold_multiplier = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_dynamic_static_min_duration_s") {
    config.initial_dynamic_static_min_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_dynamic_static_merge_gap_s") {
    config.initial_dynamic_static_merge_gap_s = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_dynamic_static_lowpass_protection") {
    config.enable_initial_dynamic_static_lowpass_protection = ParseBool(normalized_value);
  } else if (normalized_key == "initial_dynamic_static_lowpass_blend_s") {
    config.initial_dynamic_static_lowpass_blend_s = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_dynamic_static_vz_constraint") {
    config.enable_initial_dynamic_static_vz_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "initial_dynamic_static_vz_sigma_mps") {
    config.initial_dynamic_static_vz_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage2_velocity_optimization") {
    config.enable_stage2_velocity_optimization = ParseBool(normalized_value);
  } else if (normalized_key == "enable_stage2_vehicle_nhc_constraint") {
    config.enable_stage2_vehicle_nhc_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "stage2_attitude_hold_sigma_rad") {
    config.stage2_attitude_hold_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_horizontal_position_hold_sigma_m") {
    config.stage2_horizontal_position_hold_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_horizontal_velocity_hold_sigma_mps") {
    config.stage2_horizontal_velocity_hold_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_mount_leakage_prior_sigma_rad") {
    config.stage2_mount_leakage_prior_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_vehicle_y_nhc_velocity_sigma_mps") {
    config.stage2_vehicle_y_nhc_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_vehicle_y_nhc_displacement_sigma_m") {
    config.stage2_vehicle_y_nhc_displacement_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage2_lowfreq_vertical_reference_optimization") {
    config.enable_stage2_lowfreq_vertical_reference_optimization =
      ParseBool(normalized_value);
  } else if (normalized_key == "stage2_lowfreq_vertical_reference_cutoff_hz") {
    config.stage2_lowfreq_vertical_reference_cutoff_hz =
      ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_lowfreq_vertical_reference_source") {
    config.stage2_lowfreq_vertical_reference_source =
      ParseGnssVerticalReferenceSource(normalized_value);
  } else if (normalized_key == "enable_stage2_lowfreq_final_dvz_relaxation") {
    config.enable_stage2_lowfreq_final_dvz_relaxation =
      ParseBool(normalized_value);
  } else if (normalized_key == "stage2_lowfreq_final_dvz_sigma_scale") {
    config.stage2_lowfreq_final_dvz_sigma_scale =
      ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage2_lowfreq_final_hold_relaxation") {
    config.enable_stage2_lowfreq_final_hold_relaxation =
      ParseBool(normalized_value);
  } else if (normalized_key == "stage2_lowfreq_final_attitude_hold_sigma_scale") {
    config.stage2_lowfreq_final_attitude_hold_sigma_scale =
      ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_lowfreq_final_horizontal_position_hold_sigma_scale") {
    config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale =
      ParseDouble(normalized_value);
  } else if (normalized_key == "stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale") {
    config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale =
      ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage3_vertical_reference_optimization") {
    config.enable_stage3_vertical_reference_optimization = ParseBool(normalized_value);
  } else if (normalized_key == "stage3_vertical_reference_lowpass_cutoff_hz") {
    config.stage3_vertical_reference_lowpass_cutoff_hz = ParseDouble(normalized_value);
  } else if (normalized_key == "stage3_vertical_anchor_sigma_m") {
    config.stage3_vertical_anchor_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage3_initial_dynamic_static_reference_hold") {
    config.enable_stage3_initial_dynamic_static_reference_hold =
      ParseBool(normalized_value);
  } else if (normalized_key == "stage3_initial_dynamic_static_reference_hold_duration_s") {
    config.stage3_initial_dynamic_static_reference_hold_duration_s =
      ParseDouble(normalized_value);
  } else if (normalized_key == "stage3_initial_dynamic_static_reference_hold_blend_s") {
    config.stage3_initial_dynamic_static_reference_hold_blend_s =
      ParseDouble(normalized_value);
  } else if (normalized_key == "stage3_vertical_reference_constraint_mode") {
    config.stage3_vertical_reference_constraint_mode =
      ParseStage3VerticalReferenceConstraintMode(normalized_value);
  } else if (normalized_key == "stage3_vertical_envelope_half_width_m") {
    config.stage3_vertical_envelope_half_width_m = ParseDouble(normalized_value);
  } else if (normalized_key == "stage3_vertical_envelope_sigma_m") {
    config.stage3_vertical_envelope_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_stage3_vertical_envelope_center_pull") {
    config.enable_stage3_vertical_envelope_center_pull = ParseBool(normalized_value);
  } else if (normalized_key == "stage3_vertical_envelope_center_sigma_m") {
    config.stage3_vertical_envelope_center_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "stage3_vertical_envelope_center_deadband_m") {
    config.stage3_vertical_envelope_center_deadband_m = ParseDouble(normalized_value);
  } else if (normalized_key == "stage3_disable_rtk_outage_segmented_batch") {
    config.stage3_disable_rtk_outage_segmented_batch = ParseBool(normalized_value);
  } else if (normalized_key == "stage3_disable_stage2_vehicle_nhc_constraint") {
    config.stage3_disable_stage2_vehicle_nhc_constraint = ParseBool(normalized_value);
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
  } else if (normalized_key == "enable_initial_static_vertical_bias_soft_prior") {
    config.enable_initial_static_vertical_bias_soft_prior = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_vertical_bias_global_tie_sigma_ug") {
    config.initial_static_vertical_bias_global_tie_sigma_mps2 =
      MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "initial_static_vertical_bias_global_tie_sigma_mps2") {
    config.initial_static_vertical_bias_global_tie_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_static_vertical_bias_sigma_ug") {
    config.initial_static_vertical_bias_global_tie_sigma_mps2 =
      MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "initial_static_vertical_bias_sigma_mps2") {
    config.initial_static_vertical_bias_global_tie_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_static_vertical_bias_gm_tightening") {
    config.enable_initial_static_vertical_bias_gm_tightening = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_vertical_bias_gm_sigma_ug") {
    config.initial_static_vertical_bias_gm_sigma_mps2 = MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "initial_static_vertical_bias_gm_sigma_mps2") {
    config.initial_static_vertical_bias_gm_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_static_vertical_position_hold") {
    config.enable_initial_static_vertical_position_hold = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_vertical_position_hold_sigma_m") {
    config.initial_static_vertical_position_hold_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_static_position_hold") {
    config.enable_initial_static_position_hold = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_position_hold_sigma_m") {
    config.initial_static_position_hold_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_initial_static_rtk_height_reference") {
    config.enable_initial_static_rtk_height_reference = ParseBool(normalized_value);
  } else if (normalized_key == "initial_static_rtk_height_reference_sigma_m") {
    config.initial_static_rtk_height_reference_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "initial_static_rtk_height_reference_min_sample_count") {
    config.initial_static_rtk_height_reference_min_sample_count = ParseInt(normalized_value);
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
  } else if (normalized_key == "enable_vertical_envelope_center_pull") {
    config.enable_vertical_envelope_center_pull = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_envelope_center_sigma_m") {
    config.vertical_envelope_center_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_envelope_center_sigma_mode") {
    config.vertical_envelope_center_sigma_mode = ParseVerticalEnvelopeCenterSigmaMode(normalized_value);
  } else if (normalized_key == "vertical_envelope_center_deadband_m") {
    config.vertical_envelope_center_deadband_m = ParseDouble(normalized_value);
  } else if (normalized_key == "gnss_vertical_reference_source") {
    config.gnss_vertical_reference_source =
      ParseGnssVerticalReferenceSource(normalized_value);
  } else if (normalized_key == "enable_rtk_vertical_drift_reference") {
    config.enable_rtk_vertical_drift_reference = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_correlation_time_s") {
    config.rtk_vertical_drift_correlation_time_s = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_sigma_m") {
    config.rtk_vertical_drift_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_vertical_white_noise_sigma_m") {
    config.rtk_vertical_white_noise_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_huber_sigma_m") {
    config.rtk_vertical_drift_huber_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_rtk_vertical_drift_outage_segmentation") {
    config.enable_rtk_vertical_drift_outage_segmentation = ParseBool(normalized_value);
  } else if (normalized_key == "enable_rtk_vertical_drift_gate_weighting") {
    config.enable_rtk_vertical_drift_gate_weighting = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_gate_weight_floor") {
    config.rtk_vertical_drift_gate_weight_floor = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_causal_drift_reference") {
    config.enable_rtk_outage_causal_drift_reference = ParseBool(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_preoutage_vertical_fence") {
    config.enable_rtk_outage_preoutage_vertical_fence = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_outage_causal_reference_max_prefix_runs") {
    config.rtk_outage_causal_reference_max_prefix_runs = ParseInt(normalized_value);
  } else if (normalized_key == "rtk_outage_preoutage_fence_stride_s") {
    config.rtk_outage_preoutage_fence_stride_s = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_preoutage_fence_up_sigma_m") {
    config.rtk_outage_preoutage_fence_up_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_preoutage_fence_vz_sigma_mps") {
    config.rtk_outage_preoutage_fence_vz_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_max_abs_correction_m") {
    config.rtk_vertical_drift_max_abs_correction_m = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_convergence_threshold_m") {
    config.rtk_vertical_drift_convergence_threshold_m = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_outer_iterations") {
    config.rtk_vertical_drift_outer_iterations = ParseInt(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_use_for_center_pull") {
    config.rtk_vertical_drift_use_for_center_pull = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_vertical_drift_use_for_envelope_gate") {
    config.rtk_vertical_drift_use_for_envelope_gate = ParseBool(normalized_value);
  } else if (normalized_key == "enable_rtk_vertical_lowpass_reference") {
    config.enable_rtk_vertical_lowpass_reference = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_vertical_lowpass_reference_cutoff_hz") {
    config.rtk_vertical_lowpass_reference_cutoff_hz = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_smoothing") {
    config.enable_rtk_outage_smoothing = ParseBool(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_segmented_batch") {
    config.enable_rtk_outage_segmented_batch = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_outage_segmented_batch_max_outages") {
    config.rtk_outage_segmented_batch_max_outages = ParseInt(normalized_value);
  } else if (normalized_key == "rtk_outage_segmented_batch_allow_vertical_boundary_jump") {
    config.rtk_outage_segmented_batch_allow_vertical_boundary_jump =
      ParseBool(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_baz_reestimate") {
    config.enable_rtk_outage_baz_reestimate = ParseBool(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_boundary_constraints") {
    config.enable_rtk_outage_boundary_constraints = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_outage_recovery_reference_min_fix_samples") {
    config.rtk_outage_recovery_reference_min_fix_samples = ParseInt(normalized_value);
  } else if (normalized_key == "rtk_outage_recovery_reference_max_duration_s") {
    config.rtk_outage_recovery_reference_max_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_boundary_up_sigma_m") {
    config.rtk_outage_boundary_up_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_boundary_vz_sigma_mps") {
    config.rtk_outage_boundary_vz_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_boundary_baz_sigma_ug") {
    config.rtk_outage_boundary_baz_sigma_mps2 =
      MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "rtk_outage_baz_continuity_break_delta_threshold_ug") {
    config.rtk_outage_baz_continuity_break_delta_threshold_mps2 =
      MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "rtk_outage_min_gap_s") {
    config.rtk_outage_min_gap_s = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_position_ramp_sigma_m") {
    config.rtk_outage_position_ramp_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_velocity_delta_sigma_mps") {
    config.rtk_outage_velocity_delta_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_velocity_delta_target_acc_limit_mps2") {
    config.rtk_outage_velocity_delta_target_acc_limit_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_position_ramp_stride") {
    config.rtk_outage_position_ramp_stride = ParseInt(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_attitude_hold") {
    config.enable_rtk_outage_attitude_hold = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_outage_attitude_guard_duration_s") {
    config.rtk_outage_attitude_guard_duration_s = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_absolute_attitude_sigma_rad") {
    config.rtk_outage_absolute_attitude_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_outage_relative_attitude_sigma_rad") {
    config.rtk_outage_relative_attitude_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_rtk_outage_velocity_delta_3d") {
    config.enable_rtk_outage_velocity_delta_3d = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_outage_velocity_delta_3d_sigma_mps") {
    config.rtk_outage_velocity_delta_3d_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_rtk_velocity_constraint") {
    config.enable_rtk_velocity_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "rtk_velocity_window_s") {
    config.rtk_velocity_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "rtk_velocity_horizontal_sigma_mps") {
    config.rtk_velocity_horizontal_sigma_mps = ParseDouble(normalized_value);
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
  } else if (normalized_key == "enable_vertical_velocity_delta_initial_static_constraint") {
    config.enable_vertical_velocity_delta_initial_static_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "enable_vertical_velocity_delta_bias_consistent_sigma") {
    config.enable_vertical_velocity_delta_bias_consistent_sigma = ParseBool(normalized_value);
  } else if (normalized_key == "enable_vertical_velocity_delta_bias_aware_target") {
    config.enable_vertical_velocity_delta_bias_aware_target = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_bias_sigma_ug") {
    config.vertical_velocity_delta_bias_sigma_mps2 = MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "vertical_velocity_delta_bias_sigma_mps2") {
    config.vertical_velocity_delta_bias_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_attitude_sigma_rad") {
    config.vertical_velocity_delta_attitude_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_sigma_floor_mps") {
    config.vertical_velocity_delta_sigma_floor_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_sigma_ceiling_mps") {
    config.vertical_velocity_delta_sigma_ceiling_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_sigma_scale") {
    config.vertical_velocity_delta_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_velocity_delta_context_sigma_scale") {
    config.enable_vertical_velocity_delta_context_sigma_scale = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_context_normal_sigma_scale") {
    config.vertical_velocity_delta_context_normal_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_context_rough_sigma_scale") {
    config.vertical_velocity_delta_context_rough_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_context_outage_sigma_scale") {
    config.vertical_velocity_delta_context_outage_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_context_jump_sigma_scale") {
    config.vertical_velocity_delta_context_jump_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_velocity_delta_context_jump_extra_padding_s") {
    config.vertical_velocity_delta_context_jump_extra_padding_s = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_motion_adaptive_reweighting") {
    config.enable_vertical_motion_adaptive_reweighting = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_outer_iterations") {
    config.vertical_motion_adaptive_outer_iterations = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_convergence_score_epsilon") {
    config.vertical_motion_adaptive_convergence_score_epsilon = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_stability_window_s") {
    config.vertical_motion_adaptive_stability_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_horizontal_speed_rms_mps") {
    config.vertical_motion_adaptive_static_horizontal_speed_rms_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_vz_rms_mps") {
    config.vertical_motion_adaptive_static_vz_rms_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_target_acc_rms_mps2") {
    config.vertical_motion_adaptive_static_target_acc_rms_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_dvz_bias_sigma_ug") {
    config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 =
      MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "vertical_motion_adaptive_static_dvz_bias_sigma_mps2") {
    config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_attitude_sigma_rad") {
    config.vertical_motion_adaptive_static_attitude_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_sigma_floor_mps") {
    config.vertical_motion_adaptive_static_sigma_floor_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_sigma_ceiling_mps") {
    config.vertical_motion_adaptive_static_sigma_ceiling_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_motion_adaptive_static_baz_gm_sigma_ug") {
    config.vertical_motion_adaptive_static_baz_gm_sigma_mps2 =
      MicroGToMps2(ParseDouble(normalized_value));
  } else if (normalized_key == "vertical_motion_adaptive_static_baz_gm_sigma_mps2") {
    config.vertical_motion_adaptive_static_baz_gm_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_position_velocity_consistency_all_states") {
    config.enable_vertical_position_velocity_consistency_all_states = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_position_velocity_consistency_sigma_m") {
    config.vertical_position_velocity_consistency_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_position_velocity_window_consistency") {
    config.enable_vertical_position_velocity_window_consistency = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_position_velocity_window_s") {
    config.vertical_position_velocity_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_position_velocity_window_stride_s") {
    config.vertical_position_velocity_window_stride_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_position_velocity_window_sigma_m") {
    config.vertical_position_velocity_window_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_attitude_reference_constraint") {
    config.enable_attitude_reference_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "attitude_reference_sigma_rad") {
    config.attitude_reference_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "attitude_reference_relative_yaw_sigma_rad") {
    config.attitude_reference_relative_yaw_sigma_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_body_z_nhc_constraint") {
    config.enable_body_z_nhc_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "enable_body_z_nhc_global_weak_constraint") {
    config.enable_body_z_nhc_global_weak_constraint = ParseBool(normalized_value);
  } else if (normalized_key == "enable_body_z_nhc_strict_effective_weighting") {
    config.enable_body_z_nhc_strict_effective_weighting = ParseBool(normalized_value);
  } else if (normalized_key == "body_z_nhc_jump_padding_s") {
    config.body_z_nhc_jump_padding_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_merge_gap_s") {
    config.body_z_nhc_merge_gap_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_min_window_s") {
    config.body_z_nhc_min_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_jump_velocity_sigma_mps") {
    config.body_z_nhc_jump_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_jump_displacement_sigma_m") {
    config.body_z_nhc_jump_displacement_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_global_window_s") {
    config.body_z_nhc_global_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_global_stride_s") {
    config.body_z_nhc_global_stride_s = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_global_velocity_sigma_mps") {
    config.body_z_nhc_global_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_global_displacement_sigma_m") {
    config.body_z_nhc_global_displacement_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_body_z_nhc_horizontal_leakage_correction") {
    config.enable_body_z_nhc_horizontal_leakage_correction = ParseBool(normalized_value);
  } else if (normalized_key == "body_z_nhc_horizontal_leakage_min_speed_mps") {
    config.body_z_nhc_horizontal_leakage_min_speed_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_horizontal_leakage_min_sample_count") {
    config.body_z_nhc_horizontal_leakage_min_sample_count = ParseInt(normalized_value);
  } else if (normalized_key == "body_z_nhc_horizontal_leakage_huber_sigma_mps") {
    config.body_z_nhc_horizontal_leakage_huber_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_horizontal_leakage_max_abs_coeff_rad") {
    config.body_z_nhc_horizontal_leakage_max_abs_coeff_rad = ParseDouble(normalized_value);
  } else if (normalized_key == "body_z_nhc_horizontal_leakage_guard_s") {
    config.body_z_nhc_horizontal_leakage_guard_s = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_masked_imu") {
    config.enable_vertical_jump_masked_imu = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_masked_imu_padding_s") {
    config.vertical_jump_masked_imu_padding_s = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_impulse") {
    config.enable_vertical_jump_impulse = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_impulse_prior_sigma_mps") {
    config.vertical_jump_impulse_prior_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_impulse_velocity_sigma_mps") {
    config.vertical_jump_impulse_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_impulse_position_velocity_sigma_m") {
    config.vertical_jump_impulse_position_velocity_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_bias") {
    config.enable_vertical_jump_bias = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_bias_padding_s") {
    config.vertical_jump_bias_padding_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_bias_prior_sigma_mps2") {
    config.vertical_jump_bias_prior_sigma_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_bias_velocity_sigma_mps") {
    config.vertical_jump_bias_velocity_sigma_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_bias_position_velocity_sigma_m") {
    config.vertical_jump_bias_position_velocity_sigma_m = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_segmented_bias") {
    config.enable_vertical_jump_segmented_bias = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_segmented_bias_min_segment_s") {
    config.vertical_jump_segmented_bias_min_segment_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_segmented_bias_max_segments") {
    config.vertical_jump_segmented_bias_max_segments = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_segmented_bias_slope_merge_threshold_mps2") {
    config.vertical_jump_segmented_bias_slope_merge_threshold_mps2 = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_bias_highfreq_sigma_scale") {
    config.vertical_jump_bias_highfreq_sigma_scale = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_bias_highfreq_sigma_max_mps") {
    config.vertical_jump_bias_highfreq_sigma_max_mps = ParseDouble(normalized_value);
  } else if (normalized_key == "enable_vertical_jump_spectral_bias_relaxation") {
    config.enable_vertical_jump_spectral_bias_relaxation = ParseBool(normalized_value);
  } else if (normalized_key == "vertical_jump_spectral_window_s") {
    config.vertical_jump_spectral_window_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_spectral_stride_s") {
    config.vertical_jump_spectral_stride_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_spectral_reference_margin_s") {
    config.vertical_jump_spectral_reference_margin_s = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_spectral_min_reference_window_count") {
    config.vertical_jump_spectral_min_reference_window_count = ParseInt(normalized_value);
  } else if (normalized_key == "vertical_jump_spectral_response_trigger_ratio") {
    config.vertical_jump_spectral_response_trigger_ratio = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_spectral_response_full_ratio") {
    config.vertical_jump_spectral_response_full_ratio = ParseDouble(normalized_value);
  } else if (normalized_key == "vertical_jump_spectral_bias_prior_max_sigma_mps2") {
    config.vertical_jump_spectral_bias_prior_max_sigma_mps2 = ParseDouble(normalized_value);
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
    << "processing_start_time_s=" << config.processing_start_time_s << '\n'
    << "processing_end_time_s=" << config.processing_end_time_s << '\n'
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
    << "global_acc_bias_tie_sigma_ug=" << Mps2ToMicroG(config.global_acc_bias_tie_sigma_mps2) << '\n'
    << "global_acc_bias_tie_sigma_xy_ug=" << Mps2ToMicroG(config.global_acc_bias_tie_sigma_xy_mps2) << '\n'
    << "enable_global_gyro_bias=" << (config.enable_global_gyro_bias ? "true" : "false") << '\n'
    << "global_gyro_bias_tie_sigma_radps=" << config.global_gyro_bias_tie_sigma_radps << '\n'
    << "enable_vertical_acc_bias_gm_process=" << (config.enable_vertical_acc_bias_gm_process ? "true" : "false") << '\n'
    << "vertical_acc_bias_tau_s=" << config.vertical_acc_bias_tau_s << '\n'
    << "vertical_acc_bias_sigma_ug=" << Mps2ToMicroG(config.vertical_acc_bias_sigma_mps2) << '\n'
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
    << "body_z_long_bias_min_duration_s=" << config.body_z_long_bias_min_duration_s << '\n'
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
    << "enable_initial_yaw_override=" << (config.enable_initial_yaw_override ? "true" : "false") << '\n'
    << "initial_yaw_override_rad=" << config.initial_yaw_override_rad << '\n'
    << "enable_stage1_yaw_refinement=" << (config.enable_stage1_yaw_refinement ? "true" : "false") << '\n'
    << "stage1_yaw_refinement_max_iterations=" << config.stage1_yaw_refinement_max_iterations << '\n'
    << "stage1_heading_window_s=" << config.stage1_heading_window_s << '\n'
    << "stage1_heading_time_tolerance_s=" << config.stage1_heading_time_tolerance_s << '\n'
    << "stage1_heading_min_displacement_m=" << config.stage1_heading_min_displacement_m << '\n'
    << "stage1_heading_noise_floor_rad=" << config.stage1_heading_noise_floor_rad << '\n'
    << "stage1_yaw_update_max_rad=" << config.stage1_yaw_update_max_rad << '\n'
    << "enable_stage1_outage_body_y_envelope="
    << (config.enable_stage1_outage_body_y_envelope ? "true" : "false") << '\n'
    << "stage1_outage_body_y_pre_window_s="
    << config.stage1_outage_body_y_pre_window_s << '\n'
    << "stage1_outage_body_y_deadband_rmse_multiplier="
    << config.stage1_outage_body_y_deadband_rmse_multiplier << '\n'
    << "stage1_outage_body_y_min_sample_count="
    << config.stage1_outage_body_y_min_sample_count << '\n'
    << "stage1_outage_body_y_min_speed_mps="
    << config.stage1_outage_body_y_min_speed_mps << '\n'
    << "stage1_outage_body_y_min_sigma_mps="
    << config.stage1_outage_body_y_min_sigma_mps << '\n'
    << "stage1_outage_body_y_max_sigma_mps="
    << config.stage1_outage_body_y_max_sigma_mps << '\n'
    << "stage1_outage_body_y_huber_k=" << config.stage1_outage_body_y_huber_k << '\n'
    << "enable_late_static_detection="
    << (config.enable_late_static_detection ? "true" : "false") << '\n'
    << "late_static_window_s=" << config.late_static_window_s << '\n'
    << "late_static_stride_s=" << config.late_static_stride_s << '\n'
    << "late_static_min_duration_s=" << config.late_static_min_duration_s << '\n'
    << "late_static_threshold_method=" << config.late_static_threshold_method << '\n'
    << "late_static_min_rtkfix_samples=" << config.late_static_min_rtkfix_samples << '\n'
    << "late_static_merge_gap_s=" << config.late_static_merge_gap_s << '\n'
    << "late_static_exclude_initial_static="
    << (config.late_static_exclude_initial_static ? "true" : "false") << '\n'
    << "late_static_exclude_rtk_outage="
    << (config.late_static_exclude_rtk_outage ? "true" : "false") << '\n'
    << "late_static_vz_sigma_mps=" << config.late_static_vz_sigma_mps << '\n'
    << "late_static_up_sigma_m=" << config.late_static_up_sigma_m << '\n'
    << "late_static_height_hold_sigma_m=" << config.late_static_height_hold_sigma_m << '\n'
    << "enable_initial_dynamic_static_detection="
    << (config.enable_initial_dynamic_static_detection ? "true" : "false") << '\n'
    << "initial_dynamic_static_search_duration_s="
    << config.initial_dynamic_static_search_duration_s << '\n'
    << "initial_dynamic_static_threshold_multiplier="
    << config.initial_dynamic_static_threshold_multiplier << '\n'
    << "initial_dynamic_static_min_duration_s="
    << config.initial_dynamic_static_min_duration_s << '\n'
    << "initial_dynamic_static_merge_gap_s="
    << config.initial_dynamic_static_merge_gap_s << '\n'
    << "enable_initial_dynamic_static_lowpass_protection="
    << (config.enable_initial_dynamic_static_lowpass_protection ? "true" : "false") << '\n'
    << "initial_dynamic_static_lowpass_blend_s="
    << config.initial_dynamic_static_lowpass_blend_s << '\n'
    << "enable_initial_dynamic_static_vz_constraint="
    << (config.enable_initial_dynamic_static_vz_constraint ? "true" : "false") << '\n'
    << "initial_dynamic_static_vz_sigma_mps="
    << config.initial_dynamic_static_vz_sigma_mps << '\n'
    << "enable_stage2_velocity_optimization="
    << (config.enable_stage2_velocity_optimization ? "true" : "false") << '\n'
    << "enable_stage2_vehicle_nhc_constraint="
    << (config.enable_stage2_vehicle_nhc_constraint ? "true" : "false") << '\n'
    << "stage2_attitude_hold_sigma_rad=" << config.stage2_attitude_hold_sigma_rad << '\n'
    << "stage2_horizontal_position_hold_sigma_m="
    << config.stage2_horizontal_position_hold_sigma_m << '\n'
    << "stage2_horizontal_velocity_hold_sigma_mps="
    << config.stage2_horizontal_velocity_hold_sigma_mps << '\n'
    << "stage2_mount_leakage_prior_sigma_rad="
    << config.stage2_mount_leakage_prior_sigma_rad << '\n'
    << "stage2_vehicle_y_nhc_velocity_sigma_mps="
    << config.stage2_vehicle_y_nhc_velocity_sigma_mps << '\n'
    << "stage2_vehicle_y_nhc_displacement_sigma_m="
    << config.stage2_vehicle_y_nhc_displacement_sigma_m << '\n'
    << "enable_stage2_lowfreq_vertical_reference_optimization="
    << (config.enable_stage2_lowfreq_vertical_reference_optimization ? "true" : "false") << '\n'
    << "stage2_lowfreq_vertical_reference_cutoff_hz="
    << config.stage2_lowfreq_vertical_reference_cutoff_hz << '\n'
    << "stage2_lowfreq_vertical_reference_source="
    << ToString(config.stage2_lowfreq_vertical_reference_source) << '\n'
    << "enable_stage2_lowfreq_final_dvz_relaxation="
    << (config.enable_stage2_lowfreq_final_dvz_relaxation ? "true" : "false") << '\n'
    << "stage2_lowfreq_final_dvz_sigma_scale="
    << config.stage2_lowfreq_final_dvz_sigma_scale << '\n'
    << "enable_stage2_lowfreq_final_hold_relaxation="
    << (config.enable_stage2_lowfreq_final_hold_relaxation ? "true" : "false") << '\n'
    << "stage2_lowfreq_final_attitude_hold_sigma_scale="
    << config.stage2_lowfreq_final_attitude_hold_sigma_scale << '\n'
    << "stage2_lowfreq_final_horizontal_position_hold_sigma_scale="
    << config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale << '\n'
    << "stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale="
    << config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale << '\n'
    << "enable_stage3_vertical_reference_optimization="
    << (config.enable_stage3_vertical_reference_optimization ? "true" : "false") << '\n'
    << "stage3_vertical_reference_lowpass_cutoff_hz="
    << config.stage3_vertical_reference_lowpass_cutoff_hz << '\n'
    << "stage3_vertical_anchor_sigma_m="
    << config.stage3_vertical_anchor_sigma_m << '\n'
    << "enable_stage3_initial_dynamic_static_reference_hold="
    << (config.enable_stage3_initial_dynamic_static_reference_hold ? "true" : "false") << '\n'
    << "stage3_initial_dynamic_static_reference_hold_duration_s="
    << config.stage3_initial_dynamic_static_reference_hold_duration_s << '\n'
    << "stage3_initial_dynamic_static_reference_hold_blend_s="
    << config.stage3_initial_dynamic_static_reference_hold_blend_s << '\n'
    << "stage3_vertical_reference_constraint_mode="
    << ToString(config.stage3_vertical_reference_constraint_mode) << '\n'
    << "stage3_vertical_envelope_half_width_m="
    << config.stage3_vertical_envelope_half_width_m << '\n'
    << "stage3_vertical_envelope_sigma_m="
    << config.stage3_vertical_envelope_sigma_m << '\n'
    << "enable_stage3_vertical_envelope_center_pull="
    << (config.enable_stage3_vertical_envelope_center_pull ? "true" : "false") << '\n'
    << "stage3_vertical_envelope_center_sigma_m="
    << config.stage3_vertical_envelope_center_sigma_m << '\n'
    << "stage3_vertical_envelope_center_deadband_m="
    << config.stage3_vertical_envelope_center_deadband_m << '\n'
    << "stage3_disable_rtk_outage_segmented_batch="
    << (config.stage3_disable_rtk_outage_segmented_batch ? "true" : "false") << '\n'
    << "stage3_disable_stage2_vehicle_nhc_constraint="
    << (config.stage3_disable_stage2_vehicle_nhc_constraint ? "true" : "false") << '\n'
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
    << "enable_initial_static_vertical_bias_soft_prior="
    << (config.enable_initial_static_vertical_bias_soft_prior ? "true" : "false") << '\n'
    << "initial_static_vertical_bias_global_tie_sigma_ug="
    << Mps2ToMicroG(config.initial_static_vertical_bias_global_tie_sigma_mps2) << '\n'
    << "enable_initial_static_vertical_bias_gm_tightening="
    << (config.enable_initial_static_vertical_bias_gm_tightening ? "true" : "false") << '\n'
    << "initial_static_vertical_bias_gm_sigma_ug="
    << Mps2ToMicroG(config.initial_static_vertical_bias_gm_sigma_mps2) << '\n'
    << "enable_initial_static_vertical_position_hold="
    << (config.enable_initial_static_vertical_position_hold ? "true" : "false") << '\n'
    << "initial_static_vertical_position_hold_sigma_m="
    << config.initial_static_vertical_position_hold_sigma_m << '\n'
    << "enable_initial_static_position_hold="
    << (config.enable_initial_static_position_hold ? "true" : "false") << '\n'
    << "initial_static_position_hold_sigma_m="
    << config.initial_static_position_hold_sigma_m << '\n'
    << "enable_initial_static_rtk_height_reference="
    << (config.enable_initial_static_rtk_height_reference ? "true" : "false") << '\n'
    << "initial_static_rtk_height_reference_sigma_m="
    << config.initial_static_rtk_height_reference_sigma_m << '\n'
    << "initial_static_rtk_height_reference_min_sample_count="
    << config.initial_static_rtk_height_reference_min_sample_count << '\n'
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
    << "enable_vertical_envelope_center_pull="
    << (config.enable_vertical_envelope_center_pull ? "true" : "false") << '\n'
    << "vertical_envelope_center_sigma_m=" << config.vertical_envelope_center_sigma_m << '\n'
    << "vertical_envelope_center_sigma_mode=" << ToString(config.vertical_envelope_center_sigma_mode) << '\n'
    << "vertical_envelope_center_deadband_m=" << config.vertical_envelope_center_deadband_m << '\n'
    << "gnss_vertical_reference_source=" << ToString(config.gnss_vertical_reference_source) << '\n'
    << "enable_rtk_vertical_drift_reference="
    << (config.enable_rtk_vertical_drift_reference ? "true" : "false") << '\n'
    << "rtk_vertical_drift_correlation_time_s="
    << config.rtk_vertical_drift_correlation_time_s << '\n'
    << "rtk_vertical_drift_sigma_m=" << config.rtk_vertical_drift_sigma_m << '\n'
    << "rtk_vertical_white_noise_sigma_m=" << config.rtk_vertical_white_noise_sigma_m << '\n'
    << "rtk_vertical_drift_huber_sigma_m=" << config.rtk_vertical_drift_huber_sigma_m << '\n'
    << "enable_rtk_vertical_drift_outage_segmentation="
    << (config.enable_rtk_vertical_drift_outage_segmentation ? "true" : "false") << '\n'
    << "enable_rtk_vertical_drift_gate_weighting="
    << (config.enable_rtk_vertical_drift_gate_weighting ? "true" : "false") << '\n'
    << "rtk_vertical_drift_gate_weight_floor="
    << config.rtk_vertical_drift_gate_weight_floor << '\n'
    << "enable_rtk_outage_causal_drift_reference="
    << (config.enable_rtk_outage_causal_drift_reference ? "true" : "false") << '\n'
    << "enable_rtk_outage_preoutage_vertical_fence="
    << (config.enable_rtk_outage_preoutage_vertical_fence ? "true" : "false") << '\n'
    << "rtk_outage_causal_reference_max_prefix_runs="
    << config.rtk_outage_causal_reference_max_prefix_runs << '\n'
    << "rtk_outage_preoutage_fence_stride_s="
    << config.rtk_outage_preoutage_fence_stride_s << '\n'
    << "rtk_outage_preoutage_fence_up_sigma_m="
    << config.rtk_outage_preoutage_fence_up_sigma_m << '\n'
    << "rtk_outage_preoutage_fence_vz_sigma_mps="
    << config.rtk_outage_preoutage_fence_vz_sigma_mps << '\n'
    << "rtk_vertical_drift_max_abs_correction_m="
    << config.rtk_vertical_drift_max_abs_correction_m << '\n'
    << "rtk_vertical_drift_convergence_threshold_m="
    << config.rtk_vertical_drift_convergence_threshold_m << '\n'
    << "rtk_vertical_drift_outer_iterations="
    << config.rtk_vertical_drift_outer_iterations << '\n'
    << "rtk_vertical_drift_use_for_center_pull="
    << (config.rtk_vertical_drift_use_for_center_pull ? "true" : "false") << '\n'
    << "rtk_vertical_drift_use_for_envelope_gate="
    << (config.rtk_vertical_drift_use_for_envelope_gate ? "true" : "false") << '\n'
    << "enable_rtk_vertical_lowpass_reference="
    << (config.enable_rtk_vertical_lowpass_reference ? "true" : "false") << '\n'
    << "rtk_vertical_lowpass_reference_cutoff_hz="
    << config.rtk_vertical_lowpass_reference_cutoff_hz << '\n'
    << "enable_rtk_outage_smoothing="
    << (config.enable_rtk_outage_smoothing ? "true" : "false") << '\n'
    << "enable_rtk_outage_segmented_batch="
    << (config.enable_rtk_outage_segmented_batch ? "true" : "false") << '\n'
    << "rtk_outage_segmented_batch_max_outages="
    << config.rtk_outage_segmented_batch_max_outages << '\n'
    << "rtk_outage_segmented_batch_allow_vertical_boundary_jump="
    << (config.rtk_outage_segmented_batch_allow_vertical_boundary_jump ? "true" : "false") << '\n'
    << "enable_rtk_outage_baz_reestimate="
    << (config.enable_rtk_outage_baz_reestimate ? "true" : "false") << '\n'
    << "enable_rtk_outage_boundary_constraints="
    << (config.enable_rtk_outage_boundary_constraints ? "true" : "false") << '\n'
    << "rtk_outage_recovery_reference_min_fix_samples="
    << config.rtk_outage_recovery_reference_min_fix_samples << '\n'
    << "rtk_outage_recovery_reference_max_duration_s="
    << config.rtk_outage_recovery_reference_max_duration_s << '\n'
    << "rtk_outage_boundary_up_sigma_m="
    << config.rtk_outage_boundary_up_sigma_m << '\n'
    << "rtk_outage_boundary_vz_sigma_mps="
    << config.rtk_outage_boundary_vz_sigma_mps << '\n'
    << "rtk_outage_boundary_baz_sigma_ug="
    << Mps2ToMicroG(config.rtk_outage_boundary_baz_sigma_mps2) << '\n'
    << "rtk_outage_baz_continuity_break_delta_threshold_ug="
    << Mps2ToMicroG(config.rtk_outage_baz_continuity_break_delta_threshold_mps2) << '\n'
    << "rtk_outage_min_gap_s=" << config.rtk_outage_min_gap_s << '\n'
    << "rtk_outage_position_ramp_sigma_m=" << config.rtk_outage_position_ramp_sigma_m << '\n'
    << "rtk_outage_velocity_delta_sigma_mps="
    << config.rtk_outage_velocity_delta_sigma_mps << '\n'
    << "rtk_outage_velocity_delta_target_acc_limit_mps2="
    << config.rtk_outage_velocity_delta_target_acc_limit_mps2 << '\n'
    << "rtk_outage_position_ramp_stride=" << config.rtk_outage_position_ramp_stride << '\n'
    << "enable_rtk_outage_attitude_hold="
    << (config.enable_rtk_outage_attitude_hold ? "true" : "false") << '\n'
    << "rtk_outage_attitude_guard_duration_s="
    << config.rtk_outage_attitude_guard_duration_s << '\n'
    << "rtk_outage_absolute_attitude_sigma_rad="
    << config.rtk_outage_absolute_attitude_sigma_rad << '\n'
    << "rtk_outage_relative_attitude_sigma_rad="
    << config.rtk_outage_relative_attitude_sigma_rad << '\n'
    << "enable_rtk_outage_velocity_delta_3d="
    << (config.enable_rtk_outage_velocity_delta_3d ? "true" : "false") << '\n'
    << "rtk_outage_velocity_delta_3d_sigma_mps="
    << config.rtk_outage_velocity_delta_3d_sigma_mps << '\n'
    << "enable_rtk_velocity_constraint="
    << (config.enable_rtk_velocity_constraint ? "true" : "false") << '\n'
    << "rtk_velocity_window_s=" << config.rtk_velocity_window_s << '\n'
    << "rtk_velocity_horizontal_sigma_mps=" << config.rtk_velocity_horizontal_sigma_mps << '\n'
    << "enable_vertical_velocity_delta_constraint="
    << (config.enable_vertical_velocity_delta_constraint ? "true" : "false") << '\n'
    << "vertical_velocity_delta_acc_sigma_mps2=" << config.vertical_velocity_delta_acc_sigma_mps2 << '\n'
    << "vertical_velocity_delta_min_sigma_mps=" << config.vertical_velocity_delta_min_sigma_mps << '\n'
    << "vertical_velocity_delta_jump_padding_s=" << config.vertical_velocity_delta_jump_padding_s << '\n'
    << "vertical_velocity_delta_target_acc_limit_mps2="
    << config.vertical_velocity_delta_target_acc_limit_mps2 << '\n'
    << "enable_vertical_velocity_delta_initial_static_constraint="
    << (config.enable_vertical_velocity_delta_initial_static_constraint ? "true" : "false") << '\n'
    << "enable_vertical_velocity_delta_bias_consistent_sigma="
    << (config.enable_vertical_velocity_delta_bias_consistent_sigma ? "true" : "false") << '\n'
    << "enable_vertical_velocity_delta_bias_aware_target="
    << (config.enable_vertical_velocity_delta_bias_aware_target ? "true" : "false") << '\n'
    << "vertical_velocity_delta_bias_sigma_ug="
    << Mps2ToMicroG(config.vertical_velocity_delta_bias_sigma_mps2) << '\n'
    << "vertical_velocity_delta_attitude_sigma_rad="
    << config.vertical_velocity_delta_attitude_sigma_rad << '\n'
    << "vertical_velocity_delta_sigma_floor_mps="
    << config.vertical_velocity_delta_sigma_floor_mps << '\n'
    << "vertical_velocity_delta_sigma_ceiling_mps="
    << config.vertical_velocity_delta_sigma_ceiling_mps << '\n'
    << "vertical_velocity_delta_sigma_scale="
    << config.vertical_velocity_delta_sigma_scale << '\n'
    << "enable_vertical_velocity_delta_context_sigma_scale="
    << (config.enable_vertical_velocity_delta_context_sigma_scale ? "true" : "false") << '\n'
    << "vertical_velocity_delta_context_normal_sigma_scale="
    << config.vertical_velocity_delta_context_normal_sigma_scale << '\n'
    << "vertical_velocity_delta_context_rough_sigma_scale="
    << config.vertical_velocity_delta_context_rough_sigma_scale << '\n'
    << "vertical_velocity_delta_context_outage_sigma_scale="
    << config.vertical_velocity_delta_context_outage_sigma_scale << '\n'
    << "vertical_velocity_delta_context_jump_sigma_scale="
    << config.vertical_velocity_delta_context_jump_sigma_scale << '\n'
    << "vertical_velocity_delta_context_jump_extra_padding_s="
    << config.vertical_velocity_delta_context_jump_extra_padding_s << '\n'
    << "enable_vertical_motion_adaptive_reweighting="
    << (config.enable_vertical_motion_adaptive_reweighting ? "true" : "false") << '\n'
    << "vertical_motion_adaptive_outer_iterations="
    << config.vertical_motion_adaptive_outer_iterations << '\n'
    << "vertical_motion_adaptive_convergence_score_epsilon="
    << config.vertical_motion_adaptive_convergence_score_epsilon << '\n'
    << "vertical_motion_adaptive_stability_window_s="
    << config.vertical_motion_adaptive_stability_window_s << '\n'
    << "vertical_motion_adaptive_static_horizontal_speed_rms_mps="
    << config.vertical_motion_adaptive_static_horizontal_speed_rms_mps << '\n'
    << "vertical_motion_adaptive_static_vz_rms_mps="
    << config.vertical_motion_adaptive_static_vz_rms_mps << '\n'
    << "vertical_motion_adaptive_static_target_acc_rms_mps2="
    << config.vertical_motion_adaptive_static_target_acc_rms_mps2 << '\n'
    << "vertical_motion_adaptive_static_dvz_bias_sigma_ug="
    << Mps2ToMicroG(config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2) << '\n'
    << "vertical_motion_adaptive_static_attitude_sigma_rad="
    << config.vertical_motion_adaptive_static_attitude_sigma_rad << '\n'
    << "vertical_motion_adaptive_static_sigma_floor_mps="
    << config.vertical_motion_adaptive_static_sigma_floor_mps << '\n'
    << "vertical_motion_adaptive_static_sigma_ceiling_mps="
    << config.vertical_motion_adaptive_static_sigma_ceiling_mps << '\n'
    << "vertical_motion_adaptive_static_baz_gm_sigma_ug="
    << Mps2ToMicroG(config.vertical_motion_adaptive_static_baz_gm_sigma_mps2) << '\n'
    << "enable_vertical_position_velocity_consistency_all_states="
    << (config.enable_vertical_position_velocity_consistency_all_states ? "true" : "false") << '\n'
    << "vertical_position_velocity_consistency_sigma_m="
    << config.vertical_position_velocity_consistency_sigma_m << '\n'
    << "enable_vertical_position_velocity_window_consistency="
    << (config.enable_vertical_position_velocity_window_consistency ? "true" : "false") << '\n'
    << "vertical_position_velocity_window_s="
    << config.vertical_position_velocity_window_s << '\n'
    << "vertical_position_velocity_window_stride_s="
    << config.vertical_position_velocity_window_stride_s << '\n'
    << "vertical_position_velocity_window_sigma_m="
    << config.vertical_position_velocity_window_sigma_m << '\n'
    << "enable_attitude_reference_constraint="
    << (config.enable_attitude_reference_constraint ? "true" : "false") << '\n'
    << "attitude_reference_sigma_rad=" << config.attitude_reference_sigma_rad << '\n'
    << "attitude_reference_relative_yaw_sigma_rad="
    << config.attitude_reference_relative_yaw_sigma_rad << '\n'
    << "enable_body_z_nhc_constraint=" << (config.enable_body_z_nhc_constraint ? "true" : "false") << '\n'
    << "enable_body_z_nhc_global_weak_constraint="
    << (config.enable_body_z_nhc_global_weak_constraint ? "true" : "false") << '\n'
    << "enable_body_z_nhc_strict_effective_weighting="
    << (config.enable_body_z_nhc_strict_effective_weighting ? "true" : "false") << '\n'
    << "body_z_nhc_jump_padding_s=" << config.body_z_nhc_jump_padding_s << '\n'
    << "body_z_nhc_merge_gap_s=" << config.body_z_nhc_merge_gap_s << '\n'
    << "body_z_nhc_min_window_s=" << config.body_z_nhc_min_window_s << '\n'
    << "body_z_nhc_jump_velocity_sigma_mps=" << config.body_z_nhc_jump_velocity_sigma_mps << '\n'
    << "body_z_nhc_jump_displacement_sigma_m=" << config.body_z_nhc_jump_displacement_sigma_m << '\n'
    << "body_z_nhc_global_window_s=" << config.body_z_nhc_global_window_s << '\n'
    << "body_z_nhc_global_stride_s=" << config.body_z_nhc_global_stride_s << '\n'
    << "body_z_nhc_global_velocity_sigma_mps=" << config.body_z_nhc_global_velocity_sigma_mps << '\n'
    << "body_z_nhc_global_displacement_sigma_m=" << config.body_z_nhc_global_displacement_sigma_m << '\n'
    << "enable_body_z_nhc_horizontal_leakage_correction="
    << (config.enable_body_z_nhc_horizontal_leakage_correction ? "true" : "false") << '\n'
    << "body_z_nhc_horizontal_leakage_min_speed_mps="
    << config.body_z_nhc_horizontal_leakage_min_speed_mps << '\n'
    << "body_z_nhc_horizontal_leakage_min_sample_count="
    << config.body_z_nhc_horizontal_leakage_min_sample_count << '\n'
    << "body_z_nhc_horizontal_leakage_huber_sigma_mps="
    << config.body_z_nhc_horizontal_leakage_huber_sigma_mps << '\n'
    << "body_z_nhc_horizontal_leakage_max_abs_coeff_rad="
    << config.body_z_nhc_horizontal_leakage_max_abs_coeff_rad << '\n'
    << "body_z_nhc_horizontal_leakage_guard_s="
    << config.body_z_nhc_horizontal_leakage_guard_s << '\n'
    << "enable_vertical_jump_masked_imu=" << (config.enable_vertical_jump_masked_imu ? "true" : "false") << '\n'
    << "vertical_jump_masked_imu_padding_s=" << config.vertical_jump_masked_imu_padding_s << '\n'
    << "enable_vertical_jump_impulse=" << (config.enable_vertical_jump_impulse ? "true" : "false") << '\n'
    << "vertical_jump_impulse_prior_sigma_mps=" << config.vertical_jump_impulse_prior_sigma_mps << '\n'
    << "vertical_jump_impulse_velocity_sigma_mps=" << config.vertical_jump_impulse_velocity_sigma_mps << '\n'
    << "vertical_jump_impulse_position_velocity_sigma_m="
    << config.vertical_jump_impulse_position_velocity_sigma_m << '\n'
    << "enable_vertical_jump_bias=" << (config.enable_vertical_jump_bias ? "true" : "false") << '\n'
    << "vertical_jump_bias_padding_s=" << config.vertical_jump_bias_padding_s << '\n'
    << "vertical_jump_bias_prior_sigma_mps2=" << config.vertical_jump_bias_prior_sigma_mps2 << '\n'
    << "vertical_jump_bias_velocity_sigma_mps=" << config.vertical_jump_bias_velocity_sigma_mps << '\n'
    << "vertical_jump_bias_position_velocity_sigma_m="
    << config.vertical_jump_bias_position_velocity_sigma_m << '\n'
    << "enable_vertical_jump_segmented_bias="
    << (config.enable_vertical_jump_segmented_bias ? "true" : "false") << '\n'
    << "vertical_jump_segmented_bias_min_segment_s="
    << config.vertical_jump_segmented_bias_min_segment_s << '\n'
    << "vertical_jump_segmented_bias_max_segments="
    << config.vertical_jump_segmented_bias_max_segments << '\n'
    << "vertical_jump_segmented_bias_slope_merge_threshold_mps2="
    << config.vertical_jump_segmented_bias_slope_merge_threshold_mps2 << '\n'
    << "vertical_jump_bias_highfreq_sigma_scale="
    << config.vertical_jump_bias_highfreq_sigma_scale << '\n'
    << "vertical_jump_bias_highfreq_sigma_max_mps="
    << config.vertical_jump_bias_highfreq_sigma_max_mps << '\n'
    << "enable_vertical_jump_spectral_bias_relaxation="
    << (config.enable_vertical_jump_spectral_bias_relaxation ? "true" : "false") << '\n'
    << "vertical_jump_spectral_window_s=" << config.vertical_jump_spectral_window_s << '\n'
    << "vertical_jump_spectral_stride_s=" << config.vertical_jump_spectral_stride_s << '\n'
    << "vertical_jump_spectral_reference_margin_s="
    << config.vertical_jump_spectral_reference_margin_s << '\n'
    << "vertical_jump_spectral_min_reference_window_count="
    << config.vertical_jump_spectral_min_reference_window_count << '\n'
    << "vertical_jump_spectral_response_trigger_ratio="
    << config.vertical_jump_spectral_response_trigger_ratio << '\n'
    << "vertical_jump_spectral_response_full_ratio="
    << config.vertical_jump_spectral_response_full_ratio << '\n'
    << "vertical_jump_spectral_bias_prior_max_sigma_mps2="
    << config.vertical_jump_spectral_bias_prior_max_sigma_mps2 << '\n'
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
