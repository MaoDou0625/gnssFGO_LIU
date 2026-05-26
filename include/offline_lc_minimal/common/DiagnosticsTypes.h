#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "offline_lc_minimal/common/SensorTypes.h"

namespace offline_lc_minimal {

struct GnssFactorRecord {
  std::size_t sample_index = 0;
  double raw_time_s = 0.0;
  double corrected_time_s = 0.0;
  double state_time_i_s = std::numeric_limits<double>::quiet_NaN();
  double state_time_j_s = std::numeric_limits<double>::quiet_NaN();
  double duration_from_state_i_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  std::size_t synchronized_state_index = 0;
  long long trajectory_row_index_i = -1;
  long long trajectory_row_index_j = -1;
  long long synchronized_trajectory_row_index = -1;
  bool factor_used = false;
  bool vertical_direct_position_factor_used = true;
  GnssFixType gnss_fix_type = GnssFixType::kNoSolution;
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  Eigen::Vector3d measurement_enu_m = Eigen::Vector3d::Zero();
  std::string vertical_reference_source = "raw_rtk";
  double raw_rtk_up_m = std::numeric_limits<double>::quiet_NaN();
  double vertical_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double vertical_reference_highfreq_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  std::string vertical_reference_skip_reason = "UNSET";
  double residual_m = std::numeric_limits<double>::quiet_NaN();
};

struct TrajectoryRow {
  double time_s = 0.0;
  Eigen::Vector3d enu_position_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d enu_velocity_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypr_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d omega_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  bool gnss_factor_used = false;
  GnssFixType gnss_fix_type = GnssFixType::kNoSolution;
  double gnss_residual_m = std::numeric_limits<double>::quiet_NaN();
};

struct ReferenceNodeRow {
  double time_s = 0.0;
  Eigen::Vector3d enu_position_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d enu_velocity_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypr_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
};

struct ReferenceNodeState {
  double time_s = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias bias;
  gtsam::Vector3 omega = gtsam::Vector3::Zero();
};

struct BodyZSeedImuDiagnosticRow {
  double time_s = 0.0;
  double relative_time_s = 0.0;
  double body_z_specific_force_mps2 = std::numeric_limits<double>::quiet_NaN();
  double gravity_projection_z_mps2 = std::numeric_limits<double>::quiet_NaN();
  double body_z_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double body_z_acc_1s_smooth_mps2 = std::numeric_limits<double>::quiet_NaN();
  double integrated_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double integrated_body_z_velocity_0p2s_smooth_mps = std::numeric_limits<double>::quiet_NaN();
  double integrated_body_z_velocity_1s_smooth_mps = std::numeric_limits<double>::quiet_NaN();
  double signed_step_metric_mps = std::numeric_limits<double>::quiet_NaN();
  double downward_score_mps = std::numeric_limits<double>::quiet_NaN();
  double upward_score_mps = std::numeric_limits<double>::quiet_NaN();
  double body_z_axis_nav_z = std::numeric_limits<double>::quiet_NaN();
};

struct BodyZSeedJumpWindowRow {
  std::string direction = "UNKNOWN";
  long long selection_level = 0;
  long long start_state_index = -1;
  long long center_state_index = -1;
  long long end_state_index = -1;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double center_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double start_relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double center_relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double duration_s = std::numeric_limits<double>::quiet_NaN();
  double pre_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double post_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double signed_delta_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double direction_score_mps = std::numeric_limits<double>::quiet_NaN();
  double signed_step_metric_mps = std::numeric_limits<double>::quiet_NaN();
  double level_threshold_mps = std::numeric_limits<double>::quiet_NaN();
  double level_max_peak_mps = std::numeric_limits<double>::quiet_NaN();
  double level_noise_floor_mps = std::numeric_limits<double>::quiet_NaN();
  double min_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double max_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double mean_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double body_z_axis_nav_z = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_init_mps = std::numeric_limits<double>::quiet_NaN();
};

struct BodyZBiasReestimateSegmentRow {
  std::size_t segment_index = 0;
  std::string source_type = "BODY_Z_BIAS";
  std::size_t source_bias_window_index = 0;
  long long source_outage_window_index = -1;
  long long start_state_index = -1;
  long long end_state_index = -1;
  long long anchor_state_index = -1;
  double bias_window_start_time_s = std::numeric_limits<double>::quiet_NaN();
  double bias_window_end_time_s = std::numeric_limits<double>::quiet_NaN();
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double duration_s = std::numeric_limits<double>::quiet_NaN();
  double detected_bias_delta_mps2 = std::numeric_limits<double>::quiet_NaN();
  double reference_ba_z_mps2 = std::numeric_limits<double>::quiet_NaN();
  double prior_target_ba_z_mps2 = std::numeric_limits<double>::quiet_NaN();
  double prior_sigma_mps2 = std::numeric_limits<double>::quiet_NaN();
  std::size_t initialized_state_count = 0;
  bool prior_factor_added = false;
  std::string skip_reason = "UNSET";
};

struct ErrorStateRow {
  double time_s = 0.0;
  ErrorStateVector state = ErrorStateVector::Zero();
};

struct SegmentErrorDiagnostic {
  std::size_t segment_index = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  Eigen::Vector3d dtheta_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d dv_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d dp_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d dbg_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d dba_mps2 = Eigen::Vector3d::Zero();
  std::size_t gnss_factor_count = 0;
  double mean_postfit_nis = std::numeric_limits<double>::quiet_NaN();
};

struct GnssConsistencyRecord {
  std::size_t sample_index = 0;
  double raw_time_s = 0.0;
  double corrected_time_s = 0.0;
  bool factor_used = false;
  GnssFixType gnss_fix_type = GnssFixType::kNoSolution;
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  double raw_sigma_h_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_e_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_n_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  double effective_sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  double vertical_sigma_u_used_m = std::numeric_limits<double>::quiet_NaN();
  bool vertical_direct_position_factor_used = false;
  Eigen::Vector3d postfit_residual_enu_m = Eigen::Vector3d::Zero();
  double postfit_nis = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalEnvelopeDiagnosticRow {
  std::size_t sample_index = 0;
  double raw_time_s = std::numeric_limits<double>::quiet_NaN();
  double corrected_time_s = std::numeric_limits<double>::quiet_NaN();
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  std::size_t synchronized_state_index = 0;
  double state_time_i_s = std::numeric_limits<double>::quiet_NaN();
  double state_time_j_s = std::numeric_limits<double>::quiet_NaN();
  double duration_from_state_i_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_used = false;
  double rtk_up_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  double half_width_m = std::numeric_limits<double>::quiet_NaN();
  double predicted_up_m = std::numeric_limits<double>::quiet_NaN();
  double raw_residual_m = std::numeric_limits<double>::quiet_NaN();
  double violation_m = std::numeric_limits<double>::quiet_NaN();
  bool inside_envelope = false;
  bool center_pull_factor_used = false;
  std::string center_pull_reference_type = "raw_rtk";
  double center_pull_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double rtk_drift_estimate_m = std::numeric_limits<double>::quiet_NaN();
  double center_pull_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double center_pull_deadband_m = std::numeric_limits<double>::quiet_NaN();
  double center_pull_residual_m = std::numeric_limits<double>::quiet_NaN();
};

struct RtkVerticalDriftReferenceDiagnosticRow {
  std::size_t sample_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double raw_rtk_up_m = std::numeric_limits<double>::quiet_NaN();
  double nav_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  std::string nav_reference_source = "UNSET";
  std::string static_window_source = "NONE";
  double causal_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double full_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double full_minus_causal_nav_reference_m = std::numeric_limits<double>::quiet_NaN();
  double causal_reference_boundary_time_s = std::numeric_limits<double>::quiet_NaN();
  double residual_m = std::numeric_limits<double>::quiet_NaN();
  double constant_bias_m = std::numeric_limits<double>::quiet_NaN();
  double drift_estimate_m = std::numeric_limits<double>::quiet_NaN();
  double corrected_center_up_m = std::numeric_limits<double>::quiet_NaN();
  double lowpass_center_up_m = std::numeric_limits<double>::quiet_NaN();
  double lowpass_delta_m = std::numeric_limits<double>::quiet_NaN();
  double lowpass_cutoff_hz = std::numeric_limits<double>::quiet_NaN();
  double white_residual_m = std::numeric_limits<double>::quiet_NaN();
  double gate_half_width_m = std::numeric_limits<double>::quiet_NaN();
  double gate_observation_m = std::numeric_limits<double>::quiet_NaN();
  double gate_violation_m = std::numeric_limits<double>::quiet_NaN();
  double gate_weight = std::numeric_limits<double>::quiet_NaN();
  double effective_white_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double drift_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double white_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double tau_s = std::numeric_limits<double>::quiet_NaN();
  int drift_segment_index = -1;
  std::string drift_segment_role = "UNSEGMENTED";
  bool outage_boundary_blocked = false;
  bool lowpass_applied = false;
  bool static_window_flag = false;
  bool valid = false;
  std::string skip_reason = "UNSET";
};

struct Stage3VerticalReferenceDiagnosticRow {
  std::size_t state_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double stage2_up_m = std::numeric_limits<double>::quiet_NaN();
  double stage2_lowpass_up_m = std::numeric_limits<double>::quiet_NaN();
  double lowpass_delta_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_up_m = std::numeric_limits<double>::quiet_NaN();
  double residual_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_m = std::numeric_limits<double>::quiet_NaN();
  std::string constraint_mode = "gaussian";
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double envelope_half_width_m = std::numeric_limits<double>::quiet_NaN();
  double envelope_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double envelope_overflow_residual_m = std::numeric_limits<double>::quiet_NaN();
  bool center_pull_factor_added = false;
  double center_pull_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double center_pull_deadband_m = std::numeric_limits<double>::quiet_NaN();
  double center_pull_residual_m = std::numeric_limits<double>::quiet_NaN();
  bool outside_gate = false;
  bool factor_added = false;
  std::string skip_reason = "UNSET";
};

struct LateStaticFeatureDiagnosticRow {
  std::size_t window_index = 0;
  double window_start_time_s = std::numeric_limits<double>::quiet_NaN();
  double window_end_time_s = std::numeric_limits<double>::quiet_NaN();
  double window_center_time_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t rtkfix_sample_count = 0;
  std::size_t imu_sample_count = 0;
  bool overlaps_initial_static = false;
  bool overlaps_rtk_outage = false;
  bool excluded_from_detection = false;
  bool valid_features = false;
  double rtk_horizontal_speed_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double rtk_horizontal_range_m = std::numeric_limits<double>::quiet_NaN();
  double rtk_up_median_m = std::numeric_limits<double>::quiet_NaN();
  double rtk_up_range_m = std::numeric_limits<double>::quiet_NaN();
  double imu_gyro_norm_rms_radps = std::numeric_limits<double>::quiet_NaN();
  double imu_gyro_norm_p95_radps = std::numeric_limits<double>::quiet_NaN();
  double imu_acc_norm_mean_mps2 = std::numeric_limits<double>::quiet_NaN();
  double imu_acc_norm_std_mps2 = std::numeric_limits<double>::quiet_NaN();
  double log_rtk_horizontal_speed_rms = std::numeric_limits<double>::quiet_NaN();
  double log_rtk_horizontal_range = std::numeric_limits<double>::quiet_NaN();
  double log_imu_gyro_norm_rms = std::numeric_limits<double>::quiet_NaN();
  double log_imu_gyro_norm_p95 = std::numeric_limits<double>::quiet_NaN();
  bool pass_rtk_speed_rms = false;
  bool pass_rtk_range = false;
  bool pass_gyro_rms = false;
  bool pass_gyro_p95 = false;
  bool pass_acc_std = false;
  bool pass_all = false;
  std::string skip_reason = "UNSET";
};

struct LateStaticThresholdDiagnosticRow {
  std::string feature_name = "UNSET";
  std::string method = "UNSET";
  bool valid = false;
  double threshold_value = std::numeric_limits<double>::quiet_NaN();
  double log_threshold_value = std::numeric_limits<double>::quiet_NaN();
  std::size_t sample_count = 0;
  std::size_t static_side_count = 0;
  std::size_t dynamic_side_count = 0;
  double separation_score = std::numeric_limits<double>::quiet_NaN();
  std::string skip_reason = "UNSET";
};

struct LateStaticWindowRow {
  std::size_t window_index = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double duration_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t feature_window_count = 0;
  bool valid = false;
  double rtk_median_up_m = std::numeric_limits<double>::quiet_NaN();
  double rtk_up_range_m = std::numeric_limits<double>::quiet_NaN();
  double vz_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double up_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double height_hold_sigma_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t vz_factor_count = 0;
  std::size_t up_factor_count = 0;
  std::size_t height_hold_factor_count = 0;
  double max_abs_vz_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double max_abs_up_residual_m = std::numeric_limits<double>::quiet_NaN();
  double max_abs_height_hold_residual_m = std::numeric_limits<double>::quiet_NaN();
  std::string skip_reason = "UNSET";
  std::vector<std::size_t> vz_factor_state_indices;
  std::vector<std::size_t> up_factor_state_indices;
  std::vector<std::pair<std::size_t, std::size_t>> height_hold_factor_state_index_pairs;
};

struct RtkOutageCausalNavReferenceRow {
  std::size_t sample_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double raw_rtk_up_m = std::numeric_limits<double>::quiet_NaN();
  double causal_nav_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double causal_up_m = std::numeric_limits<double>::quiet_NaN();
  double causal_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double source_processing_end_time_s = std::numeric_limits<double>::quiet_NaN();
  double outage_boundary_time_s = std::numeric_limits<double>::quiet_NaN();
  long long source_state_index_i = -1;
  long long source_state_index_j = -1;
  double state_time_i_s = std::numeric_limits<double>::quiet_NaN();
  double state_time_j_s = std::numeric_limits<double>::quiet_NaN();
  double duration_from_state_i_s = std::numeric_limits<double>::quiet_NaN();
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  bool valid = false;
  std::string source_type = "UNSET";
  std::string skip_reason = "UNSET";
};

struct RtkOutageCausalStateReferenceRow {
  std::size_t state_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double reference_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double source_processing_end_time_s = std::numeric_limits<double>::quiet_NaN();
  double outage_boundary_time_s = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool fence_factor_added = false;
  std::string source_type = "UNSET";
  std::string skip_reason = "UNSET";
};

struct RtkOutageWindowRow {
  std::size_t window_index = 0;
  std::size_t pre_sample_index = 0;
  std::size_t post_sample_index = 0;
  std::size_t pre_anchor_state_index = 0;
  std::size_t post_anchor_state_index = 0;
  double pre_anchor_time_s = std::numeric_limits<double>::quiet_NaN();
  double post_anchor_time_s = std::numeric_limits<double>::quiet_NaN();
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double duration_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t interior_state_count = 0;
  std::size_t rejected_sample_count = 0;
  std::size_t body_z_jump_overlap_count = 0;
  bool initial_value_smoothing_applied = false;
  std::size_t initial_value_smoothed_state_count = 0;
  double pre_anchor_up_m = std::numeric_limits<double>::quiet_NaN();
  double post_anchor_up_m = std::numeric_limits<double>::quiet_NaN();
  double post_anchor_up_offset_m = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::size_t position_ramp_factor_count = 0;
  std::size_t velocity_delta_factor_count = 0;
  std::size_t velocity_delta_skipped_body_z_jump_count = 0;
  std::string skip_reason = "UNSET";
};

struct RtkOutageBatchSegmentRow {
  std::size_t segment_index = 0;
  std::string segment_role = "UNSET";
  long long source_outage_window_index = -1;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double duration_s = std::numeric_limits<double>::quiet_NaN();
  bool planned = false;
  bool vertical_boundary_jump_allowed = false;
  std::string start_boundary_source = "UNSET";
  std::string end_boundary_source = "UNSET";
  std::string skip_reason = "UNSET";
};

struct RtkOutageBoundaryReferenceRow {
  std::size_t window_index = 0;
  std::string boundary_role = "UNSET";
  std::string source_type = "UNSET";
  double target_time_s = std::numeric_limits<double>::quiet_NaN();
  long long target_state_index = -1;
  bool valid = false;
  bool has_up = false;
  bool has_vz = false;
  bool has_ba_z = false;
  bool add_up_constraint = false;
  bool add_vz_constraint = false;
  bool add_ba_z_constraint = false;
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double reference_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double reference_ba_z_mps2 = std::numeric_limits<double>::quiet_NaN();
  double up_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double vz_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double ba_z_sigma_mps2 = std::numeric_limits<double>::quiet_NaN();
  std::string skip_reason = "UNSET";
};

struct RtkOutageRecoveryReferenceRow {
  std::size_t window_index = 0;
  double outage_end_time_s = std::numeric_limits<double>::quiet_NaN();
  double fit_start_time_s = std::numeric_limits<double>::quiet_NaN();
  double fit_end_time_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t valid_fix_sample_count = 0;
  int min_fix_sample_count = 0;
  bool valid = false;
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double reference_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double first_sample_time_s = std::numeric_limits<double>::quiet_NaN();
  double last_sample_time_s = std::numeric_limits<double>::quiet_NaN();
  double first_sample_up_m = std::numeric_limits<double>::quiet_NaN();
  double last_sample_up_m = std::numeric_limits<double>::quiet_NaN();
  std::string skip_reason = "UNSET";
};

struct RtkOutageBiasContinuityPolicyRow {
  std::size_t window_index = 0;
  std::string boundary_role = "UNSET";
  bool ba_z_continuity_allowed = true;
  bool overlaps_reestimate_segment = false;
  double max_abs_detected_delta_ug = std::numeric_limits<double>::quiet_NaN();
  double threshold_ug = std::numeric_limits<double>::quiet_NaN();
  std::string reset_reason = "NONE";
};

struct RtkOutageBoundaryDiagnosticRow {
  std::size_t window_index = 0;
  std::string boundary_role = "UNSET";
  std::string source_type = "UNSET";
  std::size_t target_state_index = 0;
  double target_time_s = std::numeric_limits<double>::quiet_NaN();
  bool valid = false;
  bool up_factor_added = false;
  bool vz_factor_added = false;
  bool ba_z_factor_added = false;
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_up_m = std::numeric_limits<double>::quiet_NaN();
  double up_residual_m = std::numeric_limits<double>::quiet_NaN();
  double up_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double reference_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double reference_ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double optimized_ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double ba_z_residual_ug = std::numeric_limits<double>::quiet_NaN();
  double ba_z_sigma_ug = std::numeric_limits<double>::quiet_NaN();
  std::string skip_reason = "UNSET";
};

struct RtkVelocityDiagnosticRow {
  std::size_t sample_index = 0;
  std::size_t state_index = 0;
  double raw_time_s = std::numeric_limits<double>::quiet_NaN();
  double corrected_time_s = std::numeric_limits<double>::quiet_NaN();
  double state_time_s = std::numeric_limits<double>::quiet_NaN();
  double window_dt_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  double sigma_mps = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d rtk_velocity_mps =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d optimized_velocity_mps =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d velocity_residual_mps =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double horizontal_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double rtk_body_x_mps = std::numeric_limits<double>::quiet_NaN();
  double rtk_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double rtk_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_body_x_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double body_y_residual_mps = std::numeric_limits<double>::quiet_NaN();
};

struct StaticAlignmentValidationRow {
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double up_delta_m = std::numeric_limits<double>::quiet_NaN();
  double vz_mps = std::numeric_limits<double>::quiet_NaN();
  double ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double global_ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double ba_z_minus_global_ug = std::numeric_limits<double>::quiet_NaN();
  double static_bias_gm_residual_ug = std::numeric_limits<double>::quiet_NaN();
  double static_height_residual_m = std::numeric_limits<double>::quiet_NaN();
  double rtk_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double rtk_reference_residual_m = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalVelocityDeltaDiagnosticRow {
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  int outer_pass = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double dt_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  bool in_jump_padding = false;
  bool target_clamped = false;
  double raw_target_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double target_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double residual_mps = std::numeric_limits<double>::quiet_NaN();
  double sigma_mps = std::numeric_limits<double>::quiet_NaN();
  std::string sigma_model = "legacy";
  std::string sigma_context = "GLOBAL";
  double sigma_output_scale = std::numeric_limits<double>::quiet_NaN();
  double legacy_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double bias_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double attitude_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double sigma_floor_mps = std::numeric_limits<double>::quiet_NaN();
  double sigma_ceiling_mps = std::numeric_limits<double>::quiet_NaN();
  double adaptive_motion_score = std::numeric_limits<double>::quiet_NaN();
  double adaptive_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double adaptive_sigma_ratio = std::numeric_limits<double>::quiet_NaN();
  double local_horizontal_speed_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double local_vz_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double local_vz_range_mps = std::numeric_limits<double>::quiet_NaN();
  double local_target_acc_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  bool bias_aware_factor = false;
  double reference_ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double optimized_ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double bias_delta_ug = std::numeric_limits<double>::quiet_NaN();
  double bias_delta_velocity_correction_mps = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalMotionAdaptiveReweightingDiagnosticRow {
  int outer_pass = 0;
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double dt_s = std::numeric_limits<double>::quiet_NaN();
  double motion_score = std::numeric_limits<double>::quiet_NaN();
  std::string stability_class = "UNSET";
  double horizontal_speed_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_range_mps = std::numeric_limits<double>::quiet_NaN();
  double target_vertical_acc_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  double dvz_sigma_before_mps = std::numeric_limits<double>::quiet_NaN();
  double dvz_sigma_after_mps = std::numeric_limits<double>::quiet_NaN();
  double baz_gm_sigma_before_ug = std::numeric_limits<double>::quiet_NaN();
  double baz_gm_sigma_after_ug = std::numeric_limits<double>::quiet_NaN();
  bool in_jump_padding = false;
  std::string skip_reason = "UNSET";
};

struct VerticalPositionVelocityConsistencyDiagnosticRow {
  std::string constraint_type = "adjacent";
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  std::size_t state_count = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double dt_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> state_times_s;
  std::string interval_type = "UNSET";
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  double sigma_m = std::numeric_limits<double>::quiet_NaN();
  double initial_delta_z_m = std::numeric_limits<double>::quiet_NaN();
  double initial_trapezoid_vz_integral_m = std::numeric_limits<double>::quiet_NaN();
  double initial_mismatch_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_delta_z_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_trapezoid_vz_integral_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_mismatch_m = std::numeric_limits<double>::quiet_NaN();
  double normalized_residual = std::numeric_limits<double>::quiet_NaN();
};

struct AttitudeReferenceDiagnosticRow {
  std::size_t state_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  Eigen::Vector3d reference_ypr_rad = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d optimized_ypr_rad = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double residual_x_rad = std::numeric_limits<double>::quiet_NaN();
  double residual_y_rad = std::numeric_limits<double>::quiet_NaN();
  double residual_z_rad = std::numeric_limits<double>::quiet_NaN();
  double residual_norm_rad = std::numeric_limits<double>::quiet_NaN();
};

struct RelativeYawReferenceDiagnosticRow {
  std::size_t edge_index = 0;
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double time_i_s = std::numeric_limits<double>::quiet_NaN();
  double time_j_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  double sigma_rad = std::numeric_limits<double>::quiet_NaN();
  double reference_delta_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_delta_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double residual_yaw_rad = std::numeric_limits<double>::quiet_NaN();
};

struct RtkOutageAttitudeHoldDiagnosticRow {
  std::size_t window_index = 0;
  std::string constraint_type = "UNSET";
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double time_i_s = std::numeric_limits<double>::quiet_NaN();
  double time_j_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  double sigma_rad = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d reference_ypr_i_rad =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d reference_ypr_j_rad =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d optimized_ypr_i_rad =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d optimized_ypr_j_rad =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d residual_rad =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double residual_norm_rad = std::numeric_limits<double>::quiet_NaN();
  double reference_delta_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_delta_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  double residual_yaw_rad = std::numeric_limits<double>::quiet_NaN();
};

struct RtkOutageVelocityDelta3dDiagnosticRow {
  std::size_t window_index = 0;
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double time_i_s = std::numeric_limits<double>::quiet_NaN();
  double time_j_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  double sigma_mps = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d target_delta_v_mps =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d optimized_delta_v_mps =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  Eigen::Vector3d residual_mps =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  double residual_norm_mps = std::numeric_limits<double>::quiet_NaN();
};

struct BodyZHorizontalLeakageDiagnosticRow {
  bool enabled = false;
  bool estimate_valid = false;
  std::string velocity_source = "UNSET";
  std::string skip_reason = "UNSET";
  std::size_t candidate_sample_count = 0;
  std::size_t used_sample_count = 0;
  std::size_t skipped_window_count = 0;
  std::size_t skipped_low_speed_count = 0;
  std::size_t skipped_invalid_count = 0;
  double min_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double huber_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double max_abs_coeff_rad = std::numeric_limits<double>::quiet_NaN();
  double leak_x_rad = std::numeric_limits<double>::quiet_NaN();
  double leak_y_rad = std::numeric_limits<double>::quiet_NaN();
  double raw_rms_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double raw_max_abs_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double corrected_rms_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double corrected_max_abs_body_z_mps = std::numeric_limits<double>::quiet_NaN();
};

struct BodyZNHCDiagnosticRow {
  std::size_t window_index = 0;
  std::string window_type = "UNSET";
  bool from_jump_window = false;
  std::size_t source_window_index = 0;
  std::size_t source_window_count = 0;
  std::size_t start_state_index = 0;
  std::size_t end_state_index = 0;
  std::size_t state_count = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double duration_s = std::numeric_limits<double>::quiet_NaN();
  double actual_state_span_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  std::size_t velocity_factor_count = 0;
  std::size_t displacement_factor_count = 0;
  double velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double displacement_sigma_m = std::numeric_limits<double>::quiet_NaN();
  bool strict_weighting_enabled = false;
  double target_velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double applied_velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double target_displacement_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double applied_displacement_sigma_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t velocity_state_duplicate_count = 0;
  std::size_t interval_overlap_count = 0;
  std::vector<std::size_t> velocity_factor_state_indices;
  bool horizontal_leakage_correction_enabled = false;
  double horizontal_leakage_x_rad = std::numeric_limits<double>::quiet_NaN();
  double horizontal_leakage_y_rad = std::numeric_limits<double>::quiet_NaN();
  double initial_mean_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_max_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double initial_mean_abs_corrected_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_max_abs_corrected_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_corrected_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_mean_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_max_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_mean_abs_corrected_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_max_abs_corrected_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_corrected_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_mean_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_max_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_pitch_range_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_roll_range_rad = std::numeric_limits<double>::quiet_NaN();
  double max_velocity_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double displacement_residual_m = std::numeric_limits<double>::quiet_NaN();
};

struct BodyZNHCStateDiagnosticRow {
  std::size_t window_index = 0;
  std::size_t state_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  std::string nhc_region_type = "UNSET";
  bool velocity_factor_used = false;
  double effective_velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double fixed_axis_x = std::numeric_limits<double>::quiet_NaN();
  double fixed_axis_y = std::numeric_limits<double>::quiet_NaN();
  double fixed_axis_z = std::numeric_limits<double>::quiet_NaN();
  double vx_mps = std::numeric_limits<double>::quiet_NaN();
  double vy_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_mps = std::numeric_limits<double>::quiet_NaN();
  double horizontal_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double v_body_x_mps = std::numeric_limits<double>::quiet_NaN();
  double v_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double raw_v_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double horizontal_leakage_x_rad = std::numeric_limits<double>::quiet_NaN();
  double horizontal_leakage_y_rad = std::numeric_limits<double>::quiet_NaN();
  double leakage_correction_mps = std::numeric_limits<double>::quiet_NaN();
  double corrected_v_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double fixed_horizontal_projection_mps = std::numeric_limits<double>::quiet_NaN();
  double fixed_vertical_projection_mps = std::numeric_limits<double>::quiet_NaN();
  double fixed_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_axis_x = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_axis_y = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_axis_z = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_horizontal_projection_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_vertical_projection_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
};

struct Stage2MountLeakageDiagnosticRow {
  long long batch_segment_index = -1;
  std::string batch_segment_role = "UNSEGMENTED";
  bool enabled = false;
  bool estimate_valid = false;
  std::string skip_reason = "UNSET";
  std::size_t used_sample_count = 0;
  double prior_sigma_rad = std::numeric_limits<double>::quiet_NaN();
  double initial_k_zx_rad = std::numeric_limits<double>::quiet_NaN();
  double initial_k_zy_rad = std::numeric_limits<double>::quiet_NaN();
  double initial_k_yx_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_k_zx_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_k_zy_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_k_yx_rad = std::numeric_limits<double>::quiet_NaN();
  double prior_residual_norm = std::numeric_limits<double>::quiet_NaN();
  double initial_raw_y_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_raw_y_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_vehicle_y_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_vehicle_y_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_raw_y_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_raw_y_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vehicle_y_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vehicle_y_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_raw_z_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_raw_z_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_vehicle_z_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_vehicle_z_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_raw_z_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_raw_z_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vehicle_z_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vehicle_z_max_abs_mps = std::numeric_limits<double>::quiet_NaN();
};

struct Stage1OutageBodyYEnvelopeRow {
  std::size_t window_index = 0;
  double outage_start_time_s = std::numeric_limits<double>::quiet_NaN();
  double outage_end_time_s = std::numeric_limits<double>::quiet_NaN();
  double pre_window_start_time_s = std::numeric_limits<double>::quiet_NaN();
  double pre_window_end_time_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t candidate_sample_count = 0;
  std::size_t used_sample_count = 0;
  std::size_t skipped_low_speed_count = 0;
  std::size_t skipped_invalid_count = 0;
  bool valid = false;
  double min_speed_mps = std::numeric_limits<double>::quiet_NaN();
  double mean_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double rmse_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double p95_abs_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double deadband_mps = std::numeric_limits<double>::quiet_NaN();
  double sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double huber_k = std::numeric_limits<double>::quiet_NaN();
  std::size_t factor_count = 0;
  std::string skip_reason = "UNSET";
};

struct Stage1OutageBodyYStateDiagnosticRow {
  std::size_t window_index = 0;
  std::size_t state_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  double mean_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double rmse_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double deadband_mps = std::numeric_limits<double>::quiet_NaN();
  double sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double body_y_axis_x = std::numeric_limits<double>::quiet_NaN();
  double body_y_axis_y = std::numeric_limits<double>::quiet_NaN();
  double body_y_axis_z = std::numeric_limits<double>::quiet_NaN();
  double optimized_vx_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vy_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double centered_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double deadband_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double normalized_residual = std::numeric_limits<double>::quiet_NaN();
};

struct Stage2VehicleNHCStateDiagnosticRow {
  long long batch_segment_index = -1;
  std::string batch_segment_role = "UNSEGMENTED";
  std::size_t window_index = 0;
  std::size_t state_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  std::string nhc_region_type = "UNSET";
  bool velocity_factor_used = false;
  double effective_vehicle_y_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double effective_vehicle_z_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double vx_mps = std::numeric_limits<double>::quiet_NaN();
  double vy_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_mps = std::numeric_limits<double>::quiet_NaN();
  double v_body_x_mps = std::numeric_limits<double>::quiet_NaN();
  double v_body_y_mps = std::numeric_limits<double>::quiet_NaN();
  double v_body_z_mps = std::numeric_limits<double>::quiet_NaN();
  double k_zx_rad = std::numeric_limits<double>::quiet_NaN();
  double k_zy_rad = std::numeric_limits<double>::quiet_NaN();
  double k_yx_rad = std::numeric_limits<double>::quiet_NaN();
  double vehicle_y_correction_mps = std::numeric_limits<double>::quiet_NaN();
  double vehicle_z_correction_from_x_mps = std::numeric_limits<double>::quiet_NaN();
  double vehicle_z_correction_from_y_mps = std::numeric_limits<double>::quiet_NaN();
  double v_vehicle_y_mps = std::numeric_limits<double>::quiet_NaN();
  double v_vehicle_z_mps = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalJumpMaskedImuDiagnosticRow {
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  std::string factor_type = "UNSET";
  bool overlap_jump_padding = false;
  bool masked_z_position = false;
  bool masked_vz = false;
};

struct VerticalJumpImpulseDiagnosticRow {
  std::size_t span_index = 0;
  std::size_t source_window_index = 0;
  std::size_t source_window_count = 0;
  std::size_t start_state_index = 0;
  std::size_t end_state_index = 0;
  std::size_t pre_anchor_state_index = 0;
  std::size_t post_anchor_state_index = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  std::size_t replaced_imu_factor_count = 0;
  double imu_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double detected_delta_vz_init_mps = std::numeric_limits<double>::quiet_NaN();
  double detected_signed_delta_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double prior_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double estimated_jump_impulse_mps = std::numeric_limits<double>::quiet_NaN();
  double corrected_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double residual_mps = std::numeric_limits<double>::quiet_NaN();
  double pre_anchor_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double post_anchor_vz_mps = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalJumpBiasDiagnosticRow {
  std::size_t span_index = 0;
  std::size_t segment_index = 0;
  std::size_t segment_count = 1;
  std::size_t bias_key_index = 0;
  std::size_t source_window_index = 0;
  std::size_t source_window_count = 0;
  std::size_t start_state_index = 0;
  std::size_t end_state_index = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_added = false;
  std::string skip_reason = "UNSET";
  std::size_t replaced_imu_factor_count = 0;
  std::size_t velocity_factor_count = 0;
  std::size_t position_velocity_factor_count = 0;
  double source_window_duration_s = std::numeric_limits<double>::quiet_NaN();
  double factor_duration_s = 0.0;
  double imu_delta_vz_mps = 0.0;
  double detected_signed_delta_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double detected_bias_mps2 = std::numeric_limits<double>::quiet_NaN();
  bool used_segmented_estimate = false;
  double prior_sigma_mps2 = std::numeric_limits<double>::quiet_NaN();
  double effective_prior_sigma_mps2 = std::numeric_limits<double>::quiet_NaN();
  std::string spectral_skip_reason = "DISABLED";
  std::size_t spectral_target_window_count = 0;
  std::size_t spectral_reference_window_count = 0;
  double spectral_total_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double spectral_band_30_60_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double spectral_band_60_120_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double spectral_band_120_250_rms_ratio = std::numeric_limits<double>::quiet_NaN();
  double spectral_response_ratio = std::numeric_limits<double>::quiet_NaN();
  double spectral_score = 0.0;
  double base_velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double highfreq_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  double highfreq_p95_abs_mps2 = std::numeric_limits<double>::quiet_NaN();
  double highfreq_sigma_inflation_mps = 0.0;
  double velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double position_velocity_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double estimated_bias_mps2 = std::numeric_limits<double>::quiet_NaN();
  double corrected_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double residual_mps = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalJumpVelocityRampDiagnosticRow {
  std::size_t window_index = 0;
  std::size_t start_state_index = 0;
  std::size_t end_state_index = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t factor_count = 0;
  std::string skip_reason = "UNSET";
  double pre_vz_mean_mps = std::numeric_limits<double>::quiet_NaN();
  double post_vz_mean_mps = std::numeric_limits<double>::quiet_NaN();
  double jump_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double inside_vz_min_mps = std::numeric_limits<double>::quiet_NaN();
  double inside_vz_max_mps = std::numeric_limits<double>::quiet_NaN();
  double inside_vz_range_mps = std::numeric_limits<double>::quiet_NaN();
  double ramp_residual_max_mps = std::numeric_limits<double>::quiet_NaN();
  double ramp_residual_p95_mps = std::numeric_limits<double>::quiet_NaN();
  double inside_up_min_m = std::numeric_limits<double>::quiet_NaN();
  double inside_up_max_m = std::numeric_limits<double>::quiet_NaN();
  double inside_up_range_m = std::numeric_limits<double>::quiet_NaN();
  double position_ramp_residual_max_m = std::numeric_limits<double>::quiet_NaN();
  double position_ramp_residual_p95_m = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalJumpContinuityDiagnosticRow {
  std::size_t window_index = 0;
  std::size_t start_state_index = 0;
  std::size_t end_state_index = 0;
  std::size_t pre_anchor_state_index = 0;
  std::size_t post_anchor_state_index = 0;
  std::size_t pre_context_start_state_index = 0;
  std::size_t pre_context_end_state_index = 0;
  std::size_t post_context_start_state_index = 0;
  std::size_t post_context_end_state_index = 0;
  std::size_t pre_context_state_count = 0;
  std::size_t post_context_state_count = 0;
  std::size_t velocity_context_factor_count = 0;
  bool context_mean_continuity_factor_added = false;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  bool entry_factor_added = false;
  bool exit_factor_added = false;
  std::string skip_reason = "UNSET";
  double entry_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double exit_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double entry_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double exit_residual_mps = std::numeric_limits<double>::quiet_NaN();
  bool entry_position_velocity_factor_added = false;
  bool exit_position_velocity_factor_added = false;
  double entry_delta_z_m = std::numeric_limits<double>::quiet_NaN();
  double entry_velocity_integral_m = std::numeric_limits<double>::quiet_NaN();
  double entry_zv_mismatch_m = std::numeric_limits<double>::quiet_NaN();
  double exit_delta_z_m = std::numeric_limits<double>::quiet_NaN();
  double exit_velocity_integral_m = std::numeric_limits<double>::quiet_NaN();
  double exit_zv_mismatch_m = std::numeric_limits<double>::quiet_NaN();
  double pre_context_mean_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double post_context_mean_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double context_mean_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double context_mean_continuity_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double max_pre_context_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double max_post_context_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double max_inside_vz_range_mps = std::numeric_limits<double>::quiet_NaN();
  double max_boundary_step_mps = std::numeric_limits<double>::quiet_NaN();
  double max_boundary_zv_mismatch_m = std::numeric_limits<double>::quiet_NaN();
  double max_position_velocity_residual_m = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalStateCorrectionRow {
  std::size_t sample_index = 0;
  double raw_time_s = std::numeric_limits<double>::quiet_NaN();
  double corrected_time_s = std::numeric_limits<double>::quiet_NaN();
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  std::size_t state_index = 0;
  double state_time_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_used = false;
  bool vertical_direct_position_factor_used = false;
  double measurement_up_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_up_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_roll_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double postfit_residual_u_m = std::numeric_limits<double>::quiet_NaN();
};

struct ImuRateAvpRow {
  double time_s = 0.0;
  Eigen::Vector3d enu_position_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d enu_velocity_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypr_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
};

struct ImuRateIntervalDiagnostic {
  std::size_t interval_index = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  std::size_t imu_sample_count = 0;
  std::size_t emitted_sample_count = 0;
  bool used_interval = false;
  std::string status = "UNSET";
};

}  // namespace offline_lc_minimal
