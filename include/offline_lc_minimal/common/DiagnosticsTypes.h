#pragma once

#include <cstddef>
#include <limits>
#include <string>
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
  double center_pull_sigma_m = std::numeric_limits<double>::quiet_NaN();
  double center_pull_deadband_m = std::numeric_limits<double>::quiet_NaN();
  double center_pull_residual_m = std::numeric_limits<double>::quiet_NaN();
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
  double legacy_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double bias_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double attitude_sigma_mps = std::numeric_limits<double>::quiet_NaN();
  double sigma_floor_mps = std::numeric_limits<double>::quiet_NaN();
  double sigma_ceiling_mps = std::numeric_limits<double>::quiet_NaN();
  bool bias_aware_factor = false;
  double reference_ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double optimized_ba_z_ug = std::numeric_limits<double>::quiet_NaN();
  double bias_delta_ug = std::numeric_limits<double>::quiet_NaN();
  double bias_delta_velocity_correction_mps = std::numeric_limits<double>::quiet_NaN();
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
  double initial_mean_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_max_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double initial_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_mean_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_max_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_mean_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_max_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_pose_body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_pitch_range_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_roll_range_rad = std::numeric_limits<double>::quiet_NaN();
  double max_velocity_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double displacement_residual_m = std::numeric_limits<double>::quiet_NaN();
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
