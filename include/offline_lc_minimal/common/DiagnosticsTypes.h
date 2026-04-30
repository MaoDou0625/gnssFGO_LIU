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
  double mean_prefit_nis = std::numeric_limits<double>::quiet_NaN();
  double mean_postfit_nis = std::numeric_limits<double>::quiet_NaN();
  double mean_covariance_scale = std::numeric_limits<double>::quiet_NaN();
  double segment_vertical_rtk_residual_m = std::numeric_limits<double>::quiet_NaN();
  double segment_vertical_gate_inside = std::numeric_limits<double>::quiet_NaN();
  double segment_target_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double segment_feedback_attitude_scale = std::numeric_limits<double>::quiet_NaN();
};

struct GnssConsistencyRecord {
  std::size_t sample_index = 0;
  double raw_time_s = 0.0;
  double corrected_time_s = 0.0;
  bool factor_used = false;
  bool vertical_reference_used = false;
  GnssFixType gnss_fix_type = GnssFixType::kNoSolution;
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  double raw_sigma_h_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_e_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_n_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  double effective_sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  double vertical_gate_threshold_m = std::numeric_limits<double>::quiet_NaN();
  double vertical_gate_inside = std::numeric_limits<double>::quiet_NaN();
  double vertical_sigma_u_used_m = std::numeric_limits<double>::quiet_NaN();
  bool vertical_direct_position_factor_used = false;
  double vertical_feedback_target_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double vertical_feedback_attitude_scale = std::numeric_limits<double>::quiet_NaN();
  double vertical_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double local_prefit_residual_u_m = std::numeric_limits<double>::quiet_NaN();
  double local_postfit_residual_u_m = std::numeric_limits<double>::quiet_NaN();
  double confirmed_inside_before_sample = std::numeric_limits<double>::quiet_NaN();
  long long recovery_anchor_state_index = -1;
  long long nhc_jump_anchor_state_index = -1;
  double nhc_body_vy_mps = std::numeric_limits<double>::quiet_NaN();
  double nhc_body_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double nhc_body_vz_baseline_mps = std::numeric_limits<double>::quiet_NaN();
  double nhc_body_vz_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double nhc_body_vz_jump_mps = std::numeric_limits<double>::quiet_NaN();
  double nhc_body_vy_threshold_mps = std::numeric_limits<double>::quiet_NaN();
  double nhc_body_vz_threshold_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_applied_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_up_anchor_applied_m = std::numeric_limits<double>::quiet_NaN();
  double delta_roll_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_pitch_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_baz_applied_mps2 = std::numeric_limits<double>::quiet_NaN();
  double inside_bias_delta_roll_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double inside_bias_delta_pitch_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double inside_bias_delta_baz_applied_mps2 = std::numeric_limits<double>::quiet_NaN();
  double inside_bias_equivalent_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double inside_bias_residual_delta_m = std::numeric_limits<double>::quiet_NaN();
  double inside_bias_window_dt_s = std::numeric_limits<double>::quiet_NaN();
  long long inside_bias_anchor_state_index = -1;
  long long inside_bias_observation_count = 0;
  double required_up_anchor_correction_m = std::numeric_limits<double>::quiet_NaN();
  double vz_ref_global_smoothed_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_prefit_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_mismatch_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_mismatch_jump_mps = std::numeric_limits<double>::quiet_NaN();
  double jump_candidate_score = std::numeric_limits<double>::quiet_NaN();
  std::string candidate_source = "NONE";
  std::string body_z_jump_direction = "NONE";
  double body_z_signed_delta_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double body_z_direction_score_mps = std::numeric_limits<double>::quiet_NaN();
  double body_z_axis_nav_z = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_state_index = -1;
  double selected_jump_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_window_start_state_index = -1;
  long long selected_jump_window_center_state_index = -1;
  long long selected_jump_window_end_state_index = -1;
  double selected_jump_window_duration_s = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_window_point_count = 0;
  double selected_jump_delta_vz_tail_mps = std::numeric_limits<double>::quiet_NaN();
  double window_velocity_smooth_cost = std::numeric_limits<double>::quiet_NaN();
  double window_height_integral_delta_m = std::numeric_limits<double>::quiet_NaN();
  double future_trend_residual_mean_m = std::numeric_limits<double>::quiet_NaN();
  double future_trend_residual_slope_mps = std::numeric_limits<double>::quiet_NaN();
  double future_trend_cost = std::numeric_limits<double>::quiet_NaN();
  long long future_trend_fix_count = 0;
  std::string recovery_mode = "NONE";
  bool hold_window_passed = false;
  long long local_recovery_iteration_count = 0;
  long long pure_delta_up_anchor_start_iteration = -1;
  double covariance_scale = 1.0;
  double covariance_scale_e = 1.0;
  double covariance_scale_n = 1.0;
  double covariance_scale_u = 1.0;
  double prefit_residual_u_before_local_recovery_m = std::numeric_limits<double>::quiet_NaN();
  double prefit_residual_u_after_local_recovery_m = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d prefit_residual_enu_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d postfit_residual_enu_m = Eigen::Vector3d::Zero();
  double prefit_nis = std::numeric_limits<double>::quiet_NaN();
  double postfit_nis = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalStateCorrectionRow {
  std::size_t sample_index = 0;
  double raw_time_s = std::numeric_limits<double>::quiet_NaN();
  double corrected_time_s = std::numeric_limits<double>::quiet_NaN();
  StateMeasSyncStatus sync_status = StateMeasSyncStatus::kDropped;
  std::size_t state_index = 0;
  double state_time_s = std::numeric_limits<double>::quiet_NaN();
  bool factor_used = false;
  bool reference_available = false;
  double vertical_gate_inside = std::numeric_limits<double>::quiet_NaN();
  bool vertical_direct_position_factor_used = false;
  double measurement_up_m = std::numeric_limits<double>::quiet_NaN();
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double optimized_up_m = std::numeric_limits<double>::quiet_NaN();
  double delta_up_m = std::numeric_limits<double>::quiet_NaN();
  double reference_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double optimized_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double reference_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  double reference_roll_rad = std::numeric_limits<double>::quiet_NaN();
  double optimized_roll_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_roll_rad = std::numeric_limits<double>::quiet_NaN();
  double reference_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double optimized_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double delta_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double prefit_residual_u_m = std::numeric_limits<double>::quiet_NaN();
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
