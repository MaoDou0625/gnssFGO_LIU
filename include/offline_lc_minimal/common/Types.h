#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Core>
#include <gtsam/navigation/NavState.h>
#include <gtsam/navigation/ImuBias.h>

namespace offline_lc_minimal {

using ErrorStateVector = Eigen::Matrix<double, 15, 1>;
using ErrorStateMatrix = Eigen::Matrix<double, 15, 15>;

enum class GnssFixType : int {
  kRtkFix = 1,
  kRtkFloat = 2,
  kSingle = 3,
  kNoSolution = 4,
};

enum class GnssNoiseModel : int {
  kGaussian = 0,
  kCauchy = 1,
  kHuber = 2,
  kDcs = 3,
  kTukey = 4,
  kGemanMcClure = 5,
  kWelsch = 6,
};

enum class GnssConsistencyGateMode : int {
  kNone = 0,
  kNis = 1,
};

enum class GnssVerticalSigmaMode : int {
  kFromFile = 0,
  kFixed = 1,
};

enum class GnssVerticalDriftReferenceMode : int {
  kMovingAverage = 0,
};

enum class StateMeasSyncStatus : int {
  kDropped = 0,
  kSynchronizedI = 1,
  kSynchronizedJ = 2,
  kInterpolated = 3,
  kCached = 4,
};

inline std::string ToString(const GnssFixType fix_type) {
  switch (fix_type) {
    case GnssFixType::kRtkFix:
      return "RTKFIX";
    case GnssFixType::kRtkFloat:
      return "RTKFLOAT";
    case GnssFixType::kSingle:
      return "SINGLE";
    case GnssFixType::kNoSolution:
    default:
      return "NO_SOLUTION";
  }
}

inline std::string ToString(const GnssNoiseModel noise_model) {
  switch (noise_model) {
    case GnssNoiseModel::kGaussian:
      return "gaussian";
    case GnssNoiseModel::kCauchy:
      return "cauchy";
    case GnssNoiseModel::kHuber:
      return "huber";
    case GnssNoiseModel::kDcs:
      return "dcs";
    case GnssNoiseModel::kTukey:
      return "tukey";
    case GnssNoiseModel::kGemanMcClure:
      return "geman_mcclure";
    case GnssNoiseModel::kWelsch:
      return "welsch";
    default:
      return "unknown";
  }
}

inline std::string ToString(const GnssConsistencyGateMode mode) {
  switch (mode) {
    case GnssConsistencyGateMode::kNis:
      return "nis";
    case GnssConsistencyGateMode::kNone:
    default:
      return "none";
  }
}

inline std::string ToString(const GnssVerticalSigmaMode mode) {
  switch (mode) {
    case GnssVerticalSigmaMode::kFixed:
      return "fixed";
    case GnssVerticalSigmaMode::kFromFile:
    default:
      return "from_file";
  }
}

inline std::string ToString(const GnssVerticalDriftReferenceMode mode) {
  switch (mode) {
    case GnssVerticalDriftReferenceMode::kMovingAverage:
    default:
      return "moving_average";
  }
}

inline std::string ToString(const StateMeasSyncStatus status) {
  switch (status) {
    case StateMeasSyncStatus::kSynchronizedI:
      return "SYNCHRONIZED_I";
    case StateMeasSyncStatus::kSynchronizedJ:
      return "SYNCHRONIZED_J";
    case StateMeasSyncStatus::kInterpolated:
      return "INTERPOLATED";
    case StateMeasSyncStatus::kCached:
      return "CACHED";
    case StateMeasSyncStatus::kDropped:
    default:
      return "DROPPED";
  }
}

struct ImuSample {
  double time_s = 0.0;
  Eigen::Vector3d gyro_radps = Eigen::Vector3d::Zero();
  Eigen::Vector3d accel_mps2 = Eigen::Vector3d::Zero();
};

struct GnssSolutionSample {
  double time_s = 0.0;
  double lat_rad = 0.0;
  double lon_rad = 0.0;
  double h_m = 0.0;
  double sigma_lat_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_lon_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_h_m = std::numeric_limits<double>::quiet_NaN();
  double diff_age_s = std::numeric_limits<double>::quiet_NaN();
  double sol_age_s = std::numeric_limits<double>::quiet_NaN();
  int num_svs = 0;
  int best_sol_status_code = 0;
  int best_pos_type_code = 0;
  int gnssfgo_type_code = 4;
  double dop_pdop = std::numeric_limits<double>::quiet_NaN();
  double dop_hdop = std::numeric_limits<double>::quiet_NaN();
  double dop_vdop = std::numeric_limits<double>::quiet_NaN();
  int pos_source_code = 0;
  int hpd_status_code = 0;
  Eigen::Vector3d enu_position_m = Eigen::Vector3d::Zero();
  bool has_enu_position = false;

  [[nodiscard]] GnssFixType fix_type() const {
    switch (gnssfgo_type_code) {
      case 1:
        return GnssFixType::kRtkFix;
      case 2:
        return GnssFixType::kRtkFloat;
      case 3:
        return GnssFixType::kSingle;
      case 4:
      default:
        return GnssFixType::kNoSolution;
    }
  }

  [[nodiscard]] bool has_valid_position() const {
    return std::isfinite(lat_rad) && std::isfinite(lon_rad) && std::isfinite(h_m);
  }

  [[nodiscard]] bool has_finite_sigma() const {
    return std::isfinite(sigma_lat_m) && std::isfinite(sigma_lon_m) && std::isfinite(sigma_h_m);
  }
};

struct DataSummary {
  std::size_t imu_count = 0;
  std::size_t gnss_count = 0;
  std::size_t gnss_valid_position_count = 0;
  std::size_t gnss_finite_sigma_count = 0;
  std::size_t gnss_no_solution_count = 0;
  double imu_start_s = 0.0;
  double imu_end_s = 0.0;
  double imu_mean_dt_s = 0.0;
  double imu_min_dt_s = 0.0;
  double imu_max_dt_s = 0.0;
  double gnss_start_s = 0.0;
  double gnss_end_s = 0.0;
  double gnss_mean_dt_s = 0.0;
  double gnss_min_dt_s = 0.0;
  double gnss_max_dt_s = 0.0;

  [[nodiscard]] std::string ToMultilineString() const {
    std::ostringstream oss;
    oss << "imu_count=" << imu_count << '\n'
        << "gnss_count=" << gnss_count << '\n'
        << "gnss_valid_position_count=" << gnss_valid_position_count << '\n'
        << "gnss_finite_sigma_count=" << gnss_finite_sigma_count << '\n'
        << "gnss_no_solution_count=" << gnss_no_solution_count << '\n'
        << "imu_start_s=" << imu_start_s << '\n'
        << "imu_end_s=" << imu_end_s << '\n'
        << "imu_mean_dt_s=" << imu_mean_dt_s << '\n'
        << "imu_min_dt_s=" << imu_min_dt_s << '\n'
        << "imu_max_dt_s=" << imu_max_dt_s << '\n'
        << "gnss_start_s=" << gnss_start_s << '\n'
        << "gnss_end_s=" << gnss_end_s << '\n'
        << "gnss_mean_dt_s=" << gnss_mean_dt_s << '\n'
        << "gnss_min_dt_s=" << gnss_min_dt_s << '\n'
        << "gnss_max_dt_s=" << gnss_max_dt_s << '\n';
    return oss.str();
  }
};

struct DataSet {
  std::vector<ImuSample> imu_samples;
  std::vector<GnssSolutionSample> gnss_samples;
  DataSummary summary;
};

struct InitialPoseEstimate {
  gtsam::Rot3 orientation;
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_rad = 0.0;
  std::size_t stationary_sample_count = 0;
  std::string yaw_source = "fallback";
  gtsam::imuBias::ConstantBias imu_bias;
};

struct StateMeasSyncResult {
  bool found_i = false;
  std::size_t key_index_i = 0;
  std::size_t key_index_j = 0;
  double timestamp_i_s = 0.0;
  double timestamp_j_s = 0.0;
  double duration_from_state_i_s = 0.0;
  StateMeasSyncStatus status = StateMeasSyncStatus::kDropped;

  [[nodiscard]] bool state_j_exists() const {
    return status == StateMeasSyncStatus::kSynchronizedI ||
           status == StateMeasSyncStatus::kSynchronizedJ ||
           status == StateMeasSyncStatus::kInterpolated;
  }
};

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
  double covariance_scale = 1.0;
  double covariance_scale_e = 1.0;
  double covariance_scale_n = 1.0;
  double covariance_scale_u = 1.0;
  Eigen::Vector3d prefit_residual_enu_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d postfit_residual_enu_m = Eigen::Vector3d::Zero();
  double prefit_nis = std::numeric_limits<double>::quiet_NaN();
  double postfit_nis = std::numeric_limits<double>::quiet_NaN();
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
  std::size_t error_state_count = 0;
  std::size_t segment_error_count = 0;
  double initial_static_velocity_norm_mean_mps = 0.0;
  double initial_static_velocity_norm_std_mps = 0.0;
  double initial_static_velocity_norm_max_mps = 0.0;
  double static_specific_force_window_std_x_mps2 = 0.0;
  double static_specific_force_window_std_y_mps2 = 0.0;
  double static_specific_force_window_std_z_mps2 = 0.0;
  double static_specific_force_window_rms_xyz_mps2 = 0.0;
  double static_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double static_bgz_radps = std::numeric_limits<double>::quiet_NaN();
  double initial_static_horizontal_drift_max_m = 0.0;
  double initial_static_up_drift_max_m = 0.0;
  double initial_static_3d_drift_max_m = 0.0;
  double gnss_nis_mean = std::numeric_limits<double>::quiet_NaN();
  double gnss_nis_median = std::numeric_limits<double>::quiet_NaN();
  double gnss_nis_p95 = std::numeric_limits<double>::quiet_NaN();
  double axis_2sigma_pass_rate = std::numeric_limits<double>::quiet_NaN();
  std::size_t vertical_gate_inside_count = 0;
  std::size_t vertical_gate_outside_count = 0;
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
  double initial_error = 0.0;
  double final_error = 0.0;
  double origin_lat_rad = 0.0;
  double origin_lon_rad = 0.0;
  double origin_h_m = 0.0;
  double alignment_start_time_s = 0.0;
  double navigation_start_time_s = 0.0;
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
        << "error_state_count=" << error_state_count << '\n'
        << "segment_error_count=" << segment_error_count << '\n'
        << "initial_static_velocity_norm_mean_mps=" << initial_static_velocity_norm_mean_mps << '\n'
        << "initial_static_velocity_norm_std_mps=" << initial_static_velocity_norm_std_mps << '\n'
        << "initial_static_velocity_norm_max_mps=" << initial_static_velocity_norm_max_mps << '\n'
        << "static_specific_force_window_std_x_mps2=" << static_specific_force_window_std_x_mps2 << '\n'
        << "static_specific_force_window_std_y_mps2=" << static_specific_force_window_std_y_mps2 << '\n'
        << "static_specific_force_window_std_z_mps2=" << static_specific_force_window_std_z_mps2 << '\n'
        << "static_specific_force_window_rms_xyz_mps2=" << static_specific_force_window_rms_xyz_mps2 << '\n'
        << "static_baz_mps2=" << static_baz_mps2 << '\n'
        << "static_bgz_radps=" << static_bgz_radps << '\n'
        << "initial_static_horizontal_drift_max_m=" << initial_static_horizontal_drift_max_m << '\n'
        << "initial_static_up_drift_max_m=" << initial_static_up_drift_max_m << '\n'
        << "initial_static_3d_drift_max_m=" << initial_static_3d_drift_max_m << '\n'
        << "gnss_nis_mean=" << gnss_nis_mean << '\n'
        << "gnss_nis_median=" << gnss_nis_median << '\n'
        << "gnss_nis_p95=" << gnss_nis_p95 << '\n'
        << "axis_2sigma_pass_rate=" << axis_2sigma_pass_rate << '\n'
        << "vertical_gate_inside_count=" << vertical_gate_inside_count << '\n'
        << "vertical_gate_outside_count=" << vertical_gate_outside_count << '\n'
        << "feedback_forward_up_slope_10s=" << feedback_forward_up_slope_10s << '\n'
        << "feedback_forward_up_slope_30s=" << feedback_forward_up_slope_30s << '\n'
        << "feedback_forward_horizontal_slope_10s=" << feedback_forward_horizontal_slope_10s << '\n'
        << "feedback_forward_horizontal_slope_30s=" << feedback_forward_horizontal_slope_30s << '\n'
        << "optimized_first30s_mean_baz_mps2=" << optimized_first30s_mean_baz_mps2 << '\n'
        << "optimized_first30s_mean_bgz_radps=" << optimized_first30s_mean_bgz_radps << '\n'
        << "optimized_first30s_mean_roll_rad=" << optimized_first30s_mean_roll_rad << '\n'
        << "optimized_first30s_mean_pitch_rad=" << optimized_first30s_mean_pitch_rad << '\n'
        << "optimized_first30s_mean_yaw_rad=" << optimized_first30s_mean_yaw_rad << '\n'
        << "optimized_first30s_std_baz_mps2=" << optimized_first30s_std_baz_mps2 << '\n'
        << "optimized_first30s_std_pitch_rad=" << optimized_first30s_std_pitch_rad << '\n'
        << "optimized_first30s_std_roll_rad=" << optimized_first30s_std_roll_rad << '\n'
        << "optimized_first30s_up_total_variation_m=" << optimized_first30s_up_total_variation_m << '\n'
        << "optimized_first30s_vz_total_variation_mps=" << optimized_first30s_vz_total_variation_mps << '\n'
        << "forward_first30s_up_total_variation_m=" << forward_first30s_up_total_variation_m << '\n'
        << "forward_first30s_vz_total_variation_mps=" << forward_first30s_vz_total_variation_mps << '\n'
        << "initial_error=" << initial_error << '\n'
        << "final_error=" << final_error << '\n'
        << "origin_lat_rad=" << origin_lat_rad << '\n'
        << "origin_lon_rad=" << origin_lon_rad << '\n'
        << "origin_h_m=" << origin_h_m << '\n'
        << "alignment_start_time_s=" << alignment_start_time_s << '\n'
        << "navigation_start_time_s=" << navigation_start_time_s << '\n'
        << "static_alignment_duration_s=" << static_alignment_duration_s << '\n'
        << "yaw_source=" << yaw_source << '\n';
    return oss.str();
  }
};

struct OfflineRunResult {
  DataSummary data_summary;
  RunSummary run_summary;
  std::vector<TrajectoryRow> initial_static_trajectory;
  std::vector<ReferenceNodeRow> reference_node_trajectory;
  std::vector<ErrorStateRow> error_state_trajectory;
  std::vector<SegmentErrorDiagnostic> segment_error_diagnostics;
  std::vector<TrajectoryRow> trajectory;
  std::vector<ImuRateAvpRow> imu_rate_avp;
  std::vector<ImuRateIntervalDiagnostic> imu_rate_interval_diagnostics;
  std::vector<GnssFactorRecord> gnss_factor_records;
  std::vector<GnssConsistencyRecord> gnss_consistency_records;
};

}  // namespace offline_lc_minimal
