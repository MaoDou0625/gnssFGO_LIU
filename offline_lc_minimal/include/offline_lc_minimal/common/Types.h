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

enum class GnssFixType : int {
  kRtkFix = 1,
  kRtkFloat = 2,
  kSingle = 3,
  kNoSolution = 4,
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
};

struct TrajectoryRow {
  double time_s = 0.0;
  Eigen::Vector3d enu_position_m = Eigen::Vector3d::Zero();
  Eigen::Vector3d enu_velocity_mps = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypr_rad = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d bias_gyro = Eigen::Vector3d::Zero();
  bool gnss_factor_used = false;
  GnssFixType gnss_fix_type = GnssFixType::kNoSolution;
  double gnss_residual_m = std::numeric_limits<double>::quiet_NaN();
};

struct RunSummary {
  bool gnss_enabled = true;
  std::size_t state_count = 0;
  std::size_t gnss_factor_count = 0;
  std::size_t dropped_no_solution_count = 0;
  std::size_t dropped_nonfinite_sigma_count = 0;
  std::size_t dropped_bad_status_count = 0;
  double initial_error = 0.0;
  double final_error = 0.0;
  double origin_lat_rad = 0.0;
  double origin_lon_rad = 0.0;
  double origin_h_m = 0.0;
  std::string yaw_source = "fallback";

  [[nodiscard]] std::string ToMultilineString() const {
    std::ostringstream oss;
    oss << "gnss_enabled=" << (gnss_enabled ? "true" : "false") << '\n'
        << "state_count=" << state_count << '\n'
        << "gnss_factor_count=" << gnss_factor_count << '\n'
        << "dropped_no_solution_count=" << dropped_no_solution_count << '\n'
        << "dropped_nonfinite_sigma_count=" << dropped_nonfinite_sigma_count << '\n'
        << "dropped_bad_status_count=" << dropped_bad_status_count << '\n'
        << "initial_error=" << initial_error << '\n'
        << "final_error=" << final_error << '\n'
        << "origin_lat_rad=" << origin_lat_rad << '\n'
        << "origin_lon_rad=" << origin_lon_rad << '\n'
        << "origin_h_m=" << origin_h_m << '\n'
        << "yaw_source=" << yaw_source << '\n';
    return oss.str();
  }
};

struct OfflineRunResult {
  DataSummary data_summary;
  RunSummary run_summary;
  std::vector<TrajectoryRow> trajectory;
};

}  // namespace offline_lc_minimal
