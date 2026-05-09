#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Rot3.h>
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

enum class VerticalConstraintMode : int {
  kDirectZ = 0,
  kEnvelope = 1,
};

enum class VerticalEnvelopeCenterSigmaMode : int {
  kFixed = 0,
  kGateSigma = 1,
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

inline std::string ToString(const VerticalConstraintMode mode) {
  switch (mode) {
    case VerticalConstraintMode::kEnvelope:
      return "envelope";
    case VerticalConstraintMode::kDirectZ:
    default:
      return "direct_z";
  }
}

inline std::string ToString(const VerticalEnvelopeCenterSigmaMode mode) {
  switch (mode) {
    case VerticalEnvelopeCenterSigmaMode::kGateSigma:
      return "gate_sigma";
    case VerticalEnvelopeCenterSigmaMode::kFixed:
    default:
      return "fixed";
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

}  // namespace offline_lc_minimal
