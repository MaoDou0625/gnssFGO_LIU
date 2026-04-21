#include "offline_lc_minimal/core/StaticImuAlignment.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtsam/geometry/Rot3.h>

namespace offline_lc_minimal {

namespace {

constexpr double kWindowTimeEpsilonS = 1e-9;
constexpr double kVectorNormEpsilon = 1e-12;

double NormalizeYaw(const double yaw_rad) {
  constexpr double kPi = M_PI;
  double normalized = std::fmod(yaw_rad + kPi, 2.0 * kPi);
  if (normalized < 0.0) {
    normalized += 2.0 * kPi;
  }
  return normalized - kPi;
}

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const Eigen::Matrix3d matrix = rotation.matrix();
  const double yaw = std::atan2(matrix(1, 0), matrix(0, 0));
  const double pitch = std::atan2(-matrix(2, 0), std::sqrt(matrix(2, 1) * matrix(2, 1) + matrix(2, 2) * matrix(2, 2)));
  const double roll = std::atan2(matrix(2, 1), matrix(2, 2));
  return Eigen::Vector3d(yaw, pitch, roll);
}

Eigen::Vector3d SafeNormalize(const Eigen::Vector3d &vector) {
  const double vector_norm = vector.norm();
  if (vector_norm <= kVectorNormEpsilon || !std::isfinite(vector_norm)) {
    throw std::runtime_error("cannot normalize a near-zero IMU alignment vector");
  }
  return vector / vector_norm;
}

Eigen::Matrix3d BuildTriadFrame(const Eigen::Vector3d &primary, const Eigen::Vector3d &secondary) {
  const Eigen::Vector3d axis_1 = SafeNormalize(primary);
  const Eigen::Vector3d axis_2 = SafeNormalize(axis_1.cross(secondary));
  const Eigen::Vector3d axis_3 = axis_1.cross(axis_2);

  Eigen::Matrix3d frame;
  frame.col(0) = axis_1;
  frame.col(1) = axis_2;
  frame.col(2) = axis_3;
  return frame;
}

}  // namespace

StaticImuWindowSummary StaticImuAlignment::CollectWindow(
  const std::vector<ImuSample> &imu_samples,
  const double start_time_s,
  const double duration_s,
  const OfflineRunnerConfig &config,
  const bool fallback_to_all_samples) {
  if (imu_samples.empty()) {
    throw std::runtime_error("static IMU alignment requires IMU samples");
  }
  if (duration_s <= 0.0) {
    throw std::runtime_error("static IMU alignment window must be positive");
  }

  const double end_time_s = start_time_s + duration_s;
  Eigen::Vector3d stationary_acc_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d stationary_gyro_sum = Eigen::Vector3d::Zero();
  std::size_t stationary_count = 0;
  Eigen::Vector3d all_acc_sum = Eigen::Vector3d::Zero();
  Eigen::Vector3d all_gyro_sum = Eigen::Vector3d::Zero();
  std::size_t all_count = 0;

  for (const auto &imu_sample : imu_samples) {
    if (imu_sample.time_s + kWindowTimeEpsilonS < start_time_s) {
      continue;
    }
    if (imu_sample.time_s > end_time_s + kWindowTimeEpsilonS) {
      break;
    }

    all_acc_sum += imu_sample.accel_mps2;
    all_gyro_sum += imu_sample.gyro_radps;
    ++all_count;

    const double gyro_norm = imu_sample.gyro_radps.norm();
    const double accel_norm_error = std::abs(imu_sample.accel_mps2.norm() - config.gravity_mps2);
    if (gyro_norm <= config.stationary_gyro_threshold_radps &&
        accel_norm_error <= config.stationary_acc_tolerance_mps2) {
      stationary_acc_sum += imu_sample.accel_mps2;
      stationary_gyro_sum += imu_sample.gyro_radps;
      ++stationary_count;
    }
  }

  StaticImuWindowSummary summary;
  if (stationary_count > 0U) {
    summary.mean_acc_mps2 = stationary_acc_sum / static_cast<double>(stationary_count);
    summary.mean_gyro_radps = stationary_gyro_sum / static_cast<double>(stationary_count);
    summary.sample_count = stationary_count;
    summary.used_stationary_filter = true;
    return summary;
  }

  if (!fallback_to_all_samples || all_count == 0U) {
    return summary;
  }

  summary.mean_acc_mps2 = all_acc_sum / static_cast<double>(all_count);
  summary.mean_gyro_radps = all_gyro_sum / static_cast<double>(all_count);
  summary.sample_count = all_count;
  summary.used_stationary_filter = false;
  return summary;
}

bool StaticImuAlignment::TryEstimateDualVectorInitialization(
  const StaticImuWindowSummary &window_summary,
  const Eigen::Vector3d &earth_rate_enu,
  const OfflineRunnerConfig &config,
  InitialPoseEstimate &initial_pose) {
  if (window_summary.sample_count < static_cast<std::size_t>(std::max(config.imu_dual_vector_min_sample_count, 1))) {
    return false;
  }

  const Eigen::Vector3d gravity_enu(0.0, 0.0, config.gravity_mps2);
  if (earth_rate_enu.norm() <= kVectorNormEpsilon || window_summary.mean_acc_mps2.norm() <= kVectorNormEpsilon ||
      window_summary.mean_gyro_radps.norm() <= kVectorNormEpsilon) {
    return false;
  }

  const double reference_cross_norm =
    SafeNormalize(gravity_enu).cross(SafeNormalize(earth_rate_enu)).norm();
  const double measurement_cross_norm =
    SafeNormalize(window_summary.mean_acc_mps2).cross(SafeNormalize(window_summary.mean_gyro_radps)).norm();
  if (reference_cross_norm < config.imu_dual_vector_min_cross_norm ||
      measurement_cross_norm < config.imu_dual_vector_min_cross_norm) {
    return false;
  }

  const Eigen::Matrix3d navigation_frame = BuildTriadFrame(gravity_enu, earth_rate_enu);
  const Eigen::Matrix3d body_frame = BuildTriadFrame(window_summary.mean_acc_mps2, window_summary.mean_gyro_radps);
  const Eigen::Matrix3d world_from_body = navigation_frame * body_frame.transpose();
  const gtsam::Rot3 orientation(world_from_body);
  const Eigen::Vector3d ypr_rad = Rot3ToYpr(orientation);

  const Eigen::Vector3d predicted_acc_body = world_from_body.transpose() * gravity_enu;
  const Eigen::Vector3d predicted_gyro_body = world_from_body.transpose() * earth_rate_enu;
  const Eigen::Vector3d acc_bias = window_summary.mean_acc_mps2 - predicted_acc_body;
  const Eigen::Vector3d gyro_bias = window_summary.mean_gyro_radps - predicted_gyro_body;

  initial_pose.orientation = orientation;
  initial_pose.yaw_rad = NormalizeYaw(ypr_rad.x());
  initial_pose.pitch_rad = ypr_rad.y();
  initial_pose.roll_rad = ypr_rad.z();
  initial_pose.stationary_sample_count = window_summary.sample_count;
  initial_pose.yaw_source = "imu_dual_vector";
  initial_pose.imu_bias = gtsam::imuBias::ConstantBias(acc_bias, gyro_bias);
  return true;
}

}  // namespace offline_lc_minimal
