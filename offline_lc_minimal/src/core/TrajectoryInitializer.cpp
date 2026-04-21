#include "offline_lc_minimal/core/TrajectoryInitializer.h"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace offline_lc_minimal {

namespace {

double NormalizeYaw(const double yaw_rad) {
  constexpr double kPi = M_PI;
  double normalized = std::fmod(yaw_rad + kPi, 2.0 * kPi);
  if (normalized < 0.0) {
    normalized += 2.0 * kPi;
  }
  return normalized - kPi;
}

}  // namespace

InitialPoseEstimate TrajectoryInitializer::Estimate(
  const std::vector<ImuSample> &imu_samples,
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::size_t start_gnss_index,
  const OfflineRunnerConfig &config) {
  if (imu_samples.empty()) {
    throw std::runtime_error("trajectory initialization requires IMU samples");
  }
  if (start_gnss_index >= gnss_samples.size()) {
    throw std::runtime_error("trajectory initialization start_gnss_index out of range");
  }

  const double start_time_s = gnss_samples[start_gnss_index].time_s;
  const double stationary_window_end_s = start_time_s + config.stationary_window_s;

  Eigen::Vector3d accel_sum = Eigen::Vector3d::Zero();
  std::size_t stationary_count = 0;
  for (const auto &imu_sample : imu_samples) {
    if (imu_sample.time_s > stationary_window_end_s) {
      break;
    }
    const double gyro_norm = imu_sample.gyro_radps.norm();
    const double accel_norm_error = std::abs(imu_sample.accel_mps2.norm() - config.gravity_mps2);
    if (gyro_norm <= config.stationary_gyro_threshold_radps &&
        accel_norm_error <= config.stationary_acc_tolerance_mps2) {
      accel_sum += imu_sample.accel_mps2;
      ++stationary_count;
    }
  }

  if (stationary_count == 0U) {
    for (const auto &imu_sample : imu_samples) {
      if (imu_sample.time_s > stationary_window_end_s) {
        break;
      }
      accel_sum += imu_sample.accel_mps2;
      ++stationary_count;
    }
  }

  if (stationary_count == 0U) {
    throw std::runtime_error("trajectory initialization could not collect an initial IMU window");
  }

  Eigen::Vector3d gravity_alignment = accel_sum / static_cast<double>(stationary_count);
  if (gravity_alignment.z() < 0.0) {
    gravity_alignment *= -1.0;
  }

  const double roll_rad = std::atan2(gravity_alignment.y(), gravity_alignment.z());
  const double pitch_rad =
    std::atan2(-gravity_alignment.x(),
               std::sqrt(gravity_alignment.y() * gravity_alignment.y() +
                         gravity_alignment.z() * gravity_alignment.z()));

  double yaw_rad = config.fallback_initial_yaw_rad;
  std::string yaw_source = "fallback";
  const auto &start_sample = gnss_samples[start_gnss_index];
  if (start_sample.has_enu_position) {
    for (std::size_t index = start_gnss_index + 1; index < gnss_samples.size(); ++index) {
      const auto &sample = gnss_samples[index];
      if (!sample.has_enu_position) {
        continue;
      }
      const Eigen::Vector2d delta_xy = (sample.enu_position_m - start_sample.enu_position_m).head<2>();
      if (delta_xy.norm() >= config.yaw_min_distance_m) {
        yaw_rad = std::atan2(delta_xy.y(), delta_xy.x());
        yaw_source = "displacement";
        break;
      }
    }
  }

  InitialPoseEstimate initial_pose;
  initial_pose.roll_rad = roll_rad;
  initial_pose.pitch_rad = pitch_rad;
  initial_pose.yaw_rad = NormalizeYaw(yaw_rad);
  initial_pose.stationary_sample_count = stationary_count;
  initial_pose.yaw_source = yaw_source;
  initial_pose.orientation = gtsam::Rot3::Ypr(initial_pose.yaw_rad, initial_pose.pitch_rad, initial_pose.roll_rad);
  return initial_pose;
}

}  // namespace offline_lc_minimal
