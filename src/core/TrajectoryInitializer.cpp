#include "offline_lc_minimal/core/TrajectoryInitializer.h"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "offline_lc_minimal/core/StaticImuAlignment.h"

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

void UpdateBiasForOrientation(
  const StaticImuWindowSummary &window_summary,
  const Eigen::Vector3d &earth_rate_enu,
  const OfflineRunnerConfig &config,
  InitialPoseEstimate &initial_pose) {
  const Eigen::Vector3d gravity_enu(0.0, 0.0, config.gravity_mps2);
  const Eigen::Vector3d predicted_acc_body =
    initial_pose.orientation.matrix().transpose() * gravity_enu;
  const Eigen::Vector3d predicted_gyro_body =
    initial_pose.orientation.matrix().transpose() * earth_rate_enu;
  initial_pose.imu_bias = gtsam::imuBias::ConstantBias(
    window_summary.mean_acc_mps2 - predicted_acc_body,
    window_summary.mean_gyro_radps - predicted_gyro_body);
}

InitialPoseEstimate EstimateGravityPitchRollWithYaw(
  const StaticImuWindowSummary &window_summary,
  const double yaw_rad,
  const std::string &yaw_source,
  const Eigen::Vector3d &earth_rate_enu,
  const OfflineRunnerConfig &config) {
  Eigen::Vector3d gravity_alignment = window_summary.mean_acc_mps2;
  if (gravity_alignment.z() < 0.0) {
    gravity_alignment *= -1.0;
  }

  InitialPoseEstimate initial_pose;
  initial_pose.roll_rad = std::atan2(gravity_alignment.y(), gravity_alignment.z());
  initial_pose.pitch_rad =
    std::atan2(-gravity_alignment.x(),
               std::sqrt(gravity_alignment.y() * gravity_alignment.y() +
                         gravity_alignment.z() * gravity_alignment.z()));
  initial_pose.yaw_rad = NormalizeYaw(yaw_rad);
  initial_pose.stationary_sample_count = window_summary.sample_count;
  initial_pose.yaw_source = yaw_source;
  initial_pose.orientation =
    gtsam::Rot3::Ypr(initial_pose.yaw_rad, initial_pose.pitch_rad, initial_pose.roll_rad);
  UpdateBiasForOrientation(window_summary, earth_rate_enu, config, initial_pose);
  return initial_pose;
}

InitialPoseEstimate OverrideYawAndReestimateBias(
  InitialPoseEstimate initial_pose,
  const StaticImuWindowSummary &window_summary,
  const double yaw_rad,
  const Eigen::Vector3d &earth_rate_enu,
  const OfflineRunnerConfig &config) {
  initial_pose.yaw_rad = NormalizeYaw(yaw_rad);
  initial_pose.yaw_source = "override";
  initial_pose.stationary_sample_count = window_summary.sample_count;
  initial_pose.orientation =
    gtsam::Rot3::Ypr(initial_pose.yaw_rad, initial_pose.pitch_rad, initial_pose.roll_rad);
  UpdateBiasForOrientation(window_summary, earth_rate_enu, config, initial_pose);
  return initial_pose;
}

}  // namespace

InitialPoseEstimate TrajectoryInitializer::Estimate(
  const std::vector<ImuSample> &imu_samples,
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::size_t start_gnss_index,
  const double alignment_start_time_s,
  const double alignment_end_time_s,
  const double navigation_start_time_s,
  const Eigen::Vector3d &earth_rate_enu,
  const std::vector<std::size_t> &yaw_candidate_indices,
  const OfflineRunnerConfig &config) {
  if (imu_samples.empty()) {
    throw std::runtime_error("trajectory initialization requires IMU samples");
  }
  if (start_gnss_index >= gnss_samples.size()) {
    throw std::runtime_error("trajectory initialization start_gnss_index out of range");
  }
  if (alignment_end_time_s < alignment_start_time_s) {
    throw std::runtime_error("trajectory initialization alignment_end_time_s must be >= alignment_start_time_s");
  }
  if (navigation_start_time_s < alignment_end_time_s) {
    throw std::runtime_error("trajectory initialization navigation_start_time_s must be >= alignment_end_time_s");
  }

  InitialPoseEstimate initial_pose;

  const double dual_vector_duration_s = config.static_alignment_duration_s > 0.0
                                          ? alignment_end_time_s - alignment_start_time_s
                                          : config.imu_dual_vector_window_s;

  if (config.enable_initial_yaw_override) {
    const auto override_window = StaticImuAlignment::CollectWindow(
      imu_samples,
      alignment_start_time_s,
      dual_vector_duration_s,
      config,
      true);
    if (override_window.sample_count == 0U) {
      throw std::runtime_error("initial yaw override could not collect an initial IMU window");
    }

    if (config.prefer_imu_initial_yaw) {
      InitialPoseEstimate dual_vector_pose;
      if (StaticImuAlignment::TryEstimateDualVectorInitialization(
            override_window,
            earth_rate_enu,
            config,
            dual_vector_pose)) {
        return OverrideYawAndReestimateBias(
          dual_vector_pose,
          override_window,
          config.initial_yaw_override_rad,
          earth_rate_enu,
          config);
      }
    }

    return EstimateGravityPitchRollWithYaw(
      override_window,
      config.initial_yaw_override_rad,
      "override",
      earth_rate_enu,
      config);
  }

  if (config.prefer_imu_initial_yaw) {
    const auto dual_vector_window = StaticImuAlignment::CollectWindow(
      imu_samples,
      alignment_start_time_s,
      dual_vector_duration_s,
      config,
      false);
    if (StaticImuAlignment::TryEstimateDualVectorInitialization(
          dual_vector_window,
          earth_rate_enu,
          config,
          initial_pose)) {
      return initial_pose;
    }
  }

  const auto gravity_window = StaticImuAlignment::CollectWindow(
    imu_samples,
    alignment_start_time_s,
    config.stationary_window_s,
    config,
    true);
  if (gravity_window.sample_count == 0U) {
    throw std::runtime_error("trajectory initialization could not collect an initial IMU window");
  }

  double yaw_rad = config.fallback_initial_yaw_rad;
  std::string yaw_source = "fallback";
  const auto &start_sample = gnss_samples[start_gnss_index];
  if (start_sample.has_enu_position) {
    for (const std::size_t index : yaw_candidate_indices) {
      if (index <= start_gnss_index || index >= gnss_samples.size()) {
        continue;
      }
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

  return EstimateGravityPitchRollWithYaw(
    gravity_window,
    yaw_rad,
    yaw_source,
    earth_rate_enu,
    config);
}

}  // namespace offline_lc_minimal
