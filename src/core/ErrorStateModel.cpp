#include "offline_lc_minimal/core/ErrorStateModel.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace offline_lc_minimal {

namespace {

constexpr double kTimeEpsilonS = 1e-9;

Eigen::Matrix3d Skew(const Eigen::Vector3d &vector) {
  Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
  skew(0, 1) = -vector.z();
  skew(0, 2) = vector.y();
  skew(1, 0) = vector.z();
  skew(1, 2) = -vector.x();
  skew(2, 0) = -vector.y();
  skew(2, 1) = vector.x();
  return skew;
}

ErrorStateMatrix DiscretizeTransition(const ErrorStateMatrix &f_matrix, const double dt_s) {
  const ErrorStateMatrix fdt = f_matrix * dt_s;
  return ErrorStateMatrix::Identity() + fdt + 0.5 * (fdt * fdt);
}

ErrorStateMatrix BuildProcessNoiseCovariance(const double dt_s, const OfflineRunnerConfig &config) {
  ErrorStateMatrix q_continuous = ErrorStateMatrix::Zero();
  const double q_theta = std::pow(config.imu_sigma_gyro, 2.0) * config.error_process_noise_scale;
  const double q_velocity = std::pow(config.imu_sigma_acc, 2.0) * config.error_process_noise_scale;
  const double q_position = std::pow(0.5 * config.imu_sigma_acc * dt_s, 2.0) * config.error_process_noise_scale;
  const double q_dbg = std::pow(config.bias_gyro_sigma, 2.0) * config.error_process_noise_scale;
  const double q_dba = std::pow(config.bias_acc_sigma, 2.0) * config.error_process_noise_scale;

  q_continuous.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * q_theta;
  q_continuous.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * q_velocity;
  q_continuous.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * q_position;
  q_continuous.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * q_dbg;
  q_continuous.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * q_dba;

  ErrorStateMatrix q_discrete = q_continuous * std::max(dt_s, 1e-3);
  q_discrete = 0.5 * (q_discrete + q_discrete.transpose());
  q_discrete += ErrorStateMatrix::Identity() * 1e-12;
  return q_discrete;
}

}  // namespace

std::vector<double> BuildErrorStateTimestamps(
  const double start_time_s,
  const double end_time_s,
  const double error_state_frequency_hz) {
  if (error_state_frequency_hz <= 0.0) {
    throw std::runtime_error("error_state_frequency_hz must be positive");
  }
  if (end_time_s <= start_time_s) {
    throw std::runtime_error("error state timeline end must be after start");
  }

  const double dt_s = 1.0 / error_state_frequency_hz;
  std::vector<double> timestamps;
  timestamps.push_back(start_time_s);
  for (double next_time_s = start_time_s + dt_s; next_time_s < end_time_s - kTimeEpsilonS; next_time_s += dt_s) {
    timestamps.push_back(next_time_s);
  }
  if (timestamps.back() < end_time_s - kTimeEpsilonS) {
    timestamps.push_back(end_time_s);
  } else {
    timestamps.back() = end_time_s;
  }
  if (timestamps.size() < 2U) {
    throw std::runtime_error("error state timeline must contain at least two nodes");
  }
  return timestamps;
}

ErrorNodeInterpolation MapTimeToErrorNodes(const std::vector<double> &error_timestamps_s, const double time_s) {
  if (error_timestamps_s.size() < 2U) {
    throw std::runtime_error("error state timeline must contain at least two nodes");
  }

  if (time_s <= error_timestamps_s.front() + kTimeEpsilonS) {
    return ErrorNodeInterpolation{0U, 1U, 0.0};
  }
  if (time_s >= error_timestamps_s.back() - kTimeEpsilonS) {
    return ErrorNodeInterpolation{error_timestamps_s.size() - 2U, error_timestamps_s.size() - 1U, 1.0};
  }

  const auto upper_it = std::upper_bound(error_timestamps_s.begin(), error_timestamps_s.end(), time_s);
  const std::size_t right_index = static_cast<std::size_t>(std::distance(error_timestamps_s.begin(), upper_it));
  const std::size_t left_index = right_index - 1U;
  const double left_time_s = error_timestamps_s[left_index];
  const double right_time_s = error_timestamps_s[right_index];
  const double alpha = std::clamp((time_s - left_time_s) / (right_time_s - left_time_s), 0.0, 1.0);
  return ErrorNodeInterpolation{left_index, right_index, alpha};
}

ErrorStateVector InterpolateErrorState(
  const ErrorStateVector &left_state,
  const ErrorStateVector &right_state,
  const double alpha) {
  return (1.0 - alpha) * left_state + alpha * right_state;
}

ErrorProcessInterval BuildErrorProcessInterval(
  const ReferenceNodeState &reference_state,
  const ImuWindowIntegration &imu_window,
  const double end_time_s,
  const GeoReference &geo_reference,
  const OfflineRunnerConfig &config) {
  if (end_time_s <= reference_state.time_s) {
    throw std::runtime_error("error process interval end must be after start");
  }

  const double dt_s = end_time_s - reference_state.time_s;
  const Eigen::Vector3d corrected_acc_body =
    imu_window.mean_acc_mps2 - reference_state.bias.accelerometer();
  const Eigen::Vector3d specific_force_nav =
    reference_state.pose.rotation().matrix() * corrected_acc_body;
  const Eigen::Vector3d earth_rate_enu = geo_reference.EarthRateEnu();
  const Eigen::Matrix3d r_nb = reference_state.pose.rotation().matrix();

  ErrorStateMatrix f_matrix = ErrorStateMatrix::Zero();
  f_matrix.block<3, 3>(0, 0) = -Skew(earth_rate_enu);
  f_matrix.block<3, 3>(0, 9) = -r_nb;
  f_matrix.block<3, 3>(3, 0) = -Skew(specific_force_nav);
  f_matrix.block<3, 3>(3, 3) = -2.0 * Skew(earth_rate_enu);
  f_matrix.block<3, 3>(3, 12) = -r_nb;
  f_matrix.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
  f_matrix.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * (-1.0 / config.tau_gyro_bias_s);
  f_matrix.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * (-1.0 / config.tau_acc_bias_s);

  ErrorProcessInterval interval;
  interval.start_time_s = reference_state.time_s;
  interval.end_time_s = end_time_s;
  interval.phi = DiscretizeTransition(f_matrix, dt_s);
  interval.q = BuildProcessNoiseCovariance(dt_s, config);
  return interval;
}

}  // namespace offline_lc_minimal
