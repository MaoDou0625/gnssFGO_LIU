#include "offline_lc_minimal/core/ImuRateAvpReconstructor.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>

#include <Eigen/QR>

#include "offline_lc_minimal/core/ImuIntegrationUtils.h"

namespace offline_lc_minimal {

namespace {

constexpr double kTimeEpsilonS = 1e-9;
constexpr int kBackwardSolveMaxIterations = 8;
constexpr double kBackwardErrorTolerance = 1e-8;
constexpr double kBackwardDeltaTolerance = 1e-9;

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

gtsam::imuBias::ConstantBias InterpolateBias(
  const gtsam::imuBias::ConstantBias &bias_i,
  const gtsam::imuBias::ConstantBias &bias_j,
  const double alpha) {
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  const gtsam::Vector3 bias_acc =
    (1.0 - clamped_alpha) * bias_i.accelerometer() + clamped_alpha * bias_j.accelerometer();
  const gtsam::Vector3 bias_gyro =
    (1.0 - clamped_alpha) * bias_i.gyroscope() + clamped_alpha * bias_j.gyroscope();
  return gtsam::imuBias::ConstantBias(bias_acc, bias_gyro);
}

gtsam::NavState InterpolateNavState(
  const gtsam::NavState &state_i,
  const gtsam::NavState &state_j,
  const double alpha) {
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  const gtsam::Rot3 rotation = state_i.pose().rotation().slerp(clamped_alpha, state_j.pose().rotation());
  const gtsam::Vector3 position(
    (1.0 - clamped_alpha) * state_i.position().x() + clamped_alpha * state_j.position().x(),
    (1.0 - clamped_alpha) * state_i.position().y() + clamped_alpha * state_j.position().y(),
    (1.0 - clamped_alpha) * state_i.position().z() + clamped_alpha * state_j.position().z());
  const gtsam::Vector3 velocity = (1.0 - clamped_alpha) * state_i.v() + clamped_alpha * state_j.v();
  return gtsam::NavState(rotation, gtsam::Point3(position.x(), position.y(), position.z()), velocity);
}

gtsam::NavState BlendNavStates(
  const gtsam::NavState &forward_state,
  const gtsam::NavState &backward_state,
  const double alpha) {
  const double clamped_alpha = std::clamp(alpha, 0.0, 1.0);
  const gtsam::Rot3 rotation = forward_state.pose().rotation().slerp(clamped_alpha, backward_state.pose().rotation());
  const gtsam::Vector3 position(
    (1.0 - clamped_alpha) * forward_state.position().x() + clamped_alpha * backward_state.position().x(),
    (1.0 - clamped_alpha) * forward_state.position().y() + clamped_alpha * backward_state.position().y(),
    (1.0 - clamped_alpha) * forward_state.position().z() + clamped_alpha * backward_state.position().z());
  const gtsam::Vector3 velocity =
    (1.0 - clamped_alpha) * forward_state.v() + clamped_alpha * backward_state.v();
  return gtsam::NavState(rotation, gtsam::Point3(position.x(), position.y(), position.z()), velocity);
}

std::vector<std::vector<std::size_t>> AssignImuSamplesToIntervals(
  const std::vector<ImuSample> &imu_samples,
  const std::vector<OptimizedNodeState> &node_states) {
  const std::size_t interval_count = node_states.size() - 1U;
  std::vector<std::vector<std::size_t>> interval_samples(interval_count);
  std::vector<double> node_timestamps;
  node_timestamps.reserve(node_states.size());
  for (const auto &node_state : node_states) {
    node_timestamps.push_back(node_state.time_s);
  }

  for (std::size_t sample_index = 0; sample_index < imu_samples.size(); ++sample_index) {
    const double sample_time_s = imu_samples[sample_index].time_s;
    if (sample_time_s < node_timestamps.front() - kTimeEpsilonS ||
        sample_time_s > node_timestamps.back() + kTimeEpsilonS) {
      continue;
    }

    const auto upper_it =
      std::upper_bound(node_timestamps.begin(), node_timestamps.end(), sample_time_s + kTimeEpsilonS);
    std::size_t interval_index = 0U;
    if (upper_it == node_timestamps.begin()) {
      interval_index = 0U;
    } else {
      interval_index = static_cast<std::size_t>(std::distance(node_timestamps.begin(), upper_it) - 1);
      if (interval_index >= interval_count) {
        interval_index = interval_count - 1U;
      }
    }
    interval_samples[interval_index].push_back(sample_index);
  }

  return interval_samples;
}

bool SolveBackwardState(
  const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements,
  const gtsam::NavState &target_state_j,
  const gtsam::imuBias::ConstantBias &bias_t,
  const gtsam::NavState &initial_guess,
  gtsam::NavState *solution) {
  gtsam::NavState estimate = initial_guess;

  for (int iteration = 0; iteration < kBackwardSolveMaxIterations; ++iteration) {
    Eigen::Matrix<double, 9, 9> jacobian_state_i;
    const gtsam::Vector9 error =
      preintegrated_measurements.computeError(estimate, target_state_j, bias_t, jacobian_state_i, boost::none, boost::none);
    if (!error.allFinite() || !jacobian_state_i.allFinite()) {
      return false;
    }
    if (error.norm() <= kBackwardErrorTolerance) {
      *solution = estimate;
      return true;
    }

    const Eigen::ColPivHouseholderQR<Eigen::Matrix<double, 9, 9>> solver(jacobian_state_i);
    const gtsam::Vector9 delta = solver.solve(-error);
    if (!delta.allFinite()) {
      return false;
    }

    estimate = estimate.retract(delta);
    if (delta.norm() <= kBackwardDeltaTolerance) {
      *solution = estimate;
      return true;
    }
  }

  Eigen::Matrix<double, 9, 9> jacobian_state_i;
  const gtsam::Vector9 final_error =
    preintegrated_measurements.computeError(estimate, target_state_j, bias_t, jacobian_state_i, boost::none, boost::none);
  if (!final_error.allFinite() || final_error.norm() > 1e-5) {
    return false;
  }

  *solution = estimate;
  return true;
}

ImuRateAvpRow MakeOutputRow(
  const double time_s,
  const gtsam::NavState &state,
  const gtsam::imuBias::ConstantBias &bias) {
  ImuRateAvpRow row;
  row.time_s = time_s;
  row.enu_position_m = Eigen::Vector3d(state.position().x(), state.position().y(), state.position().z());
  row.enu_velocity_mps = Eigen::Vector3d(state.v().x(), state.v().y(), state.v().z());
  row.ypr_rad = Rot3ToYpr(state.pose().rotation());
  row.bias_acc = bias.accelerometer();
  row.bias_gyro = bias.gyroscope();
  return row;
}

class PrefixImuIntegrator {
 public:
  PrefixImuIntegrator(
    const std::vector<ImuSample> &imu_samples,
    const double start_time_s,
    const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &imu_params,
    const gtsam::imuBias::ConstantBias &bias_hat)
      : imu_samples_(imu_samples),
        preintegrated_measurements_(imu_params, bias_hat),
        current_time_s_(start_time_s) {
    const auto lower_it = std::lower_bound(
      imu_samples_.begin(),
      imu_samples_.end(),
      start_time_s,
      [](const ImuSample &sample, const double timestamp_s) { return sample.time_s < timestamp_s; });

    next_index_ = static_cast<std::size_t>(std::distance(imu_samples_.begin(), lower_it));
    const std::size_t hold_index = next_index_ > 0U ? next_index_ - 1U : 0U;
    held_sample_ = imu_samples_[hold_index];
  }

  bool IntegrateTo(const double target_time_s) {
    if (target_time_s < current_time_s_ - kTimeEpsilonS) {
      return false;
    }

    while (next_index_ < imu_samples_.size()) {
      const auto &current_sample = imu_samples_[next_index_];
      if (current_sample.time_s > target_time_s + kTimeEpsilonS) {
        break;
      }
      const double dt_s = current_sample.time_s - current_time_s_;
      if (dt_s > 0.0) {
        preintegrated_measurements_.integrateMeasurement(
          held_sample_.accel_mps2,
          held_sample_.gyro_radps,
          dt_s);
      }
      held_sample_ = current_sample;
      current_time_s_ = current_sample.time_s;
      ++next_index_;
    }

    if (target_time_s > current_time_s_ + kTimeEpsilonS) {
      preintegrated_measurements_.integrateMeasurement(
        held_sample_.accel_mps2,
        held_sample_.gyro_radps,
        target_time_s - current_time_s_);
      current_time_s_ = target_time_s;
    }
    return true;
  }

  [[nodiscard]] const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements() const {
    return preintegrated_measurements_;
  }

 private:
  const std::vector<ImuSample> &imu_samples_;
  gtsam::PreintegratedCombinedMeasurements preintegrated_measurements_;
  ImuSample held_sample_;
  std::size_t next_index_ = 0U;
  double current_time_s_ = 0.0;
};

}  // namespace

ImuRateAvpReconstructionResult ImuRateAvpReconstructor::Reconstruct(
  const std::vector<ImuSample> &imu_samples,
  const std::vector<OptimizedNodeState> &node_states,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &imu_params,
  const bool verbose) {
  ImuRateAvpReconstructionResult result;
  if (imu_samples.empty() || node_states.size() < 2U) {
    return result;
  }

  const std::vector<std::vector<std::size_t>> interval_samples = AssignImuSamplesToIntervals(imu_samples, node_states);
  result.diagnostics.reserve(interval_samples.size());

  for (std::size_t interval_index = 0; interval_index + 1U < node_states.size(); ++interval_index) {
    const auto &node_i = node_states[interval_index];
    const auto &node_j = node_states[interval_index + 1U];
    const auto nav_state_i = gtsam::NavState(node_i.pose, node_i.velocity);
    const auto nav_state_j = gtsam::NavState(node_j.pose, node_j.velocity);

    ImuRateIntervalDiagnostic diagnostic;
    diagnostic.interval_index = interval_index;
    diagnostic.start_time_s = node_i.time_s;
    diagnostic.end_time_s = node_j.time_s;
    diagnostic.imu_sample_count = interval_samples[interval_index].size();

    if (node_j.time_s <= node_i.time_s + kTimeEpsilonS) {
      diagnostic.status = "INVALID_INTERVAL";
      result.diagnostics.push_back(diagnostic);
      continue;
    }

    if (interval_samples[interval_index].empty()) {
      diagnostic.status = "NO_IMU_SAMPLES";
      result.diagnostics.push_back(diagnostic);
      continue;
    }

    std::vector<ImuRateAvpRow> interval_rows;
    interval_rows.reserve(interval_samples[interval_index].size());
    bool interval_failed = false;
    std::string failure_status;
    PrefixImuIntegrator prefix_integrator(imu_samples, node_i.time_s, imu_params, node_i.bias);

    for (const std::size_t sample_index : interval_samples[interval_index]) {
      const double sample_time_s = imu_samples[sample_index].time_s;
      const double alpha =
        std::clamp((sample_time_s - node_i.time_s) / (node_j.time_s - node_i.time_s), 0.0, 1.0);
      const gtsam::imuBias::ConstantBias bias_t = InterpolateBias(node_i.bias, node_j.bias, alpha);

      if (std::abs(sample_time_s - node_i.time_s) <= kTimeEpsilonS) {
        interval_rows.push_back(MakeOutputRow(sample_time_s, nav_state_i, bias_t));
        continue;
      }
      if (std::abs(sample_time_s - node_j.time_s) <= kTimeEpsilonS) {
        interval_rows.push_back(MakeOutputRow(sample_time_s, nav_state_j, bias_t));
        continue;
      }

      gtsam::NavState forward_state;
      gtsam::NavState backward_state;

      if (!prefix_integrator.IntegrateTo(sample_time_s)) {
        interval_failed = true;
        failure_status = "FORWARD_INTEGRATION_FAILED";
        break;
      }
      forward_state = prefix_integrator.preintegrated_measurements().predict(nav_state_i, bias_t);

      try {
        const auto backward_window =
          IntegrateImuWindow(imu_samples, sample_time_s, node_j.time_s, imu_params, bias_t);
        const gtsam::NavState initial_guess = InterpolateNavState(forward_state, nav_state_j, alpha);
        if (!SolveBackwardState(
              backward_window.preintegrated_measurements,
              nav_state_j,
              bias_t,
              initial_guess,
              &backward_state)) {
          interval_failed = true;
          failure_status = "BACKWARD_SOLVE_FAILED";
          break;
        }
      } catch (const std::exception &) {
        interval_failed = true;
        failure_status = "BACKWARD_INTEGRATION_FAILED";
        break;
      }

      const gtsam::NavState fused_state = BlendNavStates(forward_state, backward_state, alpha);
      interval_rows.push_back(MakeOutputRow(sample_time_s, fused_state, bias_t));
    }

    if (interval_failed) {
      diagnostic.status = failure_status;
      if (verbose) {
        std::cerr << "trajectory_imu_rate interval " << interval_index << " skipped: " << failure_status << '\n';
      }
      result.diagnostics.push_back(diagnostic);
      continue;
    }

    diagnostic.used_interval = true;
    diagnostic.emitted_sample_count = interval_rows.size();
    diagnostic.status = "OK";
    result.rows.insert(result.rows.end(), interval_rows.begin(), interval_rows.end());
    result.diagnostics.push_back(diagnostic);
  }

  return result;
}

}  // namespace offline_lc_minimal
