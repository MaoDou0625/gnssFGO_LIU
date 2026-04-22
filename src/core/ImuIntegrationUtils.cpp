#include "offline_lc_minimal/core/ImuIntegrationUtils.h"

#include <algorithm>
#include <stdexcept>

namespace offline_lc_minimal {

namespace {

constexpr double kTimeEpsilonS = 1e-9;

}  // namespace

ImuWindowIntegration IntegrateImuWindow(
  const std::vector<ImuSample> &imu_samples,
  const double start_time_s,
  const double end_time_s,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &params,
  const gtsam::imuBias::ConstantBias &bias) {
  if (imu_samples.empty()) {
    throw std::runtime_error("cannot integrate IMU window without IMU samples");
  }
  if (end_time_s <= start_time_s) {
    throw std::runtime_error("invalid IMU integration window");
  }
  if (start_time_s < imu_samples.front().time_s - kTimeEpsilonS ||
      end_time_s > imu_samples.back().time_s + kTimeEpsilonS) {
    throw std::runtime_error("IMU integration window is outside the available IMU time range");
  }

  ImuWindowIntegration window(params, bias);

  const auto lower_it = std::lower_bound(
    imu_samples.begin(),
    imu_samples.end(),
    start_time_s,
    [](const ImuSample &sample, const double timestamp_s) { return sample.time_s < timestamp_s; });

  const std::size_t begin_index = static_cast<std::size_t>(std::distance(imu_samples.begin(), lower_it));
  const std::size_t hold_index = begin_index > 0U ? begin_index - 1U : 0U;

  ImuSample held_sample = imu_samples[hold_index];
  double last_time_s = start_time_s;
  gtsam::Vector3 weighted_gyro_sum = gtsam::Vector3::Zero();
  gtsam::Vector3 weighted_acc_sum = gtsam::Vector3::Zero();
  double accumulated_dt_s = 0.0;

  for (std::size_t index = begin_index; index < imu_samples.size(); ++index) {
    const auto &current_sample = imu_samples[index];
    if (current_sample.time_s > end_time_s) {
      break;
    }

    const double dt_s = current_sample.time_s - last_time_s;
    if (dt_s > 0.0) {
      window.preintegrated_measurements.integrateMeasurement(
        held_sample.accel_mps2,
        held_sample.gyro_radps,
        dt_s);
      window.preintegrated_imu_measurements.integrateMeasurement(
        held_sample.accel_mps2,
        held_sample.gyro_radps,
        dt_s);
      weighted_acc_sum += held_sample.accel_mps2 * dt_s;
      weighted_gyro_sum += held_sample.gyro_radps * dt_s;
      accumulated_dt_s += dt_s;
      ++window.imu_segments;
    }
    held_sample = current_sample;
    last_time_s = current_sample.time_s;
  }

  if (end_time_s > last_time_s) {
    window.preintegrated_measurements.integrateMeasurement(
      held_sample.accel_mps2,
      held_sample.gyro_radps,
      end_time_s - last_time_s);
    window.preintegrated_imu_measurements.integrateMeasurement(
      held_sample.accel_mps2,
      held_sample.gyro_radps,
      end_time_s - last_time_s);
    weighted_acc_sum += held_sample.accel_mps2 * (end_time_s - last_time_s);
    weighted_gyro_sum += held_sample.gyro_radps * (end_time_s - last_time_s);
    accumulated_dt_s += end_time_s - last_time_s;
    ++window.imu_segments;
  }

  if (window.imu_segments == 0U) {
    throw std::runtime_error("no IMU data available for state interval");
  }

  window.end_gyro_radps = held_sample.gyro_radps;
  if (accumulated_dt_s > 0.0) {
    window.mean_acc_mps2 = weighted_acc_sum / accumulated_dt_s;
    window.mean_gyro_radps = weighted_gyro_sum / accumulated_dt_s;
  } else {
    window.mean_acc_mps2 = held_sample.accel_mps2;
    window.mean_gyro_radps = held_sample.gyro_radps;
  }
  return window;
}

std::size_t FindNearestImuIndex(const std::vector<ImuSample> &imu_samples, const double time_s) {
  const auto lower_it = std::lower_bound(
    imu_samples.begin(),
    imu_samples.end(),
    time_s,
    [](const ImuSample &sample, const double timestamp_s) { return sample.time_s < timestamp_s; });

  if (lower_it == imu_samples.begin()) {
    return 0U;
  }
  if (lower_it == imu_samples.end()) {
    return imu_samples.size() - 1U;
  }

  const std::size_t upper_index = static_cast<std::size_t>(std::distance(imu_samples.begin(), lower_it));
  const std::size_t lower_index = upper_index - 1U;
  const double lower_dt = std::abs(imu_samples[lower_index].time_s - time_s);
  const double upper_dt = std::abs(imu_samples[upper_index].time_s - time_s);
  return lower_dt <= upper_dt ? lower_index : upper_index;
}

}  // namespace offline_lc_minimal
