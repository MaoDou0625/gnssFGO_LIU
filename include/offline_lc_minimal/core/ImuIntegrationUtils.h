#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct ImuWindowIntegration {
  gtsam::PreintegratedCombinedMeasurements preintegrated_measurements;
  gtsam::PreintegratedImuMeasurements preintegrated_imu_measurements;
  std::size_t imu_segments = 0;
  gtsam::Vector3 end_gyro_radps = gtsam::Vector3::Zero();
  gtsam::Vector3 mean_gyro_radps = gtsam::Vector3::Zero();
  gtsam::Vector3 mean_acc_mps2 = gtsam::Vector3::Zero();

  explicit ImuWindowIntegration(
    const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &params,
    const gtsam::imuBias::ConstantBias &bias)
      : preintegrated_measurements(params, bias),
        preintegrated_imu_measurements(params, bias) {}
};

ImuWindowIntegration IntegrateImuWindow(
  const std::vector<ImuSample> &imu_samples,
  double start_time_s,
  double end_time_s,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &params,
  const gtsam::imuBias::ConstantBias &bias);

std::size_t FindNearestImuIndex(const std::vector<ImuSample> &imu_samples, double time_s);

}  // namespace offline_lc_minimal
