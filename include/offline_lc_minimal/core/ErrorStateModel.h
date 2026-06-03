#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/ImuIntegrationUtils.h"

namespace offline_lc_minimal {

struct ReferenceNodeState {
  double time_s = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias bias;
  gtsam::Vector3 omega = gtsam::Vector3::Zero();
};

struct ErrorNodeInterpolation {
  std::size_t left_index = 0U;
  std::size_t right_index = 1U;
  double alpha = 0.0;
};

struct ErrorProcessInterval {
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  ErrorStateMatrix phi = ErrorStateMatrix::Identity();
  ErrorStateMatrix q = ErrorStateMatrix::Identity();
};

std::vector<double> BuildErrorStateTimestamps(double start_time_s, double end_time_s, double error_state_frequency_hz);
ErrorNodeInterpolation MapTimeToErrorNodes(const std::vector<double> &error_timestamps_s, double time_s);
ErrorStateVector InterpolateErrorState(
  const ErrorStateVector &left_state,
  const ErrorStateVector &right_state,
  double alpha);
ErrorProcessInterval BuildErrorProcessInterval(
  const ReferenceNodeState &reference_state,
  const ImuWindowIntegration &imu_window,
  double end_time_s,
  const GeoReference &geo_reference,
  const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
