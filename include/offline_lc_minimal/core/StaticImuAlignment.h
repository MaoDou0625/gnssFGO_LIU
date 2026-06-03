#pragma once

#include <cstddef>

#include <Eigen/Core>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct StaticImuWindowSummary {
  Eigen::Vector3d mean_acc_mps2 = Eigen::Vector3d::Zero();
  Eigen::Vector3d mean_gyro_radps = Eigen::Vector3d::Zero();
  std::size_t sample_count = 0;
  bool used_stationary_filter = false;
};

class StaticImuAlignment {
 public:
  static StaticImuWindowSummary CollectWindow(
    const std::vector<ImuSample> &imu_samples,
    double start_time_s,
    double duration_s,
    const OfflineRunnerConfig &config,
    bool fallback_to_all_samples);

  static bool TryEstimateDualVectorInitialization(
    const StaticImuWindowSummary &window_summary,
    const Eigen::Vector3d &earth_rate_enu,
    const OfflineRunnerConfig &config,
    InitialPoseEstimate &initial_pose);
};

}  // namespace offline_lc_minimal
