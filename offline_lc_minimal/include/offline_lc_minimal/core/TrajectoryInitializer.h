#pragma once

#include <cstddef>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

class TrajectoryInitializer {
 public:
  static InitialPoseEstimate Estimate(
    const std::vector<ImuSample> &imu_samples,
    const std::vector<GnssSolutionSample> &gnss_samples,
    std::size_t start_gnss_index,
    const OfflineRunnerConfig &config);
};

}  // namespace offline_lc_minimal
