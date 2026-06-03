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
    double alignment_start_time_s,
    double alignment_end_time_s,
    double navigation_start_time_s,
    const Eigen::Vector3d &earth_rate_enu,
    const std::vector<std::size_t> &yaw_candidate_indices,
    const OfflineRunnerConfig &config);
};

}  // namespace offline_lc_minimal
