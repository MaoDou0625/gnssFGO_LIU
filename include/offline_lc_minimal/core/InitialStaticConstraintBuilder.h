#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/StaticImuAlignment.h"

namespace offline_lc_minimal {

struct InitialStaticConstraintData {
  StaticImuWindowSummary window_summary;
  bool valid = false;
};

class InitialStaticConstraintBuilder {
 public:
  static InitialStaticConstraintData Collect(
    const std::vector<ImuSample> &imu_samples,
    double alignment_start_time_s,
    double alignment_end_time_s,
    const OfflineRunnerConfig &config);

  static void AddFactors(
    const InitialStaticConstraintData &constraint_data,
    const Eigen::Vector3d &earth_rate_enu,
    const OfflineRunnerConfig &config,
    gtsam::NonlinearFactorGraph &graph,
    gtsam::Key pose_key,
    gtsam::Key velocity_key,
    gtsam::Key bias_key);
};

}  // namespace offline_lc_minimal
