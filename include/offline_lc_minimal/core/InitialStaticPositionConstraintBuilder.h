#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

class InitialStaticPositionConstraintBuilder {
 public:
  [[nodiscard]] static bool Enabled(const OfflineRunnerConfig &config);

  [[nodiscard]] static bool AddVerticalPositionHold(
    const OfflineRunnerConfig &config,
    gtsam::NonlinearFactorGraph &graph,
    gtsam::Key reference_pose_key,
    gtsam::Key pose_key);

  [[nodiscard]] static bool AddPositionHold(
    const OfflineRunnerConfig &config,
    gtsam::NonlinearFactorGraph &graph,
    gtsam::Key reference_pose_key,
    gtsam::Key pose_key);
};

}  // namespace offline_lc_minimal
