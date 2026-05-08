#include "offline_lc_minimal/core/InitialStaticPositionConstraintBuilder.h"

#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/StaticVerticalPositionHoldFactor.h"

namespace offline_lc_minimal {

bool InitialStaticPositionConstraintBuilder::Enabled(const OfflineRunnerConfig &config) {
  return config.enable_initial_static_vertical_position_hold;
}

bool InitialStaticPositionConstraintBuilder::AddVerticalPositionHold(
  const OfflineRunnerConfig &config,
  gtsam::NonlinearFactorGraph &graph,
  const gtsam::Key reference_pose_key,
  const gtsam::Key pose_key) {
  if (!Enabled(config)) {
    return false;
  }

  graph.add(factor::StaticVerticalPositionHoldFactor(
    reference_pose_key,
    pose_key,
    gtsam::noiseModel::Isotropic::Sigma(
      1,
      config.initial_static_vertical_position_hold_sigma_m)));
  return true;
}

}  // namespace offline_lc_minimal
