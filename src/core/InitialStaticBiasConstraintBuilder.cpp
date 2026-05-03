#include "offline_lc_minimal/core/InitialStaticBiasConstraintBuilder.h"

#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/StaticVerticalAccelBiasFactor.h"

namespace offline_lc_minimal {

bool InitialStaticBiasConstraintBuilder::Enabled(const OfflineRunnerConfig &config) {
  return config.enable_initial_static_vertical_bias_soft_prior;
}

bool InitialStaticBiasConstraintBuilder::AddVerticalAccelBiasSoftPrior(
  const OfflineRunnerConfig &config,
  gtsam::NonlinearFactorGraph &graph,
  const gtsam::Key bias_key,
  const gtsam::Key global_acc_bias_key) {
  if (!Enabled(config)) {
    return false;
  }

  graph.add(factor::StaticVerticalAccelBiasFactor(
    bias_key,
    global_acc_bias_key,
    gtsam::noiseModel::Isotropic::Sigma(
      1,
      config.initial_static_vertical_bias_sigma_mps2)));
  return true;
}

}  // namespace offline_lc_minimal
