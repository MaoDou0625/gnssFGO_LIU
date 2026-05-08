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
        config.initial_static_vertical_bias_global_tie_sigma_mps2)));
  return true;
}

double InitialStaticBiasConstraintBuilder::ResolveVerticalGmSigmaMps2(
  const OfflineRunnerConfig &config,
  const bool is_initial_static_interval) {
  if (is_initial_static_interval && config.enable_initial_static_vertical_bias_gm_tightening) {
    return config.initial_static_vertical_bias_gm_sigma_mps2;
  }
  return config.vertical_acc_bias_sigma_mps2 > 0.0 ? config.vertical_acc_bias_sigma_mps2 : config.bias_acc_sigma;
}

}  // namespace offline_lc_minimal
