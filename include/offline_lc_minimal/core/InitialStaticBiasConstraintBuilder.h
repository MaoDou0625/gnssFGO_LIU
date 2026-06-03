#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

class InitialStaticBiasConstraintBuilder {
 public:
  [[nodiscard]] static bool Enabled(const OfflineRunnerConfig &config);

  [[nodiscard]] static bool AddVerticalAccelBiasSoftPrior(
    const OfflineRunnerConfig &config,
    gtsam::NonlinearFactorGraph &graph,
    gtsam::Key bias_key,
    gtsam::Key global_acc_bias_key);

  [[nodiscard]] static double ResolveVerticalGmSigmaMps2(
    const OfflineRunnerConfig &config,
    bool is_initial_static_interval);
};

}  // namespace offline_lc_minimal
