#pragma once

#include <functional>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/VerticalMotionStabilityProfile.h"

namespace offline_lc_minimal {

struct VerticalAdaptiveReweightingPassInput {
  int pass_index = 0;
  const VerticalMotionStabilityProfile *stability_profile = nullptr;
  const gtsam::Values *initial_values = nullptr;
};

struct VerticalAdaptiveReweightingPassOutput {
  gtsam::Values optimized_values;
  double initial_error = 0.0;
  double final_error = 0.0;
};

using VerticalAdaptivePassRunner =
  std::function<VerticalAdaptiveReweightingPassOutput(const VerticalAdaptiveReweightingPassInput &)>;
using VerticalAdaptiveProfileEstimator =
  std::function<VerticalMotionStabilityProfile(const gtsam::Values &, int)>;

struct VerticalAdaptiveReweightingLoopRequest {
  const OfflineRunnerConfig *config = nullptr;
  const gtsam::Values *initial_values = nullptr;
  VerticalAdaptivePassRunner run_pass;
  VerticalAdaptiveProfileEstimator estimate_profile;
};

struct VerticalAdaptiveReweightingLoopResult {
  gtsam::Values optimized_values;
  VerticalMotionStabilityProfile final_profile;
  int pass_count = 0;
  bool converged = false;
  double initial_error = 0.0;
  double final_error = 0.0;
};

class VerticalAdaptiveReweightingLoop {
 public:
  explicit VerticalAdaptiveReweightingLoop(VerticalAdaptiveReweightingLoopRequest request);

  [[nodiscard]] VerticalAdaptiveReweightingLoopResult Run() const;

 private:
  VerticalAdaptiveReweightingLoopRequest request_;
};

}  // namespace offline_lc_minimal
