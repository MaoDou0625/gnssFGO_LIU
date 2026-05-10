#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/core/VerticalMotionStabilityProfile.h"

namespace offline_lc_minimal {

struct VerticalAccelBiasGmTransitionRecord {
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  bool is_initial_static_interval = false;
};

struct VerticalAccelBiasGmConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<VerticalAccelBiasGmTransitionRecord> *records = nullptr;
  const VerticalMotionStabilityProfile *stability_profile = nullptr;
  gtsam::Key global_acc_bias_key = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
};

class VerticalAccelBiasGmConstraintBuilder {
 public:
  explicit VerticalAccelBiasGmConstraintBuilder(VerticalAccelBiasGmConstraintBuildRequest request);

  void Build() const;

  [[nodiscard]] static double TransitionSigmaMps2(
    const OfflineRunnerConfig &config,
    double dt_s,
    bool is_initial_static_interval,
    const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry);

  [[nodiscard]] static double DrivingSigmaMps2(
    const OfflineRunnerConfig &config,
    bool is_initial_static_interval,
    const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry);

 private:
  VerticalAccelBiasGmConstraintBuildRequest request_;
};

}  // namespace offline_lc_minimal
