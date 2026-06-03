#pragma once

#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct Stage2HorizontalHoldBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<ReferenceNodeState> *reference_states = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
};

class Stage2HorizontalHoldBuilder {
 public:
  explicit Stage2HorizontalHoldBuilder(Stage2HorizontalHoldBuildRequest request);

  void Build() const;

 private:
  Stage2HorizontalHoldBuildRequest request_;
};

}  // namespace offline_lc_minimal
