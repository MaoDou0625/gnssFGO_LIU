#pragma once

#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/core/VelocityDeltaPropagationRecord.h"

namespace offline_lc_minimal {

struct HorizontalVelocityDeltaConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<VelocityDeltaPropagationRecord> *propagation_records = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
};

class HorizontalVelocityDeltaConstraintBuilder {
 public:
  explicit HorizontalVelocityDeltaConstraintBuilder(
    HorizontalVelocityDeltaConstraintBuildRequest request);

  void Build() const;

 private:
  HorizontalVelocityDeltaConstraintBuildRequest request_;
};

}  // namespace offline_lc_minimal
