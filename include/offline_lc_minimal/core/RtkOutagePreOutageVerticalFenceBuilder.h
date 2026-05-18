#pragma once

#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkOutagePreOutageVerticalFenceBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  std::vector<RtkOutageCausalStateReferenceRow> *state_references = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
};

class RtkOutagePreOutageVerticalFenceBuilder {
 public:
  explicit RtkOutagePreOutageVerticalFenceBuilder(
    RtkOutagePreOutageVerticalFenceBuildRequest request);

  void Build() const;

 private:
  RtkOutagePreOutageVerticalFenceBuildRequest request_;
};

void PopulateRtkOutagePreOutageVerticalFenceSummary(
  const gtsam::Values &optimized_values,
  const std::vector<RtkOutageCausalStateReferenceRow> &state_references,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
