#pragma once

#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkOutageBoundaryConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<RtkOutageBoundaryReferenceRow> *boundary_references = nullptr;
  const std::vector<ReferenceNodeState> *tilt_reference_states = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<RtkOutageBoundaryDiagnosticRow> *diagnostics = nullptr;
};

class RtkOutageBoundaryConstraintBuilder {
 public:
  explicit RtkOutageBoundaryConstraintBuilder(
    RtkOutageBoundaryConstraintBuildRequest request);

  void Build() const;

 private:
  RtkOutageBoundaryConstraintBuildRequest request_;
};

void PopulateRtkOutageBoundaryDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<RtkOutageBoundaryDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
