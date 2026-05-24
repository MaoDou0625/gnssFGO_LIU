#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

namespace offline_lc_minimal {

struct Stage3VerticalReferenceConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const Stage3VerticalReference *reference = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<Stage3VerticalReferenceDiagnosticRow> *diagnostics = nullptr;
};

class Stage3VerticalReferenceConstraintBuilder {
 public:
  explicit Stage3VerticalReferenceConstraintBuilder(
    Stage3VerticalReferenceConstraintBuildRequest request);

  void Build() const;

 private:
  void Validate() const;

  Stage3VerticalReferenceConstraintBuildRequest request_;
};

void PopulateStage3VerticalReferenceDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage3VerticalReferenceDiagnosticRow> &diagnostics,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
