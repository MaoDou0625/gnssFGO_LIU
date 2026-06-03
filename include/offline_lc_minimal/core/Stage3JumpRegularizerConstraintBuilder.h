#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage3JumpContextEnvelopePlanner.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

namespace offline_lc_minimal {

struct Stage3JumpRegularizerConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const Stage3VerticalReference *reference = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const gtsam::Values *initial_values = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<Stage3JumpRegularizerDiagnosticRow> *diagnostics = nullptr;
  std::vector<Stage3JumpContextEnvelopeProfileRow> *context_profiles = nullptr;
};

class Stage3JumpRegularizerConstraintBuilder {
 public:
  explicit Stage3JumpRegularizerConstraintBuilder(
    Stage3JumpRegularizerConstraintBuildRequest request);

  void Build() const;

 private:
  void Validate() const;

  Stage3JumpRegularizerConstraintBuildRequest request_;
};

void PopulateStage3JumpRegularizerDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage3JumpRegularizerDiagnosticRow> &diagnostics,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
