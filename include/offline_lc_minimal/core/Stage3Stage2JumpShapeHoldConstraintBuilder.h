#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

struct Stage3Stage2JumpShapeHoldConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const Stage2VelocityReference *stage2_reference = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<Stage3Stage2JumpShapeHoldDiagnosticRow> *diagnostics = nullptr;
};

class Stage3Stage2JumpShapeHoldConstraintBuilder {
 public:
  explicit Stage3Stage2JumpShapeHoldConstraintBuilder(
    Stage3Stage2JumpShapeHoldConstraintBuildRequest request);

  void Build() const;

 private:
  void Validate() const;

  Stage3Stage2JumpShapeHoldConstraintBuildRequest request_;
};

void PopulateStage3Stage2JumpShapeHoldDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage3Stage2JumpShapeHoldDiagnosticRow> &diagnostics,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
