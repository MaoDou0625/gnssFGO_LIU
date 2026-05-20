#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/core/Stage1OutageLateralVelocityEnvelopeEstimator.h"

namespace offline_lc_minimal {

struct Stage1OutageBodyYEnvelopeConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const Stage1OutageBodyYEnvelopeReference *reference = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<Stage1OutageBodyYEnvelopeRow> *envelopes = nullptr;
  std::vector<Stage1OutageBodyYStateDiagnosticRow> *state_diagnostics = nullptr;
};

class Stage1OutageBodyYEnvelopeConstraintBuilder {
 public:
  explicit Stage1OutageBodyYEnvelopeConstraintBuilder(
    Stage1OutageBodyYEnvelopeConstraintBuildRequest request);

  void Build() const;

 private:
  Stage1OutageBodyYEnvelopeConstraintBuildRequest request_;
};

void PopulateStage1OutageBodyYEnvelopeDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage1OutageBodyYStateDiagnosticRow> &state_diagnostics,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
