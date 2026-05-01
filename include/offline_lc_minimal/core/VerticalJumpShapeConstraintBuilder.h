#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct VerticalJumpShapeConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<VerticalJumpVelocityRampDiagnosticRow> *diagnostics = nullptr;
};

class VerticalJumpShapeConstraintBuilder {
 public:
  explicit VerticalJumpShapeConstraintBuilder(VerticalJumpShapeConstraintBuildRequest request);

  void Build() const;

 private:
  [[nodiscard]] std::vector<std::size_t> StateIndicesInWindow(double start_time_s, double end_time_s) const;

  VerticalJumpShapeConstraintBuildRequest request_;
};

void PopulateVerticalJumpVelocityRampDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalJumpVelocityRampDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
