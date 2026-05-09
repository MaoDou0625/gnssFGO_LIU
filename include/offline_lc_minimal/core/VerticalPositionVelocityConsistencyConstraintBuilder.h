#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct VerticalPositionVelocityConsistencyBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const gtsam::Values *initial_values = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<VerticalPositionVelocityConsistencyDiagnosticRow> *diagnostics = nullptr;
};

class VerticalPositionVelocityConsistencyConstraintBuilder {
 public:
  explicit VerticalPositionVelocityConsistencyConstraintBuilder(
    VerticalPositionVelocityConsistencyBuildRequest request);

  void Build() const;

 private:
  [[nodiscard]] bool OverlapsJumpPadding(double start_time_s, double end_time_s) const;
  [[nodiscard]] std::string IntervalType(
    std::size_t state_i,
    std::size_t state_j,
    double start_time_s,
    double end_time_s) const;

  VerticalPositionVelocityConsistencyBuildRequest request_;
};

void PopulateVerticalPositionVelocityConsistencyDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalPositionVelocityConsistencyDiagnosticRow> &diagnostics,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
