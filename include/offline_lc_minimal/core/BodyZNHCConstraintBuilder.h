#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"

namespace offline_lc_minimal {

struct BodyZNHCConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const gtsam::Values *initial_values = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<BodyZNHCDiagnosticRow> *diagnostics = nullptr;
};

class BodyZNHCConstraintBuilder {
 public:
  explicit BodyZNHCConstraintBuilder(BodyZNHCConstraintBuildRequest request);

  void Build() const;

 private:
  [[nodiscard]] std::vector<BodyZJumpConstraintWindow> BuildJumpWindows() const;
  [[nodiscard]] std::vector<BodyZJumpConstraintWindow> BuildGlobalWindows(
    const std::vector<BodyZJumpConstraintWindow> &jump_windows) const;
  [[nodiscard]] std::vector<std::size_t> StateIndicesInWindow(
    double start_time_s,
    double end_time_s) const;
  [[nodiscard]] bool OverlapsAnyWindow(
    double start_time_s,
    double end_time_s,
    const std::vector<BodyZJumpConstraintWindow> &windows) const;
  [[nodiscard]] BodyZNHCDiagnosticRow MakeDiagnosticRow(
    std::size_t window_index,
    const BodyZJumpConstraintWindow &window,
    const std::vector<std::size_t> &state_indices,
    double velocity_sigma_mps,
    double displacement_sigma_m) const;

  BodyZNHCConstraintBuildRequest request_;
};

void PopulateBodyZNHCDiagnostics(
  const gtsam::Values &initial_values,
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps,
  std::vector<BodyZNHCDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
