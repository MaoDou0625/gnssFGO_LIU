#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

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
  struct Window {
    std::size_t source_window_index = 0;
    std::size_t source_window_count = 1;
    std::string type = "JUMP";
    bool from_jump_window = true;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
  };

  [[nodiscard]] std::vector<Window> BuildJumpWindows() const;
  [[nodiscard]] std::vector<Window> BuildGlobalWindows(
    const std::vector<Window> &jump_windows) const;
  [[nodiscard]] std::vector<std::size_t> StateIndicesInWindow(
    double start_time_s,
    double end_time_s) const;
  [[nodiscard]] bool OverlapsAnyWindow(
    double start_time_s,
    double end_time_s,
    const std::vector<Window> &windows) const;
  [[nodiscard]] BodyZNHCDiagnosticRow MakeDiagnosticRow(
    std::size_t window_index,
    const Window &window,
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
