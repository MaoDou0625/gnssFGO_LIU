#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"

namespace offline_lc_minimal {

struct Stage2VehicleNHCConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  gtsam::Values *initial_values = nullptr;
  const std::vector<ReferenceNodeState> *reference_states = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::Key mount_leakage_key = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<Stage2MountLeakageDiagnosticRow> *mount_diagnostics = nullptr;
  std::vector<Stage2VehicleNHCStateDiagnosticRow> *state_diagnostics = nullptr;
};

class Stage2VehicleNHCConstraintBuilder {
 public:
  explicit Stage2VehicleNHCConstraintBuilder(Stage2VehicleNHCConstraintBuildRequest request);

  void Build() const;

 private:
  [[nodiscard]] std::vector<BodyZJumpConstraintWindow> BuildJumpWindows() const;
  [[nodiscard]] std::vector<BodyZJumpConstraintWindow> BuildGlobalWindows(
    const std::vector<BodyZJumpConstraintWindow> &jump_windows) const;
  [[nodiscard]] std::vector<std::size_t> StateIndicesInWindow(
    double start_time_s,
    double end_time_s) const;

  Stage2VehicleNHCConstraintBuildRequest request_;
};

void PopulateStage2VehicleNHCDiagnostics(
  const gtsam::Values &initial_values,
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  gtsam::Key mount_leakage_key,
  std::vector<Stage2MountLeakageDiagnosticRow> &mount_diagnostics,
  std::vector<Stage2VehicleNHCStateDiagnosticRow> &state_diagnostics,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
