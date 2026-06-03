#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

struct InitialDynamicStaticConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<LateStaticWindowRow> *windows = nullptr;
};

class InitialDynamicStaticConstraintBuilder {
 public:
  explicit InitialDynamicStaticConstraintBuilder(
    InitialDynamicStaticConstraintBuildRequest request);

  void Build() const;

 private:
  InitialDynamicStaticConstraintBuildRequest request_;
};

void PopulateInitialDynamicStaticDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<LateStaticWindowRow> &windows,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
