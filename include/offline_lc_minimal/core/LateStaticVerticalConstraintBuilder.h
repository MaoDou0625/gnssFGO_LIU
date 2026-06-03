#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

struct LateStaticVerticalConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  std::size_t dynamic_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<LateStaticWindowRow> *windows = nullptr;
};

class LateStaticVerticalConstraintBuilder {
 public:
  explicit LateStaticVerticalConstraintBuilder(LateStaticVerticalConstraintBuildRequest request);

  void Build() const;

 private:
  LateStaticVerticalConstraintBuildRequest request_;
};

void PopulateLateStaticVerticalDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<LateStaticWindowRow> &windows,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
