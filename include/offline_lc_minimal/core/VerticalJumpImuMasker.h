#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct VerticalJumpImuIntervalRecord {
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  std::size_t graph_factor_index = 0;
  gtsam::PreintegratedCombinedMeasurements preintegrated_measurements;
};

struct VerticalJumpImuMaskRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<VerticalJumpImuIntervalRecord> *intervals = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<VerticalJumpMaskedImuDiagnosticRow> *diagnostics = nullptr;
};

class VerticalJumpImuMasker {
 public:
  explicit VerticalJumpImuMasker(VerticalJumpImuMaskRequest request);

  void Apply() const;

 private:
  [[nodiscard]] bool OverlapsJumpPadding(double start_time_s, double end_time_s) const;

  VerticalJumpImuMaskRequest request_;
};

}  // namespace offline_lc_minimal
