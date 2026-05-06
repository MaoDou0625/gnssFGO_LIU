#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/VerticalJumpImuMasker.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

namespace offline_lc_minimal {

struct VerticalJumpImpulseConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const std::vector<VerticalJumpImuIntervalRecord> *imu_intervals = nullptr;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *propagation_records = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  gtsam::Values *initial_values = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<VerticalJumpImpulseDiagnosticRow> *diagnostics = nullptr;
};

class VerticalJumpImpulseConstraintBuilder {
 public:
  explicit VerticalJumpImpulseConstraintBuilder(VerticalJumpImpulseConstraintBuildRequest request);

  void Build() const;

 private:
  struct Span {
    std::size_t span_index = 0;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    std::vector<std::size_t> source_window_indices;
    std::vector<std::size_t> state_indices;
    std::optional<std::size_t> pre_anchor_state_index;
    std::optional<std::size_t> post_anchor_state_index;
  };

  [[nodiscard]] std::vector<Span> BuildMergedSpans() const;
  [[nodiscard]] std::vector<std::size_t> StateIndicesInWindow(double start_time_s, double end_time_s) const;
  [[nodiscard]] std::optional<double> SumImuDeltaVz(std::size_t state_i, std::size_t state_j) const;

  VerticalJumpImpulseConstraintBuildRequest request_;
};

void PopulateVerticalJumpImpulseDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalJumpImpulseDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
