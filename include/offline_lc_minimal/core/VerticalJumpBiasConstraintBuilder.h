#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/VerticalJumpBiasSegmenter.h"
#include "offline_lc_minimal/core/VerticalJumpImuMasker.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

namespace offline_lc_minimal {

struct VerticalJumpBiasConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const std::vector<BodyZSeedImuDiagnosticRow> *body_z_diagnostics = nullptr;
  const std::vector<VerticalJumpImuIntervalRecord> *imu_intervals = nullptr;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *propagation_records = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  gtsam::Values *initial_values = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<VerticalJumpBiasDiagnosticRow> *diagnostics = nullptr;
};

class VerticalJumpBiasConstraintBuilder {
 public:
  explicit VerticalJumpBiasConstraintBuilder(VerticalJumpBiasConstraintBuildRequest request);

  void Build() const;

 private:
  struct Span {
    std::size_t span_index = 0;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    std::vector<std::size_t> source_window_indices;
    std::vector<std::size_t> state_indices;
  };

  struct MatchedInterval {
    const VerticalJumpImuIntervalRecord *imu_interval = nullptr;
    const VerticalVelocityDeltaPropagationRecord *propagation_record = nullptr;
  };

  [[nodiscard]] std::vector<Span> BuildMergedSpans() const;
  [[nodiscard]] std::vector<VerticalJumpBiasSpanInput> BuildSegmenterInputs(
    const std::vector<Span> &spans) const;
  [[nodiscard]] std::vector<std::size_t> StateIndicesInWindow(double start_time_s, double end_time_s) const;
  [[nodiscard]] std::vector<MatchedInterval> FindMatchedIntervals(
    const VerticalJumpBiasSegmentEstimate &segment,
    const std::vector<VerticalJumpBiasSegmentEstimate> &all_segments) const;
  [[nodiscard]] const VerticalVelocityDeltaPropagationRecord *FindPropagationRecord(
    std::size_t state_i,
    std::size_t state_j) const;

  VerticalJumpBiasConstraintBuildRequest request_;
};

void PopulateVerticalJumpBiasDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalJumpBiasDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
