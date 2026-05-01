#pragma once

#include <cstddef>
#include <optional>
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
  std::vector<VerticalJumpContinuityDiagnosticRow> *continuity_diagnostics = nullptr;
};

class VerticalJumpShapeConstraintBuilder {
 public:
  explicit VerticalJumpShapeConstraintBuilder(VerticalJumpShapeConstraintBuildRequest request);

  void Build() const;

 private:
  struct Span {
    std::size_t window_index = 0;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
    std::vector<std::size_t> state_indices;
    std::optional<std::size_t> pre_anchor_state_index;
    std::optional<std::size_t> post_anchor_state_index;
  };

  [[nodiscard]] std::vector<Span> BuildMergedSpans() const;
  [[nodiscard]] std::vector<std::size_t> StateIndicesInWindow(double start_time_s, double end_time_s) const;

  VerticalJumpShapeConstraintBuildRequest request_;
};

void PopulateVerticalJumpVelocityRampDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalJumpVelocityRampDiagnosticRow> &diagnostics);

void PopulateVerticalJumpContinuityDiagnostics(
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps,
  std::vector<VerticalJumpContinuityDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
