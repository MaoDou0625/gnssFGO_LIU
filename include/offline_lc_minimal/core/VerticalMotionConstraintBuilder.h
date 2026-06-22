#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/core/StaticMotionWindowPlanner.h"
#include "offline_lc_minimal/core/VerticalMotionStabilityProfile.h"

namespace offline_lc_minimal {

struct VerticalVelocityDeltaPropagationRecord {
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  double target_delta_vz_mps = 0.0;
  double reference_ba_z_mps2 = 0.0;
};

struct VerticalMotionConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *propagation_records = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const std::vector<RtkOutageWindowRow> *rtk_outage_windows = nullptr;
  const std::vector<BodyZBiasReestimateSegmentRow> *bias_reestimate_segments = nullptr;
  const std::vector<StaticMotionWindow> *static_motion_windows = nullptr;
  const VerticalMotionStabilityProfile *stability_profile = nullptr;
  std::optional<double> gnss_support_end_time_s;
  std::size_t dynamic_start_index = 0;
  int outer_pass = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<VerticalVelocityDeltaDiagnosticRow> *diagnostics = nullptr;
};

class VerticalMotionConstraintBuilder {
 public:
  explicit VerticalMotionConstraintBuilder(VerticalMotionConstraintBuildRequest request);

  void Build() const;

 private:
  [[nodiscard]] bool OverlapsJumpPadding(
    double start_time_s,
    double end_time_s,
    const std::vector<BodyZJumpConstraintWindow> &jump_constraint_windows) const;

  VerticalMotionConstraintBuildRequest request_;
};

void PopulateVerticalVelocityDeltaDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalVelocityDeltaDiagnosticRow> &diagnostics);

}  // namespace offline_lc_minimal
