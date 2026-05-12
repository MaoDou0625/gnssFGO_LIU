#pragma once

#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

namespace offline_lc_minimal {

struct RtkOutageSmoothingConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *body_z_jump_windows = nullptr;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *propagation_records = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
};

class RtkOutageSmoothingConstraintBuilder {
 public:
  explicit RtkOutageSmoothingConstraintBuilder(RtkOutageSmoothingConstraintBuildRequest request);

  void Build() const;

 private:
  [[nodiscard]] bool OverlapsBodyZJump(double start_time_s, double end_time_s) const;
  [[nodiscard]] double ClampedTargetDeltaVzMps(
    const VerticalVelocityDeltaPropagationRecord &record,
    double dt_s) const;

  RtkOutageSmoothingConstraintBuildRequest request_;
};

}  // namespace offline_lc_minimal
