#pragma once

#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"

namespace offline_lc_minimal {

struct VerticalVelocityDeltaContextScaleRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<BodyZJumpConstraintWindow> *jump_constraint_windows = nullptr;
  const std::vector<RtkOutageWindowRow> *rtk_outage_windows = nullptr;
  const std::vector<BodyZBiasReestimateSegmentRow> *bias_reestimate_segments = nullptr;
};

struct VerticalVelocityDeltaContextScaleDecision {
  std::string context = "GLOBAL";
  double output_sigma_scale = 1.0;
  bool overlaps_jump = false;
  bool overlaps_rtk_outage = false;
  bool overlaps_rough_bias = false;
};

class VerticalVelocityDeltaContextScalePlanner {
 public:
  explicit VerticalVelocityDeltaContextScalePlanner(
    VerticalVelocityDeltaContextScaleRequest request);

  [[nodiscard]] VerticalVelocityDeltaContextScaleDecision Evaluate(
    double start_time_s,
    double end_time_s) const;

 private:
  [[nodiscard]] bool OverlapsJump(double start_time_s, double end_time_s) const;
  [[nodiscard]] bool OverlapsRtkOutage(double start_time_s, double end_time_s) const;
  [[nodiscard]] bool OverlapsRoughBias(double start_time_s, double end_time_s) const;

  VerticalVelocityDeltaContextScaleRequest request_;
};

}  // namespace offline_lc_minimal
