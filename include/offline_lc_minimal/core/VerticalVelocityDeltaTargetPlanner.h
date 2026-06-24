#pragma once

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

struct VerticalVelocityDeltaTargetContext {
  bool overlaps_road_high_noise_bias = false;
};

[[nodiscard]] double PlanVerticalVelocityDeltaTarget(
  const OfflineRunnerConfig &config,
  double raw_target_delta_vz_mps,
  double dt_s,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry,
  const VerticalVelocityDeltaTargetContext *target_context = nullptr);

}  // namespace offline_lc_minimal
