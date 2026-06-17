#pragma once

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

[[nodiscard]] double PlanVerticalVelocityDeltaTarget(
  const OfflineRunnerConfig &config,
  double raw_target_delta_vz_mps,
  double dt_s,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry);

}  // namespace offline_lc_minimal
