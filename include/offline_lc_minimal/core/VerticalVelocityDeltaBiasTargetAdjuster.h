#pragma once

#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

struct VerticalVelocityDeltaBiasTargetAdjustmentRequest {
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  double target_delta_vz_mps = 0.0;
  double reference_ba_z_mps2 = 0.0;
  const std::vector<BodyZBiasReestimateSegmentRow> *bias_reestimate_segments = nullptr;
};

struct VerticalVelocityDeltaBiasTargetAdjustment {
  double target_delta_vz_mps = 0.0;
  double reference_ba_z_mps2 = 0.0;
  bool applied = false;
};

[[nodiscard]] VerticalVelocityDeltaBiasTargetAdjustment
AdjustVerticalVelocityDeltaTargetForBiasReestimate(
  const VerticalVelocityDeltaBiasTargetAdjustmentRequest &request);

}  // namespace offline_lc_minimal
