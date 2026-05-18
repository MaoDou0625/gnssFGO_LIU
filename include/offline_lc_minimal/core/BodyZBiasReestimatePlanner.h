#pragma once

#include <vector>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct BodyZBiasReestimatePlannerOptions {
  double jump_padding_s = 0.0;
  double min_segment_duration_s = 0.0;
  double min_bias_window_duration_s = 0.0;
};

std::vector<BodyZBiasReestimateSegmentRow> PlanBodyZBiasReestimateSegments(
  const std::vector<BodyZSeedJumpWindowRow> &bias_windows,
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const BodyZBiasReestimatePlannerOptions &options);

}  // namespace offline_lc_minimal
