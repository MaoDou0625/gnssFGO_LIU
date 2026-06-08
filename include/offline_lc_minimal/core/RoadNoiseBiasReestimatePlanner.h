#pragma once

#include <vector>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RoadNoiseBiasReestimatePlannerOptions {
  double min_high_noise_duration_s = 0.0;
};

[[nodiscard]] std::vector<BodyZBiasReestimateSegmentRow> PlanRoadNoiseBiasReestimateSegments(
  const std::vector<RoadNoiseStateSegmentRow> &road_segments,
  const RoadNoiseBiasReestimatePlannerOptions &options);

}  // namespace offline_lc_minimal
