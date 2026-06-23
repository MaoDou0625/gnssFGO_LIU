#pragma once

#include <vector>

#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/RoadNoiseBiasDeltaEstimator.h"

namespace offline_lc_minimal {

struct RoadNoiseBiasReestimatePlannerOptions {
  double min_high_noise_duration_s = 0.0;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *propagation_records = nullptr;
  bool enable_delta_estimation = true;
  RoadNoiseBiasDeltaEstimateOptions delta_estimate_options;
};

[[nodiscard]] std::vector<BodyZBiasReestimateSegmentRow> PlanRoadNoiseBiasReestimateSegments(
  const std::vector<RoadNoiseStateSegmentRow> &road_segments,
  const RoadNoiseBiasReestimatePlannerOptions &options);

}  // namespace offline_lc_minimal
