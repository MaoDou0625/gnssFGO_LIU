#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

namespace offline_lc_minimal {

struct RoadNoiseBiasDeltaEstimateOptions {
  std::size_t min_record_count = 20;
  double mad_scale = 3.0;
  double max_abs_bias_delta_mps2 = 1.0;
};

struct RoadNoiseBiasDeltaEstimate {
  double bias_delta_mps2 = 0.0;
  std::size_t candidate_record_count = 0;
  std::size_t used_record_count = 0;
  bool estimated = false;
};

[[nodiscard]] RoadNoiseBiasDeltaEstimate EstimateRoadNoiseBiasDelta(
  const BodyZBiasReestimateSegmentRow &segment,
  const std::vector<VerticalVelocityDeltaPropagationRecord> &propagation_records,
  const RoadNoiseBiasDeltaEstimateOptions &options);

}  // namespace offline_lc_minimal
