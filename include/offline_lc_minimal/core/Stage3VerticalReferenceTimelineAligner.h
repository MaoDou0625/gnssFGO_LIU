#pragma once

#include <vector>

#include "offline_lc_minimal/core/Stage2VelocityReference.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

namespace offline_lc_minimal {

struct Stage3VerticalReferenceTimelineAlignResult {
  Stage2VelocityReference stage2_reference;
  Stage3VerticalReference stage3_reference;
};

[[nodiscard]] Stage3VerticalReferenceTimelineAlignResult
AlignStage3VerticalReferencesToTimeline(
  const Stage2VelocityReference &stage2_reference,
  const Stage3VerticalReference &stage3_reference,
  const std::vector<double> &target_timestamps_s);

}  // namespace offline_lc_minimal
