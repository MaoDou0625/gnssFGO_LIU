#pragma once

#include <vector>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkOutageBazReestimatePlannerOptions {
  double jump_padding_s = 0.0;
  double min_segment_duration_s = 0.0;
};

std::vector<BodyZBiasReestimateSegmentRow> PlanRtkOutageBazReestimateSegments(
  const std::vector<BodyZBiasReestimateSegmentRow> &body_z_segments,
  const std::vector<RtkOutageWindowRow> &rtk_outage_windows,
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const RtkOutageBazReestimatePlannerOptions &options);

}  // namespace offline_lc_minimal
