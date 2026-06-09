#pragma once

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

[[nodiscard]] bool IsRoadHighNoiseBiasReestimateSource(
  const BodyZBiasReestimateSegmentRow &segment);

[[nodiscard]] bool DecouplesVerticalGmFromGlobalBias(
  const BodyZBiasReestimateSegmentRow &segment);

}  // namespace offline_lc_minimal
