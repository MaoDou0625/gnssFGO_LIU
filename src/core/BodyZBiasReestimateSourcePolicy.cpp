#include "offline_lc_minimal/core/BodyZBiasReestimateSourcePolicy.h"

namespace offline_lc_minimal {
namespace {

constexpr const char *kRoadHighNoiseSource = "ROAD_HIGH_NOISE";

}  // namespace

bool IsRoadHighNoiseBiasReestimateSource(
  const BodyZBiasReestimateSegmentRow &segment) {
  return segment.source_type == kRoadHighNoiseSource;
}

bool DecouplesVerticalGmFromGlobalBias(
  const BodyZBiasReestimateSegmentRow &segment) {
  return IsRoadHighNoiseBiasReestimateSource(segment);
}

}  // namespace offline_lc_minimal
