#include "offline_lc_minimal/core/RoadNoiseBiasReestimatePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsFiniteSegment(const RoadNoiseStateSegmentRow &segment) {
  return std::isfinite(segment.start_time_s) &&
         std::isfinite(segment.end_time_s) &&
         segment.end_time_s > segment.start_time_s + kTimeEpsilonS;
}

bool IsHighNoiseSegment(const RoadNoiseStateSegmentRow &segment) {
  return segment.state == "HIGH_NOISE";
}

double DurationS(const RoadNoiseStateSegmentRow &segment) {
  if (std::isfinite(segment.duration_s) && segment.duration_s > 0.0) {
    return segment.duration_s;
  }
  return segment.end_time_s - segment.start_time_s;
}

}  // namespace

std::vector<BodyZBiasReestimateSegmentRow> PlanRoadNoiseBiasReestimateSegments(
  const std::vector<RoadNoiseStateSegmentRow> &road_segments,
  const RoadNoiseBiasReestimatePlannerOptions &options) {
  std::vector<BodyZBiasReestimateSegmentRow> segments;
  for (const auto &road_segment : road_segments) {
    if (!IsFiniteSegment(road_segment) || !IsHighNoiseSegment(road_segment)) {
      continue;
    }
    const double duration_s = DurationS(road_segment);
    if (duration_s + kTimeEpsilonS < std::max(0.0, options.min_high_noise_duration_s)) {
      continue;
    }

    BodyZBiasReestimateSegmentRow row;
    row.segment_index = segments.size();
    row.source_type = "ROAD_HIGH_NOISE";
    row.source_bias_window_index = road_segment.segment_index;
    row.source_outage_window_index = -1;
    row.bias_window_start_time_s = road_segment.start_time_s;
    row.bias_window_end_time_s = road_segment.end_time_s;
    row.start_time_s = road_segment.start_time_s;
    row.end_time_s = road_segment.end_time_s;
    row.duration_s = duration_s;
    row.detected_bias_delta_mps2 = 0.0;
    segments.push_back(std::move(row));
  }
  return segments;
}

}  // namespace offline_lc_minimal
