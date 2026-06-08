#include "offline_lc_minimal/core/RoadNoiseStateReference.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsFiniteSegment(const RoadNoiseStateSegmentRow &segment) {
  return std::isfinite(segment.start_time_s) &&
         std::isfinite(segment.end_time_s) &&
         segment.end_time_s > segment.start_time_s + kTimeEpsilonS;
}

}  // namespace

RoadNoiseStateReference::RoadNoiseStateReference(
  std::vector<RoadNoiseStateSegmentRow> segments)
    : segments_(std::move(segments)) {
  std::sort(
    segments_.begin(),
    segments_.end(),
    [](const RoadNoiseStateSegmentRow &left,
       const RoadNoiseStateSegmentRow &right) {
      return left.start_time_s < right.start_time_s;
    });
}

std::vector<RoadNoiseStateSegmentRow> RoadNoiseStateReference::Clip(
  const double start_time_s,
  const double end_time_s) const {
  if (!std::isfinite(start_time_s) ||
      !std::isfinite(end_time_s) ||
      end_time_s <= start_time_s + kTimeEpsilonS) {
    return {};
  }

  std::vector<RoadNoiseStateSegmentRow> clipped;
  for (const auto &segment : segments_) {
    if (!IsFiniteSegment(segment)) {
      continue;
    }
    const double clipped_start = std::max(segment.start_time_s, start_time_s);
    const double clipped_end = std::min(segment.end_time_s, end_time_s);
    if (clipped_end <= clipped_start + kTimeEpsilonS) {
      continue;
    }

    RoadNoiseStateSegmentRow row = segment;
    row.segment_index = clipped.size();
    row.start_time_s = clipped_start;
    row.end_time_s = clipped_end;
    row.duration_s = clipped_end - clipped_start;
    if (segment.source.empty()) {
      row.source = "GLOBAL_ROAD_NOISE_STATE";
    } else if (segment.source.find("_GLOBAL_CLIPPED") != std::string::npos) {
      row.source = segment.source;
    } else {
      row.source = segment.source + "_GLOBAL_CLIPPED";
    }
    row.skip_reason = "GLOBAL_CLIPPED";
    clipped.push_back(std::move(row));
  }
  return clipped;
}

}  // namespace offline_lc_minimal
