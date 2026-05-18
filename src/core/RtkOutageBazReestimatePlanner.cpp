#include "offline_lc_minimal/core/RtkOutageBazReestimatePlanner.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr const char *kBodyZBiasSource = "BODY_Z_BIAS";
constexpr const char *kRtkOutageSource = "RTK_OUTAGE";

struct TimeRange {
  double start_time_s = 0.0;
  double end_time_s = 0.0;
};

bool IsFiniteRange(const double start_time_s, const double end_time_s) {
  return std::isfinite(start_time_s) &&
         std::isfinite(end_time_s) &&
         end_time_s > start_time_s + kTimeEpsilonS;
}

bool IsFiniteJumpWindow(const BodyZSeedJumpWindowRow &window) {
  return IsFiniteRange(window.start_time_s, window.end_time_s);
}

bool IsFiniteOutageWindow(const RtkOutageWindowRow &window) {
  return IsFiniteRange(window.start_time_s, window.end_time_s);
}

bool IsFiniteSegment(const BodyZBiasReestimateSegmentRow &segment) {
  return IsFiniteRange(segment.start_time_s, segment.end_time_s);
}

double PositiveOverlapDurationS(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return std::max(0.0, std::min(left_end_s, right_end_s) - std::max(left_start_s, right_start_s));
}

bool HasPositiveOverlap(
  const BodyZBiasReestimateSegmentRow &left,
  const BodyZBiasReestimateSegmentRow &right) {
  return PositiveOverlapDurationS(
           left.start_time_s,
           left.end_time_s,
           right.start_time_s,
           right.end_time_s) > kTimeEpsilonS;
}

bool IsBodyZBiasSource(const BodyZBiasReestimateSegmentRow &segment) {
  return segment.source_type.empty() || segment.source_type == kBodyZBiasSource;
}

bool IsRtkOutageLinked(const BodyZBiasReestimateSegmentRow &segment) {
  return segment.source_outage_window_index >= 0 || segment.source_type == kRtkOutageSource;
}

std::vector<TimeRange> BuildBlockedRanges(
  const double range_start_time_s,
  const double range_end_time_s,
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const double jump_padding_s) {
  std::vector<TimeRange> blocked_ranges;
  for (const auto &jump_window : jump_windows) {
    if (!IsFiniteJumpWindow(jump_window)) {
      continue;
    }
    const double start_time_s = std::max(
      range_start_time_s,
      jump_window.start_time_s - std::max(0.0, jump_padding_s));
    const double end_time_s = std::min(
      range_end_time_s,
      jump_window.end_time_s + std::max(0.0, jump_padding_s));
    if (end_time_s <= start_time_s + kTimeEpsilonS) {
      continue;
    }
    blocked_ranges.push_back(TimeRange{start_time_s, end_time_s});
  }

  std::sort(
    blocked_ranges.begin(),
    blocked_ranges.end(),
    [](const TimeRange &left, const TimeRange &right) {
      return left.start_time_s < right.start_time_s;
    });

  std::vector<TimeRange> merged_ranges;
  for (const auto &range : blocked_ranges) {
    if (merged_ranges.empty() ||
        range.start_time_s > merged_ranges.back().end_time_s + kTimeEpsilonS) {
      merged_ranges.push_back(range);
      continue;
    }
    merged_ranges.back().end_time_s = std::max(merged_ranges.back().end_time_s, range.end_time_s);
  }
  return merged_ranges;
}

void AppendRtkOutageSegment(
  const RtkOutageWindowRow &outage_window,
  const std::size_t outage_window_index,
  const double start_time_s,
  const double end_time_s,
  const double min_segment_duration_s,
  std::vector<BodyZBiasReestimateSegmentRow> &segments) {
  const double duration_s = end_time_s - start_time_s;
  if (duration_s <= kTimeEpsilonS ||
      duration_s + kTimeEpsilonS < std::max(0.0, min_segment_duration_s)) {
    return;
  }

  BodyZBiasReestimateSegmentRow row;
  row.segment_index = segments.size();
  row.source_type = kRtkOutageSource;
  row.source_bias_window_index = std::numeric_limits<std::size_t>::max();
  row.source_outage_window_index = static_cast<long long>(outage_window_index);
  row.bias_window_start_time_s = outage_window.start_time_s;
  row.bias_window_end_time_s = outage_window.end_time_s;
  row.start_time_s = start_time_s;
  row.end_time_s = end_time_s;
  row.duration_s = duration_s;
  row.detected_bias_delta_mps2 = 0.0;
  segments.push_back(std::move(row));
}

std::vector<BodyZBiasReestimateSegmentRow> BuildRtkOutageSegments(
  const std::vector<RtkOutageWindowRow> &rtk_outage_windows,
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const RtkOutageBazReestimatePlannerOptions &options) {
  std::vector<BodyZBiasReestimateSegmentRow> segments;
  for (std::size_t outage_window_index = 0; outage_window_index < rtk_outage_windows.size(); ++outage_window_index) {
    const auto &outage_window = rtk_outage_windows[outage_window_index];
    if (!IsFiniteOutageWindow(outage_window)) {
      continue;
    }
    const std::vector<TimeRange> blocked_ranges =
      BuildBlockedRanges(
        outage_window.start_time_s,
        outage_window.end_time_s,
        jump_windows,
        options.jump_padding_s);
    double cursor_time_s = outage_window.start_time_s;
    for (const auto &blocked_range : blocked_ranges) {
      AppendRtkOutageSegment(
        outage_window,
        outage_window_index,
        cursor_time_s,
        blocked_range.start_time_s,
        options.min_segment_duration_s,
        segments);
      cursor_time_s = std::max(cursor_time_s, blocked_range.end_time_s);
    }
    AppendRtkOutageSegment(
      outage_window,
      outage_window_index,
      cursor_time_s,
      outage_window.end_time_s,
      options.min_segment_duration_s,
      segments);
  }
  return segments;
}

BodyZBiasReestimateSegmentRow NormalizeBodyZSegment(
  BodyZBiasReestimateSegmentRow segment) {
  if (segment.source_type.empty()) {
    segment.source_type = kBodyZBiasSource;
  }
  return segment;
}

void MergeSegment(
  BodyZBiasReestimateSegmentRow &target,
  const BodyZBiasReestimateSegmentRow &source) {
  const bool target_has_body_z = IsBodyZBiasSource(target);
  const bool source_has_body_z = IsBodyZBiasSource(source);

  target.start_time_s = std::min(target.start_time_s, source.start_time_s);
  target.end_time_s = std::max(target.end_time_s, source.end_time_s);
  target.duration_s = target.end_time_s - target.start_time_s;

  if (std::isfinite(source.bias_window_start_time_s)) {
    target.bias_window_start_time_s =
      std::isfinite(target.bias_window_start_time_s)
        ? std::min(target.bias_window_start_time_s, source.bias_window_start_time_s)
        : source.bias_window_start_time_s;
  }
  if (std::isfinite(source.bias_window_end_time_s)) {
    target.bias_window_end_time_s =
      std::isfinite(target.bias_window_end_time_s)
        ? std::max(target.bias_window_end_time_s, source.bias_window_end_time_s)
        : source.bias_window_end_time_s;
  }

  if (source.source_outage_window_index >= 0 && target.source_outage_window_index < 0) {
    target.source_outage_window_index = source.source_outage_window_index;
  }

  if (!target_has_body_z && source_has_body_z) {
    target.source_type = kBodyZBiasSource;
    target.source_bias_window_index = source.source_bias_window_index;
    target.detected_bias_delta_mps2 = source.detected_bias_delta_mps2;
  }

  if (!target_has_body_z && !source_has_body_z && IsRtkOutageLinked(source)) {
    target.source_type = kRtkOutageSource;
    if (target.source_outage_window_index < 0) {
      target.source_outage_window_index = source.source_outage_window_index;
    }
    target.detected_bias_delta_mps2 = 0.0;
  }
}

std::vector<BodyZBiasReestimateSegmentRow> MergeSegments(
  std::vector<BodyZBiasReestimateSegmentRow> candidates) {
  candidates.erase(
    std::remove_if(
      candidates.begin(),
      candidates.end(),
      [](const BodyZBiasReestimateSegmentRow &segment) {
        return !IsFiniteSegment(segment);
      }),
    candidates.end());
  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const BodyZBiasReestimateSegmentRow &left, const BodyZBiasReestimateSegmentRow &right) {
      if (left.start_time_s == right.start_time_s) {
        return left.end_time_s < right.end_time_s;
      }
      return left.start_time_s < right.start_time_s;
    });

  std::vector<BodyZBiasReestimateSegmentRow> merged_segments;
  for (auto candidate : candidates) {
    candidate = NormalizeBodyZSegment(std::move(candidate));
    if (merged_segments.empty() || !HasPositiveOverlap(merged_segments.back(), candidate)) {
      merged_segments.push_back(std::move(candidate));
      continue;
    }
    MergeSegment(merged_segments.back(), candidate);
  }

  for (std::size_t index = 0; index < merged_segments.size(); ++index) {
    merged_segments[index].segment_index = index;
  }
  return merged_segments;
}

}  // namespace

std::vector<BodyZBiasReestimateSegmentRow> PlanRtkOutageBazReestimateSegments(
  const std::vector<BodyZBiasReestimateSegmentRow> &body_z_segments,
  const std::vector<RtkOutageWindowRow> &rtk_outage_windows,
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const RtkOutageBazReestimatePlannerOptions &options) {
  std::vector<BodyZBiasReestimateSegmentRow> candidates;
  candidates.reserve(body_z_segments.size() + rtk_outage_windows.size());
  for (const auto &segment : body_z_segments) {
    candidates.push_back(NormalizeBodyZSegment(segment));
  }
  std::vector<BodyZBiasReestimateSegmentRow> outage_segments =
    BuildRtkOutageSegments(rtk_outage_windows, jump_windows, options);
  candidates.insert(
    candidates.end(),
    std::make_move_iterator(outage_segments.begin()),
    std::make_move_iterator(outage_segments.end()));
  return MergeSegments(std::move(candidates));
}

}  // namespace offline_lc_minimal
