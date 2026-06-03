#include "offline_lc_minimal/core/BodyZBiasReestimatePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

struct TimeRange {
  double start_time_s = 0.0;
  double end_time_s = 0.0;
};

bool IsFiniteWindow(const BodyZSeedJumpWindowRow &window) {
  return std::isfinite(window.start_time_s) &&
         std::isfinite(window.end_time_s) &&
         window.end_time_s > window.start_time_s;
}

double WindowDurationS(const BodyZSeedJumpWindowRow &window) {
  if (std::isfinite(window.duration_s) && window.duration_s > 0.0) {
    return window.duration_s;
  }
  return window.end_time_s - window.start_time_s;
}

double DetectedBiasDeltaMps2(const BodyZSeedJumpWindowRow &window) {
  const double duration_s = WindowDurationS(window);
  if (duration_s <= 0.0 || !std::isfinite(duration_s) ||
      !std::isfinite(window.signed_delta_velocity_mps)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return window.signed_delta_velocity_mps / duration_s;
}

double PositiveOverlapDurationS(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return std::max(0.0, std::min(left_end_s, right_end_s) - std::max(left_start_s, right_start_s));
}

std::vector<TimeRange> BuildBlockedRanges(
  const BodyZSeedJumpWindowRow &bias_window,
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const double jump_padding_s) {
  std::vector<TimeRange> blocked_ranges;
  for (const auto &jump_window : jump_windows) {
    if (!IsFiniteWindow(jump_window)) {
      continue;
    }
    const double start_time_s = std::max(
      bias_window.start_time_s,
      jump_window.start_time_s - std::max(0.0, jump_padding_s));
    const double end_time_s = std::min(
      bias_window.end_time_s,
      jump_window.end_time_s + std::max(0.0, jump_padding_s));
    if (PositiveOverlapDurationS(
          start_time_s,
          end_time_s,
          bias_window.start_time_s,
          bias_window.end_time_s) <= kTimeEpsilonS ||
        end_time_s <= start_time_s + kTimeEpsilonS) {
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

void AppendSegment(
  const BodyZSeedJumpWindowRow &bias_window,
  const std::size_t bias_window_index,
  const double start_time_s,
  const double end_time_s,
  const double min_segment_duration_s,
  std::vector<BodyZBiasReestimateSegmentRow> &segments) {
  const double duration_s = end_time_s - start_time_s;
  if (duration_s + kTimeEpsilonS < std::max(0.0, min_segment_duration_s)) {
    return;
  }
  if (duration_s <= kTimeEpsilonS) {
    return;
  }
  BodyZBiasReestimateSegmentRow row;
  row.segment_index = segments.size();
  row.source_bias_window_index = bias_window_index;
  row.bias_window_start_time_s = bias_window.start_time_s;
  row.bias_window_end_time_s = bias_window.end_time_s;
  row.start_time_s = start_time_s;
  row.end_time_s = end_time_s;
  row.duration_s = duration_s;
  row.detected_bias_delta_mps2 = DetectedBiasDeltaMps2(bias_window);
  segments.push_back(std::move(row));
}

}  // namespace

std::vector<BodyZBiasReestimateSegmentRow> PlanBodyZBiasReestimateSegments(
  const std::vector<BodyZSeedJumpWindowRow> &bias_windows,
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const BodyZBiasReestimatePlannerOptions &options) {
  std::vector<BodyZBiasReestimateSegmentRow> segments;
  for (std::size_t bias_window_index = 0; bias_window_index < bias_windows.size(); ++bias_window_index) {
    const auto &bias_window = bias_windows[bias_window_index];
    if (!IsFiniteWindow(bias_window)) {
      continue;
    }
    if (options.min_bias_window_duration_s > 0.0 &&
        WindowDurationS(bias_window) <= options.min_bias_window_duration_s + kTimeEpsilonS) {
      continue;
    }

    const std::vector<TimeRange> blocked_ranges =
      BuildBlockedRanges(bias_window, jump_windows, options.jump_padding_s);
    double cursor_time_s = bias_window.start_time_s;
    for (const auto &blocked_range : blocked_ranges) {
      AppendSegment(
        bias_window,
        bias_window_index,
        cursor_time_s,
        blocked_range.start_time_s,
        options.min_segment_duration_s,
        segments);
      cursor_time_s = std::max(cursor_time_s, blocked_range.end_time_s);
    }
    AppendSegment(
      bias_window,
      bias_window_index,
      cursor_time_s,
      bias_window.end_time_s,
      options.min_segment_duration_s,
      segments);
  }
  return segments;
}

}  // namespace offline_lc_minimal
