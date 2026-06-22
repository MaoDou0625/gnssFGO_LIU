#include "offline_lc_minimal/core/StaticMotionWindowPlanner.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsValidInterval(const double start_time_s, const double end_time_s) {
  return std::isfinite(start_time_s) && std::isfinite(end_time_s) &&
         end_time_s + kTimeEpsilonS >= start_time_s;
}

void AppendDetectedWindows(
  const std::vector<LateStaticWindowRow> *source,
  std::vector<StaticMotionWindow> &windows) {
  if (source == nullptr) {
    return;
  }
  for (const auto &window : *source) {
    if (!window.valid || !IsValidInterval(window.start_time_s, window.end_time_s)) {
      continue;
    }
    windows.push_back(StaticMotionWindow{
      window.start_time_s,
      window.end_time_s,
      1U});
  }
}

}  // namespace

std::vector<StaticMotionWindow> BuildStaticMotionWindows(
  const StaticMotionWindowPlanRequest &request) {
  std::vector<StaticMotionWindow> windows;
  if (IsValidInterval(request.alignment_start_time_s, request.alignment_end_time_s)) {
    windows.push_back(StaticMotionWindow{
      request.alignment_start_time_s,
      request.alignment_end_time_s,
      1U});
  }
  AppendDetectedWindows(request.initial_dynamic_static_windows, windows);
  AppendDetectedWindows(request.late_static_windows, windows);

  std::sort(
    windows.begin(),
    windows.end(),
    [](const StaticMotionWindow &lhs, const StaticMotionWindow &rhs) {
      if (lhs.start_time_s == rhs.start_time_s) {
        return lhs.end_time_s < rhs.end_time_s;
      }
      return lhs.start_time_s < rhs.start_time_s;
    });

  std::vector<StaticMotionWindow> merged;
  const double merge_gap_s = std::max(0.0, request.merge_gap_s);
  for (const auto &window : windows) {
    if (merged.empty() ||
        window.start_time_s > merged.back().end_time_s + merge_gap_s + kTimeEpsilonS) {
      merged.push_back(window);
      continue;
    }
    merged.back().end_time_s = std::max(merged.back().end_time_s, window.end_time_s);
    merged.back().source_window_count += window.source_window_count;
  }
  return merged;
}

bool IntervalOverlapsStaticMotionWindow(
  const double start_time_s,
  const double end_time_s,
  const std::vector<StaticMotionWindow> &windows) {
  if (!IsValidInterval(start_time_s, end_time_s)) {
    return false;
  }
  return std::any_of(
    windows.begin(),
    windows.end(),
    [&](const StaticMotionWindow &window) {
      return start_time_s <= window.end_time_s + kTimeEpsilonS &&
             window.start_time_s <= end_time_s + kTimeEpsilonS;
    });
}

}  // namespace offline_lc_minimal
