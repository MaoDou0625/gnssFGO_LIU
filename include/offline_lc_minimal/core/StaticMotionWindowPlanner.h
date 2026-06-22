#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

struct StaticMotionWindow {
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  std::size_t source_window_count = 0U;
};

struct StaticMotionWindowPlanRequest {
  double alignment_start_time_s = 0.0;
  double alignment_end_time_s = 0.0;
  const std::vector<LateStaticWindowRow> *initial_dynamic_static_windows = nullptr;
  const std::vector<LateStaticWindowRow> *late_static_windows = nullptr;
  double merge_gap_s = 0.0;
};

[[nodiscard]] std::vector<StaticMotionWindow> BuildStaticMotionWindows(
  const StaticMotionWindowPlanRequest &request);

[[nodiscard]] bool IntervalOverlapsStaticMotionWindow(
  double start_time_s,
  double end_time_s,
  const std::vector<StaticMotionWindow> &windows);

}  // namespace offline_lc_minimal
