#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {

std::vector<BodyZJumpConstraintWindow> BuildBodyZJumpConstraintWindows(
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const BodyZJumpConstraintWindowOptions &options) {
  std::vector<BodyZJumpConstraintWindow> windows;
  windows.reserve(jump_windows.size());
  for (std::size_t index = 0U; index < jump_windows.size(); ++index) {
    const auto &jump_window = jump_windows[index];
    if (!std::isfinite(jump_window.start_time_s) ||
        !std::isfinite(jump_window.end_time_s) ||
        jump_window.end_time_s < jump_window.start_time_s) {
      continue;
    }
    BodyZJumpConstraintWindow window;
    window.source_window_index = index;
    window.source_window_count = 1U;
    window.start_time_s = jump_window.start_time_s - options.padding_s;
    window.end_time_s = jump_window.end_time_s + options.padding_s;
    windows.push_back(window);
  }

  std::sort(
    windows.begin(),
    windows.end(),
    [](const BodyZJumpConstraintWindow &left, const BodyZJumpConstraintWindow &right) {
      return left.start_time_s < right.start_time_s;
    });

  std::vector<BodyZJumpConstraintWindow> merged_windows;
  for (const auto &window : windows) {
    if (merged_windows.empty() ||
        window.start_time_s > merged_windows.back().end_time_s + options.merge_gap_s) {
      merged_windows.push_back(window);
      continue;
    }
    merged_windows.back().end_time_s =
      std::max(merged_windows.back().end_time_s, window.end_time_s);
    merged_windows.back().source_window_count += window.source_window_count;
  }
  return merged_windows;
}

BodyZJumpConstraintWindowOptions BodyZNHCJumpConstraintWindowOptions(
  const OfflineRunnerConfig &config) {
  BodyZJumpConstraintWindowOptions options;
  options.padding_s = config.body_z_nhc_jump_padding_s;
  options.merge_gap_s = config.body_z_nhc_merge_gap_s;
  return options;
}

BodyZJumpConstraintWindowOptions VerticalVelocityDeltaJumpConstraintWindowOptions(
  const OfflineRunnerConfig &config) {
  if (config.enable_body_z_nhc_constraint) {
    return BodyZNHCJumpConstraintWindowOptions(config);
  }
  BodyZJumpConstraintWindowOptions options;
  options.padding_s = config.vertical_velocity_delta_jump_padding_s;
  options.merge_gap_s = 0.0;
  return options;
}

}  // namespace offline_lc_minimal
