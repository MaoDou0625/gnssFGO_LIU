#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct BodyZJumpConstraintWindow {
  std::size_t source_window_index = 0;
  std::size_t source_window_count = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
};

struct BodyZJumpConstraintWindowOptions {
  double padding_s = 0.0;
  double merge_gap_s = 0.0;
  double merge_max_duration_s = 0.0;
};

[[nodiscard]] std::vector<BodyZJumpConstraintWindow> BuildBodyZJumpConstraintWindows(
  const std::vector<BodyZSeedJumpWindowRow> &jump_windows,
  const BodyZJumpConstraintWindowOptions &options);

[[nodiscard]] BodyZJumpConstraintWindowOptions BodyZNHCJumpConstraintWindowOptions(
  const OfflineRunnerConfig &config);

[[nodiscard]] BodyZJumpConstraintWindowOptions VerticalVelocityDeltaJumpConstraintWindowOptions(
  const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
