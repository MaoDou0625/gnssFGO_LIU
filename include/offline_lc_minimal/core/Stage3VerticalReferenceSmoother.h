#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

[[nodiscard]] std::vector<double> BuildStage3VerticalReferenceSmoothedProfile(
  const OfflineRunnerConfig &config,
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &input_up_m,
  std::size_t first_filter_index,
  std::size_t one_past_last_filter_index);

}  // namespace offline_lc_minimal
