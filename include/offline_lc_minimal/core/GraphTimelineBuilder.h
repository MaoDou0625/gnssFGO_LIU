#pragma once

#include <cstddef>
#include <map>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct GraphTimeline {
  std::vector<double> timestamps_s;
  std::size_t dynamic_start_index = 0;
  std::size_t initial_static_state_count = 0;
};

[[nodiscard]] std::vector<double> BuildStateTimestamps(
  double start_time_s,
  double end_time_s,
  double state_frequency_hz);

[[nodiscard]] GraphTimeline BuildGraphTimeline(
  double alignment_start_time_s,
  double alignment_end_time_s,
  double navigation_start_time_s,
  double end_time_s,
  const OfflineRunnerConfig &config);

[[nodiscard]] StateMeasSyncResult FindStateForMeasurement(
  const std::map<std::size_t, double> &state_timestamp_map,
  double corrected_timestamp_s,
  const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
