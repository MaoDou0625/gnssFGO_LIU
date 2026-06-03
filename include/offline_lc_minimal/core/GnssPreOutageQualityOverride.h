#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/SensorTypes.h"

namespace offline_lc_minimal {

struct GnssPreOutageQualityOverrideSummary {
  std::size_t planned_outage_count = 0;
  std::size_t rtkfix_to_float_count = 0;
  std::size_t nonfix_to_no_solution_count = 0;
};

GnssPreOutageQualityOverrideSummary ApplyGnssPreOutageQualityOverride(
  const OfflineRunnerConfig &config,
  std::vector<GnssSolutionSample> &gnss_samples);

}  // namespace offline_lc_minimal
