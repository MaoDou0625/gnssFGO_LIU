#pragma once

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

struct RtkVerticalDriftGateWeightingResult {
  double gate_half_width_m = 0.0;
  double gate_observation_m = 0.0;
  double gate_violation_m = 0.0;
  double gate_weight = 1.0;
  double effective_white_sigma_m = 0.0;
};

[[nodiscard]] RtkVerticalDriftGateWeightingResult ComputeRtkVerticalDriftGateWeighting(
  const OfflineRunnerConfig &config,
  double observation_m,
  double sigma_u_m);

}  // namespace offline_lc_minimal
