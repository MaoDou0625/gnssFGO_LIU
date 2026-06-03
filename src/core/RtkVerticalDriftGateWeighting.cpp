#include "offline_lc_minimal/core/RtkVerticalDriftGateWeighting.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace offline_lc_minimal {
namespace {

constexpr double kTiny = 1.0e-12;

double GateHalfWidthM(const OfflineRunnerConfig &config, const double sigma_u_m) {
  if (!std::isfinite(sigma_u_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::max(
    config.vertical_envelope_min_half_width_m,
    config.vertical_envelope_gate_sigma_multiple * sigma_u_m);
}

}  // namespace

RtkVerticalDriftGateWeightingResult ComputeRtkVerticalDriftGateWeighting(
  const OfflineRunnerConfig &config,
  const double observation_m,
  const double sigma_u_m) {
  RtkVerticalDriftGateWeightingResult result;
  result.gate_half_width_m = GateHalfWidthM(config, sigma_u_m);
  result.gate_observation_m = observation_m;
  result.effective_white_sigma_m = config.rtk_vertical_white_noise_sigma_m;

  if (!std::isfinite(observation_m) ||
      !std::isfinite(result.gate_half_width_m) ||
      result.gate_half_width_m <= 0.0) {
    result.gate_violation_m = std::numeric_limits<double>::quiet_NaN();
    return result;
  }

  const double abs_observation_m = std::abs(observation_m);
  result.gate_violation_m = std::max(0.0, abs_observation_m - result.gate_half_width_m);
  if (!config.enable_rtk_vertical_drift_gate_weighting) {
    return result;
  }

  result.gate_weight = std::clamp(
    result.gate_half_width_m / std::max(abs_observation_m, kTiny),
    config.rtk_vertical_drift_gate_weight_floor,
    1.0);
  result.effective_white_sigma_m =
    config.rtk_vertical_white_noise_sigma_m / std::sqrt(result.gate_weight);
  return result;
}

}  // namespace offline_lc_minimal
