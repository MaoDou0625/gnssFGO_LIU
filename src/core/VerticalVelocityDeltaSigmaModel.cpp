#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {

VerticalVelocityDeltaSigmaModel::VerticalVelocityDeltaSigmaModel(const OfflineRunnerConfig &config)
    : config_(config) {}

VerticalVelocityDeltaSigmaResult VerticalVelocityDeltaSigmaModel::Compute(const double dt_s) const {
  const double positive_dt_s = std::max(dt_s, 0.0);
  VerticalVelocityDeltaSigmaResult result;
  result.legacy_sigma_mps = std::max(
    config_.vertical_velocity_delta_min_sigma_mps,
    config_.vertical_velocity_delta_acc_sigma_mps2 * positive_dt_s);

  if (!config_.enable_vertical_velocity_delta_bias_consistent_sigma) {
    result.model = "legacy";
    result.sigma_mps = result.legacy_sigma_mps;
    return result;
  }

  result.model = "bias_consistent";
  result.bias_sigma_mps = config_.vertical_velocity_delta_bias_sigma_mps2 * positive_dt_s;
  result.attitude_sigma_mps =
    config_.gravity_mps2 * config_.vertical_velocity_delta_attitude_sigma_rad * positive_dt_s;
  result.sigma_floor_mps = config_.vertical_velocity_delta_sigma_floor_mps;
  result.sigma_ceiling_mps = config_.vertical_velocity_delta_sigma_ceiling_mps;

  const double raw_sigma_mps = std::sqrt(
    result.bias_sigma_mps * result.bias_sigma_mps +
    result.attitude_sigma_mps * result.attitude_sigma_mps +
    result.sigma_floor_mps * result.sigma_floor_mps);
  result.sigma_mps = std::clamp(raw_sigma_mps, result.sigma_floor_mps, result.sigma_ceiling_mps);
  result.clamped_floor = result.sigma_mps <= result.sigma_floor_mps * (1.0 + 1.0e-12);
  result.clamped_ceiling = result.sigma_mps >= result.sigma_ceiling_mps && raw_sigma_mps > result.sigma_ceiling_mps;
  return result;
}

}  // namespace offline_lc_minimal
