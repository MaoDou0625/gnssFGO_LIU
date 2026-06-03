#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace offline_lc_minimal {
namespace {

void ApplyOutputScale(VerticalVelocityDeltaSigmaResult &result, const double scale) {
  if (scale == 1.0) {
    return;
  }
  result.sigma_mps *= scale;
  result.sigma_floor_mps *= scale;
  result.sigma_ceiling_mps *= scale;
}

}  // namespace

VerticalVelocityDeltaSigmaModel::VerticalVelocityDeltaSigmaModel(const OfflineRunnerConfig &config)
    : config_(config) {}

VerticalVelocityDeltaSigmaResult VerticalVelocityDeltaSigmaModel::Compute(const double dt_s) const {
  return Compute(dt_s, config_.vertical_velocity_delta_sigma_scale);
}

VerticalVelocityDeltaSigmaResult VerticalVelocityDeltaSigmaModel::Compute(
  const double dt_s,
  const double output_scale) const {
  VerticalVelocityDeltaSigmaResult result = ComputeWithoutOutputScale(dt_s);
  ApplyOutputScale(result, output_scale);
  return result;
}

VerticalVelocityDeltaSigmaResult VerticalVelocityDeltaSigmaModel::ComputeWithoutOutputScale(
  const double dt_s) const {
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

  result = ComputeBiasConsistent(
    positive_dt_s,
    config_.vertical_velocity_delta_bias_sigma_mps2,
    config_.vertical_velocity_delta_attitude_sigma_rad,
    config_.vertical_velocity_delta_sigma_floor_mps,
    config_.vertical_velocity_delta_sigma_ceiling_mps,
    "bias_consistent");
  result.legacy_sigma_mps = std::max(
    config_.vertical_velocity_delta_min_sigma_mps,
    config_.vertical_velocity_delta_acc_sigma_mps2 * positive_dt_s);
  return result;
}

VerticalVelocityDeltaSigmaResult VerticalVelocityDeltaSigmaModel::Compute(
  const double dt_s,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) const {
  return Compute(dt_s, stability_entry, config_.vertical_velocity_delta_sigma_scale);
}

VerticalVelocityDeltaSigmaResult VerticalVelocityDeltaSigmaModel::Compute(
  const double dt_s,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry,
  const double output_scale) const {
  VerticalVelocityDeltaSigmaResult base = ComputeWithoutOutputScale(dt_s);
  if (!config_.enable_vertical_motion_adaptive_reweighting ||
      stability_entry == nullptr ||
      !std::isfinite(stability_entry->motion_score) ||
      stability_entry->in_jump_padding ||
      !config_.enable_vertical_velocity_delta_bias_consistent_sigma) {
    ApplyOutputScale(base, output_scale);
    return base;
  }

  const double positive_dt_s = std::max(dt_s, 0.0);
  const VerticalVelocityDeltaSigmaResult static_result = ComputeBiasConsistent(
    positive_dt_s,
    config_.vertical_motion_adaptive_static_dvz_bias_sigma_mps2,
    config_.vertical_motion_adaptive_static_attitude_sigma_rad,
    config_.vertical_motion_adaptive_static_sigma_floor_mps,
    config_.vertical_motion_adaptive_static_sigma_ceiling_mps,
    "adaptive_static");
  const double score = std::clamp(stability_entry->motion_score, 0.0, 1.0);
  const double base_sigma = std::max(base.sigma_mps, 1.0e-12);
  const double static_sigma = std::max(static_result.sigma_mps, 1.0e-12);
  const double blended_sigma =
    std::exp((1.0 - score) * std::log(static_sigma) + score * std::log(base_sigma));

  VerticalVelocityDeltaSigmaResult result = base;
  result.model = score <= 1.0e-6 ? "adaptive_static" : "adaptive_motion";
  result.sigma_mps = blended_sigma;
  result.bias_sigma_mps =
    (1.0 - score) * static_result.bias_sigma_mps + score * base.bias_sigma_mps;
  result.attitude_sigma_mps =
    (1.0 - score) * static_result.attitude_sigma_mps + score * base.attitude_sigma_mps;
  result.sigma_floor_mps =
    (1.0 - score) * static_result.sigma_floor_mps + score * base.sigma_floor_mps;
  result.sigma_ceiling_mps =
    (1.0 - score) * static_result.sigma_ceiling_mps + score * base.sigma_ceiling_mps;
  result.sigma_mps = std::clamp(blended_sigma, result.sigma_floor_mps, result.sigma_ceiling_mps);
  result.clamped_floor = result.sigma_mps <= result.sigma_floor_mps * (1.0 + 1.0e-12);
  result.clamped_ceiling =
    result.sigma_mps >= result.sigma_ceiling_mps &&
    blended_sigma > result.sigma_ceiling_mps;
  ApplyOutputScale(result, output_scale);
  return result;
}

VerticalVelocityDeltaSigmaResult VerticalVelocityDeltaSigmaModel::ComputeBiasConsistent(
  const double dt_s,
  const double bias_sigma_mps2,
  const double attitude_sigma_rad,
  const double floor_mps,
  const double ceiling_mps,
  std::string model) const {
  const double positive_dt_s = std::max(dt_s, 0.0);
  VerticalVelocityDeltaSigmaResult result;
  result.model = std::move(model);
  result.legacy_sigma_mps = std::max(
    config_.vertical_velocity_delta_min_sigma_mps,
    config_.vertical_velocity_delta_acc_sigma_mps2 * positive_dt_s);
  result.bias_sigma_mps = bias_sigma_mps2 * positive_dt_s;
  result.attitude_sigma_mps = config_.gravity_mps2 * attitude_sigma_rad * positive_dt_s;
  result.sigma_floor_mps = floor_mps;
  result.sigma_ceiling_mps = ceiling_mps;

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
