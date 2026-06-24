#include "offline_lc_minimal/core/VerticalVelocityDeltaTargetPlanner.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {
namespace {

[[nodiscard]] double ClampWithAccelerationLimit(
  const double target_delta_vz_mps,
  const double acceleration_limit_mps2,
  const double dt_s) {
  if (dt_s <= 0.0 ||
      !std::isfinite(dt_s) ||
      !std::isfinite(target_delta_vz_mps) ||
      acceleration_limit_mps2 <= 0.0 ||
      !std::isfinite(acceleration_limit_mps2)) {
    return target_delta_vz_mps;
  }
  const double limit_mps = acceleration_limit_mps2 * dt_s;
  return std::clamp(target_delta_vz_mps, -limit_mps, limit_mps);
}

[[nodiscard]] bool IsLowSpeedVerticalHandoff(
  const OfflineRunnerConfig &config,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) {
  if (!config.enable_vertical_motion_adaptive_reweighting ||
      stability_entry == nullptr ||
      stability_entry->in_jump_padding) {
    return false;
  }
  return std::isfinite(stability_entry->horizontal_speed_rms_mps) &&
         std::isfinite(stability_entry->target_vertical_acc_rms_mps2) &&
         stability_entry->horizontal_speed_rms_mps <=
           config.vertical_motion_adaptive_static_horizontal_speed_rms_mps &&
         stability_entry->target_vertical_acc_rms_mps2 <=
           config.vertical_motion_adaptive_static_target_acc_rms_mps2;
}

[[nodiscard]] double TargetAccelerationLimitMps2(
  const OfflineRunnerConfig &config,
  const VerticalVelocityDeltaTargetContext *target_context) {
  double acceleration_limit_mps2 =
    config.vertical_velocity_delta_target_acc_limit_mps2;
  if (target_context != nullptr &&
      target_context->overlaps_road_high_noise_bias &&
      config.enable_vertical_velocity_delta_high_noise_target_acc_limit_scale) {
    acceleration_limit_mps2 *=
      config.vertical_velocity_delta_high_noise_target_acc_limit_scale;
  }
  return acceleration_limit_mps2;
}

}  // namespace

double PlanVerticalVelocityDeltaTarget(
  const OfflineRunnerConfig &config,
  const double raw_target_delta_vz_mps,
  const double dt_s,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry,
  const VerticalVelocityDeltaTargetContext *target_context) {
  double target_delta_vz_mps = ClampWithAccelerationLimit(
    raw_target_delta_vz_mps,
    TargetAccelerationLimitMps2(config, target_context),
    dt_s);

  if (IsLowSpeedVerticalHandoff(config, stability_entry)) {
    target_delta_vz_mps = ClampWithAccelerationLimit(
      target_delta_vz_mps,
      config.vertical_motion_adaptive_static_target_acc_rms_mps2,
      dt_s);
  }

  return target_delta_vz_mps;
}

}  // namespace offline_lc_minimal
