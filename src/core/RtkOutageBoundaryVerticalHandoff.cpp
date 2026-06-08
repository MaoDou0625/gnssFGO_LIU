#include "offline_lc_minimal/core/RtkOutageBoundaryVerticalHandoff.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

void RefreshReferenceValidity(RtkOutageBoundaryReferenceRow &reference) {
  reference.valid =
    reference.has_up || reference.has_vz || reference.has_ba_z ||
    reference.has_horizontal_position || reference.has_horizontal_velocity ||
    reference.has_horizontal_position_velocity_handoff ||
    reference.has_vertical_position_velocity_handoff || reference.has_attitude;
  if (!reference.valid) {
    reference.skip_reason = "nonfinite_trajectory_reference";
  } else if (reference.skip_reason == "UNSET" ||
             reference.skip_reason == "nonfinite_trajectory_reference" ||
             reference.skip_reason == "invalid_boundary_time") {
    reference.skip_reason = "OK";
  }
}

double BoundaryVerticalPositionVelocityHandoffSigmaM(
  const OfflineRunnerConfig &config) {
  if (std::isfinite(config.vertical_position_velocity_consistency_sigma_m) &&
      config.vertical_position_velocity_consistency_sigma_m > 0.0) {
    return config.vertical_position_velocity_consistency_sigma_m;
  }
  return config.rtk_outage_boundary_up_sigma_m;
}

const ReferenceNodeState *LastKeptOutageReferenceState(
  const std::vector<ReferenceNodeState> &states,
  const RtkOutageWindowRow &outage,
  const double post_first_time_s) {
  const ReferenceNodeState *best_state = nullptr;
  for (const auto &state : states) {
    if (!std::isfinite(state.time_s)) {
      continue;
    }
    if (state.time_s <= outage.start_time_s + kTimeEpsilonS) {
      continue;
    }
    if (state.time_s >= post_first_time_s - kTimeEpsilonS) {
      continue;
    }
    best_state = &state;
  }
  return best_state;
}

void StripNonVerticalBoundaryReferences(
  RtkOutageBoundaryReferenceRow &reference) {
  reference.has_attitude = false;
  reference.add_attitude_constraint = false;
  reference.attitude_sigma_rad = std::numeric_limits<double>::quiet_NaN();
  reference.has_ba_z = false;
  reference.add_ba_z_constraint = false;
  reference.reference_ba_z_mps2 = std::numeric_limits<double>::quiet_NaN();
  reference.ba_z_sigma_mps2 = std::numeric_limits<double>::quiet_NaN();
  reference.has_horizontal_position = false;
  reference.add_horizontal_position_constraint = false;
  reference.reference_horizontal_position_m =
    Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  reference.horizontal_position_sigma_m = std::numeric_limits<double>::quiet_NaN();
  reference.has_horizontal_velocity = false;
  reference.add_horizontal_velocity_constraint = false;
  reference.reference_horizontal_velocity_mps =
    Eigen::Vector2d::Constant(std::numeric_limits<double>::quiet_NaN());
  reference.horizontal_velocity_sigma_mps = std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

std::optional<double> FirstKeptOutageStateTime(
  const std::vector<double> &state_timestamps,
  const RtkOutageWindowRow &outage) {
  if (!std::isfinite(outage.start_time_s) || !std::isfinite(outage.end_time_s) ||
      outage.end_time_s <= outage.start_time_s + kTimeEpsilonS) {
    return std::nullopt;
  }
  const auto lower = std::upper_bound(
    state_timestamps.begin(),
    state_timestamps.end(),
    outage.start_time_s + kTimeEpsilonS);
  for (auto it = lower; it != state_timestamps.end(); ++it) {
    const double time_s = *it;
    if (!std::isfinite(time_s)) {
      continue;
    }
    if (time_s >= outage.end_time_s - kTimeEpsilonS) {
      break;
    }
    return time_s;
  }
  return std::nullopt;
}

void DisableDirectRtkOutageBoundaryUpConstraint(
  RtkOutageBoundaryReferenceRow &reference) {
  reference.add_up_constraint = false;
  reference.up_sigma_m = std::numeric_limits<double>::quiet_NaN();
  RefreshReferenceValidity(reference);
}

void ConfigureVerticalPositionVelocityHandoff(
  RtkOutageBoundaryReferenceRow &reference,
  const OfflineRunnerConfig &config,
  const double target_time_s,
  const double reference_time_s,
  const bool add_direct_vz_constraint,
  const std::string &missing_reference_reason) {
  reference.target_time_s = target_time_s;
  reference.has_up = std::isfinite(reference.reference_up_m);
  reference.has_vz = std::isfinite(reference.reference_vz_mps);
  reference.add_up_constraint = false;
  reference.add_vz_constraint = add_direct_vz_constraint && reference.has_vz;
  reference.up_sigma_m = std::numeric_limits<double>::quiet_NaN();
  reference.vertical_position_velocity_handoff_reference_time_s = reference_time_s;
  reference.vertical_position_velocity_handoff_sigma_m =
    BoundaryVerticalPositionVelocityHandoffSigmaM(config);
  reference.has_vertical_position_velocity_handoff =
    reference.has_up &&
    reference.has_vz &&
    std::isfinite(reference.target_time_s) &&
    std::isfinite(reference.vertical_position_velocity_handoff_reference_time_s) &&
    std::abs(reference.vertical_position_velocity_handoff_reference_time_s -
             reference.target_time_s) > kTimeEpsilonS &&
    std::isfinite(reference.vertical_position_velocity_handoff_sigma_m) &&
    reference.vertical_position_velocity_handoff_sigma_m > 0.0;
  reference.add_vertical_position_velocity_handoff_constraint =
    reference.has_vertical_position_velocity_handoff;
  RefreshReferenceValidity(reference);
  reference.skip_reason =
    reference.has_vertical_position_velocity_handoff ? "OK" : missing_reference_reason;
}

RtkOutageBoundaryReferenceRow MakeVerticalPositionVelocityHandoffReference(
  const OfflineRunnerConfig &config,
  RtkOutageBoundaryReferenceRow reference,
  const std::string &boundary_role,
  const std::string &source_type,
  const double target_time_s,
  const double reference_time_s,
  const bool add_direct_vz_constraint,
  const std::string &missing_reference_reason) {
  reference.boundary_role = boundary_role;
  reference.source_type = source_type;
  StripNonVerticalBoundaryReferences(reference);
  ConfigureVerticalPositionVelocityHandoff(
    reference,
    config,
    target_time_s,
    reference_time_s,
    add_direct_vz_constraint,
    missing_reference_reason);
  return reference;
}

void AttachPostStartVerticalPositionVelocityHandoff(
  const OfflineRunnerConfig &config,
  const OfflineRunResult &outage_result,
  const RtkOutageWindowRow &outage,
  const double post_first_time_s,
  RtkOutageBoundaryReferenceRow &reference) {
  const ReferenceNodeState *outage_last_state = LastKeptOutageReferenceState(
    outage_result.optimized_reference_states,
    outage,
    post_first_time_s);
  if (outage_last_state == nullptr) {
    return;
  }
  reference.reference_up_m = outage_last_state->pose.translation().z();
  reference.reference_vz_mps = outage_last_state->velocity.z();
  reference.has_up = std::isfinite(reference.reference_up_m);
  reference.has_vz = std::isfinite(reference.reference_vz_mps);
  ConfigureVerticalPositionVelocityHandoff(
    reference,
    config,
    post_first_time_s,
    outage_last_state->time_s,
    true,
    "missing_post_start_vertical_handoff_reference");
}

}  // namespace offline_lc_minimal
