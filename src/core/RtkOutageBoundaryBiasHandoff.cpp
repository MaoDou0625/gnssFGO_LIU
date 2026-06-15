#include "offline_lc_minimal/core/RtkOutageBoundaryBiasHandoff.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

const ReferenceNodeState *FindLastKeptOutageState(
  const std::vector<ReferenceNodeState> &states,
  const RtkOutageWindowRow &window,
  const double post_first_time_s) {
  const ReferenceNodeState *best_state = nullptr;
  for (const auto &state : states) {
    if (!std::isfinite(state.time_s)) {
      continue;
    }
    if (state.time_s <= window.start_time_s + kTimeEpsilonS) {
      continue;
    }
    if (state.time_s >= post_first_time_s - kTimeEpsilonS) {
      continue;
    }
    best_state = &state;
  }
  return best_state;
}

void RefreshReferenceValidity(RtkOutageBoundaryReferenceRow &reference) {
  reference.valid =
    reference.has_up || reference.has_vz || reference.has_ba_z ||
    reference.has_horizontal_position || reference.has_horizontal_velocity ||
    reference.has_horizontal_velocity_delta ||
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

RtkOutageBoundaryBiasHandoffResult MakeSkippedResult(
  const RtkOutageBoundaryBiasHandoffRequest &request,
  const std::string &skip_reason) {
  RtkOutageBoundaryBiasHandoffResult result;
  result.skip_reason = skip_reason;
  result.boundary_reference.source_type = kPostStartBazHandoffSource;
  result.boundary_reference.boundary_role = "POST_START";
  result.boundary_reference.target_time_s = request.post_first_time_s;
  if (request.outage_window != nullptr) {
    result.boundary_reference.window_index = request.outage_window->window_index;
  }
  result.boundary_reference.skip_reason = skip_reason;
  return result;
}

}  // namespace

double ResolveRtkOutageBoundaryBazHandoffSigmaMps2(
  const OfflineRunnerConfig &config,
  const double fallback_sigma_mps2) {
  double sigma_mps2 = fallback_sigma_mps2;
  if (config.enable_vertical_acc_bias_gm_process &&
      std::isfinite(config.vertical_acc_bias_sigma_mps2) &&
      config.vertical_acc_bias_sigma_mps2 > 0.0) {
    sigma_mps2 = std::min(
      sigma_mps2,
      config.vertical_acc_bias_sigma_mps2);
  }
  return sigma_mps2;
}

RtkOutageBoundaryBiasHandoffResult BuildRtkOutageBoundaryBiasHandoff(
  const RtkOutageBoundaryBiasHandoffRequest &request) {
  if (request.config == nullptr || request.outage_result == nullptr ||
      request.outage_window == nullptr) {
    return MakeSkippedResult(request, "incomplete_handoff_request");
  }
  if (!std::isfinite(request.post_first_time_s)) {
    return MakeSkippedResult(request, "invalid_post_first_time");
  }
  if (!std::isfinite(request.config->rtk_outage_boundary_baz_sigma_mps2) ||
      request.config->rtk_outage_boundary_baz_sigma_mps2 <= 0.0) {
    return MakeSkippedResult(request, "invalid_baz_handoff_sigma");
  }
  if (request.outage_result->optimized_reference_states.empty()) {
    return MakeSkippedResult(request, "missing_outage_reference_states");
  }

  const ReferenceNodeState *outage_last_state = FindLastKeptOutageState(
    request.outage_result->optimized_reference_states,
    *request.outage_window,
    request.post_first_time_s);
  if (outage_last_state == nullptr) {
    return MakeSkippedResult(request, "missing_kept_outage_last_state");
  }
  if (request.post_first_time_s <= outage_last_state->time_s + kTimeEpsilonS) {
    return MakeSkippedResult(request, "nonpositive_handoff_interval");
  }

  const double reference_ba_z_mps2 =
    outage_last_state->bias.accelerometer().z();
  if (!std::isfinite(reference_ba_z_mps2)) {
    return MakeSkippedResult(request, "nonfinite_outage_last_baz");
  }

  RtkOutageBoundaryBiasHandoffResult result;
  result.boundary_reference.window_index = request.outage_window->window_index;
  result.boundary_reference.boundary_role = "POST_START";
  result.boundary_reference.source_type = kPostStartBazHandoffSource;
  result.boundary_reference.target_time_s = request.post_first_time_s;
  result.boundary_reference.has_ba_z = true;
  result.boundary_reference.add_ba_z_constraint = true;
  result.boundary_reference.reference_ba_z_mps2 = reference_ba_z_mps2;
  result.boundary_reference.ba_z_sigma_mps2 =
    ResolveRtkOutageBoundaryBazHandoffSigmaMps2(
      *request.config,
      request.config->rtk_outage_boundary_baz_sigma_mps2);
  result.boundary_reference.valid = true;
  result.boundary_reference.skip_reason = "OK";
  result.valid = true;
  result.skip_reason = "OK";
  return result;
}

void AttachRtkOutageBoundaryBiasHandoff(
  const RtkOutageBoundaryBiasHandoffResult &handoff,
  RtkOutageBoundaryReferenceRow &reference) {
  if (!handoff.valid) {
    return;
  }
  if (reference.source_type == "UNSET" ||
      reference.source_type == "OUTAGE_ATTITUDE_ONLY") {
    reference.source_type = kPostStartBazHandoffSource;
  }
  reference.reference_ba_z_mps2 =
    handoff.boundary_reference.reference_ba_z_mps2;
  reference.has_ba_z = true;
  reference.add_ba_z_constraint = true;
  reference.ba_z_sigma_mps2 =
    handoff.boundary_reference.ba_z_sigma_mps2;
  RefreshReferenceValidity(reference);
}

}  // namespace offline_lc_minimal
