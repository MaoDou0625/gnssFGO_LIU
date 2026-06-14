#include "offline_lc_minimal/core/RtkOutageTerminalVelocityReference.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include <gtsam/navigation/NavState.h>

#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryVerticalHandoff.h"
#include "offline_lc_minimal/core/StageAttitudeReference.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

std::optional<double> LastKeptOutageStateTime(
  const std::vector<double> &state_timestamps,
  const RtkOutageWindowRow &outage,
  const double post_first_time_s) {
  if (!std::isfinite(outage.start_time_s) ||
      !std::isfinite(post_first_time_s) ||
      post_first_time_s <= outage.start_time_s + kTimeEpsilonS) {
    return std::nullopt;
  }

  std::optional<double> best_time_s;
  for (const double time_s : state_timestamps) {
    if (!std::isfinite(time_s)) {
      continue;
    }
    if (time_s <= outage.start_time_s + kTimeEpsilonS) {
      continue;
    }
    if (time_s >= post_first_time_s - kTimeEpsilonS) {
      break;
    }
    best_time_s = time_s;
  }
  return best_time_s;
}

bool IsFiniteVector3(const gtsam::Vector3 &vector) {
  return std::isfinite(vector.x()) &&
         std::isfinite(vector.y()) &&
         std::isfinite(vector.z());
}

double ClampVerticalDeltaVelocity(
  const OfflineRunnerConfig &config,
  const double delta_vz_mps,
  const double dt_s) {
  if (dt_s <= 0.0 || !std::isfinite(dt_s) || !std::isfinite(delta_vz_mps)) {
    return delta_vz_mps;
  }
  const double limit_mps =
    config.vertical_velocity_delta_target_acc_limit_mps2 * dt_s;
  return std::clamp(delta_vz_mps, -limit_mps, limit_mps);
}

double TerminalVerticalVelocitySigmaMps(const OfflineRunnerConfig &config) {
  double sigma_mps = config.rtk_outage_velocity_delta_3d_sigma_mps;
  if (std::isfinite(config.vertical_velocity_delta_min_sigma_mps) &&
      config.vertical_velocity_delta_min_sigma_mps > 0.0) {
    sigma_mps = std::min(sigma_mps, config.vertical_velocity_delta_min_sigma_mps);
  }
  if (std::isfinite(config.vertical_velocity_delta_sigma_ceiling_mps) &&
      config.vertical_velocity_delta_sigma_ceiling_mps > 0.0) {
    sigma_mps = std::min(sigma_mps, config.vertical_velocity_delta_sigma_ceiling_mps);
  }
  return sigma_mps;
}

std::optional<gtsam::Vector3> ImuVelocityDelta(
  const RtkOutageTerminalVelocityReferenceRequest &request,
  const ReferenceNodeState &last_state,
  const double post_first_time_s) {
  if (request.imu_samples == nullptr || request.imu_params == nullptr ||
      request.imu_samples->empty()) {
    return std::nullopt;
  }
  try {
    const ImuWindowIntegration imu_window =
      IntegrateImuWindow(
        *request.imu_samples,
        last_state.time_s,
        post_first_time_s,
        request.imu_params,
        last_state.bias);
    const gtsam::NavState last_nav_state(last_state.pose, last_state.velocity);
    const gtsam::NavState predicted_state =
      imu_window.preintegrated_measurements.predict(
        last_nav_state,
        last_state.bias);
    const gtsam::Vector3 delta_v_mps =
      predicted_state.v() - last_state.velocity;
    return IsFiniteVector3(delta_v_mps)
      ? std::optional<gtsam::Vector3>(delta_v_mps)
      : std::nullopt;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

gtsam::Vector3 ReferenceVelocityDelta(
  const std::vector<ReferenceNodeState> &reference_states,
  const ReferenceNodeState &last_state,
  const double post_first_time_s) {
  const ReferenceNodeState post_state =
    InterpolateStageReferenceState(reference_states, post_first_time_s);
  return post_state.velocity - last_state.velocity;
}

}  // namespace

std::optional<RtkOutageBoundaryReferenceRow>
BuildRtkOutageTerminalVelocityReference(
  const RtkOutageTerminalVelocityReferenceRequest &request) {
  if (request.config == nullptr ||
      request.state_timestamps == nullptr ||
      request.reference_states == nullptr ||
      request.outage == nullptr ||
      request.post_boundary_reference == nullptr) {
    throw std::runtime_error(
      "RtkOutageTerminalVelocityReference received an incomplete request");
  }
  const RtkOutageBoundaryReferenceRow &post_reference =
    *request.post_boundary_reference;
  if (!post_reference.valid || !std::isfinite(post_reference.target_time_s)) {
    return std::nullopt;
  }
  if (!post_reference.has_horizontal_velocity && !post_reference.has_vz) {
    return std::nullopt;
  }

  std::vector<ReferenceNodeState> sorted_reference_states =
    SortedFiniteReferenceStates(*request.reference_states);
  if (sorted_reference_states.empty()) {
    return std::nullopt;
  }

  const std::optional<double> last_kept_time_s =
    LastKeptOutageStateTime(
      *request.state_timestamps,
      *request.outage,
      post_reference.target_time_s);
  if (!last_kept_time_s.has_value()) {
    return std::nullopt;
  }

  const ReferenceNodeState last_state =
    InterpolateStageReferenceState(sorted_reference_states, *last_kept_time_s);
  const double dt_s = post_reference.target_time_s - last_state.time_s;
  if (dt_s <= kTimeEpsilonS || !std::isfinite(dt_s)) {
    return std::nullopt;
  }

  const std::optional<gtsam::Vector3> imu_delta =
    ImuVelocityDelta(request, last_state, post_reference.target_time_s);
  gtsam::Vector3 delta_v_mps = imu_delta.has_value()
    ? *imu_delta
    : ReferenceVelocityDelta(
        sorted_reference_states,
        last_state,
        post_reference.target_time_s);
  delta_v_mps.z() =
    ClampVerticalDeltaVelocity(*request.config, delta_v_mps.z(), dt_s);
  if (!IsFiniteVector3(delta_v_mps)) {
    return std::nullopt;
  }

  RtkOutageBoundaryReferenceRow reference;
  reference.window_index = request.outage->window_index;
  reference.boundary_role = "OUTAGE_END_TERMINAL_VELOCITY";
  reference.source_type = imu_delta.has_value()
    ? "POST_FIRST_MINUS_IMU_DELTA"
    : "POST_FIRST_MINUS_STAGE_REFERENCE_DELTA";
  reference.target_time_s = last_state.time_s;
  reference.horizontal_velocity_sigma_mps =
    request.config->rtk_outage_velocity_delta_3d_sigma_mps;
  reference.vz_sigma_mps = TerminalVerticalVelocitySigmaMps(*request.config);
  reference.skip_reason = "OK";

  if (post_reference.has_horizontal_velocity &&
      post_reference.reference_horizontal_velocity_mps.allFinite()) {
    reference.reference_horizontal_velocity_mps =
      post_reference.reference_horizontal_velocity_mps -
      Eigen::Vector2d(delta_v_mps.x(), delta_v_mps.y());
    reference.has_horizontal_velocity =
      reference.reference_horizontal_velocity_mps.allFinite() &&
      std::isfinite(reference.horizontal_velocity_sigma_mps) &&
      reference.horizontal_velocity_sigma_mps > 0.0;
    reference.add_horizontal_velocity_constraint =
      reference.has_horizontal_velocity;
  }

  if (post_reference.has_vz && std::isfinite(post_reference.reference_vz_mps)) {
    reference.reference_vz_mps = post_reference.reference_vz_mps - delta_v_mps.z();
    reference.has_vz =
      std::isfinite(reference.reference_vz_mps) &&
      std::isfinite(reference.vz_sigma_mps) &&
      reference.vz_sigma_mps > 0.0;
    reference.add_vz_constraint = reference.has_vz;
  }

  reference.valid =
    reference.has_horizontal_velocity || reference.has_vz;
  if (!reference.valid) {
    return std::nullopt;
  }
  return reference;
}

std::optional<RtkOutageBoundaryReferenceRow>
BuildRtkOutageTerminalVerticalHandoffReference(
  const RtkOutageTerminalVelocityReferenceRequest &request) {
  if (request.config == nullptr ||
      request.state_timestamps == nullptr ||
      request.outage == nullptr ||
      request.post_boundary_reference == nullptr) {
    throw std::runtime_error(
      "RtkOutageTerminalVerticalHandoffReference received an incomplete request");
  }
  const RtkOutageBoundaryReferenceRow &post_reference =
    *request.post_boundary_reference;
  if (!post_reference.valid ||
      !post_reference.has_up ||
      !post_reference.has_vz ||
      !std::isfinite(post_reference.target_time_s) ||
      !std::isfinite(post_reference.reference_up_m) ||
      !std::isfinite(post_reference.reference_vz_mps)) {
    return std::nullopt;
  }

  const std::optional<double> last_kept_time_s =
    LastKeptOutageStateTime(
      *request.state_timestamps,
      *request.outage,
      post_reference.target_time_s);
  if (!last_kept_time_s.has_value()) {
    return std::nullopt;
  }

  RtkOutageBoundaryReferenceRow reference;
  reference.window_index = request.outage->window_index;
  reference.boundary_role = "OUTAGE_END_TERMINAL_VERTICAL_HANDOFF";
  reference.source_type = "POST_FIRST_VERTICAL_POSITION_VELOCITY_HANDOFF";
  reference.reference_up_m = post_reference.reference_up_m;
  reference.reference_vz_mps = post_reference.reference_vz_mps;
  ConfigureVerticalPositionVelocityHandoff(
    reference,
    *request.config,
    *last_kept_time_s,
    post_reference.target_time_s,
    false,
    "missing_terminal_vertical_handoff_reference");
  if (!reference.valid ||
      !reference.has_vertical_position_velocity_handoff ||
      !reference.add_vertical_position_velocity_handoff_constraint) {
    return std::nullopt;
  }
  return reference;
}

std::optional<RtkOutageBoundaryReferenceRow>
BuildRtkOutageTerminalHorizontalHandoffReference(
  const RtkOutageTerminalVelocityReferenceRequest &request) {
  if (request.config == nullptr ||
      request.state_timestamps == nullptr ||
      request.outage == nullptr ||
      request.post_boundary_reference == nullptr) {
    throw std::runtime_error(
      "RtkOutageTerminalHorizontalHandoffReference received an incomplete request");
  }
  const RtkOutageBoundaryReferenceRow &post_reference =
    *request.post_boundary_reference;
  if (!post_reference.valid ||
      !post_reference.has_horizontal_position ||
      !post_reference.has_horizontal_velocity ||
      !std::isfinite(post_reference.target_time_s) ||
      !post_reference.reference_horizontal_position_m.allFinite() ||
      !post_reference.reference_horizontal_velocity_mps.allFinite()) {
    return std::nullopt;
  }

  const std::optional<double> last_kept_time_s =
    LastKeptOutageStateTime(
      *request.state_timestamps,
      *request.outage,
      post_reference.target_time_s);
  if (!last_kept_time_s.has_value()) {
    return std::nullopt;
  }
  const double dt_s = post_reference.target_time_s - *last_kept_time_s;
  if (dt_s <= kTimeEpsilonS || !std::isfinite(dt_s)) {
    return std::nullopt;
  }

  RtkOutageBoundaryReferenceRow reference;
  reference.window_index = request.outage->window_index;
  reference.boundary_role = "OUTAGE_END_HORIZONTAL_HANDOFF";
  reference.source_type = "POST_FIRST_HORIZONTAL_POSITION_VELOCITY_HANDOFF";
  reference.target_time_s = *last_kept_time_s;
  reference.reference_horizontal_position_m =
    post_reference.reference_horizontal_position_m;
  reference.reference_horizontal_velocity_mps =
    post_reference.reference_horizontal_velocity_mps;
  reference.has_horizontal_position = true;
  reference.has_horizontal_velocity = true;
  reference.has_horizontal_position_velocity_handoff = true;
  reference.add_horizontal_position_constraint = false;
  reference.add_horizontal_velocity_constraint = false;
  reference.add_horizontal_position_velocity_handoff_constraint = true;
  reference.horizontal_position_velocity_handoff_reference_time_s =
    post_reference.target_time_s;
  reference.horizontal_position_velocity_handoff_sigma_m =
    request.config->stage2_horizontal_position_hold_sigma_m;
  reference.valid =
    std::isfinite(reference.horizontal_position_velocity_handoff_sigma_m) &&
    reference.horizontal_position_velocity_handoff_sigma_m > 0.0;
  reference.skip_reason =
    reference.valid ? "OK" : "missing_terminal_horizontal_handoff_reference";
  if (!reference.valid) {
    return std::nullopt;
  }
  return reference;
}

}  // namespace offline_lc_minimal
