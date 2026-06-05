#include "offline_lc_minimal/core/RtkOutageBoundaryAttitudeHandoff.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/factor/AttitudeReferenceFactor.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

const ReferenceNodeState *FindLastKeptOutageState(
  const std::vector<ReferenceNodeState> &states,
  const RtkOutageWindowRow &window,
  const double post_first_time_s,
  std::size_t &state_index) {
  const ReferenceNodeState *best_state = nullptr;
  state_index = 0U;
  for (std::size_t index = 0; index < states.size(); ++index) {
    const ReferenceNodeState &state = states[index];
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
    state_index = index;
  }
  return best_state;
}

const ReferenceNodeState *NearestReferenceState(
  const std::vector<ReferenceNodeState> &states,
  const double time_s) {
  if (states.empty() || !std::isfinite(time_s)) {
    return nullptr;
  }
  const auto upper = std::lower_bound(
    states.begin(),
    states.end(),
    time_s,
    [](const ReferenceNodeState &state, const double target_time_s) {
      return state.time_s < target_time_s;
    });
  if (upper == states.begin()) {
    return &states.front();
  }
  if (upper == states.end()) {
    return &states.back();
  }
  const auto &right = *upper;
  const auto &left = *std::prev(upper);
  return std::abs(left.time_s - time_s) <= std::abs(right.time_s - time_s)
    ? &left
    : &right;
}

RtkOutageAttitudeHoldDiagnosticRow MakeDiagnostic(
  const RtkOutageWindowRow &window,
  const std::size_t outage_last_state_index,
  const ReferenceNodeState &outage_last_state,
  const double post_first_time_s,
  const gtsam::Rot3 &post_reference_rotation,
  const double sigma_rad) {
  RtkOutageAttitudeHoldDiagnosticRow row;
  row.window_index = window.window_index;
  row.constraint_type = "post_start_imu_handoff";
  row.state_index_i = outage_last_state_index;
  row.state_index_j = 0U;
  row.time_i_s = outage_last_state.time_s;
  row.time_j_s = post_first_time_s;
  row.factor_added = true;
  row.skip_reason = "ADDED";
  row.reference_source = kPostStartImuRelativeHandoffSource;
  row.sigma_rad = sigma_rad;
  row.reference_rotation_i = outage_last_state.pose.rotation();
  row.reference_rotation_j = post_reference_rotation;
  row.reference_ypr_i_rad = Rot3ToYpr(row.reference_rotation_i);
  row.reference_ypr_j_rad = Rot3ToYpr(row.reference_rotation_j);
  row.optimized_ypr_i_rad = row.reference_ypr_i_rad;
  row.reference_relative_rotvec_rad =
    gtsam::Rot3::Logmap(row.reference_rotation_i.between(row.reference_rotation_j));
  row.reference_relative_angle_rad = row.reference_relative_rotvec_rad.norm();
  row.reference_delta_yaw_rad =
    factor::RelativeYawRad(row.reference_rotation_i, row.reference_rotation_j);
  return row;
}

RtkOutageAttitudeHoldDiagnosticRow MakeSkippedDiagnostic(
  const RtkOutageBoundaryAttitudeHandoffRequest &request,
  const std::string &skip_reason) {
  RtkOutageAttitudeHoldDiagnosticRow row;
  row.constraint_type = "post_start_imu_handoff";
  row.factor_added = false;
  row.skip_reason = skip_reason;
  row.reference_source = kPostStartImuRelativeHandoffSource;
  if (request.outage_window != nullptr) {
    row.window_index = request.outage_window->window_index;
    row.time_i_s = request.outage_window->end_time_s;
  }
  row.time_j_s = request.post_first_time_s;
  if (request.config != nullptr) {
    row.sigma_rad = request.config->rtk_outage_absolute_attitude_sigma_rad;
  }
  return row;
}

RtkOutageBoundaryAttitudeHandoffResult MakeSkippedResult(
  const RtkOutageBoundaryAttitudeHandoffRequest &request,
  const std::string &skip_reason) {
  RtkOutageBoundaryAttitudeHandoffResult result;
  result.skip_reason = skip_reason;
  result.diagnostic = MakeSkippedDiagnostic(request, skip_reason);
  return result;
}

void PopulateRelativeDiagnostic(
  const gtsam::Rot3 &optimized_rotation_j,
  RtkOutageAttitudeHoldDiagnosticRow &row) {
  row.optimized_ypr_j_rad = Rot3ToYpr(optimized_rotation_j);
  const gtsam::Rot3 reference_delta_rotation =
    row.reference_rotation_i.between(row.reference_rotation_j);
  const gtsam::Rot3 optimized_delta_rotation =
    row.reference_rotation_i.between(optimized_rotation_j);
  row.optimized_relative_rotvec_rad = gtsam::Rot3::Logmap(optimized_delta_rotation);
  row.optimized_relative_angle_rad = row.optimized_relative_rotvec_rad.norm();
  row.residual_rad =
    gtsam::Rot3::Logmap(reference_delta_rotation.between(optimized_delta_rotation));
  row.residual_norm_rad = row.residual_rad.norm();
  row.optimized_delta_yaw_rad =
    factor::RelativeYawRad(row.reference_rotation_i, optimized_rotation_j);
  row.residual_yaw_rad =
    factor::NormalizeAngleRad(row.optimized_delta_yaw_rad - row.reference_delta_yaw_rad);
}

}  // namespace

RtkOutageBoundaryAttitudeHandoffResult BuildRtkOutageBoundaryAttitudeHandoff(
  const RtkOutageBoundaryAttitudeHandoffRequest &request) {
  RtkOutageBoundaryAttitudeHandoffResult result;
  if (request.config == nullptr || request.dataset == nullptr ||
      request.outage_result == nullptr || request.outage_window == nullptr ||
      request.imu_params == nullptr) {
    return MakeSkippedResult(request, "incomplete_handoff_request");
  }
  if (!std::isfinite(request.post_first_time_s)) {
    return MakeSkippedResult(request, "invalid_post_first_time");
  }
  if (request.outage_result->optimized_reference_states.empty()) {
    return MakeSkippedResult(request, "missing_outage_reference_states");
  }

  std::size_t outage_last_state_index = 0U;
  const ReferenceNodeState *outage_last_state = FindLastKeptOutageState(
    request.outage_result->optimized_reference_states,
    *request.outage_window,
    request.post_first_time_s,
    outage_last_state_index);
  if (outage_last_state == nullptr) {
    return MakeSkippedResult(request, "missing_kept_outage_last_state");
  }
  if (request.post_first_time_s <= outage_last_state->time_s + kTimeEpsilonS) {
    return MakeSkippedResult(request, "nonpositive_handoff_interval");
  }

  try {
    const ImuWindowIntegration imu_window = IntegrateImuWindow(
      request.dataset->imu_samples,
      outage_last_state->time_s,
      request.post_first_time_s,
      request.imu_params,
      outage_last_state->bias);
    const gtsam::NavState outage_last_nav_state(
      outage_last_state->pose,
      outage_last_state->velocity);
    const gtsam::NavState predicted_state =
      imu_window.preintegrated_measurements.predict(
        outage_last_nav_state,
        outage_last_state->bias);

    result.boundary_reference.window_index = request.outage_window->window_index;
    result.boundary_reference.boundary_role = "POST_START";
    result.boundary_reference.source_type = kPostStartImuRelativeHandoffSource;
    result.boundary_reference.target_time_s = request.post_first_time_s;
    result.boundary_reference.has_attitude = true;
    result.boundary_reference.add_attitude_constraint = true;
    result.boundary_reference.reference_rotation =
      predicted_state.pose().rotation();
    result.boundary_reference.attitude_sigma_rad =
      request.config->rtk_outage_absolute_attitude_sigma_rad;
    result.boundary_reference.valid = true;
    result.boundary_reference.skip_reason = "OK";
    result.diagnostic = MakeDiagnostic(
      *request.outage_window,
      outage_last_state_index,
      *outage_last_state,
      request.post_first_time_s,
      predicted_state.pose().rotation(),
      request.config->rtk_outage_absolute_attitude_sigma_rad);
    result.valid = true;
    result.skip_reason = "OK";
  } catch (const std::exception &exception) {
    const std::string skip_reason =
      std::string("imu_handoff_failed: ") + exception.what();
    result.skip_reason = skip_reason;
    result.diagnostic = MakeSkippedDiagnostic(request, skip_reason);
  }
  return result;
}

void AttachRtkOutageBoundaryAttitudeHandoff(
  const RtkOutageBoundaryAttitudeHandoffResult &handoff,
  RtkOutageBoundaryReferenceRow &reference) {
  if (!handoff.valid) {
    return;
  }
  reference.source_type = kPostStartImuRelativeHandoffSource;
  reference.reference_rotation = handoff.boundary_reference.reference_rotation;
  reference.has_attitude = true;
  reference.add_attitude_constraint = true;
  reference.attitude_sigma_rad = handoff.boundary_reference.attitude_sigma_rad;
  reference.valid = true;
  reference.skip_reason = "OK";
}

void PopulateRtkOutageBoundaryAttitudeHandoffDiagnostic(
  const OfflineRunResult &post_result,
  RtkOutageAttitudeHoldDiagnosticRow &diagnostic) {
  if (!diagnostic.factor_added ||
      diagnostic.constraint_type != "post_start_imu_handoff") {
    return;
  }
  const ReferenceNodeState *post_state =
    NearestReferenceState(post_result.optimized_reference_states, diagnostic.time_j_s);
  if (post_state == nullptr || !post_state->pose.rotation().matrix().allFinite()) {
    diagnostic.skip_reason = "missing_post_first_state";
    return;
  }
  PopulateRelativeDiagnostic(post_state->pose.rotation(), diagnostic);
}

}  // namespace offline_lc_minimal
