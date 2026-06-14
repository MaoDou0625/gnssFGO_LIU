#include "offline_lc_minimal/core/RtkOutageBoundaryInitialValueApplicator.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

std::size_t BoundaryStateIndex(
  const std::vector<double> &state_timestamps,
  const RtkOutageBoundaryReferenceRow &reference) {
  if (reference.target_state_index >= 0 &&
      static_cast<std::size_t>(reference.target_state_index) < state_timestamps.size()) {
    return static_cast<std::size_t>(reference.target_state_index);
  }
  if (reference.boundary_role == "OUTAGE_START") {
    const auto lower = std::lower_bound(
      state_timestamps.begin(),
      state_timestamps.end(),
      reference.target_time_s);
    return lower == state_timestamps.end()
      ? state_timestamps.size() - 1U
      : static_cast<std::size_t>(lower - state_timestamps.begin());
  }
  if (reference.boundary_role == "OUTAGE_END" ||
      reference.boundary_role == "POST_START") {
    const auto upper = std::upper_bound(
      state_timestamps.begin(),
      state_timestamps.end(),
      reference.target_time_s);
    return upper == state_timestamps.begin()
      ? 0U
      : static_cast<std::size_t>((upper - state_timestamps.begin()) - 1);
  }
  const auto lower = std::lower_bound(
    state_timestamps.begin(),
    state_timestamps.end(),
    reference.target_time_s);
  if (lower == state_timestamps.begin()) {
    return 0U;
  }
  if (lower == state_timestamps.end()) {
    return state_timestamps.size() - 1U;
  }
  const std::size_t upper_index =
    static_cast<std::size_t>(lower - state_timestamps.begin());
  const std::size_t lower_index = upper_index - 1U;
  return std::abs(state_timestamps[lower_index] - reference.target_time_s) <=
             std::abs(state_timestamps[upper_index] - reference.target_time_s)
    ? lower_index
    : upper_index;
}

bool HasPositionReference(const RtkOutageBoundaryReferenceRow &reference) {
  return (reference.has_horizontal_position &&
          reference.reference_horizontal_position_m.allFinite()) ||
         (reference.has_up && std::isfinite(reference.reference_up_m));
}

bool HasVelocityReference(const RtkOutageBoundaryReferenceRow &reference) {
  return (reference.has_horizontal_velocity &&
          reference.reference_horizontal_velocity_mps.allFinite()) ||
         (reference.has_vz && std::isfinite(reference.reference_vz_mps));
}

gtsam::Point3 BoundaryPosition(
  const RtkOutageBoundaryReferenceRow &reference,
  const gtsam::Point3 &fallback) {
  gtsam::Point3 position = fallback;
  if (reference.has_horizontal_position &&
      reference.reference_horizontal_position_m.allFinite()) {
    position = gtsam::Point3(
      reference.reference_horizontal_position_m.x(),
      reference.reference_horizontal_position_m.y(),
      position.z());
  }
  if (reference.has_up && std::isfinite(reference.reference_up_m)) {
    position = gtsam::Point3(
      position.x(),
      position.y(),
      reference.reference_up_m);
  }
  return position;
}

gtsam::Vector3 BoundaryVelocity(
  const RtkOutageBoundaryReferenceRow &reference,
  gtsam::Vector3 velocity) {
  if (reference.has_horizontal_velocity &&
      reference.reference_horizontal_velocity_mps.allFinite()) {
    velocity.x() = reference.reference_horizontal_velocity_mps.x();
    velocity.y() = reference.reference_horizontal_velocity_mps.y();
  }
  if (reference.has_vz && std::isfinite(reference.reference_vz_mps)) {
    velocity.z() = reference.reference_vz_mps;
  }
  return velocity;
}

}  // namespace

void ApplyRtkOutageBoundaryPositionVelocityInitialValues(
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  const std::vector<double> &state_timestamps,
  gtsam::Values &values) {
  if (boundary_references.empty()) {
    return;
  }
  if (state_timestamps.empty()) {
    throw std::runtime_error("RTK outage boundary initial values require graph timestamps");
  }

  for (const auto &reference : boundary_references) {
    if (!reference.valid ||
        (reference.target_state_index < 0 && !std::isfinite(reference.target_time_s)) ||
        (!HasPositionReference(reference) && !HasVelocityReference(reference))) {
      continue;
    }
    const std::size_t state_index = BoundaryStateIndex(state_timestamps, reference);
    const gtsam::Key pose_key = symbol::X(state_index);
    if (HasPositionReference(reference) && values.exists(pose_key)) {
      const auto pose = values.at<gtsam::Pose3>(pose_key);
      values.update(
        pose_key,
        gtsam::Pose3(
          pose.rotation(),
          BoundaryPosition(reference, pose.translation())));
    }
    const gtsam::Key velocity_key = symbol::V(state_index);
    if (HasVelocityReference(reference) && values.exists(velocity_key)) {
      values.update(
        velocity_key,
        BoundaryVelocity(
          reference,
          values.at<gtsam::Vector3>(velocity_key)));
    }
  }
}

}  // namespace offline_lc_minimal
