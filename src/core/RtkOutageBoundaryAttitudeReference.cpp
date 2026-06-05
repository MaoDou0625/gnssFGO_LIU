#include "offline_lc_minimal/core/RtkOutageBoundaryAttitudeReference.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

struct BoundaryPair {
  const RtkOutageBoundaryReferenceRow *start = nullptr;
  const RtkOutageBoundaryReferenceRow *end = nullptr;
};

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

std::size_t FirstStateIndexAtOrAfter(
  const std::vector<double> &state_timestamps,
  const double target_time_s) {
  const auto lower =
    std::lower_bound(state_timestamps.begin(), state_timestamps.end(), target_time_s);
  if (lower == state_timestamps.end()) {
    return state_timestamps.size() - 1U;
  }
  return static_cast<std::size_t>(lower - state_timestamps.begin());
}

std::size_t LastStateIndexAtOrBefore(
  const std::vector<double> &state_timestamps,
  const double target_time_s) {
  const auto upper =
    std::upper_bound(state_timestamps.begin(), state_timestamps.end(), target_time_s);
  if (upper == state_timestamps.begin()) {
    return 0U;
  }
  return static_cast<std::size_t>((upper - state_timestamps.begin()) - 1);
}

std::map<std::size_t, BoundaryPair> CollectBoundaryPairs(
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  const bool require_attitude) {
  std::map<std::size_t, BoundaryPair> pairs;
  for (const auto &reference : boundary_references) {
    if (!reference.valid) {
      continue;
    }
    if (require_attitude &&
        (!reference.has_attitude || !reference.reference_rotation.matrix().allFinite())) {
      continue;
    }
    auto &pair = pairs[reference.window_index];
    if (reference.boundary_role == "OUTAGE_START") {
      pair.start = &reference;
    } else if (reference.boundary_role == "OUTAGE_END") {
      pair.end = &reference;
    } else if (reference.boundary_role == "POST_START" && pair.start == nullptr) {
      pair.start = &reference;
    }
  }
  return pairs;
}

double InterpolationAlpha(
  const double time_s,
  const double start_time_s,
  const double end_time_s) {
  if (!std::isfinite(time_s) || !std::isfinite(start_time_s) ||
      !std::isfinite(end_time_s) || end_time_s <= start_time_s + kTimeEpsilonS) {
    return 0.0;
  }
  return std::clamp((time_s - start_time_s) / (end_time_s - start_time_s), 0.0, 1.0);
}

void FillBoundaryAnchoredRotations(
  std::vector<ReferenceNodeState> &reference_states,
  const std::vector<double> &state_timestamps,
  const std::size_t fill_start_index,
  const std::size_t fill_end_index,
  const std::size_t start_index,
  const std::size_t end_index,
  const RtkOutageBoundaryReferenceRow &start_reference,
  const RtkOutageBoundaryReferenceRow *end_reference) {
  const gtsam::Rot3 base_start_rotation =
    reference_states[start_index].pose.rotation();
  const gtsam::Rot3 start_rotation = start_reference.reference_rotation;
  const double start_time_s = state_timestamps[start_index];
  const double end_time_s = state_timestamps[end_index];
  gtsam::Vector3 end_correction_rotvec = gtsam::Vector3::Zero();
  if (end_reference != nullptr && end_reference->has_attitude &&
      end_reference->reference_rotation.matrix().allFinite() &&
      end_index >= start_index) {
    const gtsam::Rot3 base_end_rotation =
      reference_states[end_index].pose.rotation();
    const gtsam::Rot3 propagated_end_rotation =
      start_rotation.compose(base_start_rotation.between(base_end_rotation));
    end_correction_rotvec =
      gtsam::Rot3::Logmap(propagated_end_rotation.between(end_reference->reference_rotation));
  }

  for (std::size_t state_index = fill_start_index; state_index <= fill_end_index; ++state_index) {
    const gtsam::Rot3 base_delta =
      base_start_rotation.between(reference_states[state_index].pose.rotation());
    const double alpha = end_reference != nullptr
      ? InterpolationAlpha(
          state_timestamps[state_index],
          start_time_s,
          end_time_s)
      : 0.0;
    const gtsam::Rot3 smooth_end_correction =
      gtsam::Rot3::Expmap(alpha * end_correction_rotvec);
    const gtsam::Rot3 anchored_rotation =
      start_rotation.compose(base_delta).compose(smooth_end_correction);
    const gtsam::Point3 translation = reference_states[state_index].pose.translation();
    reference_states[state_index].pose = gtsam::Pose3(anchored_rotation, translation);
  }
}

RtkOutageWindowRow MakeSyntheticWindow(
  const std::size_t window_index,
  const std::size_t start_index,
  const std::size_t end_index,
  const double start_time_s,
  const double end_time_s) {
  RtkOutageWindowRow window;
  window.window_index = window_index;
  window.pre_anchor_state_index = start_index;
  window.post_anchor_state_index = end_index;
  window.start_time_s = start_time_s;
  window.end_time_s = end_time_s;
  window.duration_s = end_time_s - start_time_s;
  window.skip_reason = "PLANNED";
  return window;
}

std::pair<std::size_t, std::size_t> GuardedSpan(
  const RtkOutageWindowRow &window,
  const std::vector<double> &state_timestamps,
  const double guard_duration_s) {
  const std::size_t guarded_start = FirstStateIndexAtOrAfter(
    state_timestamps,
    window.start_time_s - guard_duration_s);
  const std::size_t guarded_end = LastStateIndexAtOrBefore(
    state_timestamps,
    window.end_time_s + guard_duration_s);
  return {
    std::min(window.pre_anchor_state_index, guarded_start),
    std::max(window.post_anchor_state_index, guarded_end)};
}

gtsam::Point3 BoundaryPosition(
  const RtkOutageBoundaryReferenceRow &reference,
  const gtsam::Point3 &fallback) {
  double east_m = fallback.x();
  double north_m = fallback.y();
  double up_m = fallback.z();
  if (reference.has_horizontal_position &&
      reference.reference_horizontal_position_m.allFinite()) {
    east_m = reference.reference_horizontal_position_m.x();
    north_m = reference.reference_horizontal_position_m.y();
  }
  if (reference.has_up && std::isfinite(reference.reference_up_m)) {
    up_m = reference.reference_up_m;
  }
  return gtsam::Point3(east_m, north_m, up_m);
}

gtsam::Vector3 BoundaryVelocity(
  const RtkOutageBoundaryReferenceRow &reference,
  const gtsam::Vector3 &fallback) {
  gtsam::Vector3 velocity = fallback;
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

}  // namespace

RtkOutageBoundaryAttitudeReference BuildRtkOutageBoundaryAttitudeReference(
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &imu_reference_states,
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  const double guard_duration_s) {
  if (state_timestamps.empty() || imu_reference_states.empty() ||
      boundary_references.empty()) {
    return {};
  }
  if (state_timestamps.size() != imu_reference_states.size()) {
    throw std::runtime_error(
      "boundary attitude reference requires one IMU reference state per graph state");
  }

  RtkOutageBoundaryAttitudeReference result;
  result.reference_states = imu_reference_states;
  const std::map<std::size_t, BoundaryPair> pairs =
    CollectBoundaryPairs(boundary_references, false);
  for (const auto &[window_index, pair] : pairs) {
    if (pair.start == nullptr ||
        !pair.start->has_attitude ||
        !pair.start->reference_rotation.matrix().allFinite()) {
      continue;
    }
    const std::size_t start_index =
      BoundaryStateIndex(state_timestamps, *pair.start);
    const bool has_end =
      pair.end != nullptr &&
      std::isfinite(pair.end->target_time_s) &&
      pair.end->target_time_s > pair.start->target_time_s + kTimeEpsilonS;
    const std::size_t end_index = has_end
      ? BoundaryStateIndex(state_timestamps, *pair.end)
      : start_index;
    if (end_index < start_index) {
      continue;
    }
    const bool has_end_attitude =
      has_end &&
      pair.end->has_attitude &&
      pair.end->reference_rotation.matrix().allFinite();
    const RtkOutageWindowRow window = MakeSyntheticWindow(
      window_index,
      start_index,
      end_index,
      pair.start->target_time_s,
      has_end ? pair.end->target_time_s : pair.start->target_time_s);
    const auto [fill_start_index, fill_end_index] =
      GuardedSpan(window, state_timestamps, guard_duration_s);

    FillBoundaryAnchoredRotations(
      result.reference_states,
      state_timestamps,
      fill_start_index,
      fill_end_index,
      start_index,
      end_index,
      *pair.start,
      has_end_attitude ? pair.end : nullptr);
    result.outage_windows.push_back(window);
  }

  if (result.outage_windows.empty()) {
    result.reference_states.clear();
  }
  return result;
}

void ApplyRtkOutageBoundaryAttitudeInitialValues(
  const RtkOutageBoundaryAttitudeReference &reference,
  const std::vector<double> &state_timestamps,
  const double guard_duration_s,
  gtsam::Values &values) {
  if (!reference.valid()) {
    return;
  }
  if (reference.reference_states.size() != state_timestamps.size()) {
    throw std::runtime_error(
      "boundary attitude initial value update requires one reference state per graph state");
  }
  for (const auto &window : reference.outage_windows) {
    if (window.post_anchor_state_index >= state_timestamps.size() ||
        window.pre_anchor_state_index > window.post_anchor_state_index) {
      continue;
    }
    const auto [start_index, end_index] =
      GuardedSpan(window, state_timestamps, guard_duration_s);
    for (std::size_t state_index = start_index; state_index <= end_index; ++state_index) {
      const gtsam::Key key = symbol::X(state_index);
      if (!values.exists(key)) {
        continue;
      }
      const auto pose = values.at<gtsam::Pose3>(key);
      values.update(
        key,
        gtsam::Pose3(
          reference.reference_states[state_index].pose.rotation(),
          pose.translation()));
    }
  }
}

void ApplyRtkOutageBoundaryStateInitialValues(
  const RtkOutageBoundaryAttitudeReference &reference,
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  const std::vector<double> &state_timestamps,
  const double guard_duration_s,
  gtsam::Values &values) {
  ApplyRtkOutageBoundaryAttitudeInitialValues(
    reference,
    state_timestamps,
    guard_duration_s,
    values);
  if (!reference.valid() || boundary_references.empty()) {
    return;
  }
  const std::map<std::size_t, BoundaryPair> pairs =
    CollectBoundaryPairs(boundary_references, false);
  for (const auto &window : reference.outage_windows) {
    const auto pair_it = pairs.find(window.window_index);
    if (pair_it == pairs.end() || pair_it->second.start == nullptr ||
        window.post_anchor_state_index >= state_timestamps.size() ||
        window.pre_anchor_state_index > window.post_anchor_state_index) {
      continue;
    }
    const auto &pair = pair_it->second;
    const std::size_t start_index = window.pre_anchor_state_index;
    const std::size_t end_index = window.post_anchor_state_index;
    const gtsam::Key start_pose_key = symbol::X(start_index);
    const gtsam::Key end_pose_key = symbol::X(end_index);
    if (!values.exists(start_pose_key) || !values.exists(end_pose_key)) {
      continue;
    }

    const auto start_pose = values.at<gtsam::Pose3>(start_pose_key);
    const auto end_pose = values.at<gtsam::Pose3>(end_pose_key);
    const bool has_start_position = HasPositionReference(*pair.start);
    const bool has_end_position =
      pair.end != nullptr && HasPositionReference(*pair.end);
    const bool has_start_velocity = HasVelocityReference(*pair.start);
    const bool has_end_velocity =
      pair.end != nullptr && HasVelocityReference(*pair.end);
    const gtsam::Point3 start_position =
      has_start_position ? BoundaryPosition(*pair.start, start_pose.translation())
                         : start_pose.translation();
    const gtsam::Point3 end_position =
      has_end_position ? BoundaryPosition(*pair.end, end_pose.translation())
                       : end_pose.translation();

    gtsam::Vector3 start_velocity = gtsam::Vector3::Zero();
    gtsam::Vector3 end_velocity = gtsam::Vector3::Zero();
    if (values.exists(symbol::V(start_index))) {
      start_velocity = values.at<gtsam::Vector3>(symbol::V(start_index));
    }
    if (values.exists(symbol::V(end_index))) {
      end_velocity = values.at<gtsam::Vector3>(symbol::V(end_index));
    }
    if (has_start_velocity) {
      start_velocity = BoundaryVelocity(*pair.start, start_velocity);
    }
    if (has_end_velocity) {
      end_velocity = BoundaryVelocity(*pair.end, end_velocity);
    }

    for (std::size_t state_index = start_index; state_index <= end_index; ++state_index) {
      const double alpha = InterpolationAlpha(
        state_timestamps[state_index],
        window.start_time_s,
        window.end_time_s);
      const gtsam::Key pose_key = symbol::X(state_index);
      if (values.exists(pose_key) && (has_start_position || has_end_position)) {
        const auto pose = values.at<gtsam::Pose3>(pose_key);
        const gtsam::Point3 interpolated_position =
          start_position + alpha * (end_position - start_position);
        values.update(
          pose_key,
          gtsam::Pose3(pose.rotation(), interpolated_position));
      }
      const gtsam::Key velocity_key = symbol::V(state_index);
      if (values.exists(velocity_key) && (has_start_velocity || has_end_velocity)) {
        const gtsam::Vector3 interpolated_velocity =
          start_velocity + alpha * (end_velocity - start_velocity);
        values.update(velocity_key, interpolated_velocity);
      }
    }
  }
}

}  // namespace offline_lc_minimal
