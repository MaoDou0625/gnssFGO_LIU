#include "offline_lc_minimal/core/StageAttitudeReference.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>

#include "offline_lc_minimal/core/TrajectoryResultBuilder.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

[[nodiscard]] gtsam::Vector3 VectorFromEigen(const Eigen::Vector3d &vector) {
  return gtsam::Vector3(vector.x(), vector.y(), vector.z());
}

[[nodiscard]] Eigen::Vector3d PointToEigen(const gtsam::Point3 &point) {
  return Eigen::Vector3d(point.x(), point.y(), point.z());
}

[[nodiscard]] Eigen::Vector3d VectorToEigen(const gtsam::Vector3 &vector) {
  return Eigen::Vector3d(vector.x(), vector.y(), vector.z());
}

[[nodiscard]] std::size_t LowerBracketIndex(
  const std::vector<ReferenceNodeState> &reference_states,
  const double time_s) {
  if (reference_states.size() < 2U || time_s <= reference_states.front().time_s) {
    return 0U;
  }
  if (time_s >= reference_states.back().time_s) {
    return reference_states.size() - 2U;
  }
  const auto upper =
    std::upper_bound(
      reference_states.begin(),
      reference_states.end(),
      time_s,
      [](const double value, const ReferenceNodeState &state) {
        return value < state.time_s;
      });
  return static_cast<std::size_t>(
    std::distance(reference_states.begin(), std::prev(upper)));
}

[[nodiscard]] double InterpolationAlpha(
  const double time_s,
  const double lhs_time_s,
  const double rhs_time_s) {
  const double dt_s = rhs_time_s - lhs_time_s;
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return 0.0;
  }
  return std::clamp((time_s - lhs_time_s) / dt_s, 0.0, 1.0);
}

[[nodiscard]] gtsam::Rot3 InterpolateRotation(
  const gtsam::Rot3 &left,
  const gtsam::Rot3 &right,
  const double alpha) {
  const gtsam::Vector3 delta = gtsam::Rot3::Logmap(left.between(right));
  return left.compose(gtsam::Rot3::Expmap(alpha * delta));
}

[[nodiscard]] std::size_t FirstStateIndexAtOrAfter(
  const std::vector<double> &timestamps,
  const double time_s) {
  const auto it = std::lower_bound(timestamps.begin(), timestamps.end(), time_s);
  if (it == timestamps.end()) {
    return timestamps.size() - 1U;
  }
  return static_cast<std::size_t>(std::distance(timestamps.begin(), it));
}

[[nodiscard]] std::size_t LastStateIndexAtOrBefore(
  const std::vector<double> &timestamps,
  const double time_s) {
  const auto it = std::upper_bound(timestamps.begin(), timestamps.end(), time_s);
  if (it == timestamps.begin()) {
    return 0U;
  }
  return static_cast<std::size_t>(std::distance(timestamps.begin(), it) - 1);
}

[[nodiscard]] bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED" || window.skip_reason == "ADDED";
}

}  // namespace

ReferenceNodeState ReferenceStateFromTrajectoryRow(const TrajectoryRow &row) {
  ReferenceNodeState state;
  state.time_s = row.time_s;
  state.pose = gtsam::Pose3(
    gtsam::Rot3::Ypr(row.ypr_rad.x(), row.ypr_rad.y(), row.ypr_rad.z()),
    gtsam::Point3(
      row.enu_position_m.x(),
      row.enu_position_m.y(),
      row.enu_position_m.z()));
  state.velocity = VectorFromEigen(row.enu_velocity_mps);
  state.bias = gtsam::imuBias::ConstantBias(
    VectorFromEigen(row.bias_acc),
    VectorFromEigen(row.bias_gyro));
  state.omega = VectorFromEigen(row.omega_radps);
  return state;
}

TrajectoryRow TrajectoryRowFromReferenceState(const ReferenceNodeState &state) {
  TrajectoryRow row;
  row.time_s = state.time_s;
  row.enu_position_m = PointToEigen(state.pose.translation());
  row.enu_velocity_mps = VectorToEigen(state.velocity);
  row.ypr_rad = Rot3ToYpr(state.pose.rotation());
  row.omega_radps = VectorToEigen(state.omega);
  row.bias_acc = VectorToEigen(state.bias.accelerometer());
  row.bias_gyro = VectorToEigen(state.bias.gyroscope());
  return row;
}

std::vector<ReferenceNodeState> BuildReferenceStatesFromTrajectoryRows(
  const std::vector<TrajectoryRow> &trajectory) {
  std::vector<ReferenceNodeState> states;
  states.reserve(trajectory.size());
  for (const auto &row : trajectory) {
    states.push_back(ReferenceStateFromTrajectoryRow(row));
  }
  return states;
}

std::vector<TrajectoryRow> BuildTrajectoryRowsFromReferenceStates(
  const std::vector<ReferenceNodeState> &reference_states) {
  std::vector<TrajectoryRow> trajectory;
  trajectory.reserve(reference_states.size());
  for (const auto &state : reference_states) {
    trajectory.push_back(TrajectoryRowFromReferenceState(state));
  }
  return trajectory;
}

std::vector<ReferenceNodeState> SortedFiniteReferenceStates(
  std::vector<ReferenceNodeState> reference_states) {
  reference_states.erase(
    std::remove_if(
      reference_states.begin(),
      reference_states.end(),
      [](const ReferenceNodeState &state) {
        return !std::isfinite(state.time_s);
      }),
    reference_states.end());
  std::stable_sort(
    reference_states.begin(),
    reference_states.end(),
    [](const ReferenceNodeState &lhs, const ReferenceNodeState &rhs) {
      return lhs.time_s < rhs.time_s;
    });
  reference_states.erase(
    std::unique(
      reference_states.begin(),
      reference_states.end(),
      [](const ReferenceNodeState &lhs, const ReferenceNodeState &rhs) {
        return std::abs(lhs.time_s - rhs.time_s) <= kTimeEpsilonS;
      }),
    reference_states.end());
  return reference_states;
}

bool HasReferenceStateCoverage(
  const std::vector<ReferenceNodeState> &reference_states,
  const double time_s,
  const double max_bridge_gap_s) {
  if (reference_states.empty()) {
    return false;
  }
  if (reference_states.size() == 1U) {
    return std::abs(time_s - reference_states.front().time_s) <= kTimeEpsilonS;
  }
  if (time_s < reference_states.front().time_s - kTimeEpsilonS ||
      time_s > reference_states.back().time_s + kTimeEpsilonS) {
    return false;
  }
  const std::size_t lower_index = LowerBracketIndex(reference_states, time_s);
  return reference_states[lower_index + 1U].time_s -
           reference_states[lower_index].time_s <=
         max_bridge_gap_s;
}

ReferenceNodeState InterpolateStageReferenceState(
  const std::vector<ReferenceNodeState> &reference_states,
  const double time_s) {
  if (reference_states.empty()) {
    throw std::runtime_error("stage attitude reference sequence is empty");
  }
  if (reference_states.size() == 1U ||
      time_s <= reference_states.front().time_s + kTimeEpsilonS) {
    ReferenceNodeState state = reference_states.front();
    state.time_s = time_s;
    return state;
  }
  if (time_s >= reference_states.back().time_s - kTimeEpsilonS) {
    ReferenceNodeState state = reference_states.back();
    state.time_s = time_s;
    return state;
  }

  const std::size_t lower_index = LowerBracketIndex(reference_states, time_s);
  const auto &left_state = reference_states[lower_index];
  const auto &right_state = reference_states[lower_index + 1U];
  const double alpha =
    InterpolationAlpha(time_s, left_state.time_s, right_state.time_s);

  ReferenceNodeState interpolated;
  interpolated.time_s = time_s;
  interpolated.pose = gtsam::Pose3(
    InterpolateRotation(
      left_state.pose.rotation(),
      right_state.pose.rotation(),
      alpha),
    gtsam::Point3(
      (1.0 - alpha) * left_state.pose.translation().x() +
        alpha * right_state.pose.translation().x(),
      (1.0 - alpha) * left_state.pose.translation().y() +
        alpha * right_state.pose.translation().y(),
      (1.0 - alpha) * left_state.pose.translation().z() +
        alpha * right_state.pose.translation().z()));
  interpolated.velocity =
    (1.0 - alpha) * left_state.velocity + alpha * right_state.velocity;
  interpolated.bias = gtsam::imuBias::ConstantBias(
    (1.0 - alpha) * left_state.bias.accelerometer() +
      alpha * right_state.bias.accelerometer(),
    (1.0 - alpha) * left_state.bias.gyroscope() +
      alpha * right_state.bias.gyroscope());
  interpolated.omega =
    (1.0 - alpha) * left_state.omega + alpha * right_state.omega;
  return interpolated;
}

std::vector<ReferenceNodeState> BuildImuDeltaOutageAttitudeReference(
  const std::vector<ReferenceNodeState> &base_reference_states,
  const std::vector<ReferenceNodeState> &imu_propagated_reference_states,
  const std::vector<double> &state_timestamps,
  const std::vector<RtkOutageWindowRow> &outage_windows,
  const double guard_duration_s) {
  if (base_reference_states.size() != state_timestamps.size() ||
      imu_propagated_reference_states.size() != state_timestamps.size()) {
    throw std::runtime_error(
      "outage attitude reference requires base and IMU reference states on the graph timeline");
  }

  std::vector<ReferenceNodeState> corrected = base_reference_states;
  for (const auto &window : outage_windows) {
    if (!IsPlannedOutage(window) ||
        window.pre_anchor_state_index >= state_timestamps.size() ||
        window.post_anchor_state_index >= state_timestamps.size() ||
        window.pre_anchor_state_index > window.post_anchor_state_index) {
      continue;
    }

    const std::size_t hold_start_index = std::min(
      window.pre_anchor_state_index,
      FirstStateIndexAtOrAfter(state_timestamps, window.start_time_s - guard_duration_s));
    const std::size_t hold_end_index = std::max(
      window.post_anchor_state_index,
      LastStateIndexAtOrBefore(state_timestamps, window.end_time_s + guard_duration_s));
    const std::size_t anchor_index = window.pre_anchor_state_index;
    const gtsam::Rot3 anchor_rotation =
      base_reference_states[anchor_index].pose.rotation();
    const gtsam::Rot3 imu_anchor_rotation =
      imu_propagated_reference_states[anchor_index].pose.rotation();

    for (std::size_t state_index = hold_start_index;
         state_index <= hold_end_index;
         ++state_index) {
      const gtsam::Rot3 imu_delta =
        imu_anchor_rotation.between(
          imu_propagated_reference_states[state_index].pose.rotation());
      corrected[state_index].pose = gtsam::Pose3(
        anchor_rotation.compose(imu_delta),
        corrected[state_index].pose.translation());
    }
  }
  return corrected;
}

double MaxAdjacentRotationStepRad(
  const std::vector<ReferenceNodeState> &reference_states) {
  double max_step_rad = 0.0;
  bool has_step = false;
  for (std::size_t index = 1; index < reference_states.size(); ++index) {
    const gtsam::Vector3 rotvec = gtsam::Rot3::Logmap(
      reference_states[index - 1U].pose.rotation().between(
        reference_states[index].pose.rotation()));
    max_step_rad = std::max(max_step_rad, rotvec.norm());
    has_step = true;
  }
  return has_step ? max_step_rad : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace offline_lc_minimal
