#include "offline_lc_minimal/core/TrajectoryResultBuilder.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimeEpsilonS = 1e-9;

gtsam::Rot3 InterpolateRotation(const gtsam::Rot3 &left, const gtsam::Rot3 &right, const double alpha) {
  const gtsam::Vector3 delta = gtsam::Rot3::Logmap(left.between(right));
  return left.compose(gtsam::Rot3::Expmap(alpha * delta));
}

TrajectoryRow MakeTrajectoryRow(
  const double time_s,
  const gtsam::Pose3 &pose,
  const gtsam::Vector3 &velocity,
  const gtsam::imuBias::ConstantBias &bias,
  const gtsam::Vector3 &omega) {
  TrajectoryRow row;
  row.time_s = time_s;
  row.enu_position_m = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
  row.enu_velocity_mps = Eigen::Vector3d(velocity.x(), velocity.y(), velocity.z());
  row.ypr_rad = Rot3ToYpr(pose.rotation());
  row.omega_radps = Eigen::Vector3d(omega.x(), omega.y(), omega.z());
  row.bias_acc = bias.accelerometer();
  row.bias_gyro = bias.gyroscope();
  return row;
}

}  // namespace

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

ReferenceNodeState MakeReferenceNodeState(
  const double time_s,
  const gtsam::NavState &nav_state,
  const gtsam::imuBias::ConstantBias &bias,
  const gtsam::Vector3 &omega) {
  ReferenceNodeState state;
  state.time_s = time_s;
  state.pose = nav_state.pose();
  state.velocity = nav_state.v();
  state.bias = bias;
  state.omega = omega;
  return state;
}

ReferenceNodeRow MakeReferenceNodeRow(const ReferenceNodeState &state) {
  ReferenceNodeRow row;
  row.time_s = state.time_s;
  row.enu_position_m =
    Eigen::Vector3d(state.pose.translation().x(), state.pose.translation().y(), state.pose.translation().z());
  row.enu_velocity_mps = Eigen::Vector3d(state.velocity.x(), state.velocity.y(), state.velocity.z());
  row.ypr_rad = Rot3ToYpr(state.pose.rotation());
  row.bias_acc = state.bias.accelerometer();
  row.bias_gyro = state.bias.gyroscope();
  return row;
}

std::vector<ReferenceNodeState> BuildReferenceStatesFromOptimizedValues(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values) {
  std::vector<ReferenceNodeState> states;
  states.reserve(state_timestamps.size());
  for (std::size_t state_index = 0; state_index < state_timestamps.size(); ++state_index) {
    states.push_back(MakeReferenceNodeState(
      state_timestamps[state_index],
      gtsam::NavState(
        optimized_values.at<gtsam::Pose3>(X(state_index)),
        optimized_values.at<gtsam::Vector3>(V(state_index))),
      optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index)),
      optimized_values.at<gtsam::Vector3>(W(state_index))));
  }
  return states;
}

ReferenceNodeState InterpolateReferenceState(
  const std::vector<ReferenceNodeState> &reference_states,
  const double time_s) {
  if (reference_states.empty()) {
    throw std::runtime_error("reference node state sequence is empty");
  }
  if (reference_states.size() == 1U || time_s <= reference_states.front().time_s + kTimeEpsilonS) {
    return reference_states.front();
  }
  if (time_s >= reference_states.back().time_s - kTimeEpsilonS) {
    return reference_states.back();
  }

  const auto upper_it = std::upper_bound(
    reference_states.begin(),
    reference_states.end(),
    time_s,
    [](const double timestamp_s, const ReferenceNodeState &state) { return timestamp_s < state.time_s; });
  const std::size_t right_index = static_cast<std::size_t>(std::distance(reference_states.begin(), upper_it));
  const std::size_t left_index = right_index - 1U;
  const auto &left_state = reference_states[left_index];
  const auto &right_state = reference_states[right_index];
  const double alpha =
    std::clamp((time_s - left_state.time_s) / (right_state.time_s - left_state.time_s), 0.0, 1.0);
  const double dt_s = std::max(right_state.time_s - left_state.time_s, kTimeEpsilonS);

  const double alpha2 = alpha * alpha;
  const double alpha3 = alpha2 * alpha;
  const double h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
  const double h10 = alpha3 - 2.0 * alpha2 + alpha;
  const double h01 = -2.0 * alpha3 + 3.0 * alpha2;
  const double h11 = alpha3 - alpha2;
  const double interpolated_up_m =
    h00 * left_state.pose.translation().z() +
    h10 * dt_s * left_state.velocity.z() +
    h01 * right_state.pose.translation().z() +
    h11 * dt_s * right_state.velocity.z();
  Eigen::Vector3d interpolated_velocity =
    (1.0 - alpha) * left_state.velocity + alpha * right_state.velocity;
  interpolated_velocity.z() =
    ((6.0 * alpha2 - 6.0 * alpha) * left_state.pose.translation().z() +
     (-6.0 * alpha2 + 6.0 * alpha) * right_state.pose.translation().z()) /
      dt_s +
    (3.0 * alpha2 - 4.0 * alpha + 1.0) * left_state.velocity.z() +
    (3.0 * alpha2 - 2.0 * alpha) * right_state.velocity.z();

  ReferenceNodeState interpolated;
  interpolated.time_s = time_s;
  interpolated.pose = gtsam::Pose3(
    InterpolateRotation(left_state.pose.rotation(), right_state.pose.rotation(), alpha),
    gtsam::Point3(
      (1.0 - alpha) * left_state.pose.translation().x() + alpha * right_state.pose.translation().x(),
      (1.0 - alpha) * left_state.pose.translation().y() + alpha * right_state.pose.translation().y(),
      interpolated_up_m));
  interpolated.velocity = interpolated_velocity;
  interpolated.bias = gtsam::imuBias::ConstantBias(
    (1.0 - alpha) * left_state.bias.accelerometer() + alpha * right_state.bias.accelerometer(),
    (1.0 - alpha) * left_state.bias.gyroscope() + alpha * right_state.bias.gyroscope());
  interpolated.omega = (1.0 - alpha) * left_state.omega + alpha * right_state.omega;
  return interpolated;
}

std::vector<TrajectoryRow> BuildForwardTrajectoryRows(
  const std::vector<ImuSample> &imu_samples,
  const gtsam::Pose3 &start_pose,
  const gtsam::Vector3 &start_velocity,
  const gtsam::imuBias::ConstantBias &bias,
  const double start_time_s,
  const double duration_s,
  const double gravity_mps2) {
  std::vector<TrajectoryRow> rows;
  if (imu_samples.size() < 2U || duration_s <= 0.0) {
    return rows;
  }

  const double end_time_s = start_time_s + duration_s;
  gtsam::Rot3 current_rotation = start_pose.rotation();
  Eigen::Vector3d current_position(
    start_pose.translation().x(),
    start_pose.translation().y(),
    start_pose.translation().z());
  Eigen::Vector3d current_velocity(start_velocity.x(), start_velocity.y(), start_velocity.z());
  const Eigen::Vector3d bias_acc = bias.accelerometer();
  const Eigen::Vector3d bias_gyro = bias.gyroscope();
  const Eigen::Vector3d gravity_enu(0.0, 0.0, gravity_mps2);

  auto append_row = [&](const double time_s, const Eigen::Vector3d &omega_radps) {
    TrajectoryRow row;
    row.time_s = time_s;
    row.enu_position_m = current_position;
    row.enu_velocity_mps = current_velocity;
    row.ypr_rad = Rot3ToYpr(current_rotation);
    row.omega_radps = omega_radps;
    row.bias_acc = bias_acc;
    row.bias_gyro = bias_gyro;
    rows.push_back(row);
  };

  append_row(start_time_s, Eigen::Vector3d::Zero());
  for (std::size_t imu_index = 0; imu_index + 1U < imu_samples.size(); ++imu_index) {
    const auto &current_sample = imu_samples[imu_index];
    const auto &next_sample = imu_samples[imu_index + 1U];
    const double interval_start_s = std::max(current_sample.time_s, start_time_s);
    const double interval_end_s = std::min(next_sample.time_s, end_time_s);
    if (interval_end_s <= interval_start_s + kTimeEpsilonS) {
      continue;
    }

    const double delta_time_s = interval_end_s - interval_start_s;
    const Eigen::Vector3d corrected_gyro = current_sample.gyro_radps - bias_gyro;
    const Eigen::Vector3d corrected_acc = current_sample.accel_mps2 - bias_acc;
    const Eigen::Vector3d nav_specific_force = current_rotation.matrix() * corrected_acc;
    const Eigen::Vector3d nav_acc = nav_specific_force - gravity_enu;

    current_position += current_velocity * delta_time_s + 0.5 * nav_acc * delta_time_s * delta_time_s;
    current_velocity += nav_acc * delta_time_s;
    current_rotation =
      current_rotation.compose(gtsam::Rot3::Expmap(gtsam::Vector3(
        corrected_gyro.x() * delta_time_s,
        corrected_gyro.y() * delta_time_s,
        corrected_gyro.z() * delta_time_s)));

    append_row(interval_end_s, corrected_gyro);
    if (interval_end_s >= end_time_s - kTimeEpsilonS) {
      break;
    }
  }

  return rows;
}

void UpdateTrajectoryRowsFromOptimizedValues(
  const gtsam::Values &optimized_values,
  std::vector<TrajectoryRow> &trajectory_rows) {
  for (std::size_t index = 0; index < trajectory_rows.size(); ++index) {
    auto &row = trajectory_rows[index];
    const auto pose = optimized_values.at<gtsam::Pose3>(X(index));
    const auto velocity = optimized_values.at<gtsam::Vector3>(V(index));
    const auto bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(index));
    const auto omega = optimized_values.at<gtsam::Vector3>(W(index));

    row.enu_position_m = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
    row.enu_velocity_mps = Eigen::Vector3d(velocity.x(), velocity.y(), velocity.z());
    row.ypr_rad = Rot3ToYpr(pose.rotation());
    row.omega_radps = Eigen::Vector3d(omega.x(), omega.y(), omega.z());
    row.bias_acc = bias.accelerometer();
    row.bias_gyro = bias.gyroscope();
  }
}

std::vector<TrajectoryRow> BuildInitialStaticTrajectoryRows(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values,
  const std::size_t initial_static_state_count) {
  std::vector<TrajectoryRow> rows;
  rows.reserve(initial_static_state_count);
  for (std::size_t graph_index = 0; graph_index < initial_static_state_count; ++graph_index) {
    rows.push_back(MakeTrajectoryRow(
      state_timestamps[graph_index],
      optimized_values.at<gtsam::Pose3>(X(graph_index)),
      optimized_values.at<gtsam::Vector3>(V(graph_index)),
      optimized_values.at<gtsam::imuBias::ConstantBias>(B(graph_index)),
      optimized_values.at<gtsam::Vector3>(W(graph_index))));
  }
  return rows;
}

}  // namespace offline_lc_minimal
