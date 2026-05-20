#include "offline_lc_minimal/core/Stage2VelocityReference.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "offline_lc_minimal/core/RtkHeadingAlignmentEstimator.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimestampToleranceS = 1.0e-6;

gtsam::Pose3 PoseFromTrajectoryRow(const TrajectoryRow &row) {
  return gtsam::Pose3(
    gtsam::Rot3::Ypr(row.ypr_rad.x(), row.ypr_rad.y(), row.ypr_rad.z()),
    gtsam::Point3(
      row.enu_position_m.x(),
      row.enu_position_m.y(),
      row.enu_position_m.z()));
}

gtsam::Vector3 VectorFromEigen(const Eigen::Vector3d &vector) {
  return gtsam::Vector3(vector.x(), vector.y(), vector.z());
}

gtsam::Pose3 MergedReferencePose(
  const TrajectoryRow &row,
  const gtsam::Pose3 &existing_pose,
  const Stage2ReferenceApplicationOptions &options) {
  const gtsam::Pose3 reference_pose = PoseFromTrajectoryRow(row);
  return gtsam::Pose3(
    reference_pose.rotation(),
    gtsam::Point3(
      reference_pose.translation().x(),
      reference_pose.translation().y(),
      options.apply_vertical_position
        ? reference_pose.translation().z()
        : existing_pose.translation().z()));
}

gtsam::Vector3 MergedReferenceVelocity(
  const TrajectoryRow &row,
  const gtsam::Vector3 &existing_velocity,
  const Stage2ReferenceApplicationOptions &options) {
  gtsam::Vector3 reference_velocity = VectorFromEigen(row.enu_velocity_mps);
  if (!options.apply_vertical_velocity) {
    reference_velocity.z() = existing_velocity.z();
  }
  return reference_velocity;
}

gtsam::imuBias::ConstantBias MergedReferenceBias(
  const TrajectoryRow &row,
  const gtsam::imuBias::ConstantBias &existing_bias,
  const Stage2ReferenceApplicationOptions &options) {
  gtsam::Vector3 reference_accel_bias = VectorFromEigen(row.bias_acc);
  if (!options.apply_accel_z_bias) {
    reference_accel_bias.z() = existing_bias.accelerometer().z();
  }
  return gtsam::imuBias::ConstantBias(
    reference_accel_bias,
    VectorFromEigen(row.bias_gyro));
}

void ValidateReferenceSizeAndTimes(
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &state_timestamps) {
  if (trajectory.size() != state_timestamps.size()) {
      std::ostringstream oss;
      oss.precision(17);
      oss << "stage2 reference trajectory size does not match graph timeline"
        << ": reference_size=" << trajectory.size()
        << ", state_count=" << state_timestamps.size();
    if (!trajectory.empty()) {
      oss << ", reference_start_s=" << trajectory.front().time_s
          << ", reference_end_s=" << trajectory.back().time_s;
    }
    if (!state_timestamps.empty()) {
      oss << ", state_start_s=" << state_timestamps.front()
          << ", state_end_s=" << state_timestamps.back();
    }
    throw std::runtime_error(oss.str());
  }
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    if (!std::isfinite(trajectory[index].time_s) ||
        std::abs(trajectory[index].time_s - state_timestamps[index]) >
          kTimestampToleranceS) {
      std::ostringstream oss;
      oss.precision(17);
      oss << "stage2 reference trajectory timestamps do not match graph timeline"
          << ": index=" << index
          << ", reference_time_s=" << trajectory[index].time_s
          << ", state_time_s=" << state_timestamps[index];
      throw std::runtime_error(oss.str());
    }
  }
}

}  // namespace

Stage2ReferenceApplicationOptions
Stage2AttitudeHorizontalReferenceApplicationOptions() {
  Stage2ReferenceApplicationOptions options;
  options.apply_vertical_position = false;
  options.apply_vertical_velocity = false;
  options.apply_accel_z_bias = false;
  return options;
}

std::vector<ReferenceNodeState> BuildStage2ReferenceStatesFromTrajectory(
  const std::vector<TrajectoryRow> &trajectory) {
  std::vector<ReferenceNodeState> states;
  states.reserve(trajectory.size());
  for (const auto &row : trajectory) {
    ReferenceNodeState state;
    state.time_s = row.time_s;
    state.pose = PoseFromTrajectoryRow(row);
    state.velocity = VectorFromEigen(row.enu_velocity_mps);
    state.bias = gtsam::imuBias::ConstantBias(
      VectorFromEigen(row.bias_acc),
      VectorFromEigen(row.bias_gyro));
    state.omega = VectorFromEigen(row.omega_radps);
    states.push_back(state);
  }
  return states;
}

void ApplyStage2ReferenceTrajectoryToInitialValues(
  const Stage2VelocityReference &reference,
  const std::vector<double> &state_timestamps,
  gtsam::Values &values,
  const Stage2ReferenceApplicationOptions &options) {
  ValidateReferenceSizeAndTimes(reference.trajectory, state_timestamps);
  for (std::size_t state_index = 0; state_index < reference.trajectory.size(); ++state_index) {
    const auto &row = reference.trajectory[state_index];
    values.update(
      X(state_index),
      MergedReferencePose(
        row,
        values.at<gtsam::Pose3>(X(state_index)),
        options));
    values.update(
      V(state_index),
      MergedReferenceVelocity(
        row,
        values.at<gtsam::Vector3>(V(state_index)),
        options));
    values.update(
      B(state_index),
      MergedReferenceBias(
        row,
        values.at<gtsam::imuBias::ConstantBias>(B(state_index)),
        options));
    values.update(W(state_index), VectorFromEigen(row.omega_radps));
  }
}

double ComputeMaxAbsYawDeltaRad(
  const std::vector<ReferenceNodeState> &reference_states,
  const gtsam::Values &optimized_values) {
  double max_abs_delta_rad = 0.0;
  bool has_delta = false;
  for (std::size_t state_index = 0; state_index < reference_states.size(); ++state_index) {
    const auto optimized_pose = optimized_values.at<gtsam::Pose3>(X(state_index));
    const double optimized_yaw_rad = optimized_pose.rotation().ypr().x();
    const double reference_yaw_rad = reference_states[state_index].pose.rotation().ypr().x();
    const double delta_rad =
      NormalizeHeadingAngleRad(optimized_yaw_rad - reference_yaw_rad);
    max_abs_delta_rad = std::max(max_abs_delta_rad, std::abs(delta_rad));
    has_delta = true;
  }
  return has_delta ? max_abs_delta_rad : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace offline_lc_minimal
