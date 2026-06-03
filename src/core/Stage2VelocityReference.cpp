#include "offline_lc_minimal/core/Stage2VelocityReference.h"

#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>

#include "offline_lc_minimal/core/RtkHeadingAlignmentEstimator.h"
#include "offline_lc_minimal/core/StageAttitudeReference.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimestampToleranceS = 1.0e-6;

gtsam::Pose3 MergedReferencePose(
  const ReferenceNodeState &reference_state,
  const gtsam::Pose3 &existing_pose,
  const Stage2ReferenceApplicationOptions &options) {
  const gtsam::Pose3 &reference_pose = reference_state.pose;
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
  const ReferenceNodeState &reference_state,
  const gtsam::Vector3 &existing_velocity,
  const Stage2ReferenceApplicationOptions &options) {
  gtsam::Vector3 reference_velocity = reference_state.velocity;
  if (!options.apply_vertical_velocity) {
    reference_velocity.z() = existing_velocity.z();
  }
  return reference_velocity;
}

gtsam::imuBias::ConstantBias MergedReferenceBias(
  const ReferenceNodeState &reference_state,
  const gtsam::imuBias::ConstantBias &existing_bias,
  const Stage2ReferenceApplicationOptions &options) {
  gtsam::Vector3 reference_accel_bias = reference_state.bias.accelerometer();
  if (!options.apply_accel_z_bias) {
    reference_accel_bias.z() = existing_bias.accelerometer().z();
  }
  return gtsam::imuBias::ConstantBias(
    reference_accel_bias,
    reference_state.bias.gyroscope());
}

void ValidateReferenceSizeAndTimes(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<double> &state_timestamps) {
  if (reference_states.size() != state_timestamps.size()) {
      std::ostringstream oss;
      oss.precision(17);
      oss << "stage2 reference trajectory size does not match graph timeline"
        << ": reference_size=" << reference_states.size()
        << ", state_count=" << state_timestamps.size();
    if (!reference_states.empty()) {
      oss << ", reference_start_s=" << reference_states.front().time_s
          << ", reference_end_s=" << reference_states.back().time_s;
    }
    if (!state_timestamps.empty()) {
      oss << ", state_start_s=" << state_timestamps.front()
          << ", state_end_s=" << state_timestamps.back();
    }
    throw std::runtime_error(oss.str());
  }
  for (std::size_t index = 0; index < reference_states.size(); ++index) {
    if (!std::isfinite(reference_states[index].time_s) ||
        std::abs(reference_states[index].time_s - state_timestamps[index]) >
          kTimestampToleranceS) {
      std::ostringstream oss;
      oss.precision(17);
      oss << "stage2 reference trajectory timestamps do not match graph timeline"
          << ": index=" << index
          << ", reference_time_s=" << reference_states[index].time_s
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
  return BuildReferenceStatesFromTrajectoryRows(trajectory);
}

std::vector<ReferenceNodeState> BuildStage2ReferenceStates(
  const Stage2VelocityReference &reference) {
  if (!reference.reference_states.empty()) {
    return reference.reference_states;
  }
  return BuildStage2ReferenceStatesFromTrajectory(reference.trajectory);
}

void ApplyStage2ReferenceTrajectoryToInitialValues(
  const Stage2VelocityReference &reference,
  const std::vector<double> &state_timestamps,
  gtsam::Values &values,
  const Stage2ReferenceApplicationOptions &options) {
  const std::vector<ReferenceNodeState> reference_states =
    BuildStage2ReferenceStates(reference);
  ValidateReferenceSizeAndTimes(reference_states, state_timestamps);
  for (std::size_t state_index = 0; state_index < reference_states.size(); ++state_index) {
    const auto &reference_state = reference_states[state_index];
    values.update(
      X(state_index),
      MergedReferencePose(
        reference_state,
        values.at<gtsam::Pose3>(X(state_index)),
        options));
    values.update(
      V(state_index),
      MergedReferenceVelocity(
        reference_state,
        values.at<gtsam::Vector3>(V(state_index)),
        options));
    values.update(
      B(state_index),
      MergedReferenceBias(
        reference_state,
        values.at<gtsam::imuBias::ConstantBias>(B(state_index)),
        options));
    values.update(W(state_index), reference_state.omega);
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
