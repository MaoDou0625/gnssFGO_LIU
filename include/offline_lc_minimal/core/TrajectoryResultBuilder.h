#pragma once

#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

[[nodiscard]] Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation);

[[nodiscard]] ReferenceNodeState MakeReferenceNodeState(
  double time_s,
  const gtsam::NavState &nav_state,
  const gtsam::imuBias::ConstantBias &bias,
  const gtsam::Vector3 &omega);

[[nodiscard]] ReferenceNodeRow MakeReferenceNodeRow(const ReferenceNodeState &state);

[[nodiscard]] std::vector<ReferenceNodeState> BuildReferenceStatesFromOptimizedValues(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values);

[[nodiscard]] ReferenceNodeState InterpolateReferenceState(
  const std::vector<ReferenceNodeState> &reference_states,
  double time_s);

[[nodiscard]] std::vector<TrajectoryRow> BuildForwardTrajectoryRows(
  const std::vector<ImuSample> &imu_samples,
  const gtsam::Pose3 &start_pose,
  const gtsam::Vector3 &start_velocity,
  const gtsam::imuBias::ConstantBias &bias,
  double start_time_s,
  double duration_s,
  double gravity_mps2);

void UpdateTrajectoryRowsFromOptimizedValues(
  const gtsam::Values &optimized_values,
  std::vector<TrajectoryRow> &trajectory_rows);

[[nodiscard]] std::vector<TrajectoryRow> BuildInitialStaticTrajectoryRows(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values,
  std::size_t initial_static_state_count);

}  // namespace offline_lc_minimal
