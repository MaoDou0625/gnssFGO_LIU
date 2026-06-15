#pragma once

#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

void ApplyRtkOutageBoundaryPositionVelocityInitialValues(
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  const std::vector<double> &state_timestamps,
  gtsam::Values &values);

void ApplyRtkOutageBoundaryPriorTarget(
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  double initial_time_s,
  gtsam::Pose3 &initial_pose_world,
  gtsam::Vector3 &initial_velocity,
  gtsam::Vector6 &pose_sigmas,
  gtsam::Vector3 &velocity_sigmas);

}  // namespace offline_lc_minimal
