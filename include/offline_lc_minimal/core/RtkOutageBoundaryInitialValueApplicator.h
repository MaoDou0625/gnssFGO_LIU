#pragma once

#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

void ApplyRtkOutageBoundaryPositionVelocityInitialValues(
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  const std::vector<double> &state_timestamps,
  gtsam::Values &values);

}  // namespace offline_lc_minimal
