#pragma once

#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

struct RtkOutageBoundaryAttitudeReference {
  std::vector<ReferenceNodeState> reference_states;
  std::vector<RtkOutageWindowRow> outage_windows;
  std::string source = "boundary_anchor_imu_delta";

  [[nodiscard]] bool valid() const {
    return !reference_states.empty() && !outage_windows.empty();
  }
};

[[nodiscard]] RtkOutageBoundaryAttitudeReference
BuildRtkOutageBoundaryAttitudeReference(
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &imu_reference_states,
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references);

void ApplyRtkOutageBoundaryAttitudeInitialValues(
  const RtkOutageBoundaryAttitudeReference &reference,
  const std::vector<double> &state_timestamps,
  double guard_duration_s,
  gtsam::Values &values);

void ApplyRtkOutageBoundaryStateInitialValues(
  const RtkOutageBoundaryAttitudeReference &reference,
  const std::vector<RtkOutageBoundaryReferenceRow> &boundary_references,
  const std::vector<double> &state_timestamps,
  double guard_duration_s,
  gtsam::Values &values);

}  // namespace offline_lc_minimal
