#pragma once

#include <string>
#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

[[nodiscard]] ReferenceNodeState ReferenceStateFromTrajectoryRow(
  const TrajectoryRow &row);

[[nodiscard]] TrajectoryRow TrajectoryRowFromReferenceState(
  const ReferenceNodeState &state);

[[nodiscard]] std::vector<ReferenceNodeState> BuildReferenceStatesFromTrajectoryRows(
  const std::vector<TrajectoryRow> &trajectory);

[[nodiscard]] std::vector<TrajectoryRow> BuildTrajectoryRowsFromReferenceStates(
  const std::vector<ReferenceNodeState> &reference_states);

[[nodiscard]] std::vector<ReferenceNodeState> SortedFiniteReferenceStates(
  std::vector<ReferenceNodeState> reference_states);

[[nodiscard]] bool HasReferenceStateCoverage(
  const std::vector<ReferenceNodeState> &reference_states,
  double time_s,
  double max_bridge_gap_s);

[[nodiscard]] ReferenceNodeState InterpolateStageReferenceState(
  const std::vector<ReferenceNodeState> &reference_states,
  double time_s);

[[nodiscard]] std::vector<ReferenceNodeState> BuildImuDeltaOutageAttitudeReference(
  const std::vector<ReferenceNodeState> &base_reference_states,
  const std::vector<ReferenceNodeState> &imu_propagated_reference_states,
  const std::vector<double> &state_timestamps,
  const std::vector<RtkOutageWindowRow> &outage_windows,
  double guard_duration_s);

[[nodiscard]] double MaxAdjacentRotationStepRad(
  const std::vector<ReferenceNodeState> &reference_states);

}  // namespace offline_lc_minimal
