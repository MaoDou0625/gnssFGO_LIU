#pragma once

#include <memory>
#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct Stage2VelocityReference {
  std::vector<TrajectoryRow> trajectory;
  std::shared_ptr<const OfflineRunnerConfig> source_config;
};

struct Stage2ReferenceApplicationOptions {
  bool apply_vertical_position = true;
  bool apply_vertical_velocity = true;
  bool apply_accel_z_bias = true;
};

[[nodiscard]] Stage2ReferenceApplicationOptions
Stage2AttitudeHorizontalReferenceApplicationOptions();

[[nodiscard]] std::vector<ReferenceNodeState> BuildStage2ReferenceStatesFromTrajectory(
  const std::vector<TrajectoryRow> &trajectory);

void ApplyStage2ReferenceTrajectoryToInitialValues(
  const Stage2VelocityReference &reference,
  const std::vector<double> &state_timestamps,
  gtsam::Values &values,
  const Stage2ReferenceApplicationOptions &options = {});

[[nodiscard]] double ComputeMaxAbsYawDeltaRad(
  const std::vector<ReferenceNodeState> &reference_states,
  const gtsam::Values &optimized_values);

}  // namespace offline_lc_minimal
