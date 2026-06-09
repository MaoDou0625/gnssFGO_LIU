#pragma once

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

[[nodiscard]] OfflineRunnerConfig MakeStage1YawRefinementConfig(
  const OfflineRunnerConfig &config);

[[nodiscard]] OfflineRunnerConfig MakeStage2VelocityOptimizationConfig(
  const OfflineRunnerConfig &config);

[[nodiscard]] OfflineRunnerConfig DisableStage3VerticalReferenceOptimization(
  OfflineRunnerConfig config);

}  // namespace offline_lc_minimal
