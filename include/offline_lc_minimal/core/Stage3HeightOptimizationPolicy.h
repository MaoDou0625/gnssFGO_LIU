#pragma once

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

[[nodiscard]] OfflineRunnerConfig MakeStage3HeightReferenceSourceConfig(
  OfflineRunnerConfig config);

[[nodiscard]] OfflineRunnerConfig MakeStage3HeightOptimizationConfig(
  OfflineRunnerConfig config);

}  // namespace offline_lc_minimal
