#pragma once

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

[[nodiscard]] OfflineRunnerConfig MakeStage1YawRefinementConfig(
  const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
