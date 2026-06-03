#pragma once

#include <string>

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

enum class RtkOutageSegmentationContext {
  kStage1Source,
  kSegmentChild,
  kStage3Final,
  kPassthrough
};

[[nodiscard]] const char *ToString(RtkOutageSegmentationContext context);

[[nodiscard]] OfflineRunnerConfig DisableRtkOutageSegmentedBatchRecursion(
  OfflineRunnerConfig config);

[[nodiscard]] std::string ExplainRtkOutageSegmentedBatchInactive(
  const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
