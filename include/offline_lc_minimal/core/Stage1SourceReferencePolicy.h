#pragma once

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

struct Stage1SourceReferencePolicyRequest {
  const OfflineRunnerConfig *requested_config = nullptr;
  const OfflineRunnerConfig *source_config = nullptr;
};

void ApplyStage1SourceReferencePolicy(
  const Stage1SourceReferencePolicyRequest &request,
  OfflineRunResult &source_result);

}  // namespace offline_lc_minimal
