#pragma once

#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/SharedVerticalReferenceBuilder.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"
#include "offline_lc_minimal/io/TrajectoryCsvReader.h"

namespace offline_lc_minimal {

struct Stage3SharedReferenceMapRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<TrajectoryCsvRow> *stage2_trajectory = nullptr;
  const std::vector<SharedVerticalReferenceRow> *shared_reference = nullptr;
  const std::vector<SharedReferenceLinePoint> *shared_reference_line = nullptr;
};

[[nodiscard]] Stage3VerticalReference BuildStage3ReferenceFromSharedVerticalReference(
  Stage3SharedReferenceMapRequest request);

}  // namespace offline_lc_minimal
