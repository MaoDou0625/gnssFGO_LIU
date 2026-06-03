#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

struct RtkVerticalLowpassReferenceFilterSummary {
  std::size_t valid_count = 0;
  double max_abs_delta_m = 0.0;
};

[[nodiscard]] RtkVerticalLowpassReferenceFilterSummary ApplyRtkVerticalLowpassReferenceFilter(
  const OfflineRunnerConfig &config,
  std::vector<RtkVerticalDriftReferenceDiagnosticRow> *profile);

}  // namespace offline_lc_minimal
