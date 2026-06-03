#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

using VerticalMotionStabilityProfile =
  std::vector<VerticalMotionAdaptiveReweightingDiagnosticRow>;

[[nodiscard]] const VerticalMotionAdaptiveReweightingDiagnosticRow *FindStabilityProfileEntry(
  const VerticalMotionStabilityProfile *profile,
  std::size_t state_index_i,
  std::size_t state_index_j);

[[nodiscard]] double MaxMotionScoreDelta(
  const VerticalMotionStabilityProfile &lhs,
  const VerticalMotionStabilityProfile &rhs);

}  // namespace offline_lc_minimal
