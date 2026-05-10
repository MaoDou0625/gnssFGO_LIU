#include "offline_lc_minimal/core/VerticalMotionStabilityProfile.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace offline_lc_minimal {

const VerticalMotionAdaptiveReweightingDiagnosticRow *FindStabilityProfileEntry(
  const VerticalMotionStabilityProfile *profile,
  const std::size_t state_index_i,
  const std::size_t state_index_j) {
  if (profile == nullptr) {
    return nullptr;
  }
  const auto it = std::find_if(
    profile->begin(),
    profile->end(),
    [&](const auto &entry) {
      return entry.state_index_i == state_index_i && entry.state_index_j == state_index_j;
    });
  return it == profile->end() ? nullptr : &(*it);
}

double MaxMotionScoreDelta(
  const VerticalMotionStabilityProfile &lhs,
  const VerticalMotionStabilityProfile &rhs) {
  if (lhs.size() != rhs.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double max_delta = 0.0;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    if (lhs[index].state_index_i != rhs[index].state_index_i ||
        lhs[index].state_index_j != rhs[index].state_index_j ||
        !std::isfinite(lhs[index].motion_score) ||
        !std::isfinite(rhs[index].motion_score)) {
      return std::numeric_limits<double>::infinity();
    }
    max_delta = std::max(max_delta, std::abs(lhs[index].motion_score - rhs[index].motion_score));
  }
  return max_delta;
}

}  // namespace offline_lc_minimal
