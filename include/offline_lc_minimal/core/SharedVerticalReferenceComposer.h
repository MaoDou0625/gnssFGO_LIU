#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/core/SharedVerticalReferenceBuilder.h"

namespace offline_lc_minimal {

struct SharedVerticalReferenceCompositionRequest {
  double grid_spacing_m = 1.0;
  double sigma_m = 0.015;
  std::vector<double> nav_up_by_bin;
  std::vector<std::size_t> nav_sample_count_by_bin;
};

[[nodiscard]] std::vector<SharedVerticalReferenceRow> ComposeNavOnlySharedVerticalReference(
  SharedVerticalReferenceCompositionRequest request);

}  // namespace offline_lc_minimal
