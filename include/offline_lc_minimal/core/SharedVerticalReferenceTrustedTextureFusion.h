#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "offline_lc_minimal/core/SharedVerticalReferenceBuilder.h"

namespace offline_lc_minimal {

struct SharedVerticalReferenceMemberHeightGrid {
  std::string member_id;
  std::vector<double> up_by_bin;
  std::vector<double> vz_mps_by_bin;
  std::vector<std::size_t> sample_count_by_bin;
};

struct SharedVerticalReferenceTrustedTextureFusionRequest {
  double grid_spacing_m = 1.0;
  double sigma_m = 0.015;
  double lowpass_radius_m = 20.0;
  double source_margin_min = 0.03;
  std::vector<SharedVerticalReferenceMemberHeightGrid> members;
};

[[nodiscard]] std::vector<SharedVerticalReferenceRow>
ComposeTrustedTextureSharedVerticalReference(
  SharedVerticalReferenceTrustedTextureFusionRequest request);

}  // namespace offline_lc_minimal
