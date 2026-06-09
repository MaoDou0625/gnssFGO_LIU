#pragma once

#include <filesystem>
#include <vector>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

[[nodiscard]] std::vector<BodyZBiasReestimateSegmentRow>
ReadBodyZBiasReestimateSegmentCsv(const std::filesystem::path &path);

}  // namespace offline_lc_minimal
