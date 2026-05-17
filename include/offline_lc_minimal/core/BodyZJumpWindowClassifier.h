#pragma once

#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"

namespace offline_lc_minimal {

struct BodyZJumpWindowClassification {
  std::vector<BodyZJumpWindowCandidate> jump_windows;
  std::vector<BodyZJumpWindowCandidate> bias_windows;
};

[[nodiscard]] BodyZJumpWindowClassification ClassifyBodyZJumpWindowsForBias(
  const std::vector<BodyZJumpWindowCandidate> &merged_windows,
  const std::vector<BodyZJumpWindowCandidate> &transition_candidates,
  const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
