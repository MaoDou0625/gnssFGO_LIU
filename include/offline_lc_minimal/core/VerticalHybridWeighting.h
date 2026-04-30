#pragma once

#include <cstddef>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"

namespace offline_lc_minimal {

struct VerticalHybridWeighting {
  double direct_vertical_sigma_m = 0.0;
  double inside_kinematic_sigma_scale = 1.0;
  bool inside_body_z_window = false;
};

[[nodiscard]] bool IsStateInsideBodyZJumpWindow(
  const std::vector<BodyZJumpWindowCandidate> &windows,
  std::size_t state_index);

[[nodiscard]] VerticalHybridWeighting ComputeVerticalHybridWeighting(
  const OfflineRunnerConfig &config,
  const std::vector<BodyZJumpWindowCandidate> &body_z_windows,
  std::size_t state_index,
  double base_vertical_sigma_m,
  bool initial_anchor,
  bool inside_gate);

[[nodiscard]] double ResolveVerticalInsideBazSigmaMps2(const OfflineRunnerConfig &config);

}  // namespace offline_lc_minimal
