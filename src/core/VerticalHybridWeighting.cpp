#include "offline_lc_minimal/core/VerticalHybridWeighting.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {

namespace {

constexpr double kMinimumSigma = 1e-12;

[[nodiscard]] double PositiveOrFloor(const double value) {
  return std::max(value, kMinimumSigma);
}

}  // namespace

bool IsStateInsideBodyZJumpWindow(
  const std::vector<BodyZJumpWindowCandidate> &windows,
  const std::size_t state_index) {
  return std::any_of(
    windows.begin(),
    windows.end(),
    [state_index](const BodyZJumpWindowCandidate &window) {
      return window.start_state_index <= state_index && state_index <= window.end_state_index;
    });
}

VerticalHybridWeighting ComputeVerticalHybridWeighting(
  const OfflineRunnerConfig &config,
  const std::vector<BodyZJumpWindowCandidate> &body_z_windows,
  const std::size_t state_index,
  const double base_vertical_sigma_m,
  const bool initial_anchor,
  const bool inside_gate) {
  VerticalHybridWeighting weighting;
  const double rtk_scale =
    initial_anchor ? 1.0
                   : (inside_gate ? config.vertical_rtk_inside_gate_sigma_scale
                                  : config.vertical_rtk_outside_gate_sigma_scale);
  weighting.direct_vertical_sigma_m = PositiveOrFloor(base_vertical_sigma_m * rtk_scale);
  weighting.inside_body_z_window = IsStateInsideBodyZJumpWindow(body_z_windows, state_index);
  weighting.inside_kinematic_sigma_scale =
    weighting.inside_body_z_window ? config.vertical_rtk_jump_inside_sigma_scale : 1.0;
  return weighting;
}

double ResolveVerticalInsideBazSigmaMps2(const OfflineRunnerConfig &config) {
  return PositiveOrFloor(
    config.vertical_acc_bias_sigma_mps2 > 0.0 ? config.vertical_acc_bias_sigma_mps2 : config.bias_acc_sigma);
}

}  // namespace offline_lc_minimal
