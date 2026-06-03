#include "offline_lc_minimal/core/VerticalAdaptiveReweightingLoop.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {

VerticalAdaptiveReweightingLoop::VerticalAdaptiveReweightingLoop(
  VerticalAdaptiveReweightingLoopRequest request)
    : request_(std::move(request)) {}

VerticalAdaptiveReweightingLoopResult VerticalAdaptiveReweightingLoop::Run() const {
  if (request_.config == nullptr || request_.initial_values == nullptr ||
      !request_.run_pass || !request_.estimate_profile) {
    throw std::runtime_error("VerticalAdaptiveReweightingLoop received an incomplete request");
  }

  const int adaptive_iterations =
    request_.config->enable_vertical_motion_adaptive_reweighting
      ? std::max(0, request_.config->vertical_motion_adaptive_outer_iterations)
      : 0;
  const int max_passes = 1 + adaptive_iterations;

  VerticalAdaptiveReweightingLoopResult result;
  const gtsam::Values *initial_values = request_.initial_values;
  VerticalMotionStabilityProfile profile;
  bool has_profile = false;

  for (int pass_index = 0; pass_index < max_passes; ++pass_index) {
    const VerticalAdaptiveReweightingPassInput input{
      pass_index,
      has_profile ? &profile : nullptr,
      initial_values};
    const VerticalAdaptiveReweightingPassOutput output = request_.run_pass(input);
    result.optimized_values = output.optimized_values;
    result.initial_error = output.initial_error;
    result.final_error = output.final_error;
    result.pass_count = pass_index + 1;

    if (pass_index == max_passes - 1) {
      break;
    }

    VerticalMotionStabilityProfile next_profile =
      request_.estimate_profile(output.optimized_values, pass_index + 1);
    if (has_profile &&
        MaxMotionScoreDelta(profile, next_profile) <=
          request_.config->vertical_motion_adaptive_convergence_score_epsilon) {
      result.converged = true;
      result.final_profile = std::move(next_profile);
      break;
    }
    profile = std::move(next_profile);
    result.final_profile = profile;
    has_profile = true;
    initial_values = &result.optimized_values;
  }

  if (has_profile && result.final_profile.empty()) {
    result.final_profile = profile;
  }
  return result;
}

}  // namespace offline_lc_minimal
