#include "offline_lc_minimal/core/BodyZJumpWindowClassifier.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsFiniteWindow(const BodyZJumpWindowCandidate &window) {
  return std::isfinite(window.start_time_s) &&
         std::isfinite(window.end_time_s) &&
         window.end_time_s >= window.start_time_s;
}

bool IsLongBiasWindow(
  const BodyZJumpWindowCandidate &window,
  const OfflineRunnerConfig &config) {
  return std::isfinite(config.body_z_long_bias_min_duration_s) &&
         config.body_z_long_bias_min_duration_s > 0.0 &&
         window.duration_s > config.body_z_long_bias_min_duration_s + kTimeEpsilonS;
}

bool CandidateBelongsToMergedWindow(
  const BodyZJumpWindowCandidate &candidate,
  const BodyZJumpWindowCandidate &merged) {
  return candidate.direction == merged.direction &&
         candidate.start_time_s + kTimeEpsilonS >= merged.start_time_s &&
         candidate.end_time_s <= merged.end_time_s + kTimeEpsilonS;
}

bool SameWindow(
  const BodyZJumpWindowCandidate &left,
  const BodyZJumpWindowCandidate &right) {
  return left.direction == right.direction &&
         std::abs(left.start_time_s - right.start_time_s) <= kTimeEpsilonS &&
         std::abs(left.center_time_s - right.center_time_s) <= kTimeEpsilonS &&
         std::abs(left.end_time_s - right.end_time_s) <= kTimeEpsilonS;
}

void AppendUnique(
  std::vector<BodyZJumpWindowCandidate> &windows,
  const BodyZJumpWindowCandidate &window) {
  if (!IsFiniteWindow(window)) {
    return;
  }
  const auto existing_it = std::find_if(
    windows.begin(),
    windows.end(),
    [&](const BodyZJumpWindowCandidate &existing) { return SameWindow(existing, window); });
  if (existing_it == windows.end()) {
    windows.push_back(window);
  }
}

void SortByTime(std::vector<BodyZJumpWindowCandidate> &windows) {
  std::sort(
    windows.begin(),
    windows.end(),
    [](const BodyZJumpWindowCandidate &left, const BodyZJumpWindowCandidate &right) {
      return left.center_time_s < right.center_time_s;
    });
}

std::vector<BodyZJumpWindowCandidate> BoundaryTransitionsForLongWindow(
  const BodyZJumpWindowCandidate &merged_window,
  const std::vector<BodyZJumpWindowCandidate> &transition_candidates) {
  std::vector<BodyZJumpWindowCandidate> transitions;
  for (const auto &candidate : transition_candidates) {
    if (!IsFiniteWindow(candidate) ||
        !CandidateBelongsToMergedWindow(candidate, merged_window)) {
      continue;
    }
    transitions.push_back(candidate);
  }
  if (transitions.size() <= 2U) {
    return transitions;
  }
  SortByTime(transitions);
  return {transitions.front(), transitions.back()};
}

}  // namespace

BodyZJumpWindowClassification ClassifyBodyZJumpWindowsForBias(
  const std::vector<BodyZJumpWindowCandidate> &merged_windows,
  const std::vector<BodyZJumpWindowCandidate> &transition_candidates,
  const OfflineRunnerConfig &config) {
  BodyZJumpWindowClassification classification;
  classification.jump_windows.reserve(merged_windows.size());
  classification.bias_windows.reserve(merged_windows.size());

  for (const auto &merged_window : merged_windows) {
    if (!IsFiniteWindow(merged_window)) {
      continue;
    }
    classification.bias_windows.push_back(merged_window);
    if (!IsLongBiasWindow(merged_window, config)) {
      AppendUnique(classification.jump_windows, merged_window);
      continue;
    }

    const std::vector<BodyZJumpWindowCandidate> boundary_transitions =
      BoundaryTransitionsForLongWindow(merged_window, transition_candidates);
    bool added_transition = false;
    for (const auto &candidate : boundary_transitions) {
      AppendUnique(classification.jump_windows, candidate);
      added_transition = true;
    }
    if (!added_transition) {
      AppendUnique(classification.jump_windows, merged_window);
    }
  }

  SortByTime(classification.jump_windows);
  SortByTime(classification.bias_windows);
  return classification;
}

}  // namespace offline_lc_minimal
