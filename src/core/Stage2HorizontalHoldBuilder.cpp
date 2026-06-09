#include "offline_lc_minimal/core/Stage2HorizontalHoldBuilder.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/HorizontalHoldFactor.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

constexpr double kBoundaryEpsilonS = 1.0e-9;

struct OutageHorizontalHoldSkipRange {
  double start_time_s = 0.0;
  double end_time_s = 0.0;
};

std::vector<OutageHorizontalHoldSkipRange> BuildOutageHorizontalHoldSkipRanges(
  const std::vector<RtkOutageBoundaryReferenceRow> *boundary_references) {
  if (boundary_references == nullptr || boundary_references->empty()) {
    return {};
  }
  struct Pair {
    bool has_start = false;
    bool has_end = false;
    double start_time_s = 0.0;
    double end_time_s = 0.0;
  };
  std::map<std::size_t, Pair> pairs;
  for (const RtkOutageBoundaryReferenceRow &reference : *boundary_references) {
    if (!reference.valid || !std::isfinite(reference.target_time_s)) {
      continue;
    }
    Pair &pair = pairs[reference.window_index];
    if (reference.boundary_role == "OUTAGE_START") {
      pair.has_start = true;
      pair.start_time_s = reference.target_time_s;
    } else if (reference.boundary_role == "OUTAGE_END") {
      pair.has_end = true;
      pair.end_time_s = reference.target_time_s;
    }
  }
  std::vector<OutageHorizontalHoldSkipRange> ranges;
  for (const auto &[_, pair] : pairs) {
    if (!pair.has_start || !pair.has_end ||
        pair.end_time_s <= pair.start_time_s + kBoundaryEpsilonS) {
      continue;
    }
    ranges.push_back(OutageHorizontalHoldSkipRange{
      pair.start_time_s,
      pair.end_time_s});
  }
  std::sort(
    ranges.begin(),
    ranges.end(),
    [](const OutageHorizontalHoldSkipRange &lhs,
       const OutageHorizontalHoldSkipRange &rhs) {
      return lhs.start_time_s < rhs.start_time_s;
    });
  return ranges;
}

bool ShouldSkipHorizontalHold(
  const double time_s,
  const std::vector<OutageHorizontalHoldSkipRange> &skip_ranges) {
  if (!std::isfinite(time_s)) {
    return false;
  }
  return std::any_of(
    skip_ranges.begin(),
    skip_ranges.end(),
    [time_s](const OutageHorizontalHoldSkipRange &range) {
      return time_s + kBoundaryEpsilonS >= range.start_time_s &&
             time_s <= range.end_time_s + kBoundaryEpsilonS;
    });
}

}  // namespace

Stage2HorizontalHoldBuilder::Stage2HorizontalHoldBuilder(
  Stage2HorizontalHoldBuildRequest request)
    : request_(std::move(request)) {}

void Stage2HorizontalHoldBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.reference_states == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr) {
    throw std::runtime_error("Stage2HorizontalHoldBuilder received an incomplete request");
  }
  if (!request_.config->enable_stage2_velocity_optimization) {
    return;
  }
  if (request_.reference_states->size() != request_.state_timestamps->size()) {
    throw std::runtime_error("stage2 horizontal hold requires one reference state per graph state");
  }

  const auto position_noise =
    gtsam::noiseModel::Isotropic::Sigma(
      2,
      request_.config->stage2_horizontal_position_hold_sigma_m);
  const auto velocity_noise =
    gtsam::noiseModel::Isotropic::Sigma(
      2,
      request_.config->stage2_horizontal_velocity_hold_sigma_mps);
  const std::vector<OutageHorizontalHoldSkipRange> horizontal_hold_skip_ranges =
    BuildOutageHorizontalHoldSkipRanges(request_.boundary_references);
  for (std::size_t state_index = 0; state_index < request_.state_timestamps->size(); ++state_index) {
    const ReferenceNodeState &reference_state = (*request_.reference_states)[state_index];
    if (ShouldSkipHorizontalHold(
          (*request_.state_timestamps)[state_index],
          horizontal_hold_skip_ranges)) {
      continue;
    }
    request_.graph->add(factor::HorizontalPositionHoldFactor(
      X(state_index),
      (gtsam::Vector2()
        << reference_state.pose.translation().x(),
           reference_state.pose.translation().y())
        .finished(),
      position_noise));
    ++request_.run_summary->stage2_horizontal_position_hold_factor_count;

    request_.graph->add(factor::HorizontalVelocityHoldFactor(
      V(state_index),
      (gtsam::Vector2()
        << reference_state.velocity.x(),
           reference_state.velocity.y())
        .finished(),
      velocity_noise));
    ++request_.run_summary->stage2_horizontal_velocity_hold_factor_count;
  }
}

}  // namespace offline_lc_minimal
