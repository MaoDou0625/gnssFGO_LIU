#include "offline_lc_minimal/core/Stage2HorizontalHoldBuilder.h"

#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/HorizontalHoldFactor.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

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
  for (std::size_t state_index = 0; state_index < request_.state_timestamps->size(); ++state_index) {
    const ReferenceNodeState &reference_state = (*request_.reference_states)[state_index];
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
