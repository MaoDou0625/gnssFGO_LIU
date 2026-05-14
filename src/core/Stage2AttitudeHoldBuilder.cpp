#include "offline_lc_minimal/core/Stage2AttitudeHoldBuilder.h"

#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/AttitudeHoldFactor.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::X;

}  // namespace

Stage2AttitudeHoldBuilder::Stage2AttitudeHoldBuilder(Stage2AttitudeHoldBuildRequest request)
    : request_(std::move(request)) {}

void Stage2AttitudeHoldBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.reference_states == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr) {
    throw std::runtime_error("Stage2AttitudeHoldBuilder received an incomplete request");
  }
  if (!request_.config->enable_stage2_velocity_optimization) {
    return;
  }
  if (request_.reference_states->size() != request_.state_timestamps->size()) {
    throw std::runtime_error("stage2 attitude hold requires one reference state per graph state");
  }

  const auto noise =
    gtsam::noiseModel::Isotropic::Sigma(
      3,
      request_.config->stage2_attitude_hold_sigma_rad);
  for (std::size_t state_index = 0; state_index < request_.state_timestamps->size(); ++state_index) {
    request_.graph->add(factor::AttitudeHoldFactor(
      X(state_index),
      (*request_.reference_states)[state_index].pose.rotation(),
      noise));
    ++request_.run_summary->stage2_attitude_hold_factor_count;
  }
}

}  // namespace offline_lc_minimal
