#include "offline_lc_minimal/core/HorizontalVelocityDeltaConstraintBuilder.h"

#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/HorizontalVelocityDeltaFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

gtsam::Vector2 HorizontalTargetDelta(const VelocityDeltaPropagationRecord &record) {
  return gtsam::Vector2(record.target_delta_v_mps.x(), record.target_delta_v_mps.y());
}

}  // namespace

HorizontalVelocityDeltaConstraintBuilder::HorizontalVelocityDeltaConstraintBuilder(
  HorizontalVelocityDeltaConstraintBuildRequest request)
    : request_(std::move(request)) {}

void HorizontalVelocityDeltaConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.propagation_records == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr) {
    throw std::runtime_error(
      "HorizontalVelocityDeltaConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_horizontal_velocity_delta_constraint) {
    return;
  }

  const auto noise = gtsam::noiseModel::Isotropic::Sigma(
    2,
    request_.config->rtk_valid_horizontal_velocity_delta_sigma_mps);
  for (const auto &record : *request_.propagation_records) {
    const gtsam::Vector2 target_delta_v_mps = HorizontalTargetDelta(record);
    if (!target_delta_v_mps.allFinite()) {
      continue;
    }
    request_.graph->add(factor::HorizontalVelocityDeltaFactor(
      symbol::V(record.state_index_i),
      symbol::V(record.state_index_j),
      target_delta_v_mps,
      noise));
    ++request_.run_summary->horizontal_velocity_delta_factor_count;
  }
}

}  // namespace offline_lc_minimal
