#include "offline_lc_minimal/core/VerticalAccelBiasGmConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/InitialStaticBiasConstraintBuilder.h"
#include "offline_lc_minimal/factor/VerticalAccelBiasGmTransitionFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

double BiasDecay(const double dt_s, const double tau_s) {
  return std::exp(-std::max(dt_s, 0.0) / std::max(tau_s, 1.0e-9));
}

double BaseGmSigmaMps2(
  const OfflineRunnerConfig &config,
  const bool is_initial_static_interval) {
  return InitialStaticBiasConstraintBuilder::ResolveVerticalGmSigmaMps2(
    config,
    is_initial_static_interval);
}

bool ShouldApplyAdaptiveGmSigma(
  const OfflineRunnerConfig &config,
  const bool is_initial_static_interval,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) {
  return config.enable_vertical_motion_adaptive_reweighting &&
         !is_initial_static_interval &&
         stability_entry != nullptr &&
         !stability_entry->in_jump_padding &&
         std::isfinite(stability_entry->motion_score) &&
         std::clamp(stability_entry->motion_score, 0.0, 1.0) < 1.0 - 1.0e-12;
}

double AdaptiveGmDrivingSigmaMps2(
  const OfflineRunnerConfig &config,
  const bool is_initial_static_interval,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) {
  const double base_sigma = BaseGmSigmaMps2(config, is_initial_static_interval);
  if (!ShouldApplyAdaptiveGmSigma(config, is_initial_static_interval, stability_entry)) {
    return base_sigma;
  }
  const double score = std::clamp(stability_entry->motion_score, 0.0, 1.0);
  const double static_sigma = config.vertical_motion_adaptive_static_baz_gm_sigma_mps2;
  return std::exp((1.0 - score) * std::log(std::max(static_sigma, 1.0e-12)) +
                  score * std::log(std::max(base_sigma, 1.0e-12)));
}

double TransitionVarianceFloor(
  const OfflineRunnerConfig &config,
  const bool is_initial_static_interval,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) {
  return ShouldApplyAdaptiveGmSigma(config, is_initial_static_interval, stability_entry)
           ? 1.0e-24
           : 1.0e-12;
}

}  // namespace

VerticalAccelBiasGmConstraintBuilder::VerticalAccelBiasGmConstraintBuilder(
  VerticalAccelBiasGmConstraintBuildRequest request)
    : request_(std::move(request)) {}

void VerticalAccelBiasGmConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.records == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr) {
    throw std::runtime_error("VerticalAccelBiasGmConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_vertical_acc_bias_gm_process) {
    return;
  }
  for (const auto &record : *request_.records) {
    const double dt_s = record.end_time_s - record.start_time_s;
    const double phi = BiasDecay(dt_s, request_.config->vertical_acc_bias_tau_s);
    const auto *stability_entry = FindStabilityProfileEntry(
      request_.stability_profile,
      record.state_index_i,
      record.state_index_j);
    const double sigma_mps2 = TransitionSigmaMps2(
      *request_.config,
      dt_s,
      record.is_initial_static_interval,
      stability_entry);
    request_.graph->add(factor::VerticalAccelBiasGmTransitionFactor(
      symbol::B(record.state_index_i),
      symbol::B(record.state_index_j),
      request_.global_acc_bias_key,
      phi,
      gtsam::noiseModel::Isotropic::Sigma(1, sigma_mps2)));
    if (record.is_initial_static_interval &&
        request_.config->enable_initial_static_vertical_bias_gm_tightening) {
      ++request_.run_summary->initial_static_vertical_bias_gm_tightened_factor_count;
    }
  }
}

double VerticalAccelBiasGmConstraintBuilder::TransitionSigmaMps2(
  const OfflineRunnerConfig &config,
  const double dt_s,
  const bool is_initial_static_interval,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) {
  const double bounded_dt_s = std::max(dt_s, 1.0e-6);
  const double driving_sigma_mps2 = DrivingSigmaMps2(
    config,
    is_initial_static_interval,
    stability_entry);
  const double variance =
    driving_sigma_mps2 * driving_sigma_mps2 *
    config.vertical_acc_bias_process_noise_scale *
    std::max(
      1.0 - std::exp(-2.0 * bounded_dt_s / std::max(config.vertical_acc_bias_tau_s, 1.0e-9)),
      1.0e-9);
  return std::sqrt(std::max(
    variance,
    TransitionVarianceFloor(config, is_initial_static_interval, stability_entry)));
}

double VerticalAccelBiasGmConstraintBuilder::DrivingSigmaMps2(
  const OfflineRunnerConfig &config,
  const bool is_initial_static_interval,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) {
  return AdaptiveGmDrivingSigmaMps2(config, is_initial_static_interval, stability_entry);
}

}  // namespace offline_lc_minimal
