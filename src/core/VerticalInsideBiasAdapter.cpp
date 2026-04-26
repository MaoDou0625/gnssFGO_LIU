#include "offline_lc_minimal/core/VerticalInsideBiasAdapter.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {

VerticalInsideBiasAdapter::VerticalInsideBiasAdapter(const OfflineRunnerConfig &config)
    : config_(config) {}

void VerticalInsideBiasAdapter::RewindFromStateIndex(const std::size_t state_index) {
  observations_.erase(
    std::remove_if(
      observations_.begin(),
      observations_.end(),
      [&](const Observation &observation) { return observation.state_index >= state_index; }),
    observations_.end());
}

void VerticalInsideBiasAdapter::AcceptUpdate(const VerticalInsideBiasUpdate &update) {
  if (std::isfinite(update.current_time_s)) {
    last_update_time_s_ = update.current_time_s;
  }
  RewindFromStateIndex(update.anchor_state_index);
}

std::optional<VerticalInsideBiasUpdate> VerticalInsideBiasAdapter::ObserveInsideResidual(
  const std::size_t state_index,
  const double time_s,
  const double residual_u_m,
  const double sigma_u_m,
  const double gate_threshold_m) {
  if (!config_.enable_vertical_inside_bias_adaptation) {
    return std::nullopt;
  }
  if (!std::isfinite(time_s) || !std::isfinite(residual_u_m) || !std::isfinite(sigma_u_m) ||
      !std::isfinite(gate_threshold_m) || sigma_u_m <= 0.0 || gate_threshold_m <= 0.0) {
    return std::nullopt;
  }
  if (std::abs(residual_u_m) > config_.vertical_inside_bias_gate_fraction * gate_threshold_m) {
    return std::nullopt;
  }

  observations_.push_back(Observation{state_index, time_s, residual_u_m, sigma_u_m, gate_threshold_m});
  PruneHistory(time_s);
  const auto candidate = BuildUpdateCandidate();
  return candidate;
}

void VerticalInsideBiasAdapter::PruneHistory(const double latest_time_s) {
  const double window_s = std::max(config_.vertical_inside_bias_window_s, 1e-6);
  observations_.erase(
    std::remove_if(
      observations_.begin(),
      observations_.end(),
      [&](const Observation &observation) {
        return observation.time_s < latest_time_s - window_s || observation.time_s > latest_time_s + 1e-6;
      }),
    observations_.end());
}

std::optional<VerticalInsideBiasUpdate> VerticalInsideBiasAdapter::BuildUpdateCandidate() const {
  if (static_cast<int>(observations_.size()) < config_.vertical_inside_bias_min_observations) {
    return std::nullopt;
  }
  const Observation &oldest = observations_.front();
  const Observation &latest = observations_.back();
  const double window_dt_s = latest.time_s - oldest.time_s;
  if (window_dt_s < config_.vertical_inside_bias_min_window_s) {
    return std::nullopt;
  }
  if (latest.time_s - last_update_time_s_ < config_.vertical_inside_bias_update_interval_s) {
    return std::nullopt;
  }

  const double residual_delta_m = latest.residual_u_m - oldest.residual_u_m;
  if (std::abs(latest.residual_u_m) < config_.vertical_inside_bias_min_abs_residual_m) {
    return std::nullopt;
  }
  if (latest.residual_u_m * residual_delta_m <= 0.0) {
    return std::nullopt;
  }
  if (std::abs(residual_delta_m) < config_.vertical_inside_bias_min_residual_delta_m) {
    return std::nullopt;
  }

  const double equivalent_acc_mps2 = 2.0 * residual_delta_m / (window_dt_s * window_dt_s);
  const double raw_delta_baz_mps2 = config_.vertical_inside_bias_gain * equivalent_acc_mps2;
  const double bounded_delta_baz_mps2 = std::clamp(
    raw_delta_baz_mps2,
    -config_.vertical_inside_bias_max_delta_mps2,
    config_.vertical_inside_bias_max_delta_mps2);
  if (std::abs(bounded_delta_baz_mps2) <= 0.0) {
    return std::nullopt;
  }

  VerticalInsideBiasUpdate update;
  update.anchor_state_index = latest.state_index;
  update.current_state_index = latest.state_index;
  update.current_time_s = latest.time_s;
  update.delta_baz_mps2 = bounded_delta_baz_mps2;
  update.equivalent_acc_mps2 = equivalent_acc_mps2;
  update.residual_delta_m = residual_delta_m;
  update.window_dt_s = window_dt_s;
  update.observation_count = static_cast<int>(observations_.size());
  return update;
}

}  // namespace offline_lc_minimal
