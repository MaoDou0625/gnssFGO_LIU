#include "offline_lc_minimal/core/VerticalInsideBiasAdapter.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {

namespace {

double ResolveVerticalAccBiasSigmaMps2(const OfflineRunnerConfig &config) {
  return config.vertical_acc_bias_sigma_mps2 > 0.0 ? config.vertical_acc_bias_sigma_mps2 : config.bias_acc_sigma;
}

}  // namespace

VerticalInsideBiasAdapter::VerticalInsideBiasAdapter(const OfflineRunnerConfig &config)
    : config_(config) {}

void VerticalInsideBiasAdapter::RewindFromStateIndex(const std::size_t state_index) {
  filter_history_.erase(
    std::remove_if(
      filter_history_.begin(),
      filter_history_.end(),
      [&](const FilterSnapshot &snapshot) { return snapshot.state_index >= state_index; }),
    filter_history_.end());
  RestoreFilterFromHistory();
}

void VerticalInsideBiasAdapter::AcceptUpdate(const VerticalInsideBiasUpdate &update) {
  filter_history_.erase(
    std::remove_if(
      filter_history_.begin(),
      filter_history_.end(),
      [&](const FilterSnapshot &snapshot) { return snapshot.state_index >= update.anchor_state_index; }),
    filter_history_.end());
  RestoreFilterFromHistory();
  filter_initialized_ = true;
  filter_height_residual_m_ = update.filter_height_residual_m;
  filter_baz_mps2_ = update.filter_baz_mps2;
  filter_covariance_ = update.filter_covariance;
  filter_time_s_ = std::isfinite(update.current_time_s) ? update.current_time_s : filter_time_s_;
  const Observation accepted_observation{
    update.anchor_state_index,
    update.current_time_s,
    update.residual_u_m,
    update.sigma_u_m,
    update.gate_threshold_m,
    update.pitch_rad,
    update.roll_rad};
  filter_history_.push_back(FilterSnapshot{
    update.anchor_state_index,
    filter_time_s_,
    filter_height_residual_m_,
    filter_baz_mps2_,
    filter_covariance_,
    accepted_observation});
  last_observation_ = accepted_observation;
}

std::optional<VerticalInsideBiasUpdate> VerticalInsideBiasAdapter::ObserveInsideResidual(
  const std::size_t state_index,
  const double time_s,
  const double residual_u_m,
  const double sigma_u_m,
  const double gate_threshold_m,
  const double pitch_rad,
  const double roll_rad) {
  if (!config_.enable_vertical_inside_bias_adaptation) {
    return std::nullopt;
  }
  if (!std::isfinite(time_s) || !std::isfinite(residual_u_m) || !std::isfinite(sigma_u_m) ||
      !std::isfinite(gate_threshold_m) || !std::isfinite(pitch_rad) || !std::isfinite(roll_rad) ||
      sigma_u_m <= 0.0 || gate_threshold_m <= 0.0) {
    return std::nullopt;
  }
  if (std::abs(residual_u_m) > gate_threshold_m) {
    return std::nullopt;
  }

  const Observation observation{state_index, time_s, residual_u_m, sigma_u_m, gate_threshold_m, pitch_rad, roll_rad};
  if (!filter_initialized_) {
    const double bias_std_mps2 = std::max(ResolveVerticalAccBiasSigmaMps2(config_), 1e-12);
    filter_initialized_ = true;
    filter_height_residual_m_ = residual_u_m;
    filter_baz_mps2_ = 0.0;
    filter_covariance_ = Eigen::Matrix2d::Zero();
    filter_covariance_(0, 0) = sigma_u_m * sigma_u_m;
    filter_covariance_(1, 1) = bias_std_mps2 * bias_std_mps2;
    filter_time_s_ = time_s;
    last_observation_ = observation;
    return std::nullopt;
  }
  return BuildKalmanUpdateCandidate(observation);
}

void VerticalInsideBiasAdapter::RestoreFilterFromHistory() {
  if (filter_history_.empty()) {
    filter_initialized_ = false;
    filter_height_residual_m_ = 0.0;
    filter_baz_mps2_ = 0.0;
    filter_covariance_ = Eigen::Matrix2d::Zero();
    filter_time_s_ = 0.0;
    last_observation_.reset();
    return;
  }
  const FilterSnapshot &snapshot = filter_history_.back();
  filter_initialized_ = true;
  filter_height_residual_m_ = snapshot.height_residual_m;
  filter_baz_mps2_ = snapshot.baz_mps2;
  filter_covariance_ = snapshot.covariance;
  filter_time_s_ = snapshot.time_s;
  last_observation_ = snapshot.baseline_observation;
}

std::optional<VerticalInsideBiasUpdate> VerticalInsideBiasAdapter::BuildKalmanUpdateCandidate(
  const Observation &observation) const {
  if (!filter_initialized_) {
    return std::nullopt;
  }

  const double dt_s = observation.time_s - filter_time_s_;
  if (dt_s <= 1e-6) {
    return std::nullopt;
  }

  const double bias_std_mps2 = std::max(ResolveVerticalAccBiasSigmaMps2(config_), 1e-12);
  const double stationary_variance_mps4 = bias_std_mps2 * bias_std_mps2;
  const double phi = std::exp(-dt_s / std::max(config_.vertical_acc_bias_tau_s, 1e-9));

  Eigen::Vector2d predicted_state;
  predicted_state << filter_height_residual_m_ + 0.5 * dt_s * dt_s * filter_baz_mps2_,
    phi * filter_baz_mps2_;

  Eigen::Matrix2d transition = Eigen::Matrix2d::Identity();
  transition(0, 1) = 0.5 * dt_s * dt_s;
  transition(1, 1) = phi;

  Eigen::Matrix2d process_noise = Eigen::Matrix2d::Zero();
  process_noise(1, 1) = stationary_variance_mps4 * config_.vertical_acc_bias_process_noise_scale *
                        std::max(1.0 - phi * phi, 0.0);
  const Eigen::Matrix2d predicted_covariance =
    transition * filter_covariance_ * transition.transpose() + process_noise;

  const double innovation_m = observation.residual_u_m - predicted_state[0];
  const double innovation_variance_m2 =
    predicted_covariance(0, 0) + std::max(observation.sigma_u_m * observation.sigma_u_m, 1e-12);
  if (!std::isfinite(innovation_variance_m2) || innovation_variance_m2 <= 0.0) {
    return std::nullopt;
  }

  const Eigen::Vector2d kalman_gain = predicted_covariance.col(0) / innovation_variance_m2;
  const Eigen::Vector2d posterior_state = predicted_state + kalman_gain * innovation_m;
  const Eigen::Matrix2d posterior_covariance =
    (Eigen::Matrix2d::Identity() - kalman_gain * Eigen::RowVector2d(1.0, 0.0)) * predicted_covariance;
  const double delta_baz_mps2 = posterior_state[1] - filter_baz_mps2_;
  if (!std::isfinite(delta_baz_mps2) || std::abs(delta_baz_mps2) <= 1e-12) {
    return std::nullopt;
  }

  double bounded_delta_pitch_rad = 0.0;
  double bounded_delta_roll_rad = 0.0;
  const double residual_delta_m =
    last_observation_.has_value() ? observation.residual_u_m - last_observation_->residual_u_m : innovation_m;
  const double diagnostic_dt_s =
    last_observation_.has_value() ? std::max(observation.time_s - last_observation_->time_s, 1e-6) : dt_s;
  const double equivalent_acc_mps2 = 2.0 * residual_delta_m / (diagnostic_dt_s * diagnostic_dt_s);
  const double desired_attitude_acc_mps2 =
    config_.vertical_inside_attitude_gain * equivalent_acc_mps2;
  if (std::abs(desired_attitude_acc_mps2) > 0.0) {
    const double gravity_mps2 = std::max(std::abs(config_.gravity_mps2), 1e-6);
    const double d_acc_d_pitch =
      -gravity_mps2 * std::sin(observation.pitch_rad) * std::cos(observation.roll_rad);
    const double d_acc_d_roll =
      -gravity_mps2 * std::cos(observation.pitch_rad) * std::sin(observation.roll_rad);
    const double gradient_norm2 = d_acc_d_pitch * d_acc_d_pitch + d_acc_d_roll * d_acc_d_roll;
    if (gradient_norm2 > 1e-12) {
      bounded_delta_pitch_rad = std::clamp(
        desired_attitude_acc_mps2 * d_acc_d_pitch / gradient_norm2,
        -config_.vertical_inside_max_delta_attitude_rad,
        config_.vertical_inside_max_delta_attitude_rad);
      bounded_delta_roll_rad = std::clamp(
        desired_attitude_acc_mps2 * d_acc_d_roll / gradient_norm2,
        -config_.vertical_inside_max_delta_attitude_rad,
        config_.vertical_inside_max_delta_attitude_rad);
    }
  }

  VerticalInsideBiasUpdate update;
  update.anchor_state_index = observation.state_index;
  update.current_state_index = observation.state_index;
  update.current_time_s = observation.time_s;
  update.delta_pitch_rad = bounded_delta_pitch_rad;
  update.delta_roll_rad = bounded_delta_roll_rad;
  update.delta_baz_mps2 = delta_baz_mps2;
  update.equivalent_acc_mps2 = equivalent_acc_mps2;
  update.residual_delta_m = residual_delta_m;
  update.window_dt_s = dt_s;
  update.filter_variance_mps4 = posterior_covariance(1, 1);
  update.residual_u_m = observation.residual_u_m;
  update.sigma_u_m = observation.sigma_u_m;
  update.gate_threshold_m = observation.gate_threshold_m;
  update.pitch_rad = observation.pitch_rad;
  update.roll_rad = observation.roll_rad;
  update.filter_height_residual_m = posterior_state[0];
  update.filter_baz_mps2 = posterior_state[1];
  update.filter_covariance = posterior_covariance;
  update.observation_count = last_observation_.has_value() ? 2 : 1;
  return update;
}

}  // namespace offline_lc_minimal
