#include "offline_lc_minimal/core/SequentialNhcJumpDetector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace offline_lc_minimal {

namespace {

double WeightedAbsPercentile(
  const std::vector<std::pair<double, double>> &weighted_values,
  const double percentile) {
  if (weighted_values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  std::vector<std::pair<double, double>> sorted = weighted_values;
  std::sort(
    sorted.begin(),
    sorted.end(),
    [](const auto &left, const auto &right) { return left.first < right.first; });

  double total_weight = 0.0;
  for (const auto &[value, weight] : sorted) {
    (void)value;
    total_weight += std::max(weight, 0.0);
  }
  if (total_weight <= 0.0) {
    return sorted.back().first;
  }

  const double target_weight = std::clamp(percentile, 0.0, 1.0) * total_weight;
  double cumulative_weight = 0.0;
  for (const auto &[value, weight] : sorted) {
    cumulative_weight += std::max(weight, 0.0);
    if (cumulative_weight >= target_weight) {
      return value;
    }
  }
  return sorted.back().first;
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 0U) {
    return 0.5 * (values[middle - 1U] + values[middle]);
  }
  return values[middle];
}

}  // namespace

SequentialNhcJumpDetector::SequentialNhcJumpDetector(const OfflineRunnerConfig &config)
    : config_(config) {}

void SequentialNhcJumpDetector::SeedWithConfirmedStates(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t start_index,
  const std::size_t end_index) {
  ObserveConfirmedWindow(reference_states, start_index, end_index);
}

void SequentialNhcJumpDetector::ObserveConfirmedWindow(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t start_index,
  const std::size_t end_index) {
  if (reference_states.empty() || start_index >= reference_states.size()) {
    return;
  }
  const std::size_t bounded_end = std::min(end_index, reference_states.size() - 1U);
  for (std::size_t state_index = start_index; state_index <= bounded_end; ++state_index) {
    AppendState(reference_states[state_index], state_index);
  }
  PruneHistory(reference_states[bounded_end].time_s);
}

NhcThresholdSnapshot SequentialNhcJumpDetector::CurrentThresholds(const double evaluation_time_s) const {
  NhcThresholdSnapshot snapshot;
  snapshot.body_vz_baseline_mps = ComputeBodyVzBaseline(evaluation_time_s);
  snapshot.body_vy_threshold_mps = config_.nhc_body_vy_min_threshold_mps;
  snapshot.body_vz_threshold_mps = config_.nhc_body_vz_min_threshold_mps;
  if (history_.empty()) {
    return snapshot;
  }

  std::vector<std::pair<double, double>> weighted_abs_vy;
  std::vector<std::pair<double, double>> weighted_abs_vz_jump;
  weighted_abs_vy.reserve(history_.size());
  weighted_abs_vz_jump.reserve(history_.size());
  for (const auto &sample : history_) {
    const double age_s = evaluation_time_s - sample.time_s;
    if (age_s < -1e-6 || age_s > config_.nhc_history_max_age_s) {
      continue;
    }
    const double weight = std::exp(-std::max(age_s, 0.0) / std::max(config_.nhc_history_half_life_s, 1e-6));
    weighted_abs_vy.emplace_back(std::abs(sample.body_vy_mps), weight);
  }
  for (std::size_t sample_index = 1; sample_index < history_.size(); ++sample_index) {
    const auto &previous_sample = history_[sample_index - 1U];
    const auto &sample = history_[sample_index];
    if (previous_sample.is_jump || sample.is_jump) {
      continue;
    }
    const double age_s = evaluation_time_s - sample.time_s;
    if (age_s < -1e-6 || age_s > config_.nhc_history_max_age_s) {
      continue;
    }
    const double weight = std::exp(-std::max(age_s, 0.0) / std::max(config_.nhc_history_half_life_s, 1e-6));
    weighted_abs_vz_jump.emplace_back(std::abs(sample.body_vz_mps - previous_sample.body_vz_mps), weight);
  }

  const double weighted_abs_p99_vy = WeightedAbsPercentile(weighted_abs_vy, 0.99);
  const double weighted_abs_p99_vz = WeightedAbsPercentile(weighted_abs_vz_jump, 0.99);
  if (std::isfinite(weighted_abs_p99_vy)) {
    snapshot.body_vy_threshold_mps =
      std::max(config_.nhc_body_vy_min_threshold_mps, config_.nhc_body_vy_percentile_scale * weighted_abs_p99_vy);
  }
  if (std::isfinite(weighted_abs_p99_vz)) {
    snapshot.body_vz_threshold_mps =
      std::max(config_.nhc_body_vz_min_threshold_mps, config_.nhc_body_vz_percentile_scale * weighted_abs_p99_vz);
  }
  return snapshot;
}

NhcStateEvaluation SequentialNhcJumpDetector::EvaluateState(
  const ReferenceNodeState &state,
  const double evaluation_time_s) const {
  const Eigen::Vector3d body_velocity = ComputeBodyVelocity(state);
  const double previous_body_vz_mps = history_.empty() ? body_velocity.z() : history_.back().body_vz_mps;
  return EvaluateWithPreviousBodyVz(body_velocity.y(), body_velocity.z(), previous_body_vz_mps, evaluation_time_s);
}

NhcStateEvaluation SequentialNhcJumpDetector::EvaluateTransition(
  const ReferenceNodeState &previous_state,
  const ReferenceNodeState &state,
  const double evaluation_time_s) const {
  const Eigen::Vector3d previous_body_velocity = ComputeBodyVelocity(previous_state);
  const Eigen::Vector3d body_velocity = ComputeBodyVelocity(state);
  return EvaluateWithPreviousBodyVz(
    body_velocity.y(),
    body_velocity.z(),
    previous_body_velocity.z(),
    evaluation_time_s);
}

std::optional<std::size_t> SequentialNhcJumpDetector::FindJumpAnchor(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t start_index,
  const std::size_t end_index) const {
  if (!config_.enable_nhc_jump_reference || reference_states.empty() || start_index >= end_index ||
      end_index >= reference_states.size()) {
    return std::nullopt;
  }

  std::optional<std::size_t> first_crossing_index;
  for (std::size_t state_index = start_index + 1U; state_index <= end_index; ++state_index) {
    const NhcStateEvaluation evaluation =
      EvaluateTransition(reference_states[state_index - 1U], reference_states[state_index], reference_states[state_index].time_s);
    if (evaluation.exceeds_threshold) {
      first_crossing_index = state_index;
      break;
    }
  }
  if (!first_crossing_index.has_value()) {
    return std::nullopt;
  }
  return first_crossing_index;
}

std::optional<std::size_t> SequentialNhcJumpDetector::FindRecentJumpAnchor(
  const std::size_t start_index,
  const std::size_t end_index) const {
  if (!config_.enable_nhc_jump_reference || start_index > end_index || jump_history_.empty()) {
    return std::nullopt;
  }

  std::optional<DetectedJump> strongest_jump;
  for (const auto &jump : jump_history_) {
    if (jump.state_index < start_index || jump.state_index > end_index) {
      continue;
    }
    if (!strongest_jump.has_value() ||
        std::abs(jump.body_vz_jump_mps) > std::abs(strongest_jump->body_vz_jump_mps)) {
      strongest_jump = jump;
    }
  }
  if (!strongest_jump.has_value()) {
    return std::nullopt;
  }
  return strongest_jump->state_index;
}

Eigen::Vector3d SequentialNhcJumpDetector::ComputeBodyVelocity(const ReferenceNodeState &state) const {
  const Eigen::Vector3d nav_velocity(state.velocity.x(), state.velocity.y(), state.velocity.z());
  return state.pose.rotation().matrix().transpose() * nav_velocity;
}

NhcStateEvaluation SequentialNhcJumpDetector::EvaluateWithPreviousBodyVz(
  const double body_vy_mps,
  const double body_vz_mps,
  const double previous_body_vz_mps,
  const double evaluation_time_s) const {
  NhcStateEvaluation evaluation;
  const NhcThresholdSnapshot snapshot = CurrentThresholds(evaluation_time_s);
  evaluation.body_vy_mps = body_vy_mps;
  evaluation.body_vz_mps = body_vz_mps;
  const double body_vz_baseline_mps =
    std::isfinite(snapshot.body_vz_baseline_mps) ? snapshot.body_vz_baseline_mps : 0.0;
  evaluation.body_vz_residual_mps = evaluation.body_vz_mps - body_vz_baseline_mps;
  evaluation.body_vz_jump_mps = evaluation.body_vz_mps - previous_body_vz_mps;
  evaluation.exceeds_threshold =
    std::abs(evaluation.body_vz_jump_mps) > snapshot.body_vz_threshold_mps;
  return evaluation;
}

double SequentialNhcJumpDetector::ComputeBodyVzBaseline(const double evaluation_time_s) const {
  std::vector<double> recent_body_vz_samples;
  recent_body_vz_samples.reserve(history_.size());
  for (const auto &sample : history_) {
    const double age_s = evaluation_time_s - sample.time_s;
    if (age_s < -1e-6 || age_s > 1.0) {
      continue;
    }
    recent_body_vz_samples.push_back(sample.body_vz_mps);
  }
  if (recent_body_vz_samples.empty() && !history_.empty()) {
    recent_body_vz_samples.push_back(history_.back().body_vz_mps);
  }
  return Median(std::move(recent_body_vz_samples));
}

void SequentialNhcJumpDetector::AppendState(const ReferenceNodeState &state, const std::size_t state_index) {
  const Eigen::Vector3d body_velocity = ComputeBodyVelocity(state);
  bool is_jump = false;
  if (config_.enable_nhc_jump_reference && !history_.empty()) {
    const NhcThresholdSnapshot snapshot = CurrentThresholds(state.time_s);
    const double body_vz_jump_mps = body_velocity.z() - history_.back().body_vz_mps;
    if (std::isfinite(body_vz_jump_mps) && std::isfinite(snapshot.body_vz_threshold_mps) &&
        std::abs(body_vz_jump_mps) > snapshot.body_vz_threshold_mps) {
      is_jump = true;
      DetectedJump jump{
        .state_index = state_index,
        .time_s = state.time_s,
        .body_vz_jump_mps = body_vz_jump_mps,
        .body_vz_threshold_mps = snapshot.body_vz_threshold_mps,
      };
      if (!jump_history_.empty() &&
          state.time_s - jump_history_.back().time_s < config_.nhc_jump_min_separation_s) {
        if (std::abs(body_vz_jump_mps) > std::abs(jump_history_.back().body_vz_jump_mps)) {
          jump_history_.back() = jump;
        }
      } else {
        jump_history_.push_back(jump);
      }
    }
  }
  history_.push_back(AcceptedSample{
    .state_index = state_index,
    .time_s = state.time_s,
    .body_vy_mps = body_velocity.y(),
    .body_vz_mps = body_velocity.z(),
    .is_jump = is_jump,
  });
}

void SequentialNhcJumpDetector::PruneHistory(const double evaluation_time_s) {
  const auto history_begin = history_.begin();
  const auto retained_begin = std::find_if(
    history_begin,
    history_.end(),
    [&](const AcceptedSample &sample) {
      return evaluation_time_s - sample.time_s <= config_.nhc_history_max_age_s + 1e-6;
    });
  history_.erase(history_begin, retained_begin);

  const auto jump_begin = jump_history_.begin();
  const auto retained_jump_begin = std::find_if(
    jump_begin,
    jump_history_.end(),
    [&](const DetectedJump &jump) {
      return evaluation_time_s - jump.time_s <= config_.nhc_history_max_age_s + 1e-6;
    });
  jump_history_.erase(jump_begin, retained_jump_begin);
}

}  // namespace offline_lc_minimal
