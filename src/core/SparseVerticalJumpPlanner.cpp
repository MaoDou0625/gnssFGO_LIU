#include "offline_lc_minimal/core/SparseVerticalJumpPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace offline_lc_minimal {

namespace {

constexpr double kTimeEpsilonS = 1e-9;

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

struct RtkVerticalVelocitySample {
  double time_s = 0.0;
  double vz_mps = std::numeric_limits<double>::quiet_NaN();
};

std::size_t FindIndexAtOrBefore(const std::vector<double> &times, const double target_time_s) {
  const auto it = std::lower_bound(times.begin(), times.end(), target_time_s);
  if (it == times.begin()) {
    return 0U;
  }
  if (it != times.end() && *it == target_time_s) {
    return static_cast<std::size_t>(std::distance(times.begin(), it));
  }
  return static_cast<std::size_t>(std::distance(times.begin(), it - 1));
}

std::size_t FindIndexAtOrAfter(const std::vector<double> &times, const double target_time_s) {
  const auto it = std::lower_bound(times.begin(), times.end(), target_time_s);
  if (it == times.end()) {
    return times.size() - 1U;
  }
  return static_cast<std::size_t>(std::distance(times.begin(), it));
}

double InterpolateByTime(
  const std::vector<RtkVerticalVelocitySample> &samples,
  const double time_s) {
  if (samples.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (time_s < samples.front().time_s - kTimeEpsilonS ||
      time_s > samples.back().time_s + kTimeEpsilonS) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto upper_it = std::lower_bound(
    samples.begin(),
    samples.end(),
    time_s,
    [](const RtkVerticalVelocitySample &sample, const double value) { return sample.time_s < value; });
  if (upper_it == samples.begin()) {
    return upper_it->vz_mps;
  }
  if (upper_it == samples.end()) {
    return samples.back().vz_mps;
  }
  if (std::abs(upper_it->time_s - time_s) <= kTimeEpsilonS) {
    return upper_it->vz_mps;
  }

  const auto &right = *upper_it;
  const auto &left = *(upper_it - 1);
  const double alpha = std::clamp((time_s - left.time_s) / (right.time_s - left.time_s), 0.0, 1.0);
  return (1.0 - alpha) * left.vz_mps + alpha * right.vz_mps;
}

}  // namespace

SparseVerticalJumpPlanner::SparseVerticalJumpPlanner(const OfflineRunnerConfig &config)
    : config_(config) {}

std::vector<VerticalVzReferenceSample> SparseVerticalJumpPlanner::BuildGlobalReference(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<double> &state_timestamps,
  const double dynamic_start_time_s) const {
  std::vector<VerticalVzReferenceSample> reference(state_timestamps.size());
  for (std::size_t state_index = 0; state_index < state_timestamps.size(); ++state_index) {
    reference[state_index].time_s = state_timestamps[state_index];
  }
  if (gnss_samples.empty() || state_timestamps.empty()) {
    return reference;
  }

  const double diff_window_s = std::max(config_.vertical_global_vz_window_s, 1e-6);
  const double half_diff_window_s = 0.5 * diff_window_s;
  const double smooth_window_s = std::max(config_.vertical_global_vz_smooth_window_s, 1e-6);
  const double half_smooth_window_s = 0.5 * smooth_window_s;

  struct ValidRtkSample {
    double time_s = 0.0;
    double up_m = 0.0;
  };
  std::vector<ValidRtkSample> valid_rtk_samples;
  valid_rtk_samples.reserve(gnss_samples.size());
  for (const auto &sample : gnss_samples) {
    if (sample.fix_type() != GnssFixType::kRtkFix || !sample.has_valid_position() || !sample.has_enu_position) {
      continue;
    }
    const double corrected_time_s = sample.time_s - config_.gnss_time_offset_s;
    if (corrected_time_s + kTimeEpsilonS < dynamic_start_time_s) {
      continue;
    }
    if (!std::isfinite(sample.enu_position_m.z())) {
      continue;
    }
    valid_rtk_samples.push_back(ValidRtkSample{corrected_time_s, sample.enu_position_m.z()});
  }
  if (valid_rtk_samples.size() < 3U) {
    return reference;
  }

  std::vector<double> rtk_times;
  rtk_times.reserve(valid_rtk_samples.size());
  for (const auto &sample : valid_rtk_samples) {
    rtk_times.push_back(sample.time_s);
  }

  std::vector<RtkVerticalVelocitySample> rtk_velocity_samples;
  rtk_velocity_samples.reserve(valid_rtk_samples.size());
  for (std::size_t center_index = 0; center_index < valid_rtk_samples.size(); ++center_index) {
    const double center_time_s = valid_rtk_samples[center_index].time_s;
    const double target_left_s = center_time_s - half_diff_window_s;
    const double target_right_s = center_time_s + half_diff_window_s;
    if (target_left_s < rtk_times.front() - kTimeEpsilonS ||
        target_right_s > rtk_times.back() + kTimeEpsilonS) {
      continue;
    }
    const std::size_t left_index = FindIndexAtOrBefore(rtk_times, target_left_s);
    const std::size_t right_index = FindIndexAtOrAfter(rtk_times, target_right_s);
    if (left_index >= center_index || right_index <= center_index || left_index >= right_index) {
      continue;
    }
    const double left_gap_s = target_left_s - rtk_times[left_index];
    const double right_gap_s = rtk_times[right_index] - target_right_s;
    if (left_gap_s > half_diff_window_s + kTimeEpsilonS ||
        right_gap_s > half_diff_window_s + kTimeEpsilonS) {
      continue;
    }
    const double dt_s = rtk_times[right_index] - rtk_times[left_index];
    if (dt_s <= 1e-6) {
      continue;
    }
    const double delta_up_m = valid_rtk_samples[right_index].up_m - valid_rtk_samples[left_index].up_m;
    rtk_velocity_samples.push_back(RtkVerticalVelocitySample{
      center_time_s,
      delta_up_m / dt_s,
    });
  }
  if (rtk_velocity_samples.size() < 2U) {
    return reference;
  }

  std::vector<double> raw_reference_by_state(reference.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t state_index = 0; state_index < state_timestamps.size(); ++state_index) {
    const double time_s = state_timestamps[state_index];
    if (time_s + kTimeEpsilonS < dynamic_start_time_s) {
      continue;
    }
    const double interpolated_vz_mps = InterpolateByTime(rtk_velocity_samples, time_s);
    if (!std::isfinite(interpolated_vz_mps)) {
      continue;
    }
    reference[state_index].valid = true;
    reference[state_index].vz_ref_global_mps = interpolated_vz_mps;
    raw_reference_by_state[state_index] = interpolated_vz_mps;
  }

  for (std::size_t center_index = 0; center_index < state_timestamps.size(); ++center_index) {
    if (!reference[center_index].valid) {
      continue;
    }
    std::vector<double> window_values;
    for (std::size_t state_index = 0; state_index < state_timestamps.size(); ++state_index) {
      if (!reference[state_index].valid) {
        continue;
      }
      if (std::abs(state_timestamps[state_index] - state_timestamps[center_index]) > half_smooth_window_s + kTimeEpsilonS) {
        continue;
      }
      window_values.push_back(raw_reference_by_state[state_index]);
    }
    if (!window_values.empty()) {
      reference[center_index].vz_ref_global_smoothed_mps = Median(std::move(window_values));
    }
  }

  return reference;
}

void SparseVerticalJumpPlanner::SeedWithConfirmedStates(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const std::size_t start_index,
  const std::size_t end_index) {
  ObserveStateRange(reference_states, vertical_vz_reference, start_index, end_index);
}

void SparseVerticalJumpPlanner::ObserveConfirmedWindow(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const std::size_t start_index,
  const std::size_t end_index) {
  ObserveStateRange(reference_states, vertical_vz_reference, start_index, end_index);
}

double SparseVerticalJumpPlanner::CurrentJumpStepThreshold(const double evaluation_time_s) const {
  std::vector<std::pair<double, double>> weighted_abs_jump_values;
  weighted_abs_jump_values.reserve(mismatch_jump_history_.size());
  for (const auto &sample : mismatch_jump_history_) {
    const double age_s = evaluation_time_s - sample.time_s;
    if (age_s < -1e-6 || age_s > config_.nhc_history_max_age_s) {
      continue;
    }
    const double weight =
      std::exp(-std::max(age_s, 0.0) / std::max(config_.nhc_history_half_life_s, 1e-6));
    weighted_abs_jump_values.emplace_back(std::abs(sample.mismatch_jump_mps), weight);
  }
  const double weighted_abs_p99 = WeightedAbsPercentile(weighted_abs_jump_values, 0.99);
  if (!std::isfinite(weighted_abs_p99)) {
    return config_.vertical_jump_step_min_threshold_mps;
  }
  return std::max(config_.vertical_jump_step_min_threshold_mps, weighted_abs_p99);
}

std::vector<SparseVerticalJumpCandidate> SparseVerticalJumpPlanner::BuildCandidates(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const std::size_t start_index,
  const std::size_t end_index,
  const std::function<bool(std::size_t)> &nhc_support) const {
  std::vector<SparseVerticalJumpCandidate> candidates;
  if (reference_states.empty() || vertical_vz_reference.size() != reference_states.size() ||
      start_index >= end_index || end_index >= reference_states.size()) {
    return candidates;
  }

  struct CandidateSeed {
    SparseVerticalJumpCandidate candidate;
    double peak_value = 0.0;
  };
  std::vector<CandidateSeed> seeds;
  for (std::size_t state_index = std::max<std::size_t>(start_index + 1U, 1U);
       state_index <= end_index;
       ++state_index) {
    if (!vertical_vz_reference[state_index].valid ||
        !vertical_vz_reference[state_index - 1U].valid ||
        !std::isfinite(vertical_vz_reference[state_index].vz_ref_global_smoothed_mps) ||
        !std::isfinite(vertical_vz_reference[state_index - 1U].vz_ref_global_smoothed_mps)) {
      continue;
    }
    const double mismatch_mps =
      reference_states[state_index].velocity.z() - vertical_vz_reference[state_index].vz_ref_global_smoothed_mps;
    const double previous_mismatch_mps =
      reference_states[state_index - 1U].velocity.z() -
      vertical_vz_reference[state_index - 1U].vz_ref_global_smoothed_mps;
    const double mismatch_jump_mps = mismatch_mps - previous_mismatch_mps;
    const double threshold_mps = CurrentJumpStepThreshold(reference_states[state_index].time_s);
    if (!std::isfinite(mismatch_jump_mps) || std::abs(mismatch_jump_mps) <= threshold_mps) {
      continue;
    }

    const double previous_abs_jump =
      (state_index > start_index + 1U && vertical_vz_reference[state_index - 2U].valid &&
       std::isfinite(vertical_vz_reference[state_index - 2U].vz_ref_global_smoothed_mps))
        ? std::abs(
            (reference_states[state_index - 1U].velocity.z() -
             vertical_vz_reference[state_index - 1U].vz_ref_global_smoothed_mps) -
            (reference_states[state_index - 2U].velocity.z() -
             vertical_vz_reference[state_index - 2U].vz_ref_global_smoothed_mps))
        : -std::numeric_limits<double>::infinity();
    const double next_abs_jump =
      (state_index + 1U <= end_index && vertical_vz_reference[state_index + 1U].valid &&
       std::isfinite(vertical_vz_reference[state_index + 1U].vz_ref_global_smoothed_mps))
        ? std::abs(
            (reference_states[state_index + 1U].velocity.z() -
             vertical_vz_reference[state_index + 1U].vz_ref_global_smoothed_mps) -
            mismatch_mps)
        : -std::numeric_limits<double>::infinity();
    const double abs_jump = std::abs(mismatch_jump_mps);
    if (abs_jump + 1e-9 < previous_abs_jump || abs_jump + 1e-9 < next_abs_jump) {
      continue;
    }

    SparseVerticalJumpCandidate candidate;
    candidate.state_index = state_index;
    candidate.time_s = reference_states[state_index].time_s;
    candidate.vz_prefit_mps = reference_states[state_index].velocity.z();
    candidate.vz_ref_global_smoothed_mps = vertical_vz_reference[state_index].vz_ref_global_smoothed_mps;
    candidate.vz_mismatch_mps = mismatch_mps;
    candidate.vz_mismatch_jump_mps = mismatch_jump_mps;
    candidate.jump_step_threshold_mps = threshold_mps;
    candidate.delta_vz_init_mps = -mismatch_jump_mps;
    candidate.nhc_supported = nhc_support ? nhc_support(state_index) : false;
    candidate.score = abs_jump - threshold_mps + (candidate.nhc_supported ? 0.25 * threshold_mps : 0.0);
    seeds.push_back(CandidateSeed{candidate, abs_jump});
  }

  std::sort(
    seeds.begin(),
    seeds.end(),
    [](const CandidateSeed &left, const CandidateSeed &right) {
      if (left.candidate.score == right.candidate.score) {
        return left.candidate.time_s < right.candidate.time_s;
      }
      return left.candidate.score > right.candidate.score;
    });

  std::vector<SparseVerticalJumpCandidate> selected_candidates;
  selected_candidates.reserve(std::min<std::size_t>(
    seeds.size(),
    static_cast<std::size_t>(std::max(config_.vertical_jump_max_candidates_per_segment, 0))));
  for (const auto &seed : seeds) {
    bool suppressed = false;
    for (const auto &selected_candidate : selected_candidates) {
      if (std::abs(seed.candidate.time_s - selected_candidate.time_s) <
          config_.vertical_jump_candidate_min_separation_s - kTimeEpsilonS) {
        suppressed = true;
        break;
      }
    }
    if (suppressed) {
      continue;
    }
    selected_candidates.push_back(seed.candidate);
    if (selected_candidates.size() >=
        static_cast<std::size_t>(std::max(config_.vertical_jump_max_candidates_per_segment, 0))) {
      break;
    }
  }
  return selected_candidates;
}

void SparseVerticalJumpPlanner::ObserveStateRange(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const std::size_t start_index,
  const std::size_t end_index) {
  if (reference_states.empty() || vertical_vz_reference.size() != reference_states.size() ||
      start_index >= reference_states.size()) {
    return;
  }
  const std::size_t bounded_end = std::min(end_index, reference_states.size() - 1U);
  double previous_mismatch_mps = std::numeric_limits<double>::quiet_NaN();
  bool has_previous_mismatch = false;
  for (std::size_t state_index = start_index; state_index <= bounded_end; ++state_index) {
    if (!vertical_vz_reference[state_index].valid ||
        !std::isfinite(vertical_vz_reference[state_index].vz_ref_global_smoothed_mps)) {
      has_previous_mismatch = false;
      continue;
    }
    const double mismatch_mps =
      reference_states[state_index].velocity.z() - vertical_vz_reference[state_index].vz_ref_global_smoothed_mps;
    if (has_previous_mismatch) {
      mismatch_jump_history_.push_back(AcceptedMismatchJumpSample{
        state_index,
        reference_states[state_index].time_s,
        mismatch_mps - previous_mismatch_mps,
      });
    }
    previous_mismatch_mps = mismatch_mps;
    has_previous_mismatch = true;
  }
  PruneHistory(reference_states[bounded_end].time_s);
}

void SparseVerticalJumpPlanner::PruneHistory(const double evaluation_time_s) {
  const auto retained_begin = std::find_if(
    mismatch_jump_history_.begin(),
    mismatch_jump_history_.end(),
    [&](const AcceptedMismatchJumpSample &sample) {
      return evaluation_time_s - sample.time_s <= config_.nhc_history_max_age_s + 1e-6;
    });
  mismatch_jump_history_.erase(mismatch_jump_history_.begin(), retained_begin);
}

}  // namespace offline_lc_minimal
