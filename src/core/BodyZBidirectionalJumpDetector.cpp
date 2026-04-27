#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>

namespace offline_lc_minimal {

namespace {

constexpr double kTimeEpsilonS = 1e-9;

double Median(std::vector<double> values) {
  values.erase(
    std::remove_if(values.begin(), values.end(), [](const double value) { return !std::isfinite(value); }),
    values.end());
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

double RobustSigma(std::vector<double> values) {
  values.erase(
    std::remove_if(values.begin(), values.end(), [](const double value) { return !std::isfinite(value); }),
    values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double median = Median(values);
  std::vector<double> absolute_deviations;
  absolute_deviations.reserve(values.size());
  for (const double value : values) {
    absolute_deviations.push_back(std::abs(value - median));
  }
  return 1.4826 * Median(std::move(absolute_deviations));
}

int OddWindowSamples(const double window_s, const double dt_s) {
  int samples = std::max(1, static_cast<int>(std::llround(window_s / std::max(dt_s, 1e-9))));
  if (samples % 2 == 0) {
    ++samples;
  }
  return samples;
}

std::vector<double> CenteredMean(
  const std::vector<double> &values,
  const double window_s,
  const double dt_s) {
  std::vector<double> result(values.size(), std::numeric_limits<double>::quiet_NaN());
  if (values.empty()) {
    return result;
  }
  const int count = OddWindowSamples(window_s, dt_s);
  const int half_count = count / 2;
  const int min_periods = std::max(1, count / 4);
  std::vector<double> prefix_sum(values.size() + 1U, 0.0);
  std::vector<int> prefix_count(values.size() + 1U, 0);
  for (std::size_t index = 0; index < values.size(); ++index) {
    prefix_sum[index + 1U] = prefix_sum[index];
    prefix_count[index + 1U] = prefix_count[index];
    if (std::isfinite(values[index])) {
      prefix_sum[index + 1U] += values[index];
      ++prefix_count[index + 1U];
    }
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    const std::size_t begin =
      index > static_cast<std::size_t>(half_count) ? index - static_cast<std::size_t>(half_count) : 0U;
    const std::size_t end =
      std::min(values.size(), index + static_cast<std::size_t>(half_count) + 1U);
    const int valid_count = prefix_count[end] - prefix_count[begin];
    if (valid_count >= min_periods) {
      result[index] = (prefix_sum[end] - prefix_sum[begin]) / static_cast<double>(valid_count);
    }
  }
  return result;
}

std::vector<double> CenteredStepMetric(
  const std::vector<double> &values,
  const double dt_s,
  const OfflineRunnerConfig &config) {
  std::vector<double> result(values.size(), std::numeric_limits<double>::quiet_NaN());
  if (values.empty()) {
    return result;
  }
  const int window_count =
    std::max(3, static_cast<int>(std::llround(config.body_z_jump_pre_post_window_s / std::max(dt_s, 1e-9))));
  const int gap_count =
    std::max(0, static_cast<int>(std::llround(config.body_z_jump_center_gap_s / std::max(dt_s, 1e-9))));
  const int min_periods = std::max(3, window_count / 3);

  for (std::size_t index = 0; index < values.size(); ++index) {
    if (index <= static_cast<std::size_t>(gap_count)) {
      continue;
    }
    const std::size_t left_end = index - static_cast<std::size_t>(gap_count + 1);
    const std::size_t left_begin =
      left_end + 1U > static_cast<std::size_t>(window_count)
        ? left_end + 1U - static_cast<std::size_t>(window_count)
        : 0U;
    const std::size_t right_begin = index + static_cast<std::size_t>(gap_count + 1);
    if (right_begin >= values.size()) {
      continue;
    }
    const std::size_t right_end =
      std::min(values.size() - 1U, right_begin + static_cast<std::size_t>(window_count) - 1U);

    std::vector<double> left_values;
    std::vector<double> right_values;
    left_values.reserve(left_end - left_begin + 1U);
    right_values.reserve(right_end - right_begin + 1U);
    for (std::size_t left_index = left_begin; left_index <= left_end; ++left_index) {
      if (std::isfinite(values[left_index])) {
        left_values.push_back(values[left_index]);
      }
    }
    for (std::size_t right_index = right_begin; right_index <= right_end; ++right_index) {
      if (std::isfinite(values[right_index])) {
        right_values.push_back(values[right_index]);
      }
    }
    if (left_values.size() < static_cast<std::size_t>(min_periods) ||
        right_values.size() < static_cast<std::size_t>(min_periods)) {
      continue;
    }
    result[index] = Median(std::move(right_values)) - Median(std::move(left_values));
  }
  return result;
}

std::vector<std::size_t> LocalPeakIndices(const std::vector<double> &score) {
  std::vector<std::size_t> peaks;
  if (score.size() < 3U) {
    return peaks;
  }
  for (std::size_t index = 1U; index + 1U < score.size(); ++index) {
    const double value = score[index];
    if (!std::isfinite(value) || value <= 0.0) {
      continue;
    }
    const double previous = std::isfinite(score[index - 1U]) ? score[index - 1U] : -std::numeric_limits<double>::infinity();
    const double next = std::isfinite(score[index + 1U]) ? score[index + 1U] : -std::numeric_limits<double>::infinity();
    if (value + 1e-12 >= previous && value + 1e-12 >= next) {
      peaks.push_back(index);
    }
  }
  return peaks;
}

gtsam::Rot3 InterpolateRotation(const gtsam::Rot3 &left, const gtsam::Rot3 &right, const double alpha) {
  const gtsam::Vector3 delta = gtsam::Rot3::Logmap(left.between(right));
  return left.compose(gtsam::Rot3::Expmap(alpha * delta));
}

ReferenceNodeState InterpolateReferenceState(
  const std::vector<ReferenceNodeState> &reference_states,
  const double time_s) {
  if (reference_states.empty()) {
    return {};
  }
  if (reference_states.size() == 1U || time_s <= reference_states.front().time_s + kTimeEpsilonS) {
    return reference_states.front();
  }
  if (time_s >= reference_states.back().time_s - kTimeEpsilonS) {
    return reference_states.back();
  }
  const auto upper_it = std::upper_bound(
    reference_states.begin(),
    reference_states.end(),
    time_s,
    [](const double timestamp_s, const ReferenceNodeState &state) { return timestamp_s < state.time_s; });
  const std::size_t right_index = static_cast<std::size_t>(std::distance(reference_states.begin(), upper_it));
  const std::size_t left_index = right_index - 1U;
  const auto &left_state = reference_states[left_index];
  const auto &right_state = reference_states[right_index];
  const double alpha =
    std::clamp((time_s - left_state.time_s) / (right_state.time_s - left_state.time_s), 0.0, 1.0);

  ReferenceNodeState state;
  state.time_s = time_s;
  state.pose = gtsam::Pose3(
    InterpolateRotation(left_state.pose.rotation(), right_state.pose.rotation(), alpha),
    gtsam::Point3(
      (1.0 - alpha) * left_state.pose.translation().x() + alpha * right_state.pose.translation().x(),
      (1.0 - alpha) * left_state.pose.translation().y() + alpha * right_state.pose.translation().y(),
      (1.0 - alpha) * left_state.pose.translation().z() + alpha * right_state.pose.translation().z()));
  state.velocity = (1.0 - alpha) * left_state.velocity + alpha * right_state.velocity;
  state.bias = gtsam::imuBias::ConstantBias(
    (1.0 - alpha) * left_state.bias.accelerometer() + alpha * right_state.bias.accelerometer(),
    (1.0 - alpha) * left_state.bias.gyroscope() + alpha * right_state.bias.gyroscope());
  state.omega = (1.0 - alpha) * left_state.omega + alpha * right_state.omega;
  return state;
}

std::size_t NearestStateIndex(const std::vector<double> &state_timestamps, const double time_s) {
  if (state_timestamps.empty()) {
    return 0U;
  }
  const auto lower = std::lower_bound(state_timestamps.begin(), state_timestamps.end(), time_s);
  if (lower == state_timestamps.begin()) {
    return 0U;
  }
  if (lower == state_timestamps.end()) {
    return state_timestamps.size() - 1U;
  }
  const std::size_t right_index = static_cast<std::size_t>(std::distance(state_timestamps.begin(), lower));
  const std::size_t left_index = right_index - 1U;
  return std::abs(state_timestamps[right_index] - time_s) < std::abs(time_s - state_timestamps[left_index])
           ? right_index
           : left_index;
}

void ExpandWindowByElapsedPadding(
  const std::vector<BodyZJumpSignalSample> &signal,
  const double left_target_time_s,
  const double right_target_time_s,
  const double max_duration_s,
  std::size_t &start_index,
  std::size_t &end_index) {
  while (start_index > 0U &&
         signal[start_index - 1U].time_s >= left_target_time_s - kTimeEpsilonS &&
         signal[end_index].time_s - signal[start_index - 1U].time_s <= max_duration_s + kTimeEpsilonS) {
    --start_index;
  }
  while (end_index + 1U < signal.size() &&
         signal[end_index + 1U].time_s <= right_target_time_s + kTimeEpsilonS &&
         signal[end_index + 1U].time_s - signal[start_index].time_s <= max_duration_s + kTimeEpsilonS) {
    ++end_index;
  }
}

std::pair<std::size_t, std::size_t> BuildWindow(
  const std::size_t center_index,
  const std::vector<double> &score,
  const std::vector<BodyZJumpSignalSample> &signal,
  const double threshold_mps,
  const OfflineRunnerConfig &config) {
  const double center_score = score[center_index];
  const double support_threshold = std::max(
    config.body_z_jump_min_score_mps,
    std::min(config.body_z_jump_support_ratio * threshold_mps,
             config.body_z_jump_support_ratio * center_score));
  const double max_half_duration_s = 0.5 * config.body_z_jump_max_window_duration_s;
  std::size_t start_index = center_index;
  std::size_t end_index = center_index;
  while (start_index > 0U) {
    const std::size_t next_start = start_index - 1U;
    if (signal[center_index].time_s - signal[next_start].time_s > max_half_duration_s + kTimeEpsilonS) {
      break;
    }
    if (!std::isfinite(score[next_start]) || score[next_start] < support_threshold) {
      break;
    }
    start_index = next_start;
  }
  while (end_index + 1U < signal.size()) {
    const std::size_t next_end = end_index + 1U;
    if (signal[next_end].time_s - signal[center_index].time_s > max_half_duration_s + kTimeEpsilonS) {
      break;
    }
    if (!std::isfinite(score[next_end]) || score[next_end] < support_threshold) {
      break;
    }
    end_index = next_end;
  }

  const double left_target_time_s = signal[start_index].time_s - config.body_z_jump_redundant_padding_s;
  const double right_target_time_s = signal[end_index].time_s + config.body_z_jump_redundant_padding_s;
  ExpandWindowByElapsedPadding(
    signal,
    left_target_time_s,
    right_target_time_s,
    config.body_z_jump_max_window_duration_s,
    start_index,
    end_index);
  if (start_index == center_index && center_index > 0U) {
    --start_index;
  }
  if (end_index == center_index && center_index + 1U < signal.size()) {
    ++end_index;
  }
  while (start_index < center_index &&
         signal[end_index].time_s - signal[start_index].time_s > config.body_z_jump_max_window_duration_s) {
    ++start_index;
  }
  while (end_index > center_index &&
         signal[end_index].time_s - signal[start_index].time_s > config.body_z_jump_max_window_duration_s) {
    --end_index;
  }
  if (center_score < support_threshold) {
    return {center_index, center_index};
  }
  return {start_index, end_index};
}

std::vector<BodyZJumpWindowCandidate> SelectDirection(
  const std::string &direction,
  const double sign,
  const std::vector<double> &signed_step,
  const std::vector<double> &score,
  const std::vector<BodyZJumpSignalSample> &signal,
  const std::vector<double> &state_timestamps,
  const OfflineRunnerConfig &config) {
  std::vector<BodyZJumpWindowCandidate> windows;
  const std::vector<std::size_t> peaks = LocalPeakIndices(score);
  std::vector<bool> suppressed(score.size(), false);

  std::vector<double> positive_scores;
  positive_scores.reserve(score.size());
  for (const double value : score) {
    if (std::isfinite(value) && value > 0.0) {
      positive_scores.push_back(value);
    }
  }
  const double direction_sigma = RobustSigma(std::move(positive_scores));
  const double signed_sigma = RobustSigma(signed_step);
  const double base_noise_floor = std::max(
    {config.body_z_jump_min_score_mps,
     std::isfinite(signed_sigma) ? 1.5 * signed_sigma : config.body_z_jump_min_score_mps,
     std::isfinite(direction_sigma) ? 1.5 * direction_sigma : config.body_z_jump_min_score_mps});

  for (int level = 1; level <= config.body_z_jump_max_levels; ++level) {
    std::vector<std::size_t> remaining_peaks;
    remaining_peaks.reserve(peaks.size());
    for (const std::size_t peak_index : peaks) {
      if (!suppressed[peak_index] && std::isfinite(score[peak_index])) {
        remaining_peaks.push_back(peak_index);
      }
    }
    if (remaining_peaks.empty()) {
      break;
    }

    double level_max_peak = -std::numeric_limits<double>::infinity();
    for (const std::size_t peak_index : remaining_peaks) {
      level_max_peak = std::max(level_max_peak, score[peak_index]);
    }
    if (!std::isfinite(level_max_peak) || level_max_peak < base_noise_floor) {
      break;
    }
    const double threshold = std::max(base_noise_floor, config.body_z_jump_threshold_ratio * level_max_peak);
    std::vector<std::size_t> level_peak_indices;
    for (const std::size_t peak_index : remaining_peaks) {
      if (score[peak_index] >= threshold) {
        level_peak_indices.push_back(peak_index);
      }
    }
    std::sort(
      level_peak_indices.begin(),
      level_peak_indices.end(),
      [&](const std::size_t left, const std::size_t right) { return score[left] > score[right]; });

    std::vector<double> ordered_times;
    ordered_times.reserve(level_peak_indices.size());
    for (const std::size_t peak_index : level_peak_indices) {
      ordered_times.push_back(signal[peak_index].time_s);
    }
    std::sort(ordered_times.begin(), ordered_times.end());
    std::vector<double> gaps;
    for (std::size_t index = 1U; index < ordered_times.size(); ++index) {
      gaps.push_back(ordered_times[index] - ordered_times[index - 1U]);
    }
    const double median_gap_s = gaps.empty() ? std::numeric_limits<double>::infinity() : Median(std::move(gaps));
    if (level_peak_indices.size() >= static_cast<std::size_t>(config.body_z_jump_dense_peak_count) &&
        median_gap_s < config.body_z_jump_dense_gap_s &&
        level_max_peak < config.body_z_jump_dense_peak_floor_ratio * base_noise_floor) {
      break;
    }

    int selected_this_level = 0;
    for (const std::size_t center_index : level_peak_indices) {
      if (suppressed[center_index]) {
        continue;
      }
      const auto [start_index, end_index] =
        BuildWindow(center_index, score, signal, threshold, config);
      const double signed_delta =
        signal[end_index].integrated_body_z_velocity_mps - signal[start_index].integrated_body_z_velocity_mps;
      if (direction == "DOWN" && signed_delta >= 0.0) {
        continue;
      }
      if (direction == "UP" && signed_delta <= 0.0) {
        continue;
      }
      bool too_close = false;
      for (const auto &window : windows) {
        if (std::abs(signal[center_index].time_s - window.center_time_s) < config.body_z_jump_min_separation_s) {
          too_close = true;
          break;
        }
      }
      if (too_close) {
        continue;
      }

      std::vector<double> acc_values;
      acc_values.reserve(end_index - start_index + 1U);
      for (std::size_t index = start_index; index <= end_index; ++index) {
        acc_values.push_back(signal[index].body_z_acc_mps2);
      }
      const auto [min_acc_it, max_acc_it] = std::minmax_element(acc_values.begin(), acc_values.end());
      const double mean_acc =
        std::accumulate(acc_values.begin(), acc_values.end(), 0.0) / static_cast<double>(acc_values.size());
      const double body_z_axis_nav_z = signal[center_index].body_z_axis_nav_z;
      BodyZJumpWindowCandidate window;
      window.direction = direction;
      window.selection_level = level;
      window.start_signal_index = start_index;
      window.center_signal_index = center_index;
      window.end_signal_index = end_index;
      window.start_state_index = NearestStateIndex(state_timestamps, signal[start_index].time_s);
      window.center_state_index = NearestStateIndex(state_timestamps, signal[center_index].time_s);
      window.end_state_index = NearestStateIndex(state_timestamps, signal[end_index].time_s);
      window.start_time_s = signal[start_index].time_s;
      window.center_time_s = signal[center_index].time_s;
      window.end_time_s = signal[end_index].time_s;
      window.start_relative_time_s = signal[start_index].relative_time_s;
      window.center_relative_time_s = signal[center_index].relative_time_s;
      window.end_relative_time_s = signal[end_index].relative_time_s;
      window.duration_s = window.end_time_s - window.start_time_s;
      window.pre_velocity_mps = signal[start_index].integrated_body_z_velocity_mps;
      window.post_velocity_mps = signal[end_index].integrated_body_z_velocity_mps;
      window.signed_delta_velocity_mps = signed_delta;
      window.direction_score_mps = sign * signed_step[center_index];
      window.signed_step_metric_mps = signed_step[center_index];
      window.level_threshold_mps = threshold;
      window.level_max_peak_mps = level_max_peak;
      window.level_noise_floor_mps = base_noise_floor;
      window.min_acc_mps2 = *min_acc_it;
      window.max_acc_mps2 = *max_acc_it;
      window.mean_acc_mps2 = mean_acc;
      window.body_z_axis_nav_z = body_z_axis_nav_z;
      window.delta_vz_init_mps = -signed_delta * body_z_axis_nav_z;
      windows.push_back(window);

      ++selected_this_level;
      const double suppress_start_time_s =
        signal[start_index].time_s - 0.5 * config.body_z_jump_min_separation_s;
      const double suppress_end_time_s =
        signal[end_index].time_s + 0.5 * config.body_z_jump_min_separation_s;
      for (std::size_t index = 0; index < signal.size(); ++index) {
        if (signal[index].time_s >= suppress_start_time_s &&
            signal[index].time_s <= suppress_end_time_s) {
          suppressed[index] = true;
        }
      }
    }
    if (selected_this_level == 0) {
      for (const std::size_t center_index : level_peak_indices) {
        suppressed[center_index] = true;
      }
      continue;
    }
  }
  return windows;
}

BodyZJumpWindowCandidate RebuildMergedWindow(
  const std::vector<BodyZJumpWindowCandidate> &windows,
  const std::size_t begin_index,
  const std::size_t end_index,
  const std::vector<BodyZJumpSignalSample> &signal,
  const std::vector<double> &state_timestamps) {
  BodyZJumpWindowCandidate merged = windows[begin_index];
  std::size_t best_window_index = begin_index;
  for (std::size_t index = begin_index; index <= end_index; ++index) {
    const auto &window = windows[index];
    if (window.start_time_s < merged.start_time_s) {
      merged.start_time_s = window.start_time_s;
      merged.start_relative_time_s = window.start_relative_time_s;
      merged.start_signal_index = window.start_signal_index;
    }
    if (window.end_time_s > merged.end_time_s) {
      merged.end_time_s = window.end_time_s;
      merged.end_relative_time_s = window.end_relative_time_s;
      merged.end_signal_index = window.end_signal_index;
    }
    if (window.direction_score_mps > windows[best_window_index].direction_score_mps) {
      best_window_index = index;
    }
    merged.selection_level = std::min(merged.selection_level, window.selection_level);
    merged.level_noise_floor_mps =
      std::max(merged.level_noise_floor_mps, window.level_noise_floor_mps);
    merged.level_max_peak_mps =
      std::max(merged.level_max_peak_mps, window.level_max_peak_mps);
    merged.level_threshold_mps =
      std::min(merged.level_threshold_mps, window.level_threshold_mps);
  }

  const auto &best = windows[best_window_index];
  merged.center_signal_index = best.center_signal_index;
  merged.center_time_s = best.center_time_s;
  merged.center_relative_time_s = best.center_relative_time_s;
  merged.direction_score_mps = best.direction_score_mps;
  merged.signed_step_metric_mps = best.signed_step_metric_mps;
  merged.body_z_axis_nav_z = best.body_z_axis_nav_z;

  merged.start_state_index = NearestStateIndex(state_timestamps, merged.start_time_s);
  merged.center_state_index = NearestStateIndex(state_timestamps, merged.center_time_s);
  merged.end_state_index = NearestStateIndex(state_timestamps, merged.end_time_s);
  merged.duration_s = merged.end_time_s - merged.start_time_s;
  merged.pre_velocity_mps = signal[merged.start_signal_index].integrated_body_z_velocity_mps;
  merged.post_velocity_mps = signal[merged.end_signal_index].integrated_body_z_velocity_mps;
  merged.signed_delta_velocity_mps = merged.post_velocity_mps - merged.pre_velocity_mps;
  merged.delta_vz_init_mps = -merged.signed_delta_velocity_mps * merged.body_z_axis_nav_z;

  double acc_sum = 0.0;
  std::size_t acc_count = 0U;
  merged.min_acc_mps2 = std::numeric_limits<double>::infinity();
  merged.max_acc_mps2 = -std::numeric_limits<double>::infinity();
  for (std::size_t signal_index = merged.start_signal_index;
       signal_index <= merged.end_signal_index && signal_index < signal.size();
       ++signal_index) {
    const double acc_mps2 = signal[signal_index].body_z_acc_mps2;
    if (!std::isfinite(acc_mps2)) {
      continue;
    }
    merged.min_acc_mps2 = std::min(merged.min_acc_mps2, acc_mps2);
    merged.max_acc_mps2 = std::max(merged.max_acc_mps2, acc_mps2);
    acc_sum += acc_mps2;
    ++acc_count;
  }
  if (acc_count > 0U) {
    merged.mean_acc_mps2 = acc_sum / static_cast<double>(acc_count);
  } else {
    merged.min_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
    merged.max_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
    merged.mean_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  }
  return merged;
}

std::vector<BodyZJumpWindowCandidate> MergeNearbySameDirectionWindows(
  std::vector<BodyZJumpWindowCandidate> windows,
  const std::vector<BodyZJumpSignalSample> &signal,
  const std::vector<double> &state_timestamps,
  const OfflineRunnerConfig &config) {
  if (windows.size() < 2U) {
    return windows;
  }
  std::sort(
    windows.begin(),
    windows.end(),
    [](const BodyZJumpWindowCandidate &left, const BodyZJumpWindowCandidate &right) {
      if (left.direction == right.direction) {
        return left.start_time_s < right.start_time_s;
      }
      return left.direction < right.direction;
    });

  std::vector<BodyZJumpWindowCandidate> merged_windows;
  const double merge_gap_s = std::max(config.body_z_jump_redundant_padding_s, 0.0);
  std::size_t group_begin = 0U;
  while (group_begin < windows.size()) {
    std::size_t group_end = group_begin;
    double group_end_time_s = windows[group_begin].end_time_s;
    while (group_end + 1U < windows.size() &&
           windows[group_end + 1U].direction == windows[group_begin].direction &&
           windows[group_end + 1U].start_time_s <= group_end_time_s + merge_gap_s) {
      ++group_end;
      group_end_time_s = std::max(group_end_time_s, windows[group_end].end_time_s);
    }
    merged_windows.push_back(
      RebuildMergedWindow(windows, group_begin, group_end, signal, state_timestamps));
    group_begin = group_end + 1U;
  }

  std::sort(
    merged_windows.begin(),
    merged_windows.end(),
    [](const BodyZJumpWindowCandidate &left, const BodyZJumpWindowCandidate &right) {
      return left.center_time_s < right.center_time_s;
    });
  return merged_windows;
}

}  // namespace

BodyZBidirectionalJumpDetector::BodyZBidirectionalJumpDetector(const OfflineRunnerConfig &config)
    : config_(config) {}

BodyZJumpDetectionResult BodyZBidirectionalJumpDetector::Detect(
  const std::vector<ImuSample> &imu_samples,
  const std::vector<ReferenceNodeState> &seed_reference_states,
  const std::vector<double> &state_timestamps,
  const double dynamic_start_time_s,
  const double end_time_s) const {
  BodyZJumpDetectionResult result;
  if (imu_samples.size() < 2U || seed_reference_states.empty() || state_timestamps.empty()) {
    return result;
  }

  std::vector<const ImuSample *> selected_imu_samples;
  selected_imu_samples.reserve(imu_samples.size());
  for (const auto &sample : imu_samples) {
    if (sample.time_s + kTimeEpsilonS < dynamic_start_time_s ||
        sample.time_s > end_time_s + kTimeEpsilonS ||
        sample.time_s < seed_reference_states.front().time_s - kTimeEpsilonS ||
        sample.time_s > seed_reference_states.back().time_s + kTimeEpsilonS) {
      continue;
    }
    selected_imu_samples.push_back(&sample);
  }
  if (selected_imu_samples.size() < 3U) {
    return result;
  }

  result.signal.reserve(selected_imu_samples.size());
  double integrated_velocity_mps = 0.0;
  double previous_time_s = selected_imu_samples.front()->time_s;
  double previous_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  const Eigen::Vector3d gravity_nav(0.0, 0.0, config_.gravity_mps2);
  for (const auto *sample : selected_imu_samples) {
    const ReferenceNodeState seed_state = InterpolateReferenceState(seed_reference_states, sample->time_s);
    const Eigen::Vector3d bias_acc = seed_state.bias.accelerometer();
    const Eigen::Matrix3d rotation_matrix = seed_state.pose.rotation().matrix();
    const Eigen::Vector3d gravity_body = rotation_matrix.transpose() * gravity_nav;
    const double body_z_specific_force_mps2 = sample->accel_mps2.z() - bias_acc.z();
    const double body_z_acc_mps2 = body_z_specific_force_mps2 - gravity_body.z();
    if (std::isfinite(previous_acc_mps2)) {
      const double dt_s = std::max(sample->time_s - previous_time_s, 0.0);
      integrated_velocity_mps += 0.5 * (previous_acc_mps2 + body_z_acc_mps2) * dt_s;
    }

    BodyZJumpSignalSample signal;
    signal.time_s = sample->time_s;
    signal.relative_time_s = sample->time_s - dynamic_start_time_s;
    signal.body_z_specific_force_mps2 = body_z_specific_force_mps2;
    signal.gravity_projection_z_mps2 = gravity_body.z();
    signal.body_z_acc_mps2 = body_z_acc_mps2;
    signal.integrated_body_z_velocity_mps = integrated_velocity_mps;
    signal.body_z_axis_nav_z = rotation_matrix.col(2).z();
    result.signal.push_back(signal);
    previous_time_s = sample->time_s;
    previous_acc_mps2 = body_z_acc_mps2;
  }

  std::vector<double> sample_times;
  sample_times.reserve(result.signal.size());
  for (const auto &sample : result.signal) {
    sample_times.push_back(sample.time_s);
  }
  std::vector<double> sample_dts;
  sample_dts.reserve(sample_times.size() - 1U);
  for (std::size_t index = 1U; index < sample_times.size(); ++index) {
    sample_dts.push_back(sample_times[index] - sample_times[index - 1U]);
  }
  const double dt_s = std::max(Median(std::move(sample_dts)), 1e-9);

  std::vector<double> body_z_acc;
  std::vector<double> integrated_velocity;
  body_z_acc.reserve(result.signal.size());
  integrated_velocity.reserve(result.signal.size());
  for (const auto &sample : result.signal) {
    body_z_acc.push_back(sample.body_z_acc_mps2);
    integrated_velocity.push_back(sample.integrated_body_z_velocity_mps);
  }
  const std::vector<double> body_z_acc_1s = CenteredMean(body_z_acc, 1.0, dt_s);
  const std::vector<double> velocity_0p2 =
    CenteredMean(integrated_velocity, config_.body_z_jump_velocity_smooth_s, dt_s);
  const std::vector<double> velocity_1s = CenteredMean(integrated_velocity, 1.0, dt_s);
  const std::vector<double> signed_step =
    CenteredStepMetric(velocity_0p2, dt_s, config_);
  std::vector<double> downward_score(signed_step.size(), std::numeric_limits<double>::quiet_NaN());
  std::vector<double> upward_score(signed_step.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < result.signal.size(); ++index) {
    result.signal[index].body_z_acc_1s_smooth_mps2 = body_z_acc_1s[index];
    result.signal[index].integrated_body_z_velocity_0p2s_smooth_mps = velocity_0p2[index];
    result.signal[index].integrated_body_z_velocity_1s_smooth_mps = velocity_1s[index];
    result.signal[index].signed_step_metric_mps = signed_step[index];
    if (std::isfinite(signed_step[index])) {
      downward_score[index] = std::max(-signed_step[index], 0.0);
      upward_score[index] = std::max(signed_step[index], 0.0);
    }
    result.signal[index].downward_score_mps = downward_score[index];
    result.signal[index].upward_score_mps = upward_score[index];
  }

  std::vector<BodyZJumpWindowCandidate> down_windows =
    SelectDirection("DOWN", -1.0, signed_step, downward_score, result.signal, state_timestamps, config_);
  std::vector<BodyZJumpWindowCandidate> up_windows =
    SelectDirection("UP", 1.0, signed_step, upward_score, result.signal, state_timestamps, config_);
  result.windows = std::move(down_windows);
  result.windows.insert(
    result.windows.end(),
    std::make_move_iterator(up_windows.begin()),
    std::make_move_iterator(up_windows.end()));
  std::sort(
    result.windows.begin(),
    result.windows.end(),
    [](const BodyZJumpWindowCandidate &left, const BodyZJumpWindowCandidate &right) {
      return left.center_time_s < right.center_time_s;
    });
  result.windows = MergeNearbySameDirectionWindows(
    std::move(result.windows),
    result.signal,
    state_timestamps,
    config_);
  return result;
}

}  // namespace offline_lc_minimal
