#include "offline_lc_minimal/core/RoadNoiseStateEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

struct NoiseWindow {
  double center_time_s = 0.0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  double rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  std::size_t sample_count = 0;
  bool high_noise = false;
};

struct MutableSegment {
  bool high_noise = false;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  double weighted_score_sum = 0.0;
  std::size_t window_count = 0;

  [[nodiscard]] double DurationS() const {
    return end_time_s - start_time_s;
  }

  [[nodiscard]] double MeanScore() const {
    return window_count == 0 ? std::numeric_limits<double>::quiet_NaN()
                             : weighted_score_sum / static_cast<double>(window_count);
  }
};

bool IsFiniteWindow(const BodyZSeedJumpWindowRow &window) {
  return std::isfinite(window.start_time_s) &&
         std::isfinite(window.end_time_s) &&
         window.end_time_s > window.start_time_s;
}

bool IsInJumpMask(
  const double time_s,
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows,
  const double padding_s) {
  if (jump_windows == nullptr) {
    return false;
  }
  for (const auto &window : *jump_windows) {
    if (!IsFiniteWindow(window)) {
      continue;
    }
    if (time_s + kTimeEpsilonS >= window.start_time_s - padding_s &&
        time_s <= window.end_time_s + padding_s + kTimeEpsilonS) {
      return true;
    }
  }
  return false;
}

double BodyZHighFrequencyResidualMps2(const BodyZJumpSignalSample &sample) {
  if (!std::isfinite(sample.body_z_acc_mps2) ||
      !std::isfinite(sample.body_z_acc_1s_smooth_mps2)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return sample.body_z_acc_mps2 - sample.body_z_acc_1s_smooth_mps2;
}

std::vector<NoiseWindow> BuildNoiseWindows(
  const OfflineRunnerConfig &config,
  const std::vector<BodyZJumpSignalSample> &signal,
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows) {
  std::vector<NoiseWindow> windows;
  if (signal.empty()) {
    return windows;
  }

  const double start_time_s = signal.front().time_s;
  const double end_time_s = signal.back().time_s;
  const double half_window_s = 0.5 * config.road_noise_state_window_s;
  const double stride_s = config.road_noise_state_stride_s;
  const double padding_s = std::max(0.0, config.vertical_velocity_delta_jump_padding_s);

  std::size_t left_index = 0;
  std::size_t right_index = 0;
  for (double center_time_s = start_time_s + half_window_s;
       center_time_s <= end_time_s - half_window_s + kTimeEpsilonS;
       center_time_s += stride_s) {
    const double window_start_s = center_time_s - half_window_s;
    const double window_end_s = center_time_s + half_window_s;
    while (left_index < signal.size() &&
           signal[left_index].time_s < window_start_s - kTimeEpsilonS) {
      ++left_index;
    }
    right_index = std::max(right_index, left_index);
    while (right_index < signal.size() &&
           signal[right_index].time_s <= window_end_s + kTimeEpsilonS) {
      ++right_index;
    }

    double squared_sum = 0.0;
    std::size_t sample_count = 0;
    for (std::size_t index = left_index; index < right_index; ++index) {
      const auto &sample = signal[index];
      if (IsInJumpMask(sample.time_s, jump_windows, padding_s)) {
        continue;
      }
      const double residual = BodyZHighFrequencyResidualMps2(sample);
      if (!std::isfinite(residual)) {
        continue;
      }
      squared_sum += residual * residual;
      ++sample_count;
    }
    if (sample_count < static_cast<std::size_t>(config.road_noise_state_min_sample_count)) {
      continue;
    }
    NoiseWindow window;
    window.center_time_s = center_time_s;
    window.start_time_s = window_start_s;
    window.end_time_s = window_end_s;
    window.rms_mps2 = std::sqrt(squared_sum / static_cast<double>(sample_count));
    window.sample_count = sample_count;
    windows.push_back(window);
  }
  return windows;
}

std::pair<double, double> EstimateTwoNoiseCenters(const std::vector<NoiseWindow> &windows) {
  std::vector<double> scores;
  scores.reserve(windows.size());
  for (const auto &window : windows) {
    if (std::isfinite(window.rms_mps2)) {
      scores.push_back(window.rms_mps2);
    }
  }
  if (scores.empty()) {
    return {
      std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::quiet_NaN()};
  }
  std::sort(scores.begin(), scores.end());
  if (scores.size() == 1U) {
    return {scores.front(), scores.front()};
  }

  double low_center = scores[scores.size() / 4U];
  double high_center = scores[(3U * scores.size()) / 4U];
  for (int iteration = 0; iteration < 30; ++iteration) {
    double low_sum = 0.0;
    double high_sum = 0.0;
    std::size_t low_count = 0;
    std::size_t high_count = 0;
    for (const double score : scores) {
      if (std::abs(score - low_center) <= std::abs(score - high_center)) {
        low_sum += score;
        ++low_count;
      } else {
        high_sum += score;
        ++high_count;
      }
    }
    if (low_count == 0 || high_count == 0) {
      break;
    }
    const double next_low = low_sum / static_cast<double>(low_count);
    const double next_high = high_sum / static_cast<double>(high_count);
    if (std::abs(next_low - low_center) < 1.0e-12 &&
        std::abs(next_high - high_center) < 1.0e-12) {
      break;
    }
    low_center = next_low;
    high_center = next_high;
  }
  if (low_center > high_center) {
    std::swap(low_center, high_center);
  }
  return {low_center, high_center};
}

void ClassifyNoiseWindows(
  const OfflineRunnerConfig &config,
  const double low_center,
  const double high_center,
  std::vector<NoiseWindow> &windows) {
  if (windows.empty()) {
    return;
  }
  if (!std::isfinite(low_center) || !std::isfinite(high_center) ||
      high_center <= low_center + 1.0e-12) {
    for (auto &window : windows) {
      window.high_noise = false;
    }
    return;
  }
  const double center_delta = high_center - low_center;
  const double midpoint = 0.5 * (low_center + high_center);
  const double hysteresis = std::clamp(config.road_noise_state_hysteresis_ratio, 0.0, 0.49) *
                            center_delta;
  const double enter_high_threshold = midpoint + hysteresis;
  const double exit_high_threshold = midpoint - hysteresis;

  bool high_noise = windows.front().rms_mps2 >= midpoint;
  for (auto &window : windows) {
    if (!high_noise && window.rms_mps2 >= enter_high_threshold) {
      high_noise = true;
    } else if (high_noise && window.rms_mps2 <= exit_high_threshold) {
      high_noise = false;
    }
    window.high_noise = high_noise;
  }
}

std::vector<MutableSegment> BuildMutableSegments(const std::vector<NoiseWindow> &windows) {
  std::vector<MutableSegment> segments;
  for (std::size_t index = 0; index < windows.size(); ++index) {
    const auto &window = windows[index];
    const double start_time_s =
      index == 0U ? window.start_time_s
                  : 0.5 * (windows[index - 1U].center_time_s + window.center_time_s);
    const double end_time_s =
      index + 1U == windows.size()
        ? window.end_time_s
        : 0.5 * (window.center_time_s + windows[index + 1U].center_time_s);

    if (segments.empty() || segments.back().high_noise != window.high_noise) {
      MutableSegment segment;
      segment.high_noise = window.high_noise;
      segment.start_time_s = start_time_s;
      segment.end_time_s = end_time_s;
      segment.weighted_score_sum = window.rms_mps2;
      segment.window_count = 1U;
      segments.push_back(segment);
      continue;
    }
    segments.back().end_time_s = end_time_s;
    segments.back().weighted_score_sum += window.rms_mps2;
    ++segments.back().window_count;
  }
  return segments;
}

void MergeAdjacentSameState(std::vector<MutableSegment> &segments) {
  if (segments.size() < 2U) {
    return;
  }
  std::vector<MutableSegment> merged;
  for (const auto &segment : segments) {
    if (merged.empty() || merged.back().high_noise != segment.high_noise) {
      merged.push_back(segment);
      continue;
    }
    merged.back().end_time_s = segment.end_time_s;
    merged.back().weighted_score_sum += segment.weighted_score_sum;
    merged.back().window_count += segment.window_count;
  }
  segments = std::move(merged);
}

void MergeShortSegments(
  const double min_segment_s,
  std::vector<MutableSegment> &segments) {
  if (min_segment_s <= 0.0) {
    return;
  }
  while (segments.size() > 1U) {
    auto shortest_it = std::min_element(
      segments.begin(),
      segments.end(),
      [](const MutableSegment &left, const MutableSegment &right) {
        return left.DurationS() < right.DurationS();
      });
    if (shortest_it == segments.end() ||
        shortest_it->DurationS() + kTimeEpsilonS >= min_segment_s) {
      break;
    }

    const std::size_t index = static_cast<std::size_t>(
      std::distance(segments.begin(), shortest_it));
    std::size_t merge_index = 0U;
    if (index == 0U) {
      merge_index = 1U;
    } else if (index + 1U == segments.size()) {
      merge_index = index - 1U;
    } else {
      const double previous_diff =
        std::abs(segments[index].MeanScore() - segments[index - 1U].MeanScore());
      const double next_diff =
        std::abs(segments[index].MeanScore() - segments[index + 1U].MeanScore());
      merge_index = previous_diff <= next_diff ? index - 1U : index + 1U;
    }

    const std::size_t left = std::min(index, merge_index);
    const std::size_t right = std::max(index, merge_index);
    MutableSegment merged = segments[left];
    merged.high_noise = segments[merge_index].high_noise;
    merged.start_time_s = segments[left].start_time_s;
    merged.end_time_s = segments[right].end_time_s;
    merged.weighted_score_sum =
      segments[left].weighted_score_sum + segments[right].weighted_score_sum;
    merged.window_count = segments[left].window_count + segments[right].window_count;
    segments.erase(segments.begin() + static_cast<long long>(right));
    segments.erase(segments.begin() + static_cast<long long>(left));
    segments.insert(segments.begin() + static_cast<long long>(left), merged);
    MergeAdjacentSameState(segments);
  }
}

std::vector<RoadNoiseStateSegmentRow> BuildRows(
  const std::vector<MutableSegment> &segments,
  const double low_center,
  const double high_center,
  const double hysteresis_ratio) {
  const double center_delta =
    std::isfinite(low_center) && std::isfinite(high_center)
      ? std::max(0.0, high_center - low_center)
      : std::numeric_limits<double>::quiet_NaN();
  const double midpoint =
    std::isfinite(center_delta) ? 0.5 * (low_center + high_center)
                                : std::numeric_limits<double>::quiet_NaN();
  const double hysteresis =
    std::isfinite(center_delta) ? std::clamp(hysteresis_ratio, 0.0, 0.49) * center_delta
                                : std::numeric_limits<double>::quiet_NaN();
  const double low_threshold =
    std::isfinite(midpoint) ? midpoint - hysteresis
                            : std::numeric_limits<double>::quiet_NaN();
  const double high_threshold =
    std::isfinite(midpoint) ? midpoint + hysteresis
                            : std::numeric_limits<double>::quiet_NaN();

  std::vector<RoadNoiseStateSegmentRow> rows;
  rows.reserve(segments.size());
  for (const auto &segment : segments) {
    if (segment.end_time_s <= segment.start_time_s + kTimeEpsilonS) {
      continue;
    }
    RoadNoiseStateSegmentRow row;
    row.segment_index = rows.size();
    row.state = segment.high_noise ? "HIGH_NOISE" : "LOW_NOISE";
    row.start_time_s = segment.start_time_s;
    row.end_time_s = segment.end_time_s;
    row.duration_s = segment.DurationS();
    row.mean_noise_rms_mps2 = segment.MeanScore();
    row.low_noise_center_mps2 = low_center;
    row.high_noise_center_mps2 = high_center;
    row.low_threshold_mps2 = low_threshold;
    row.high_threshold_mps2 = high_threshold;
    row.window_count = segment.window_count;
    rows.push_back(std::move(row));
  }
  return rows;
}

}  // namespace

RoadNoiseStateEstimator::RoadNoiseStateEstimator(RoadNoiseStateEstimatorRequest request)
    : request_(std::move(request)) {}

std::vector<RoadNoiseStateSegmentRow> RoadNoiseStateEstimator::Estimate() const {
  if (request_.config == nullptr || request_.signal == nullptr) {
    throw std::runtime_error("RoadNoiseStateEstimator received an incomplete request");
  }
  std::vector<NoiseWindow> windows =
    BuildNoiseWindows(*request_.config, *request_.signal, request_.jump_windows);
  if (windows.empty()) {
    return {};
  }

  const auto [low_center, high_center] = EstimateTwoNoiseCenters(windows);
  ClassifyNoiseWindows(*request_.config, low_center, high_center, windows);

  std::vector<MutableSegment> segments = BuildMutableSegments(windows);
  MergeShortSegments(request_.config->road_noise_state_min_segment_s, segments);
  return BuildRows(
    segments,
    low_center,
    high_center,
    request_.config->road_noise_state_hysteresis_ratio);
}

}  // namespace offline_lc_minimal
