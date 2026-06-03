#include "offline_lc_minimal/core/VerticalJumpBiasSegmenter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr std::size_t kMaxFitSampleCount = 500;

struct BiasFitSample {
  double time_s = 0.0;
  double velocity_mps = 0.0;
  double acceleration_mps2 = std::numeric_limits<double>::quiet_NaN();
};

struct FitRange {
  std::size_t begin = 0;
  std::size_t end = 0;
};

struct LineFit {
  double slope_mps2 = 0.0;
  double intercept_mps = 0.0;
  double sse = 0.0;
};

bool IsFinite(const double value) {
  return std::isfinite(value);
}

double SourceWindowDurationS(
  const std::vector<BodyZSeedJumpWindowRow> &windows,
  const std::vector<std::size_t> &window_indices) {
  double duration_s = 0.0;
  for (const std::size_t window_index : window_indices) {
    if (window_index >= windows.size()) {
      continue;
    }
    const auto &window = windows[window_index];
    if (!IsFinite(window.start_time_s) || !IsFinite(window.end_time_s) ||
        window.end_time_s <= window.start_time_s) {
      continue;
    }
    duration_s += window.end_time_s - window.start_time_s;
  }
  return duration_s;
}

double SumDetectedSignedDeltaVelocity(
  const std::vector<BodyZSeedJumpWindowRow> &windows,
  const std::vector<std::size_t> &window_indices) {
  double sum = 0.0;
  bool has_value = false;
  for (const std::size_t window_index : window_indices) {
    if (window_index >= windows.size()) {
      continue;
    }
    const double value = windows[window_index].signed_delta_velocity_mps;
    if (!IsFinite(value)) {
      continue;
    }
    sum += value;
    has_value = true;
  }
  return has_value ? sum : std::numeric_limits<double>::quiet_NaN();
}

double VelocityForSegmenting(const BodyZSeedImuDiagnosticRow &row) {
  if (IsFinite(row.integrated_body_z_velocity_0p2s_smooth_mps)) {
    return row.integrated_body_z_velocity_0p2s_smooth_mps;
  }
  if (IsFinite(row.integrated_body_z_velocity_mps)) {
    return row.integrated_body_z_velocity_mps;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

std::vector<BiasFitSample> CollectSamples(
  const std::vector<BodyZSeedImuDiagnosticRow> &diagnostics,
  const double start_time_s,
  const double end_time_s) {
  std::vector<BiasFitSample> samples;
  samples.reserve(diagnostics.size());
  for (const auto &row : diagnostics) {
    const double velocity_mps = VelocityForSegmenting(row);
    if (!IsFinite(row.time_s) || !IsFinite(velocity_mps) ||
        row.time_s + kTimeEpsilonS < start_time_s ||
        row.time_s > end_time_s + kTimeEpsilonS) {
      continue;
    }
    samples.push_back(BiasFitSample{row.time_s, velocity_mps, row.body_z_acc_mps2});
  }
  std::sort(samples.begin(), samples.end(), [](const BiasFitSample &left, const BiasFitSample &right) {
    return left.time_s < right.time_s;
  });
  return samples;
}

std::vector<BiasFitSample> DownsampleForFit(std::vector<BiasFitSample> samples) {
  if (samples.size() <= kMaxFitSampleCount) {
    return samples;
  }
  std::vector<BiasFitSample> downsampled;
  downsampled.reserve(kMaxFitSampleCount);
  const double step = static_cast<double>(samples.size() - 1U) /
                      static_cast<double>(kMaxFitSampleCount - 1U);
  std::size_t previous_index = samples.size();
  for (std::size_t output_index = 0; output_index < kMaxFitSampleCount; ++output_index) {
    std::size_t sample_index = static_cast<std::size_t>(std::llround(step * output_index));
    sample_index = std::min(sample_index, samples.size() - 1U);
    if (sample_index == previous_index) {
      continue;
    }
    downsampled.push_back(samples[sample_index]);
    previous_index = sample_index;
  }
  return downsampled;
}

LineFit FitLine(
  const std::vector<BiasFitSample> &samples,
  const std::size_t begin,
  const std::size_t end) {
  if (end <= begin + 1U) {
    return {};
  }
  const std::size_t count = end - begin;
  const double t0 = samples[begin].time_s;
  double sum_t = 0.0;
  double sum_v = 0.0;
  double sum_tt = 0.0;
  double sum_tv = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    const double t = samples[index].time_s - t0;
    const double v = samples[index].velocity_mps;
    sum_t += t;
    sum_v += v;
    sum_tt += t * t;
    sum_tv += t * v;
  }
  const double denominator = static_cast<double>(count) * sum_tt - sum_t * sum_t;
  LineFit fit;
  if (std::abs(denominator) > 1.0e-12) {
    fit.slope_mps2 = (static_cast<double>(count) * sum_tv - sum_t * sum_v) / denominator;
  }
  fit.intercept_mps = (sum_v - fit.slope_mps2 * sum_t) / static_cast<double>(count);

  for (std::size_t index = begin; index < end; ++index) {
    const double predicted = fit.intercept_mps + fit.slope_mps2 * (samples[index].time_s - t0);
    const double residual = samples[index].velocity_mps - predicted;
    fit.sse += residual * residual;
  }
  return fit;
}

double DurationS(const std::vector<BiasFitSample> &samples, const FitRange &range) {
  if (range.end <= range.begin + 1U) {
    return 0.0;
  }
  return samples[range.end - 1U].time_s - samples[range.begin].time_s;
}

double BicForModel(
  const std::size_t sample_count,
  const double sse,
  const std::size_t segment_count) {
  const double n = static_cast<double>(std::max<std::size_t>(sample_count, 2U));
  const double variance = std::max(sse / n, 1.0e-12);
  return n * std::log(variance) + 2.0 * static_cast<double>(segment_count) * std::log(n);
}

std::vector<FitRange> SplitByPiecewiseLinearFit(
  const std::vector<BiasFitSample> &samples,
  const OfflineRunnerConfig &config) {
  std::vector<FitRange> ranges{{0U, samples.size()}};
  const std::size_t max_segments =
    static_cast<std::size_t>(std::max(1, config.vertical_jump_segmented_bias_max_segments));
  const double min_segment_s = config.vertical_jump_segmented_bias_min_segment_s;
  while (ranges.size() < max_segments) {
    double current_sse = 0.0;
    for (const auto &range : ranges) {
      current_sse += FitLine(samples, range.begin, range.end).sse;
    }
    const double current_bic = BicForModel(samples.size(), current_sse, ranges.size());

    bool has_candidate = false;
    std::size_t best_range_index = 0;
    std::size_t best_cut = 0;
    double best_candidate_sse = current_sse;
    double best_candidate_bic = current_bic;
    for (std::size_t range_index = 0; range_index < ranges.size(); ++range_index) {
      const FitRange range = ranges[range_index];
      if (range.end <= range.begin + 2U) {
        continue;
      }
      const double range_sse = FitLine(samples, range.begin, range.end).sse;
      for (std::size_t cut = range.begin + 1U; cut < range.end; ++cut) {
        const FitRange left{range.begin, cut};
        const FitRange right{cut, range.end};
        if (DurationS(samples, left) + kTimeEpsilonS < min_segment_s ||
            DurationS(samples, right) + kTimeEpsilonS < min_segment_s) {
          continue;
        }
        const double candidate_sse =
          current_sse - range_sse +
          FitLine(samples, left.begin, left.end).sse +
          FitLine(samples, right.begin, right.end).sse;
        const double candidate_bic = BicForModel(samples.size(), candidate_sse, ranges.size() + 1U);
        if (candidate_bic + 1.0e-9 < best_candidate_bic) {
          has_candidate = true;
          best_range_index = range_index;
          best_cut = cut;
          best_candidate_sse = candidate_sse;
          best_candidate_bic = candidate_bic;
        }
      }
    }
    (void)best_candidate_sse;
    if (!has_candidate || best_candidate_bic + 1.0e-9 >= current_bic) {
      break;
    }
    const FitRange range = ranges[best_range_index];
    ranges.erase(ranges.begin() + static_cast<std::ptrdiff_t>(best_range_index));
    ranges.push_back(FitRange{range.begin, best_cut});
    ranges.push_back(FitRange{best_cut, range.end});
    std::sort(ranges.begin(), ranges.end(), [](const FitRange &left, const FitRange &right) {
      return left.begin < right.begin;
    });
  }
  return ranges;
}

void MergeSimilarSlopeRanges(
  const std::vector<BiasFitSample> &samples,
  const OfflineRunnerConfig &config,
  std::vector<FitRange> &ranges) {
  bool merged = true;
  while (merged && ranges.size() > 1U) {
    merged = false;
    for (std::size_t index = 0; index + 1U < ranges.size(); ++index) {
      const double left_slope = FitLine(samples, ranges[index].begin, ranges[index].end).slope_mps2;
      const double right_slope = FitLine(samples, ranges[index + 1U].begin, ranges[index + 1U].end).slope_mps2;
      if (std::abs(left_slope - right_slope) >
          config.vertical_jump_segmented_bias_slope_merge_threshold_mps2) {
        continue;
      }
      ranges[index].end = ranges[index + 1U].end;
      ranges.erase(ranges.begin() + static_cast<std::ptrdiff_t>(index + 1U));
      merged = true;
      break;
    }
  }
}

double Percentile(std::vector<double> values, const double percentile) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double position = percentile * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  const double weight = position - static_cast<double>(lower);
  return values[lower] * (1.0 - weight) + values[upper] * weight;
}

void FillHighFrequencyStats(
  const std::vector<BiasFitSample> &samples,
  const double start_time_s,
  const double end_time_s,
  const double slope_mps2,
  VerticalJumpBiasSegmentEstimate &estimate) {
  double sum_square = 0.0;
  std::size_t count = 0;
  std::vector<double> abs_residuals;
  for (const auto &sample : samples) {
    if (!IsFinite(sample.acceleration_mps2) ||
        sample.time_s + kTimeEpsilonS < start_time_s ||
        sample.time_s > end_time_s + kTimeEpsilonS) {
      continue;
    }
    const double residual = sample.acceleration_mps2 - slope_mps2;
    sum_square += residual * residual;
    abs_residuals.push_back(std::abs(residual));
    ++count;
  }
  if (count == 0U) {
    estimate.highfreq_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
    estimate.highfreq_p95_abs_mps2 = std::numeric_limits<double>::quiet_NaN();
    return;
  }
  estimate.highfreq_rms_mps2 = std::sqrt(sum_square / static_cast<double>(count));
  estimate.highfreq_p95_abs_mps2 = Percentile(std::move(abs_residuals), 0.95);
}

VerticalJumpBiasSegmentEstimate MakeFallbackEstimate(
  const VerticalJumpBiasSpanInput &span,
  const std::vector<BodyZSeedJumpWindowRow> &windows) {
  VerticalJumpBiasSegmentEstimate estimate;
  estimate.span_index = span.span_index;
  estimate.segment_index = 0;
  estimate.segment_count = 1;
  estimate.source_window_index = span.source_window_indices.empty() ? 0U : span.source_window_indices.front();
  estimate.source_window_count = span.source_window_indices.size();
  estimate.start_time_s = span.start_time_s;
  estimate.end_time_s = span.end_time_s;
  estimate.source_window_duration_s = SourceWindowDurationS(windows, span.source_window_indices);
  estimate.detected_signed_delta_velocity_mps =
    SumDetectedSignedDeltaVelocity(windows, span.source_window_indices);
  estimate.detected_bias_mps2 =
    estimate.source_window_duration_s > 0.0 && IsFinite(estimate.detected_signed_delta_velocity_mps)
      ? estimate.detected_signed_delta_velocity_mps / estimate.source_window_duration_s
      : std::numeric_limits<double>::quiet_NaN();
  estimate.used_segmented_estimate = false;
  estimate.highfreq_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  estimate.highfreq_p95_abs_mps2 = std::numeric_limits<double>::quiet_NaN();
  return estimate;
}

std::vector<VerticalJumpBiasSegmentEstimate> EstimateSegmentedSpan(
  const VerticalJumpBiasSpanInput &span,
  const std::vector<BodyZSeedJumpWindowRow> &windows,
  const std::vector<BodyZSeedImuDiagnosticRow> &diagnostics,
  const OfflineRunnerConfig &config) {
  const std::vector<BiasFitSample> all_samples = CollectSamples(diagnostics, span.start_time_s, span.end_time_s);
  if (all_samples.size() < 2U ||
      all_samples.back().time_s - all_samples.front().time_s + kTimeEpsilonS <
        config.vertical_jump_segmented_bias_min_segment_s) {
    return {MakeFallbackEstimate(span, windows)};
  }

  const std::vector<BiasFitSample> fit_samples = DownsampleForFit(all_samples);
  std::vector<FitRange> ranges = SplitByPiecewiseLinearFit(fit_samples, config);
  MergeSimilarSlopeRanges(fit_samples, config, ranges);

  std::vector<VerticalJumpBiasSegmentEstimate> estimates;
  estimates.reserve(ranges.size());
  std::vector<double> segment_boundaries(ranges.size() + 1U, span.start_time_s);
  segment_boundaries.front() = span.start_time_s;
  segment_boundaries.back() = span.end_time_s;
  for (std::size_t index = 0; index + 1U < ranges.size(); ++index) {
    const double left_time_s = fit_samples[ranges[index].end - 1U].time_s;
    const double right_time_s = fit_samples[ranges[index + 1U].begin].time_s;
    segment_boundaries[index + 1U] = 0.5 * (left_time_s + right_time_s);
  }
  for (std::size_t segment_index = 0; segment_index < ranges.size(); ++segment_index) {
    const auto &range = ranges[segment_index];
    const LineFit fit = FitLine(fit_samples, range.begin, range.end);
    const double start_time_s = segment_boundaries[segment_index];
    const double end_time_s = segment_boundaries[segment_index + 1U];
    const double duration_s = std::max(end_time_s - start_time_s, 0.0);

    VerticalJumpBiasSegmentEstimate estimate;
    estimate.span_index = span.span_index;
    estimate.segment_index = segment_index;
    estimate.segment_count = ranges.size();
    estimate.source_window_index = span.source_window_indices.empty() ? 0U : span.source_window_indices.front();
    estimate.source_window_count = span.source_window_indices.size();
    estimate.start_time_s = start_time_s;
    estimate.end_time_s = end_time_s;
    estimate.source_window_duration_s = duration_s;
    estimate.detected_bias_mps2 = fit.slope_mps2;
    estimate.detected_signed_delta_velocity_mps = fit.slope_mps2 * duration_s;
    estimate.used_segmented_estimate = true;
    FillHighFrequencyStats(all_samples, estimate.start_time_s, estimate.end_time_s, fit.slope_mps2, estimate);
    estimates.push_back(estimate);
  }
  return estimates.empty() ? std::vector<VerticalJumpBiasSegmentEstimate>{MakeFallbackEstimate(span, windows)}
                           : estimates;
}

}  // namespace

std::vector<VerticalJumpBiasSegmentEstimate> EstimateVerticalJumpBiasSegments(
  const VerticalJumpBiasSegmenterRequest &request) {
  if (request.config == nullptr || request.jump_windows == nullptr || request.spans == nullptr) {
    throw std::runtime_error("EstimateVerticalJumpBiasSegments received an incomplete request");
  }

  std::vector<VerticalJumpBiasSegmentEstimate> estimates;
  for (const auto &span : *request.spans) {
    std::vector<VerticalJumpBiasSegmentEstimate> span_estimates;
    if (request.config->enable_vertical_jump_segmented_bias &&
        request.body_z_diagnostics != nullptr &&
        !request.body_z_diagnostics->empty()) {
      span_estimates = EstimateSegmentedSpan(
        span,
        *request.jump_windows,
        *request.body_z_diagnostics,
        *request.config);
    } else {
      span_estimates.push_back(MakeFallbackEstimate(span, *request.jump_windows));
    }
    for (auto &estimate : span_estimates) {
      estimate.bias_key_index = estimates.size();
      estimates.push_back(std::move(estimate));
    }
  }
  return estimates;
}

}  // namespace offline_lc_minimal
