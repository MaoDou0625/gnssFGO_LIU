#include "offline_lc_minimal/core/VerticalJumpSpectralResponseEstimator.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <numeric>
#include <vector>

namespace offline_lc_minimal {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kMinReferenceEnergy = 1.0e-12;

struct SpectralSample {
  double time_s = 0.0;
  double acceleration_mps2 = 0.0;
};

struct WindowSpectrum {
  bool valid = false;
  double total_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  double band_30_60_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  double band_60_120_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
  double band_120_250_rms_mps2 = std::numeric_limits<double>::quiet_NaN();
};

struct SpectrumAccumulator {
  std::size_t count = 0;
  double total_rms_sum = 0.0;
  double band_30_60_rms_sum = 0.0;
  double band_60_120_rms_sum = 0.0;
  double band_120_250_rms_sum = 0.0;

  void Add(const WindowSpectrum &spectrum) {
    if (!spectrum.valid) {
      return;
    }
    ++count;
    total_rms_sum += spectrum.total_rms_mps2;
    band_30_60_rms_sum += spectrum.band_30_60_rms_mps2;
    band_60_120_rms_sum += spectrum.band_60_120_rms_mps2;
    band_120_250_rms_sum += spectrum.band_120_250_rms_mps2;
  }

  [[nodiscard]] WindowSpectrum Mean() const {
    WindowSpectrum result;
    if (count == 0U) {
      return result;
    }
    result.valid = true;
    const double inv_count = 1.0 / static_cast<double>(count);
    result.total_rms_mps2 = total_rms_sum * inv_count;
    result.band_30_60_rms_mps2 = band_30_60_rms_sum * inv_count;
    result.band_60_120_rms_mps2 = band_60_120_rms_sum * inv_count;
    result.band_120_250_rms_mps2 = band_120_250_rms_sum * inv_count;
    return result;
  }
};

struct ReferenceCandidate {
  double distance_s = 0.0;
  double center_time_s = 0.0;
};

bool IsFinite(const double value) {
  return std::isfinite(value);
}

double Clamp01(const double value) {
  return std::clamp(value, 0.0, 1.0);
}

double SafeRatio(const double numerator, const double denominator) {
  if (!IsFinite(numerator) || !IsFinite(denominator) || denominator <= kMinReferenceEnergy) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return numerator / denominator;
}

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s < right_end_s && right_start_s < left_end_s;
}

bool OverlapsExcludedSpan(
  const double start_time_s,
  const double end_time_s,
  const std::vector<VerticalJumpBiasSpanInput> *excluded_spans) {
  if (excluded_spans == nullptr) {
    return false;
  }
  for (const auto &span : *excluded_spans) {
    if (!IsFinite(span.start_time_s) || !IsFinite(span.end_time_s)) {
      continue;
    }
    if (IntervalsOverlap(start_time_s, end_time_s, span.start_time_s, span.end_time_s)) {
      return true;
    }
  }
  return false;
}

std::vector<SpectralSample> BuildFiniteSamples(
  const std::vector<BodyZSeedImuDiagnosticRow> &diagnostics) {
  std::vector<SpectralSample> samples;
  samples.reserve(diagnostics.size());
  for (const auto &row : diagnostics) {
    if (!IsFinite(row.time_s) || !IsFinite(row.body_z_acc_mps2)) {
      continue;
    }
    samples.push_back(SpectralSample{row.time_s, row.body_z_acc_mps2});
  }
  std::sort(samples.begin(), samples.end(), [](const SpectralSample &left, const SpectralSample &right) {
    return left.time_s < right.time_s;
  });
  return samples;
}

std::size_t LowerBoundSampleIndex(
  const std::vector<SpectralSample> &samples,
  const double time_s) {
  return static_cast<std::size_t>(
    std::lower_bound(
      samples.begin(),
      samples.end(),
      time_s,
      [](const SpectralSample &sample, const double value) {
        return sample.time_s < value;
      }) -
    samples.begin());
}

double BandRmsFromDft(
  const std::vector<double> &windowed_values,
  const double fs_hz,
  const double low_hz,
  const double high_hz,
  const double mean_window_square) {
  const std::size_t sample_count = windowed_values.size();
  if (sample_count < 2U || !IsFinite(fs_hz) || fs_hz <= 0.0 || mean_window_square <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double bin_hz = fs_hz / static_cast<double>(sample_count);
  const std::size_t k_min = static_cast<std::size_t>(
    std::max(1.0, std::ceil(low_hz / bin_hz)));
  const std::size_t k_max = static_cast<std::size_t>(
    std::floor(std::min(high_hz, 0.5 * fs_hz) / bin_hz));
  if (k_max < k_min) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double band_power = 0.0;
  const std::size_t bin_stride =
    std::max<std::size_t>(1U, static_cast<std::size_t>(std::llround(2.0 / bin_hz)));
  const double inv_n2 =
    1.0 / (static_cast<double>(sample_count) * static_cast<double>(sample_count));
  for (std::size_t k = k_min; k <= k_max; k += bin_stride) {
    double real = 0.0;
    double imag = 0.0;
    const double angle_step = -2.0 * kPi * static_cast<double>(k) /
                              static_cast<double>(sample_count);
    for (std::size_t n = 0; n < sample_count; ++n) {
      const double angle = angle_step * static_cast<double>(n);
      real += windowed_values[n] * std::cos(angle);
      imag += windowed_values[n] * std::sin(angle);
    }
    const bool is_nyquist =
      sample_count % 2U == 0U && k == sample_count / 2U;
    const double one_sided_weight = is_nyquist ? 1.0 : 2.0;
    band_power += static_cast<double>(bin_stride) * one_sided_weight *
                  (real * real + imag * imag) * inv_n2 /
                  mean_window_square;
  }
  return std::sqrt(std::max(0.0, band_power));
}

WindowSpectrum ComputeWindowSpectrum(
  const std::vector<SpectralSample> &samples,
  const double start_time_s,
  const double end_time_s) {
  WindowSpectrum result;
  if (end_time_s <= start_time_s) {
    return result;
  }
  const std::size_t begin = LowerBoundSampleIndex(samples, start_time_s);
  const std::size_t end = LowerBoundSampleIndex(samples, end_time_s);
  if (end <= begin + 7U) {
    return result;
  }

  const std::size_t count = end - begin;
  std::vector<double> values;
  values.reserve(count);
  double mean = 0.0;
  for (std::size_t index = begin; index < end; ++index) {
    values.push_back(samples[index].acceleration_mps2);
    mean += samples[index].acceleration_mps2;
  }
  mean /= static_cast<double>(count);

  double total_square = 0.0;
  std::vector<double> windowed_values;
  windowed_values.reserve(count);
  double window_square_sum = 0.0;
  for (std::size_t index = 0; index < count; ++index) {
    const double demeaned = values[index] - mean;
    total_square += demeaned * demeaned;
    const double window = count > 1U
                            ? 0.5 * (1.0 - std::cos(2.0 * kPi * static_cast<double>(index) /
                                                    static_cast<double>(count - 1U)))
                            : 1.0;
    windowed_values.push_back(demeaned * window);
    window_square_sum += window * window;
  }

  const double dt_s =
    (samples[end - 1U].time_s - samples[begin].time_s) / static_cast<double>(count - 1U);
  if (!IsFinite(dt_s) || dt_s <= 0.0 || window_square_sum <= 0.0) {
    return result;
  }
  const double fs_hz = 1.0 / dt_s;
  const double mean_window_square = window_square_sum / static_cast<double>(count);

  result.valid = true;
  result.total_rms_mps2 = std::sqrt(total_square / static_cast<double>(count));
  result.band_30_60_rms_mps2 =
    BandRmsFromDft(windowed_values, fs_hz, 30.0, 60.0, mean_window_square);
  result.band_60_120_rms_mps2 =
    BandRmsFromDft(windowed_values, fs_hz, 60.0, 120.0, mean_window_square);
  result.band_120_250_rms_mps2 =
    BandRmsFromDft(windowed_values, fs_hz, 120.0, 250.0, mean_window_square);
  result.valid = IsFinite(result.total_rms_mps2) &&
                 IsFinite(result.band_30_60_rms_mps2) &&
                 IsFinite(result.band_60_120_rms_mps2) &&
                 IsFinite(result.band_120_250_rms_mps2);
  return result;
}

void AccumulateTargetWindows(
  const std::vector<SpectralSample> &samples,
  const OfflineRunnerConfig &config,
  const double start_time_s,
  const double end_time_s,
  SpectrumAccumulator &accumulator) {
  const double half_window_s = 0.5 * config.vertical_jump_spectral_window_s;
  for (double center_time_s = start_time_s;
       center_time_s <= end_time_s + 1.0e-9;
       center_time_s += config.vertical_jump_spectral_stride_s) {
    accumulator.Add(ComputeWindowSpectrum(
      samples,
      center_time_s - half_window_s,
      center_time_s + half_window_s));
  }
}

void AccumulateReferenceWindows(
  const std::vector<SpectralSample> &samples,
  const OfflineRunnerConfig &config,
  const std::vector<VerticalJumpBiasSpanInput> *excluded_spans,
  const double start_time_s,
  const double end_time_s,
  SpectrumAccumulator &accumulator) {
  const double half_window_s = 0.5 * config.vertical_jump_spectral_window_s;
  const double reference_start_s =
    start_time_s - config.vertical_jump_spectral_reference_margin_s + half_window_s;
  const double reference_end_s =
    end_time_s + config.vertical_jump_spectral_reference_margin_s - half_window_s;
  for (double center_time_s = reference_start_s;
       center_time_s <= reference_end_s + 1.0e-9;
       center_time_s += config.vertical_jump_spectral_stride_s) {
    const double window_start_s = center_time_s - half_window_s;
    const double window_end_s = center_time_s + half_window_s;
    if (OverlapsExcludedSpan(window_start_s, window_end_s, excluded_spans)) {
      continue;
    }
    accumulator.Add(ComputeWindowSpectrum(samples, window_start_s, window_end_s));
  }
}

void AccumulateNearestReferenceWindows(
  const std::vector<SpectralSample> &samples,
  const OfflineRunnerConfig &config,
  const std::vector<VerticalJumpBiasSpanInput> *excluded_spans,
  const double start_time_s,
  const double end_time_s,
  SpectrumAccumulator &accumulator) {
  if (samples.empty()) {
    return;
  }
  std::vector<ReferenceCandidate> candidates;
  const double half_window_s = 0.5 * config.vertical_jump_spectral_window_s;
  const double first_center_s = samples.front().time_s + half_window_s;
  const double last_center_s = samples.back().time_s - half_window_s;
  for (double center_time_s = first_center_s;
       center_time_s <= last_center_s + 1.0e-9;
       center_time_s += config.vertical_jump_spectral_stride_s) {
    const double window_start_s = center_time_s - half_window_s;
    const double window_end_s = center_time_s + half_window_s;
    if (OverlapsExcludedSpan(window_start_s, window_end_s, excluded_spans)) {
      continue;
    }
    const double distance_s = center_time_s < start_time_s
                                ? start_time_s - center_time_s
                                : std::max(0.0, center_time_s - end_time_s);
    candidates.push_back(ReferenceCandidate{distance_s, center_time_s});
  }
  std::sort(candidates.begin(), candidates.end(), [](const auto &left, const auto &right) {
    return left.distance_s < right.distance_s;
  });

  for (const auto &candidate : candidates) {
    if (accumulator.count >= static_cast<std::size_t>(
                               config.vertical_jump_spectral_min_reference_window_count)) {
      break;
    }
    accumulator.Add(ComputeWindowSpectrum(
      samples,
      candidate.center_time_s - half_window_s,
      candidate.center_time_s + half_window_s));
  }
}

double MaxFiniteRatio(const std::initializer_list<double> values) {
  double result = std::numeric_limits<double>::quiet_NaN();
  for (const double value : values) {
    if (!IsFinite(value)) {
      continue;
    }
    if (!IsFinite(result) || value > result) {
      result = value;
    }
  }
  return result;
}

}  // namespace

VerticalJumpSpectralResponseEstimate EstimateVerticalJumpSpectralResponse(
  const VerticalJumpSpectralResponseRequest &request) {
  VerticalJumpSpectralResponseEstimate estimate;
  if (request.config == nullptr || !request.config->enable_vertical_jump_spectral_bias_relaxation) {
    return estimate;
  }
  estimate.enabled = true;
  estimate.skip_reason = "UNSET";
  if (request.body_z_diagnostics == nullptr || request.body_z_diagnostics->empty()) {
    estimate.skip_reason = "MISSING_DIAGNOSTICS";
    return estimate;
  }
  if (!IsFinite(request.start_time_s) || !IsFinite(request.end_time_s) ||
      request.end_time_s <= request.start_time_s) {
    estimate.skip_reason = "INVALID_SPAN";
    return estimate;
  }

  const std::vector<SpectralSample> samples = BuildFiniteSamples(*request.body_z_diagnostics);
  if (samples.empty()) {
    estimate.skip_reason = "MISSING_DIAGNOSTICS";
    return estimate;
  }

  SpectrumAccumulator target;
  AccumulateTargetWindows(
    samples,
    *request.config,
    request.start_time_s,
    request.end_time_s,
    target);
  estimate.target_window_count = target.count;
  if (target.count == 0U) {
    estimate.skip_reason = "MISSING_TARGET";
    return estimate;
  }

  SpectrumAccumulator reference;
  AccumulateReferenceWindows(
    samples,
    *request.config,
    request.excluded_spans,
    request.start_time_s,
    request.end_time_s,
    reference);
  bool used_nearest_reference = false;
  if (reference.count < static_cast<std::size_t>(
                          request.config->vertical_jump_spectral_min_reference_window_count)) {
    SpectrumAccumulator nearest_reference;
    AccumulateNearestReferenceWindows(
      samples,
      *request.config,
      request.excluded_spans,
      request.start_time_s,
      request.end_time_s,
      nearest_reference);
    if (nearest_reference.count >= static_cast<std::size_t>(
                                     request.config->vertical_jump_spectral_min_reference_window_count)) {
      reference = nearest_reference;
    }
    used_nearest_reference =
      reference.count >= static_cast<std::size_t>(
                           request.config->vertical_jump_spectral_min_reference_window_count);
  }
  estimate.reference_window_count = reference.count;
  if (reference.count < static_cast<std::size_t>(
                          request.config->vertical_jump_spectral_min_reference_window_count)) {
    estimate.skip_reason = "MISSING_REFERENCE";
    return estimate;
  }

  const WindowSpectrum target_mean = target.Mean();
  const WindowSpectrum reference_mean = reference.Mean();
  estimate.target_total_rms_mps2 = target_mean.total_rms_mps2;
  estimate.reference_total_rms_mps2 = reference_mean.total_rms_mps2;
  estimate.total_rms_ratio =
    SafeRatio(target_mean.total_rms_mps2, reference_mean.total_rms_mps2);
  estimate.band_30_60_rms_ratio =
    SafeRatio(target_mean.band_30_60_rms_mps2, reference_mean.band_30_60_rms_mps2);
  estimate.band_60_120_rms_ratio =
    SafeRatio(target_mean.band_60_120_rms_mps2, reference_mean.band_60_120_rms_mps2);
  estimate.band_120_250_rms_ratio =
    SafeRatio(target_mean.band_120_250_rms_mps2, reference_mean.band_120_250_rms_mps2);
  estimate.response_ratio = MaxFiniteRatio({
    estimate.total_rms_ratio,
    estimate.band_30_60_rms_ratio,
    estimate.band_60_120_rms_ratio,
    estimate.band_120_250_rms_ratio});
  if (!IsFinite(estimate.response_ratio)) {
    estimate.skip_reason = "INVALID_REFERENCE_ENERGY";
    return estimate;
  }

  const double denominator =
    request.config->vertical_jump_spectral_response_full_ratio -
    request.config->vertical_jump_spectral_response_trigger_ratio;
  estimate.score = Clamp01(
    (estimate.response_ratio -
     request.config->vertical_jump_spectral_response_trigger_ratio) /
    denominator);
  estimate.valid = true;
  estimate.skip_reason = used_nearest_reference ? "ADDED_NEAREST_REFERENCE" : "ADDED";
  return estimate;
}

double ComputeVerticalJumpSpectralEffectivePriorSigma(
  const OfflineRunnerConfig &config,
  const double base_prior_sigma_mps2,
  const VerticalJumpSpectralResponseEstimate &estimate) {
  if (!config.enable_vertical_jump_spectral_bias_relaxation ||
      !estimate.valid ||
      estimate.score <= 0.0 ||
      !IsFinite(base_prior_sigma_mps2)) {
    return base_prior_sigma_mps2;
  }
  const double max_sigma =
    std::max(base_prior_sigma_mps2, config.vertical_jump_spectral_bias_prior_max_sigma_mps2);
  return base_prior_sigma_mps2 +
         estimate.score * (max_sigma - base_prior_sigma_mps2);
}

}  // namespace offline_lc_minimal
