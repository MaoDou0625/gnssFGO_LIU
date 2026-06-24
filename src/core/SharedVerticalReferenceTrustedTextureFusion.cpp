#include "offline_lc_minimal/core/SharedVerticalReferenceTrustedTextureFusion.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace offline_lc_minimal {
namespace {

constexpr double kDistanceEpsilonM = 1.0e-6;
constexpr double kConfidenceWindowM = 31.0;
constexpr double kConfidenceSmoothRadiusM = 2.0;
constexpr double kGateGoodConfidence = 0.95;
constexpr double kGateBadConfidence = 0.75;
constexpr double kCorrelationGoodOffset = 0.05;
constexpr double kCorrelationSpan = 0.40;
constexpr double kSourceStrengthScale = 0.04;
constexpr double kSoftSourceTemperature = 4.0e-5;
constexpr double kSourceScoreSmoothingRadiusM = 30.0;
constexpr double kFinalReferenceSmoothingRadiusM = 4.0;
constexpr double kTrustedObservationGapScaleM = 3.0;
constexpr double kTrustedObservationMinWeight = 0.25;
constexpr double kVelocitySourceLowpassRadiusM = 20.0;
constexpr double kVelocityDeltaScoreDistanceScaleM = 10.0;

struct MemberTextureProfile {
  std::string member_id;
  std::vector<double> filled_up_m;
  std::vector<double> filled_vz_mps;
  std::vector<double> lowpass_vz_mps;
  std::vector<double> height_gradient;
  std::vector<double> velocity_delta_mps_per_m;
  std::vector<double> observation_weight;
  std::vector<std::size_t> sample_count_by_bin;
  std::size_t first_valid = 0U;
  std::size_t last_valid = 0U;
  bool valid = false;
};

bool IsFinite(const double value) {
  return std::isfinite(value);
}

double Clamp01(const double value) {
  return std::clamp(value, 0.0, 1.0);
}

double Median(std::vector<double> values) {
  values.erase(
    std::remove_if(values.begin(), values.end(), [](const double value) {
      return !IsFinite(value);
    }),
    values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t mid = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  double median = values[mid];
  if ((values.size() % 2U) == 0U) {
    const auto lower_max =
      std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
    median = 0.5 * (median + *lower_max);
  }
  return median;
}

double Percentile(std::vector<double> values, const double q) {
  values.erase(
    std::remove_if(values.begin(), values.end(), [](const double value) {
      return !IsFinite(value);
    }),
    values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double clamped_q = Clamp01(q);
  const double position = clamped_q * static_cast<double>(values.size() - 1U);
  const std::size_t left = static_cast<std::size_t>(std::floor(position));
  const std::size_t right = static_cast<std::size_t>(std::ceil(position));
  if (left == right) {
    return values[left];
  }
  const double alpha = position - static_cast<double>(left);
  return (1.0 - alpha) * values[left] + alpha * values[right];
}

std::pair<std::size_t, std::size_t> FiniteRange(const std::vector<double> &values) {
  std::size_t first = values.size();
  std::size_t last = values.size();
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (IsFinite(values[index])) {
      if (first == values.size()) {
        first = index;
      }
      last = index;
    }
  }
  return {first, last};
}

std::vector<double> BuildObservationWeight(
  const std::vector<std::size_t> &sample_count_by_bin,
  const double grid_spacing_m) {
  const int unreachable = static_cast<int>(sample_count_by_bin.size()) + 1;
  std::vector<int> nearest_observed_distance_bins(sample_count_by_bin.size(), unreachable);
  int last_observed = -unreachable;
  for (std::size_t index = 0; index < sample_count_by_bin.size(); ++index) {
    if (sample_count_by_bin[index] > 0U) {
      last_observed = static_cast<int>(index);
    }
    nearest_observed_distance_bins[index] =
      std::min(nearest_observed_distance_bins[index], static_cast<int>(index) - last_observed);
  }
  last_observed = 2 * unreachable;
  for (int index = static_cast<int>(sample_count_by_bin.size()) - 1; index >= 0; --index) {
    if (sample_count_by_bin[static_cast<std::size_t>(index)] > 0U) {
      last_observed = index;
    }
    nearest_observed_distance_bins[static_cast<std::size_t>(index)] =
      std::min(
        nearest_observed_distance_bins[static_cast<std::size_t>(index)],
        last_observed - index);
  }

  std::vector<double> weights(sample_count_by_bin.size(), 0.0);
  for (std::size_t index = 0; index < sample_count_by_bin.size(); ++index) {
    const int distance_bins = nearest_observed_distance_bins[index];
    if (distance_bins >= unreachable) {
      weights[index] = 0.0;
      continue;
    }
    const double distance_m = static_cast<double>(distance_bins) * grid_spacing_m;
    weights[index] =
      std::exp(-0.5 * std::pow(distance_m / kTrustedObservationGapScaleM, 2.0));
  }
  return weights;
}

bool HasTrustedObservation(
  const MemberTextureProfile &profile,
  const std::size_t index) {
  return index < profile.observation_weight.size() &&
         profile.observation_weight[index] >= kTrustedObservationMinWeight;
}

std::vector<double> FillWithinFiniteRangeByInterpolation(
  const std::vector<double> &values,
  std::size_t *first_valid,
  std::size_t *last_valid) {
  std::vector<double> filled(values.size(), std::numeric_limits<double>::quiet_NaN());
  const auto [first, last] = FiniteRange(values);
  if (first == values.size()) {
    return filled;
  }
  *first_valid = first;
  *last_valid = last;
  std::vector<std::size_t> finite_indices;
  for (std::size_t index = first; index <= last; ++index) {
    if (IsFinite(values[index])) {
      finite_indices.push_back(index);
      filled[index] = values[index];
    }
  }
  for (std::size_t index = first; index <= last; ++index) {
    if (IsFinite(filled[index])) {
      continue;
    }
    const auto right_it =
      std::lower_bound(finite_indices.begin(), finite_indices.end(), index);
    if (right_it == finite_indices.begin() || right_it == finite_indices.end()) {
      continue;
    }
    const std::size_t right = *right_it;
    const std::size_t left = *(right_it - 1);
    const double alpha =
      static_cast<double>(index - left) /
      std::max(static_cast<double>(right - left), 1.0);
    filled[index] = (1.0 - alpha) * filled[left] + alpha * filled[right];
  }
  return filled;
}

std::vector<double> FillFiniteSeriesByInterpolation(std::vector<double> values) {
  std::vector<std::size_t> finite_indices;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (IsFinite(values[index])) {
      finite_indices.push_back(index);
    }
  }
  if (finite_indices.empty()) {
    throw std::runtime_error("trusted texture fusion has no finite shared height bins");
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (IsFinite(values[index])) {
      continue;
    }
    const auto right_it =
      std::lower_bound(finite_indices.begin(), finite_indices.end(), index);
    if (right_it == finite_indices.begin()) {
      values[index] = values[*right_it];
    } else if (right_it == finite_indices.end()) {
      values[index] = values[finite_indices.back()];
    } else {
      const std::size_t right = *right_it;
      const std::size_t left = *(right_it - 1);
      const double alpha =
        static_cast<double>(index - left) /
        std::max(static_cast<double>(right - left), 1.0);
      values[index] = (1.0 - alpha) * values[left] + alpha * values[right];
    }
  }
  return values;
}

std::vector<double> LocalLinearSmooth(
  const std::vector<double> &values,
  const double grid_spacing_m,
  const double radius_m) {
  const int radius_bins =
    std::max(1, static_cast<int>(std::ceil(radius_m / grid_spacing_m)));
  const double sigma_m = std::max(0.5 * radius_m, grid_spacing_m);
  std::vector<double> smoothed(values.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < values.size(); ++index) {
    double weight_sum = 0.0;
    double weighted_sum = 0.0;
    double weighted_x_sum = 0.0;
    double weighted_xx_sum = 0.0;
    double weighted_xy_sum = 0.0;
    const int begin = std::max(0, static_cast<int>(index) - radius_bins);
    const int end =
      std::min(static_cast<int>(values.size()) - 1, static_cast<int>(index) + radius_bins);
    for (int candidate = begin; candidate <= end; ++candidate) {
      const double value = values[static_cast<std::size_t>(candidate)];
      if (!IsFinite(value)) {
        continue;
      }
      const double x_m =
        (static_cast<double>(candidate) - static_cast<double>(index)) *
        grid_spacing_m;
      const double weight = std::exp(-0.5 * std::pow(x_m / sigma_m, 2.0));
      weight_sum += weight;
      weighted_sum += weight * value;
      weighted_x_sum += weight * x_m;
      weighted_xx_sum += weight * x_m * x_m;
      weighted_xy_sum += weight * x_m * value;
    }
    if (weight_sum <= 0.0) {
      continue;
    }
    const double determinant =
      weight_sum * weighted_xx_sum - weighted_x_sum * weighted_x_sum;
    smoothed[index] =
      determinant > kDistanceEpsilonM
        ? (weighted_sum * weighted_xx_sum -
           weighted_x_sum * weighted_xy_sum) / determinant
        : weighted_sum / weight_sum;
  }
  return smoothed;
}

std::vector<double> SmoothScalarSeries(
  const std::vector<double> &values,
  const double grid_spacing_m,
  const double radius_m) {
  const int radius_bins =
    std::max(1, static_cast<int>(std::ceil(radius_m / grid_spacing_m)));
  const double sigma_m = std::max(0.5 * radius_m, grid_spacing_m);
  std::vector<double> smoothed(values.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < values.size(); ++index) {
    double weight_sum = 0.0;
    double weighted_sum = 0.0;
    const int begin = std::max(0, static_cast<int>(index) - radius_bins);
    const int end =
      std::min(static_cast<int>(values.size()) - 1, static_cast<int>(index) + radius_bins);
    for (int candidate = begin; candidate <= end; ++candidate) {
      const double value = values[static_cast<std::size_t>(candidate)];
      if (!IsFinite(value)) {
        continue;
      }
      const double distance_m =
        std::abs(static_cast<double>(candidate) - static_cast<double>(index)) *
        grid_spacing_m;
      const double weight =
        std::exp(-0.5 * std::pow(distance_m / sigma_m, 2.0));
      weight_sum += weight;
      weighted_sum += weight * value;
    }
    if (weight_sum > 0.0) {
      smoothed[index] = weighted_sum / weight_sum;
    }
  }
  return smoothed;
}

std::vector<double> Gradient(
  const std::vector<double> &values,
  const double grid_spacing_m) {
  std::vector<double> gradient(values.size(), std::numeric_limits<double>::quiet_NaN());
  if (values.size() < 2U) {
    return gradient;
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    std::size_t left = index;
    while (left > 0U && !IsFinite(values[left])) {
      --left;
    }
    std::size_t right = index;
    while (right + 1U < values.size() && !IsFinite(values[right])) {
      ++right;
    }
    if (index > 0U && IsFinite(values[index - 1U])) {
      left = index - 1U;
    }
    if (index + 1U < values.size() && IsFinite(values[index + 1U])) {
      right = index + 1U;
    }
    if (left == right || !IsFinite(values[left]) || !IsFinite(values[right])) {
      continue;
    }
    gradient[index] =
      (values[right] - values[left]) /
      (std::max(static_cast<double>(right - left) * grid_spacing_m, kDistanceEpsilonM));
  }
  return gradient;
}

double LocalRms(
  const std::vector<double> &values,
  const std::size_t center,
  const int radius_bins) {
  double sum_sq = 0.0;
  std::size_t count = 0U;
  const int begin = std::max(0, static_cast<int>(center) - radius_bins);
  const int end =
    std::min(static_cast<int>(values.size()) - 1, static_cast<int>(center) + radius_bins);
  for (int index = begin; index <= end; ++index) {
    const double value = values[static_cast<std::size_t>(index)];
    if (!IsFinite(value)) {
      continue;
    }
    sum_sq += value * value;
    ++count;
  }
  if (count == 0U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::sqrt(sum_sq / static_cast<double>(count));
}

double LocalCorrelation(
  const std::vector<double> &a,
  const std::vector<double> &b,
  const std::size_t center,
  const int radius_bins) {
  double sum_a = 0.0;
  double sum_b = 0.0;
  std::size_t count = 0U;
  const int begin = std::max(0, static_cast<int>(center) - radius_bins);
  const int end =
    std::min(static_cast<int>(a.size()) - 1, static_cast<int>(center) + radius_bins);
  for (int index = begin; index <= end; ++index) {
    const double av = a[static_cast<std::size_t>(index)];
    const double bv = b[static_cast<std::size_t>(index)];
    if (!IsFinite(av) || !IsFinite(bv)) {
      continue;
    }
    sum_a += av;
    sum_b += bv;
    ++count;
  }
  if (count < 3U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double mean_a = sum_a / static_cast<double>(count);
  const double mean_b = sum_b / static_cast<double>(count);
  double covariance = 0.0;
  double variance_a = 0.0;
  double variance_b = 0.0;
  for (int index = begin; index <= end; ++index) {
    const double av = a[static_cast<std::size_t>(index)];
    const double bv = b[static_cast<std::size_t>(index)];
    if (!IsFinite(av) || !IsFinite(bv)) {
      continue;
    }
    const double da = av - mean_a;
    const double db = bv - mean_b;
    covariance += da * db;
    variance_a += da * da;
    variance_b += db * db;
  }
  const double denominator = std::sqrt(variance_a * variance_b);
  if (denominator <= 1.0e-18) {
    if (variance_a <= 1.0e-18 && variance_b <= 1.0e-18) {
      return 1.0;
    }
    return 0.0;
  }
  return std::clamp(covariance / denominator, -1.0, 1.0);
}

MemberTextureProfile BuildMemberProfile(
  SharedVerticalReferenceMemberHeightGrid member,
  const double grid_spacing_m) {
  if (member.up_by_bin.empty() ||
      member.vz_mps_by_bin.size() != member.up_by_bin.size() ||
      member.sample_count_by_bin.size() != member.up_by_bin.size()) {
    throw std::runtime_error(
      "trusted texture fusion member grid size mismatch: " + member.member_id);
  }
  MemberTextureProfile profile;
  profile.member_id = std::move(member.member_id);
  profile.sample_count_by_bin = std::move(member.sample_count_by_bin);
  profile.observation_weight =
    BuildObservationWeight(profile.sample_count_by_bin, grid_spacing_m);
  profile.filled_up_m =
    FillWithinFiniteRangeByInterpolation(member.up_by_bin, &profile.first_valid, &profile.last_valid);
  profile.valid = profile.first_valid < profile.filled_up_m.size();
  if (!profile.valid) {
    return profile;
  }
  profile.height_gradient = Gradient(profile.filled_up_m, grid_spacing_m);
  std::size_t vz_first_valid = 0U;
  std::size_t vz_last_valid = 0U;
  profile.filled_vz_mps =
    FillWithinFiniteRangeByInterpolation(member.vz_mps_by_bin, &vz_first_valid, &vz_last_valid);
  profile.lowpass_vz_mps =
    SmoothScalarSeries(profile.filled_vz_mps, grid_spacing_m, kVelocitySourceLowpassRadiusM);
  profile.velocity_delta_mps_per_m = Gradient(profile.lowpass_vz_mps, grid_spacing_m);
  return profile;
}

std::vector<double> BuildCommonSeries(
  const std::vector<MemberTextureProfile> &profiles,
  const std::size_t grid_count,
  const std::vector<double> MemberTextureProfile::*field) {
  std::vector<double> common(grid_count, std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < grid_count; ++index) {
    std::vector<double> values;
    for (const auto &profile : profiles) {
      const auto &series = profile.*field;
      if (profile.valid && HasTrustedObservation(profile, index) &&
          index < series.size() && IsFinite(series[index])) {
        values.push_back(series[index]);
      }
    }
    common[index] = Median(std::move(values));
  }
  return common;
}

std::vector<double> BuildVelocityDeltaConfidence(
  const std::vector<MemberTextureProfile> &profiles,
  const std::size_t grid_count,
  const double grid_spacing_m) {
  const int radius_bins =
    std::max(1, static_cast<int>(std::ceil(0.5 * kConfidenceWindowM / grid_spacing_m)));
  std::vector<double> correlations(grid_count, std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < grid_count; ++index) {
    std::vector<double> local_correlations;
    for (std::size_t a = 0; a < profiles.size(); ++a) {
      if (!profiles[a].valid) {
        continue;
      }
      for (std::size_t b = a + 1U; b < profiles.size(); ++b) {
        if (!profiles[b].valid) {
          continue;
        }
        const double r =
          LocalCorrelation(
            profiles[a].velocity_delta_mps_per_m,
            profiles[b].velocity_delta_mps_per_m,
            index,
            radius_bins);
        if (IsFinite(r)) {
          local_correlations.push_back(r);
        }
      }
    }
    correlations[index] = Median(std::move(local_correlations));
  }

  const double r_ref_raw = Percentile(correlations, 0.80);
  const double r_ref = IsFinite(r_ref_raw) ? r_ref_raw : 1.0;
  const double r_good = std::clamp(r_ref - kCorrelationGoodOffset, -1.0, 1.0);
  const double r_bad = std::clamp(r_good - kCorrelationSpan, -1.0, 1.0);
  std::vector<double> confidence(grid_count, 1.0);
  for (std::size_t index = 0; index < grid_count; ++index) {
    if (!IsFinite(correlations[index])) {
      confidence[index] = 1.0;
      continue;
    }
    confidence[index] =
      Clamp01((correlations[index] - r_bad) / std::max(r_good - r_bad, 1.0e-6));
  }
  const std::vector<double> smoothed =
    SmoothScalarSeries(confidence, grid_spacing_m, kConfidenceSmoothRadiusM);
  for (std::size_t index = 0; index < grid_count; ++index) {
    if (IsFinite(smoothed[index])) {
      confidence[index] = smoothed[index];
    }
  }
  return confidence;
}

std::vector<double> BuildSourceGate(
  const std::vector<double> &confidence,
  const double grid_spacing_m) {
  std::vector<double> gate(confidence.size(), 0.0);
  for (std::size_t index = 0; index < confidence.size(); ++index) {
    if (!IsFinite(confidence[index])) {
      continue;
    }
    gate[index] =
      Clamp01((kGateGoodConfidence - confidence[index]) /
              std::max(kGateGoodConfidence - kGateBadConfidence, 1.0e-6));
  }
  const std::vector<double> smoothed =
    SmoothScalarSeries(gate, grid_spacing_m, kConfidenceSmoothRadiusM);
  for (std::size_t index = 0; index < gate.size(); ++index) {
    gate[index] = IsFinite(smoothed[index]) ? Clamp01(smoothed[index]) : gate[index];
  }
  return gate;
}

double SourceScore(
  const MemberTextureProfile &profile,
  const std::size_t index,
  const int radius_bins) {
  if (!HasTrustedObservation(profile, index)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double local_delta_rms =
    LocalRms(profile.velocity_delta_mps_per_m, index, radius_bins);
  const double local_velocity_rms =
    LocalRms(profile.lowpass_vz_mps, index, radius_bins);
  if (!IsFinite(local_delta_rms) && !IsFinite(local_velocity_rms)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double delta_score =
    IsFinite(local_delta_rms)
      ? kVelocityDeltaScoreDistanceScaleM * local_delta_rms
      : 0.0;
  const double velocity_score = IsFinite(local_velocity_rms) ? local_velocity_rms : 0.0;
  return delta_score + velocity_score;
}

std::vector<double> SourceWeights(
  const std::vector<double> &scores,
  const double source_margin_min) {
  std::vector<std::size_t> valid_indices;
  valid_indices.reserve(scores.size());
  for (std::size_t index = 0; index < scores.size(); ++index) {
    if (IsFinite(scores[index])) {
      valid_indices.push_back(index);
    }
  }
  std::vector<double> weights(scores.size(), 0.0);
  if (valid_indices.empty()) {
    return weights;
  }
  if (valid_indices.size() == 1U) {
    weights[valid_indices.front()] = 1.0;
    return weights;
  }

  std::vector<double> valid_scores;
  valid_scores.reserve(valid_indices.size());
  for (const std::size_t index : valid_indices) {
    valid_scores.push_back(scores[index]);
  }
  const double best_score = *std::min_element(valid_scores.begin(), valid_scores.end());
  std::sort(valid_scores.begin(), valid_scores.end());
  const double second_score = valid_scores[1U];
  const double relative_margin =
    (second_score - best_score) / std::max(std::abs(second_score), 1.0e-9);
  const double strength =
    Clamp01((relative_margin - std::max(0.0, source_margin_min)) / kSourceStrengthScale);
  const double uniform_weight = 1.0 / static_cast<double>(valid_indices.size());

  double raw_sum = 0.0;
  for (const std::size_t index : valid_indices) {
    const double raw = std::exp(
      -(scores[index] - best_score) /
      std::max(kSoftSourceTemperature, 1.0e-9));
    weights[index] = raw;
    raw_sum += raw;
  }
  if (raw_sum <= 0.0) {
    for (const std::size_t index : valid_indices) {
      weights[index] = uniform_weight;
    }
    return weights;
  }
  for (const std::size_t index : valid_indices) {
    const double raw_weight = weights[index] / raw_sum;
    weights[index] = (1.0 - strength) * uniform_weight + strength * raw_weight;
  }
  return weights;
}

double WeightedMemberValue(
  const std::vector<MemberTextureProfile> &profiles,
  const std::vector<double> &weights,
  const std::size_t index,
  const std::vector<double> MemberTextureProfile::*field) {
  double weight_sum = 0.0;
  double weighted_sum = 0.0;
  for (std::size_t member_index = 0; member_index < profiles.size(); ++member_index) {
    const auto &series = profiles[member_index].*field;
    if (index >= series.size() || !IsFinite(series[index]) || weights[member_index] <= 0.0) {
      continue;
    }
    weight_sum += weights[member_index];
    weighted_sum += weights[member_index] * series[index];
  }
  if (weight_sum <= 0.0) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return weighted_sum / weight_sum;
}

std::pair<double, double> ActiveHeightEnvelope(
  const std::vector<MemberTextureProfile> &profiles,
  const std::size_t index) {
  double min_height = std::numeric_limits<double>::infinity();
  double max_height = -std::numeric_limits<double>::infinity();
  for (const auto &profile : profiles) {
    if (index >= profile.filled_up_m.size() || !IsFinite(profile.filled_up_m[index])) {
      continue;
    }
    min_height = std::min(min_height, profile.filled_up_m[index]);
    max_height = std::max(max_height, profile.filled_up_m[index]);
  }
  return {min_height, max_height};
}

std::vector<MemberTextureProfile> BuildProfiles(
  std::vector<SharedVerticalReferenceMemberHeightGrid> members,
  const double grid_spacing_m) {
  std::vector<MemberTextureProfile> profiles;
  profiles.reserve(members.size());
  for (auto &member : members) {
    profiles.push_back(BuildMemberProfile(std::move(member), grid_spacing_m));
  }
  profiles.erase(
    std::remove_if(profiles.begin(), profiles.end(), [](const MemberTextureProfile &profile) {
      return !profile.valid;
    }),
    profiles.end());
  if (profiles.size() < 2U) {
    throw std::runtime_error("trusted texture fusion requires at least two members with finite height bins");
  }
  return profiles;
}

std::vector<std::vector<double>> BuildSmoothedSourceScores(
  const std::vector<MemberTextureProfile> &profiles,
  const std::size_t grid_count,
  const int source_radius_bins,
  const double grid_spacing_m) {
  std::vector<std::vector<double>> scores_by_member;
  scores_by_member.reserve(profiles.size());
  for (const auto &profile : profiles) {
    std::vector<double> raw_scores(grid_count, std::numeric_limits<double>::quiet_NaN());
    for (std::size_t bin = 0; bin < grid_count; ++bin) {
      raw_scores[bin] =
        SourceScore(profile, bin, source_radius_bins);
    }
    std::vector<double> smoothed_scores =
      SmoothScalarSeries(raw_scores, grid_spacing_m, kSourceScoreSmoothingRadiusM);
    for (std::size_t bin = 0; bin < grid_count; ++bin) {
      if (!HasTrustedObservation(profile, bin)) {
        smoothed_scores[bin] = std::numeric_limits<double>::quiet_NaN();
      }
    }
    scores_by_member.push_back(std::move(smoothed_scores));
  }
  return scores_by_member;
}

void SmoothReferenceRows(
  std::vector<SharedVerticalReferenceRow> &rows,
  const double grid_spacing_m) {
  std::vector<double> reference_up_m;
  reference_up_m.reserve(rows.size());
  for (const auto &row : rows) {
    reference_up_m.push_back(row.reference_up_m);
  }
  const std::vector<double> smoothed =
    LocalLinearSmooth(reference_up_m, grid_spacing_m, kFinalReferenceSmoothingRadiusM);
  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (IsFinite(smoothed[index])) {
      rows[index].reference_up_m = smoothed[index];
    }
  }
}

}  // namespace

std::vector<SharedVerticalReferenceRow> ComposeTrustedTextureSharedVerticalReference(
  SharedVerticalReferenceTrustedTextureFusionRequest request) {
  if (!std::isfinite(request.grid_spacing_m) || request.grid_spacing_m <= 0.0 ||
      !std::isfinite(request.sigma_m) || request.sigma_m <= 0.0 ||
      !std::isfinite(request.lowpass_radius_m) || request.lowpass_radius_m <= 0.0) {
    throw std::runtime_error("trusted texture fusion grid spacing, sigma, and radius must be positive");
  }
  if (request.members.size() < 2U) {
    throw std::runtime_error("trusted texture fusion requires at least two members");
  }
  const std::size_t grid_count = request.members.front().up_by_bin.size();
  if (grid_count == 0U) {
    throw std::runtime_error("trusted texture fusion requires non-empty member grids");
  }
  for (const auto &member : request.members) {
    if (member.up_by_bin.size() != grid_count ||
        member.vz_mps_by_bin.size() != grid_count ||
        member.sample_count_by_bin.size() != grid_count) {
      throw std::runtime_error("trusted texture fusion member grids must have equal length");
    }
  }

  const std::vector<MemberTextureProfile> profiles =
    BuildProfiles(std::move(request.members), request.grid_spacing_m);
  std::vector<double> common_height =
    BuildCommonSeries(profiles, grid_count, &MemberTextureProfile::filled_up_m);
  std::vector<double> common_height_gradient =
    BuildCommonSeries(profiles, grid_count, &MemberTextureProfile::height_gradient);
  common_height = FillFiniteSeriesByInterpolation(std::move(common_height));
  common_height_gradient =
    FillFiniteSeriesByInterpolation(std::move(common_height_gradient));

  const std::vector<double> confidence =
    BuildVelocityDeltaConfidence(profiles, grid_count, request.grid_spacing_m);
  const std::vector<double> source_gate =
    BuildSourceGate(confidence, request.grid_spacing_m);
  const int source_radius_bins =
    std::max(1, static_cast<int>(std::ceil(request.lowpass_radius_m / request.grid_spacing_m)));
  const std::vector<std::vector<double>> source_scores_by_member =
    BuildSmoothedSourceScores(
      profiles,
      grid_count,
      source_radius_bins,
      request.grid_spacing_m);

  std::vector<double> shared_height_gradient(
    grid_count,
    std::numeric_limits<double>::quiet_NaN());
  std::vector<std::size_t> sample_counts(grid_count, 0U);
  for (std::size_t bin = 0; bin < grid_count; ++bin) {
    std::vector<double> scores(profiles.size(), std::numeric_limits<double>::quiet_NaN());
    std::size_t sample_count = 0U;
    for (std::size_t member_index = 0; member_index < profiles.size(); ++member_index) {
      const auto &profile = profiles[member_index];
      if (bin < profile.sample_count_by_bin.size()) {
        sample_count += profile.sample_count_by_bin[bin];
      }
      if (member_index < source_scores_by_member.size() &&
          bin < source_scores_by_member[member_index].size()) {
        scores[member_index] = source_scores_by_member[member_index][bin];
      }
    }
    const std::vector<double> source_weights =
      SourceWeights(scores, request.source_margin_min);
    const double trusted_height_gradient =
      WeightedMemberValue(
        profiles,
        source_weights,
        bin,
        &MemberTextureProfile::height_gradient);
    const double gate = source_gate[bin];
    shared_height_gradient[bin] = common_height_gradient[bin];
    if (IsFinite(trusted_height_gradient)) {
      shared_height_gradient[bin] =
        (1.0 - gate) * common_height_gradient[bin] +
        gate * trusted_height_gradient;
    }
    sample_counts[bin] = sample_count;
  }

  std::vector<double> shared_height = common_height;
  if (!shared_height.empty()) {
    shared_height.front() = common_height.front();
    for (std::size_t bin = 1; bin < shared_height.size(); ++bin) {
      if (IsFinite(shared_height_gradient[bin - 1U]) &&
          IsFinite(shared_height_gradient[bin])) {
        shared_height[bin] =
          shared_height[bin - 1U] +
          0.5 * (shared_height_gradient[bin - 1U] + shared_height_gradient[bin]) *
            request.grid_spacing_m;
      }
    }
    std::vector<double> height_offset;
    height_offset.reserve(shared_height.size());
    for (std::size_t bin = 0; bin < shared_height.size(); ++bin) {
      if (IsFinite(shared_height[bin]) && IsFinite(common_height[bin])) {
        height_offset.push_back(shared_height[bin] - common_height[bin]);
      }
    }
    const double offset_m = Median(std::move(height_offset));
    if (IsFinite(offset_m)) {
      for (double &value : shared_height) {
        if (IsFinite(value)) {
          value -= offset_m;
        }
      }
    }
  }

  std::vector<SharedVerticalReferenceRow> rows;
  rows.reserve(grid_count);
  for (std::size_t bin = 0; bin < grid_count; ++bin) {
    double reference_up_m = shared_height[bin];
    const auto [min_height, max_height] = ActiveHeightEnvelope(profiles, bin);
    if (IsFinite(min_height) && IsFinite(max_height) && min_height <= max_height) {
      reference_up_m = std::clamp(reference_up_m, min_height, max_height);
    }

    SharedVerticalReferenceRow row;
    row.s_m = static_cast<double>(bin) * request.grid_spacing_m;
    row.reference_up_m = reference_up_m;
    row.sigma_m = request.sigma_m;
    row.source =
      sample_counts[bin] > 0U ? "TRUSTED_TEXTURE_FUSION" : "TRUSTED_TEXTURE_FUSION_INTERPOLATED";
    row.rtk_weight = 0.0;
    row.nav_bridge_weight = 1.0;
    row.sample_count = sample_counts[bin];
    rows.push_back(row);
  }
  SmoothReferenceRows(rows, request.grid_spacing_m);
  return rows;
}

}  // namespace offline_lc_minimal
