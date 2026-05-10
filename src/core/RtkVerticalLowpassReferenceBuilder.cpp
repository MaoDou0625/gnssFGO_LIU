#include "offline_lc_minimal/core/RtkVerticalLowpassReferenceBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kMinSigmaM = 1.0e-6;

struct CandidateSample {
  std::size_t sample_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double up_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
};

bool IsAllowedFixType(const OfflineRunnerConfig &config, const GnssFixType fix_type) {
  if (!config.drop_non_rtkfix) {
    return true;
  }
  return fix_type == GnssFixType::kRtkFix;
}

bool PassesGnssQualityFiltersWithoutCounters(
  const GnssSolutionSample &sample,
  const OfflineRunnerConfig &config) {
  if (!sample.has_valid_position() || !sample.has_enu_position) {
    return false;
  }
  if (config.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config.required_best_sol_status_code) {
    return false;
  }
  if (config.drop_no_solution && sample.fix_type() == GnssFixType::kNoSolution) {
    return false;
  }
  if (!IsAllowedFixType(config, sample.fix_type())) {
    return false;
  }
  if (config.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return true;
}

double WeightedMedian(std::vector<std::pair<double, double>> value_weights) {
  if (value_weights.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(
    value_weights.begin(),
    value_weights.end(),
    [](const auto &lhs, const auto &rhs) {
      return lhs.first < rhs.first;
    });
  double total_weight = 0.0;
  for (const auto &value_weight : value_weights) {
    total_weight += value_weight.second;
  }
  if (!(total_weight > 0.0)) {
    return value_weights[value_weights.size() / 2U].first;
  }
  const double half_weight = 0.5 * total_weight;
  double accumulated_weight = 0.0;
  for (const auto &[value, weight] : value_weights) {
    accumulated_weight += weight;
    if (accumulated_weight >= half_weight) {
      return value;
    }
  }
  return value_weights.back().first;
}

double RobustHuberMean(
  const std::vector<const CandidateSample *> &window_samples,
  const double huber_sigma_m) {
  std::vector<std::pair<double, double>> value_weights;
  value_weights.reserve(window_samples.size());
  for (const CandidateSample *sample : window_samples) {
    const double sigma = std::max(std::abs(sample->sigma_u_m), kMinSigmaM);
    value_weights.emplace_back(sample->up_m, 1.0 / (sigma * sigma));
  }
  const double center = WeightedMedian(value_weights);
  if (!std::isfinite(center)) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double weighted_sum = 0.0;
  double total_weight = 0.0;
  for (const CandidateSample *sample : window_samples) {
    const double sigma = std::max(std::abs(sample->sigma_u_m), kMinSigmaM);
    double weight = 1.0 / (sigma * sigma);
    const double abs_residual = std::abs(sample->up_m - center);
    if (abs_residual > huber_sigma_m) {
      weight *= huber_sigma_m / abs_residual;
    }
    weighted_sum += weight * sample->up_m;
    total_weight += weight;
  }
  if (!(total_weight > 0.0)) {
    return center;
  }
  return weighted_sum / total_weight;
}

double ComputeStdDev(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                      static_cast<double>(values.size());
  double variance = 0.0;
  for (const double value : values) {
    const double centered = value - mean;
    variance += centered * centered;
  }
  return std::sqrt(variance / static_cast<double>(values.size()));
}

}  // namespace

RtkVerticalLowpassReferenceBuilder::RtkVerticalLowpassReferenceBuilder(
    RtkVerticalLowpassReferenceBuildRequest request)
    : request_(std::move(request)) {}

void RtkVerticalLowpassReferenceBuilder::Validate() const {
  if (request_.config == nullptr || request_.gnss_samples == nullptr ||
      !request_.is_within_imu_coverage ||
      !request_.corrected_time_s || !request_.clamped_sigma_m) {
    throw std::runtime_error("RtkVerticalLowpassReferenceBuilder received an incomplete request");
  }
}

RtkVerticalLowpassReferenceBuildResult RtkVerticalLowpassReferenceBuilder::Build() const {
  Validate();

  RtkVerticalLowpassReferenceBuildResult result;
  result.rows.resize(request_.gnss_samples->size());
  for (std::size_t index = 0; index < result.rows.size(); ++index) {
    result.rows[index].sample_index = index;
    if (index < request_.gnss_samples->size()) {
      const auto &sample = (*request_.gnss_samples)[index];
      result.rows[index].time_s = request_.corrected_time_s(sample);
      result.rows[index].raw_up_m = sample.enu_position_m.z();
    }
    result.rows[index].lowpass_valid = false;
    result.rows[index].skip_reason = "NOT_EVALUATED";
  }

  std::vector<CandidateSample> candidates;
  candidates.reserve(request_.gnss_samples->size());
  for (std::size_t sample_index = request_.first_sample_index;
       sample_index < request_.gnss_samples->size();
       ++sample_index) {
    const GnssSolutionSample &sample = (*request_.gnss_samples)[sample_index];
    RtkVerticalLowpassReferenceRow &row = result.rows[sample_index];
    const double corrected_time_s = request_.corrected_time_s(sample);
    row.time_s = corrected_time_s;
    row.raw_up_m = sample.enu_position_m.z();

    if (!PassesGnssQualityFiltersWithoutCounters(sample, *request_.config)) {
      row.skip_reason = "SAMPLE_REJECTED";
      continue;
    }
    if (!request_.is_within_imu_coverage(corrected_time_s)) {
      row.skip_reason = "OUT_OF_IMU_COVERAGE";
      continue;
    }
    const Eigen::Vector3d sigma_m = request_.clamped_sigma_m(sample);
    if (!std::isfinite(corrected_time_s) || !std::isfinite(sample.enu_position_m.z()) ||
        !std::isfinite(sigma_m.z()) || sigma_m.z() <= 0.0) {
      row.skip_reason = "INVALID_SAMPLE";
      continue;
    }
    candidates.push_back(CandidateSample{
      sample_index,
      corrected_time_s,
      sample.enu_position_m.z(),
      sigma_m.z()});
  }

  std::sort(
    candidates.begin(),
    candidates.end(),
    [](const CandidateSample &lhs, const CandidateSample &rhs) {
      return lhs.time_s < rhs.time_s;
    });

  const double half_window_s = 0.5 * request_.config->rtk_vertical_lowpass_window_s;
  std::vector<double> highpass_values;
  std::size_t left = 0;
  std::size_t right = 0;
  for (std::size_t center = 0; center < candidates.size(); ++center) {
    const double center_time_s = candidates[center].time_s;
    while (left < candidates.size() &&
           candidates[left].time_s < center_time_s - half_window_s) {
      ++left;
    }
    while (right < candidates.size() &&
           candidates[right].time_s <= center_time_s + half_window_s) {
      ++right;
    }

    std::vector<const CandidateSample *> window_samples;
    window_samples.reserve(right - left);
    for (std::size_t index = left; index < right; ++index) {
      window_samples.push_back(&candidates[index]);
    }

    RtkVerticalLowpassReferenceRow &row = result.rows[candidates[center].sample_index];
    row.window_sample_count = static_cast<int>(window_samples.size());
    if (window_samples.size() <
        static_cast<std::size_t>(request_.config->rtk_vertical_lowpass_min_sample_count)) {
      row.lowpass_valid = false;
      row.skip_reason = "INSUFFICIENT_WINDOW_SAMPLES";
      continue;
    }

    const double lowpass_up_m =
      RobustHuberMean(window_samples, request_.config->rtk_vertical_lowpass_huber_sigma_m);
    if (!std::isfinite(lowpass_up_m)) {
      row.lowpass_valid = false;
      row.skip_reason = "INVALID_LOWPASS_REFERENCE";
      continue;
    }

    row.lowpass_up_m = lowpass_up_m;
    row.raw_minus_lowpass_m = row.raw_up_m - row.lowpass_up_m;
    row.lowpass_valid = true;
    row.skip_reason = "NONE";
    ++result.valid_count;
    highpass_values.push_back(row.raw_minus_lowpass_m);
    result.raw_minus_lowpass_max_abs_m =
      std::isfinite(result.raw_minus_lowpass_max_abs_m)
        ? std::max(result.raw_minus_lowpass_max_abs_m, std::abs(row.raw_minus_lowpass_m))
        : std::abs(row.raw_minus_lowpass_m);
  }

  result.raw_minus_lowpass_std_m = ComputeStdDev(highpass_values);
  return result;
}

}  // namespace offline_lc_minimal
