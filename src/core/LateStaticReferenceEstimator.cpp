#include "offline_lc_minimal/core/LateStaticReferenceEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kHuberC = 1.345;
constexpr double kMinScaleM = 1.0e-9;
constexpr int kMaxHuberIterations = 20;

bool TimeInWindow(const double time_s, const LateStaticWindowRow &window) {
  return std::isfinite(time_s) &&
         time_s + kTimeEpsilonS >= window.start_time_s &&
         time_s <= window.end_time_s + kTimeEpsilonS;
}

bool IsUsableRtkFix(
  const GnssSolutionSample &sample,
  const LateStaticReferenceEstimationRequest &request) {
  if (!sample.has_enu_position || !sample.enu_position_m.allFinite() ||
      sample.fix_type() != GnssFixType::kRtkFix) {
    return false;
  }
  return !request.should_use_rtkfix_sample || request.should_use_rtkfix_sample(sample);
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t mid = values.size() / 2U;
  if ((values.size() % 2U) != 0U) {
    return values[mid];
  }
  return 0.5 * (values[mid - 1U] + values[mid]);
}

double HuberMean(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double location = Median(values);
  std::vector<double> absolute_deviations;
  absolute_deviations.reserve(values.size());
  for (const double value : values) {
    absolute_deviations.push_back(std::abs(value - location));
  }
  const double scale = 1.4826 * Median(std::move(absolute_deviations));
  if (!std::isfinite(scale) || scale <= kMinScaleM) {
    return location;
  }

  for (int iteration = 0; iteration < kMaxHuberIterations; ++iteration) {
    double weighted_sum = 0.0;
    double weight_sum = 0.0;
    for (const double value : values) {
      const double standardized_residual = (value - location) / scale;
      const double abs_residual = std::abs(standardized_residual);
      const double weight =
        abs_residual <= kHuberC ? 1.0 : kHuberC / abs_residual;
      weighted_sum += weight * value;
      weight_sum += weight;
    }
    if (weight_sum <= 0.0 || !std::isfinite(weight_sum)) {
      break;
    }
    const double next_location = weighted_sum / weight_sum;
    if (std::abs(next_location - location) <= 1.0e-12) {
      location = next_location;
      break;
    }
    location = next_location;
  }
  return location;
}

std::vector<double> CollectRtkUpSamples(
  const LateStaticWindowRow &window,
  const LateStaticReferenceEstimationRequest &request) {
  std::vector<double> up_samples;
  for (const auto &sample : *request.gnss_samples) {
    if (!IsUsableRtkFix(sample, request)) {
      continue;
    }
    const double time_s =
      request.corrected_time_s ? request.corrected_time_s(sample) : sample.time_s;
    if (!TimeInWindow(time_s, window)) {
      continue;
    }
    up_samples.push_back(sample.enu_position_m.z());
  }
  return up_samples;
}

}  // namespace

LateStaticReferenceEstimator::LateStaticReferenceEstimator(
  LateStaticReferenceEstimationRequest request)
    : request_(std::move(request)) {}

void LateStaticReferenceEstimator::Estimate() const {
  if (request_.gnss_samples == nullptr || request_.windows == nullptr) {
    throw std::runtime_error("LateStaticReferenceEstimator received an incomplete request");
  }

  for (auto &window : *request_.windows) {
    if (!window.valid) {
      continue;
    }
    std::vector<double> up_samples = CollectRtkUpSamples(window, request_);
    if (up_samples.empty()) {
      window.valid = false;
      window.skip_reason = "NO_RTK_REFERENCE_SAMPLES";
      continue;
    }
    window.rtk_median_up_m = HuberMean(up_samples);
    const auto [min_it, max_it] =
      std::minmax_element(up_samples.begin(), up_samples.end());
    window.rtk_up_range_m = *max_it - *min_it;
  }
}

}  // namespace offline_lc_minimal
