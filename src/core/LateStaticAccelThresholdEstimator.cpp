#include "offline_lc_minimal/core/LateStaticAccelThresholdEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace offline_lc_minimal {
namespace {

constexpr char kFeatureName[] = "imu_acc_norm_std_mps2";

double SafeLog10(const double value) {
  return value > 0.0 ? std::log10(value) : std::numeric_limits<double>::quiet_NaN();
}

double Mean(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::accumulate(values.begin(), values.end(), 0.0) /
         static_cast<double>(values.size());
}

double StdDevPopulation(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double mean = Mean(values);
  double sum_sq = 0.0;
  for (const double value : values) {
    const double delta = value - mean;
    sum_sq += delta * delta;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

LateStaticAccelThresholdEstimate FixedConfigEstimate(
  const OfflineRunnerConfig &config,
  const std::string &method,
  const std::string &skip_reason,
  const std::size_t sample_count = 0U,
  const double reference_std_mps2 = std::numeric_limits<double>::quiet_NaN()) {
  LateStaticAccelThresholdEstimate estimate;
  estimate.threshold_mps2 = config.late_static_acc_norm_std_threshold_mps2;
  estimate.reference_std_mps2 = reference_std_mps2;
  estimate.sample_count = sample_count;
  estimate.method = method;
  estimate.valid = std::isfinite(estimate.threshold_mps2) && estimate.threshold_mps2 > 0.0;
  estimate.skip_reason = estimate.valid ? skip_reason : "INVALID_FIXED_THRESHOLD";
  return estimate;
}

}  // namespace

LateStaticAccelThresholdEstimate EstimateLateStaticAccelNormStdThreshold(
  const LateStaticAccelThresholdRequest &request) {
  if (request.config == nullptr) {
    LateStaticAccelThresholdEstimate estimate;
    estimate.method = "missing_config";
    estimate.skip_reason = "MISSING_CONFIG";
    return estimate;
  }

  const OfflineRunnerConfig &config = *request.config;
  if (!config.enable_late_static_initial_acc_norm_std_threshold) {
    return FixedConfigEstimate(config, "fixed_config", "OK");
  }
  if (request.imu_samples == nullptr || request.imu_samples->empty() ||
      request.alignment_end_time_s <= request.alignment_start_time_s) {
    return FixedConfigEstimate(config, "fixed_config_fallback", "NO_INITIAL_STATIC_IMU");
  }

  std::vector<double> accel_norms;
  accel_norms.reserve(request.imu_samples->size());
  for (const auto &sample : *request.imu_samples) {
    if (sample.time_s < request.alignment_start_time_s ||
        sample.time_s > request.alignment_end_time_s ||
        !sample.accel_mps2.allFinite()) {
      continue;
    }
    accel_norms.push_back(sample.accel_mps2.norm());
  }

  const auto sample_count = accel_norms.size();
  if (sample_count <
      static_cast<std::size_t>(config.late_static_initial_acc_norm_std_min_sample_count)) {
    return FixedConfigEstimate(
      config,
      "fixed_config_fallback",
      "INSUFFICIENT_INITIAL_STATIC_IMU",
      sample_count);
  }

  const double reference_std_mps2 = StdDevPopulation(accel_norms);
  if (!std::isfinite(reference_std_mps2) || reference_std_mps2 <= 0.0) {
    return FixedConfigEstimate(
      config,
      "fixed_config_fallback",
      "INVALID_INITIAL_STATIC_IMU_STD",
      sample_count,
      reference_std_mps2);
  }

  LateStaticAccelThresholdEstimate estimate;
  estimate.reference_std_mps2 = reference_std_mps2;
  estimate.sample_count = sample_count;
  estimate.method = "initial_static_scaled";
  estimate.threshold_mps2 =
    std::max(
      config.late_static_acc_norm_std_threshold_mps2,
      reference_std_mps2 * config.late_static_initial_acc_norm_std_scale);
  estimate.valid = std::isfinite(estimate.threshold_mps2) && estimate.threshold_mps2 > 0.0;
  estimate.skip_reason = estimate.valid ? "OK" : "INVALID_INITIAL_STATIC_THRESHOLD";
  return estimate;
}

LateStaticThresholdDiagnosticRow MakeLateStaticAccelNormStdThresholdDiagnostic(
  const LateStaticAccelThresholdEstimate &estimate) {
  LateStaticThresholdDiagnosticRow row;
  row.feature_name = kFeatureName;
  row.method = estimate.method;
  row.valid = estimate.valid;
  row.threshold_value = estimate.threshold_mps2;
  row.log_threshold_value = SafeLog10(estimate.threshold_mps2);
  row.sample_count = estimate.sample_count;
  row.static_side_count = estimate.sample_count;
  row.dynamic_side_count = 0U;
  row.separation_score = estimate.reference_std_mps2;
  row.skip_reason = estimate.skip_reason;
  return row;
}

}  // namespace offline_lc_minimal
