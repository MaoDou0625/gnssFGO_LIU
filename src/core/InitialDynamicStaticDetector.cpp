#include "offline_lc_minimal/core/InitialDynamicStaticDetector.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>
#include <vector>

#include "offline_lc_minimal/core/LateStaticDetector.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

struct Thresholds {
  bool valid = false;
  double rtk_speed_rms_mps = std::numeric_limits<double>::quiet_NaN();
  double rtk_horizontal_range_m = std::numeric_limits<double>::quiet_NaN();
  double gyro_rms_radps = std::numeric_limits<double>::quiet_NaN();
  double gyro_p95_radps = std::numeric_limits<double>::quiet_NaN();
  double acc_std_mps2 = std::numeric_limits<double>::quiet_NaN();
};

struct ActiveWindow {
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> up_medians;
  std::vector<double> up_lows;
  std::vector<double> up_highs;
  std::size_t feature_count = 0;
};

double Median(std::vector<double> values) {
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

double Percentile(std::vector<double> values, const double percentile) {
  values.erase(
    std::remove_if(
      values.begin(),
      values.end(),
      [](const double value) { return !std::isfinite(value) || value < 0.0; }),
    values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double position =
    std::clamp(percentile, 0.0, 1.0) * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  const double alpha = position - static_cast<double>(lower);
  return (1.0 - alpha) * values[lower] + alpha * values[upper];
}

std::vector<LateStaticFeatureDiagnosticRow> ExtractFeatures(
  const InitialDynamicStaticDetectionRequest &request,
  const double start_time_s,
  const double end_time_s,
  const bool exclude_outage) {
  OfflineRunnerConfig feature_config = *request.config;
  feature_config.enable_late_static_detection = true;
  feature_config.late_static_exclude_initial_static = false;
  feature_config.late_static_exclude_rtk_outage = exclude_outage;

  LateStaticDetectionRequest feature_request;
  feature_request.config = &feature_config;
  feature_request.imu_samples = request.imu_samples;
  feature_request.gnss_samples = request.gnss_samples;
  feature_request.rtk_outage_windows = request.rtk_outage_windows;
  feature_request.processing_start_time_s = start_time_s;
  feature_request.processing_end_time_s = end_time_s;
  feature_request.alignment_start_time_s = request.alignment_start_time_s;
  feature_request.alignment_end_time_s = request.alignment_end_time_s;
  feature_request.should_use_rtkfix_sample = request.should_use_rtkfix_sample;
  feature_request.corrected_time_s = request.corrected_time_s;
  return LateStaticFeatureExtractor(std::move(feature_request)).Extract();
}

std::vector<double> Collect(
  const std::vector<LateStaticFeatureDiagnosticRow> &features,
  const double LateStaticFeatureDiagnosticRow::*field) {
  std::vector<double> values;
  values.reserve(features.size());
  for (const auto &row : features) {
    const double value = row.*field;
    if (row.valid_features && std::isfinite(value) && value >= 0.0) {
      values.push_back(value);
    }
  }
  return values;
}

LateStaticThresholdDiagnosticRow MakeScaledThresholdRow(
  const std::string &feature_name,
  const std::vector<double> &baseline_values,
  const double multiplier) {
  LateStaticThresholdDiagnosticRow row;
  row.feature_name = feature_name;
  row.method = "initial_dynamic_static_baseline_p95_scale";
  row.sample_count = baseline_values.size();
  row.static_side_count = baseline_values.size();
  row.dynamic_side_count = 0U;
  const double p95 = Percentile(baseline_values, 0.95);
  if (!std::isfinite(p95) || baseline_values.empty()) {
    row.valid = false;
    row.skip_reason = "INSUFFICIENT_BASELINE";
    return row;
  }
  row.threshold_value = std::max(p95 * multiplier, p95 + 1.0e-12);
  row.log_threshold_value =
    row.threshold_value > 0.0 ? std::log10(row.threshold_value)
                              : std::numeric_limits<double>::quiet_NaN();
  row.separation_score = multiplier;
  row.valid = true;
  row.skip_reason = "OK";
  return row;
}

Thresholds BuildThresholds(
  const OfflineRunnerConfig &config,
  const std::vector<LateStaticFeatureDiagnosticRow> &baseline_features,
  std::vector<LateStaticThresholdDiagnosticRow> &diagnostics) {
  diagnostics.clear();
  diagnostics.reserve(5U);
  const double multiplier = config.initial_dynamic_static_threshold_multiplier;
  diagnostics.push_back(
    MakeScaledThresholdRow(
      "rtk_horizontal_speed_rms_mps",
      Collect(baseline_features, &LateStaticFeatureDiagnosticRow::rtk_horizontal_speed_rms_mps),
      multiplier));
  diagnostics.push_back(
    MakeScaledThresholdRow(
      "rtk_horizontal_range_m",
      Collect(baseline_features, &LateStaticFeatureDiagnosticRow::rtk_horizontal_range_m),
      multiplier));
  diagnostics.push_back(
    MakeScaledThresholdRow(
      "imu_gyro_norm_rms_radps",
      Collect(baseline_features, &LateStaticFeatureDiagnosticRow::imu_gyro_norm_rms_radps),
      multiplier));
  diagnostics.push_back(
    MakeScaledThresholdRow(
      "imu_gyro_norm_p95_radps",
      Collect(baseline_features, &LateStaticFeatureDiagnosticRow::imu_gyro_norm_p95_radps),
      multiplier));
  diagnostics.push_back(
    MakeScaledThresholdRow(
      "imu_acc_norm_std_mps2",
      Collect(baseline_features, &LateStaticFeatureDiagnosticRow::imu_acc_norm_std_mps2),
      multiplier));

  Thresholds thresholds;
  thresholds.valid =
    std::all_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const LateStaticThresholdDiagnosticRow &row) { return row.valid; });
  if (!thresholds.valid) {
    return thresholds;
  }
  thresholds.rtk_speed_rms_mps = diagnostics[0].threshold_value;
  thresholds.rtk_horizontal_range_m = diagnostics[1].threshold_value;
  thresholds.gyro_rms_radps = diagnostics[2].threshold_value;
  thresholds.gyro_p95_radps = diagnostics[3].threshold_value;
  thresholds.acc_std_mps2 = diagnostics[4].threshold_value;
  return thresholds;
}

LateStaticWindowRow MakeWindowRow(
  const ActiveWindow &active,
  const OfflineRunnerConfig &config,
  const std::size_t index) {
  LateStaticWindowRow row;
  row.window_index = index;
  row.start_time_s = active.start_time_s;
  row.end_time_s = active.end_time_s;
  row.duration_s = active.end_time_s - active.start_time_s;
  row.feature_window_count = active.feature_count;
  row.rtk_median_up_m = Median(active.up_medians);
  if (!active.up_lows.empty() && !active.up_highs.empty()) {
    row.rtk_up_range_m =
      *std::max_element(active.up_highs.begin(), active.up_highs.end()) -
      *std::min_element(active.up_lows.begin(), active.up_lows.end());
  }
  row.vz_sigma_mps = config.initial_dynamic_static_vz_sigma_mps;
  row.up_sigma_m = std::numeric_limits<double>::quiet_NaN();
  row.height_hold_sigma_m = std::numeric_limits<double>::quiet_NaN();
  row.valid =
    row.duration_s + kTimeEpsilonS >= config.initial_dynamic_static_min_duration_s &&
    std::isfinite(row.rtk_median_up_m);
  row.skip_reason = row.valid ? "OK" : "SHORT_OR_INVALID";
  return row;
}

std::vector<LateStaticWindowRow> DetectWindows(
  const OfflineRunnerConfig &config,
  const Thresholds &thresholds,
  std::vector<LateStaticFeatureDiagnosticRow> &features) {
  std::vector<LateStaticWindowRow> windows;
  if (!thresholds.valid) {
    return windows;
  }

  ActiveWindow active;
  bool has_active = false;
  auto close_active = [&]() {
    if (!has_active) {
      return;
    }
    LateStaticWindowRow row = MakeWindowRow(active, config, windows.size());
    if (row.valid) {
      windows.push_back(std::move(row));
    }
    active = ActiveWindow{};
    has_active = false;
  };

  for (auto &row : features) {
    if (!row.valid_features || row.excluded_from_detection) {
      row.pass_all = false;
      if (row.excluded_from_detection) {
        row.skip_reason = "EXCLUDED_FROM_DETECTION";
      }
      close_active();
      continue;
    }
    row.pass_rtk_speed_rms =
      row.rtk_horizontal_speed_rms_mps <= thresholds.rtk_speed_rms_mps;
    row.pass_rtk_range =
      row.rtk_horizontal_range_m <= thresholds.rtk_horizontal_range_m;
    row.pass_gyro_rms =
      row.imu_gyro_norm_rms_radps <= thresholds.gyro_rms_radps;
    row.pass_gyro_p95 =
      row.imu_gyro_norm_p95_radps <= thresholds.gyro_p95_radps;
    row.pass_acc_std =
      row.imu_acc_norm_std_mps2 <= thresholds.acc_std_mps2;
    row.pass_all =
      row.pass_rtk_speed_rms && row.pass_rtk_range &&
      row.pass_gyro_rms && row.pass_gyro_p95 && row.pass_acc_std;
    if (!row.pass_all) {
      row.skip_reason = "DYNAMIC_EVIDENCE";
      close_active();
      continue;
    }
    row.skip_reason = "OK";

    if (!has_active ||
        row.window_start_time_s >
          active.end_time_s + config.initial_dynamic_static_merge_gap_s) {
      close_active();
      active.start_time_s = row.window_start_time_s;
      active.end_time_s = row.window_end_time_s;
      has_active = true;
    } else {
      active.end_time_s = std::max(active.end_time_s, row.window_end_time_s);
    }
    active.up_medians.push_back(row.rtk_up_median_m);
    active.up_lows.push_back(row.rtk_up_median_m - 0.5 * row.rtk_up_range_m);
    active.up_highs.push_back(row.rtk_up_median_m + 0.5 * row.rtk_up_range_m);
    ++active.feature_count;
  }
  close_active();
  return windows;
}

}  // namespace

InitialDynamicStaticDetector::InitialDynamicStaticDetector(
  InitialDynamicStaticDetectionRequest request)
    : request_(std::move(request)) {}

InitialDynamicStaticDetectionResult InitialDynamicStaticDetector::Detect() const {
  if (request_.config == nullptr || request_.imu_samples == nullptr ||
      request_.gnss_samples == nullptr || !request_.corrected_time_s ||
      !request_.should_use_rtkfix_sample) {
    throw std::runtime_error(
      "InitialDynamicStaticDetector received an incomplete request");
  }

  InitialDynamicStaticDetectionResult result;
  if (!request_.config->enable_initial_dynamic_static_detection) {
    return result;
  }
  if (request_.alignment_end_time_s <=
        request_.alignment_start_time_s + request_.config->late_static_window_s ||
      request_.processing_end_time_s <=
        request_.dynamic_start_time_s + request_.config->late_static_window_s) {
    return result;
  }

  const auto baseline_features =
    ExtractFeatures(
      request_,
      request_.alignment_start_time_s,
      request_.alignment_end_time_s,
      false);
  const double candidate_end_time_s =
    std::min(
      request_.processing_end_time_s,
      request_.dynamic_start_time_s +
        request_.config->initial_dynamic_static_search_duration_s);
  result.feature_diagnostics =
    ExtractFeatures(
      request_,
      request_.dynamic_start_time_s,
      candidate_end_time_s,
      true);
  const Thresholds thresholds =
    BuildThresholds(
      *request_.config,
      baseline_features,
      result.threshold_diagnostics);
  result.windows =
    DetectWindows(
      *request_.config,
      thresholds,
      result.feature_diagnostics);
  return result;
}

}  // namespace offline_lc_minimal
