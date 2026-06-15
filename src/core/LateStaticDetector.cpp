#include "offline_lc_minimal/core/LateStaticDetector.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kLogFloor = 1.0e-12;

bool OverlapsInterval(
  const double start_a,
  const double end_a,
  const double start_b,
  const double end_b) {
  return std::isfinite(start_a) && std::isfinite(end_a) &&
         std::isfinite(start_b) && std::isfinite(end_b) &&
         start_a <= end_b + kTimeEpsilonS &&
         end_a >= start_b - kTimeEpsilonS;
}

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
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double clamped = std::clamp(percentile, 0.0, 1.0);
  const double position = clamped * static_cast<double>(values.size() - 1U);
  const auto lower = static_cast<std::size_t>(std::floor(position));
  const auto upper = static_cast<std::size_t>(std::ceil(position));
  if (lower == upper) {
    return values[lower];
  }
  const double alpha = position - static_cast<double>(lower);
  return (1.0 - alpha) * values[lower] + alpha * values[upper];
}

double Rms(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum_sq = 0.0;
  for (const double value : values) {
    sum_sq += value * value;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
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

double SafeLog10(const double value) {
  return std::log10(std::max(value, kLogFloor));
}

std::string Lowercase(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(), [](const unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return value;
}

bool IsRtkFixSample(
  const GnssSolutionSample &sample,
  const LateStaticDetectionRequest &request) {
  if (!sample.has_enu_position || !sample.enu_position_m.allFinite() ||
      sample.fix_type() != GnssFixType::kRtkFix) {
    return false;
  }
  return !request.should_use_rtkfix_sample || request.should_use_rtkfix_sample(sample);
}

bool OverlapsAnyOutage(
  const double start_time_s,
  const double end_time_s,
  const std::vector<RtkOutageWindowRow> *outage_windows) {
  if (outage_windows == nullptr) {
    return false;
  }
  return std::any_of(
    outage_windows->begin(),
    outage_windows->end(),
    [&](const RtkOutageWindowRow &window) {
      return OverlapsInterval(start_time_s, end_time_s, window.start_time_s, window.end_time_s);
    });
}

double DetectionStartTime(const LateStaticDetectionRequest &request) {
  if (request.processing_start_time_s > 0.0) {
    return request.processing_start_time_s;
  }
  double start = std::numeric_limits<double>::infinity();
  if (request.imu_samples != nullptr && !request.imu_samples->empty()) {
    start = std::min(start, request.imu_samples->front().time_s);
  }
  if (request.gnss_samples != nullptr && !request.gnss_samples->empty() &&
      request.corrected_time_s) {
    for (const auto &sample : *request.gnss_samples) {
      start = std::min(start, request.corrected_time_s(sample));
    }
  }
  return std::isfinite(start) ? start : 0.0;
}

double DetectionEndTime(const LateStaticDetectionRequest &request) {
  if (request.processing_end_time_s > 0.0) {
    return request.processing_end_time_s;
  }
  double end = -std::numeric_limits<double>::infinity();
  if (request.imu_samples != nullptr && !request.imu_samples->empty()) {
    end = std::max(end, request.imu_samples->back().time_s);
  }
  if (request.gnss_samples != nullptr && !request.gnss_samples->empty() &&
      request.corrected_time_s) {
    for (const auto &sample : *request.gnss_samples) {
      end = std::max(end, request.corrected_time_s(sample));
    }
  }
  return std::isfinite(end) ? end : 0.0;
}

struct OtsuResult {
  bool valid = false;
  double value_threshold = std::numeric_limits<double>::quiet_NaN();
  double log_threshold = std::numeric_limits<double>::quiet_NaN();
  std::size_t sample_count = 0;
  std::size_t static_side_count = 0;
  std::size_t dynamic_side_count = 0;
  double separation_score = std::numeric_limits<double>::quiet_NaN();
  std::string skip_reason = "UNSET";
};

OtsuResult EstimateLogOtsuThreshold(std::vector<double> values) {
  values.erase(
    std::remove_if(
      values.begin(),
      values.end(),
      [](const double value) {
        return !std::isfinite(value) || value < 0.0;
      }),
    values.end());

  OtsuResult result;
  result.sample_count = values.size();
  if (values.size() < 2U) {
    result.skip_reason = "INSUFFICIENT_SAMPLES";
    return result;
  }

  std::vector<double> logs;
  logs.reserve(values.size());
  for (const double value : values) {
    logs.push_back(SafeLog10(value));
  }
  std::sort(logs.begin(), logs.end());
  if (logs.front() == logs.back()) {
    result.skip_reason = "NON_SEPARABLE";
    return result;
  }

  const double total_sum = std::accumulate(logs.begin(), logs.end(), 0.0);
  const double total_mean = total_sum / static_cast<double>(logs.size());
  double total_variance = 0.0;
  for (const double value : logs) {
    const double delta = value - total_mean;
    total_variance += delta * delta;
  }
  total_variance /= static_cast<double>(logs.size());
  if (total_variance <= 0.0 || !std::isfinite(total_variance)) {
    result.skip_reason = "NON_SEPARABLE";
    return result;
  }

  double prefix_sum = 0.0;
  double best_between = -1.0;
  std::size_t best_split = 0U;
  for (std::size_t split = 1U; split < logs.size(); ++split) {
    prefix_sum += logs[split - 1U];
    if (logs[split - 1U] == logs[split]) {
      continue;
    }
    const double weight_static = static_cast<double>(split) / static_cast<double>(logs.size());
    const double weight_dynamic = 1.0 - weight_static;
    const double mean_static = prefix_sum / static_cast<double>(split);
    const double mean_dynamic =
      (total_sum - prefix_sum) / static_cast<double>(logs.size() - split);
    const double between =
      weight_static * weight_dynamic *
      (mean_static - mean_dynamic) * (mean_static - mean_dynamic);
    if (between > best_between) {
      best_between = between;
      best_split = split;
    }
  }

  if (best_split == 0U || best_between <= 0.0 || !std::isfinite(best_between)) {
    result.skip_reason = "NON_SEPARABLE";
    return result;
  }
  result.log_threshold = 0.5 * (logs[best_split - 1U] + logs[best_split]);
  result.value_threshold = std::pow(10.0, result.log_threshold);
  result.static_side_count = best_split;
  result.dynamic_side_count = logs.size() - best_split;
  result.separation_score = best_between / total_variance;
  result.valid = true;
  result.skip_reason = "OK";
  return result;
}

LateStaticThresholdDiagnosticRow MakeThresholdRow(
  const std::string &feature_name,
  const std::string &method,
  const OtsuResult &result) {
  LateStaticThresholdDiagnosticRow row;
  row.feature_name = feature_name;
  row.method = method;
  row.valid = result.valid;
  row.threshold_value = result.value_threshold;
  row.log_threshold_value = result.log_threshold;
  row.sample_count = result.sample_count;
  row.static_side_count = result.static_side_count;
  row.dynamic_side_count = result.dynamic_side_count;
  row.separation_score = result.separation_score;
  row.skip_reason = result.skip_reason;
  return row;
}

OtsuResult ApplyThresholdScale(OtsuResult result, const double scale) {
  if (result.valid) {
    result.value_threshold *= scale;
    result.log_threshold =
      result.value_threshold > 0.0 ? std::log10(result.value_threshold)
                                   : std::numeric_limits<double>::quiet_NaN();
  }
  return result;
}

LateStaticThresholdDiagnosticRow MakeFixedThresholdRow(
  const std::string &feature_name,
  const double threshold_value,
  const std::size_t sample_count) {
  LateStaticThresholdDiagnosticRow row;
  row.feature_name = feature_name;
  row.method = "fixed_config";
  row.valid = std::isfinite(threshold_value) && threshold_value > 0.0;
  row.threshold_value = threshold_value;
  row.log_threshold_value =
    threshold_value > 0.0 ? std::log10(threshold_value)
                          : std::numeric_limits<double>::quiet_NaN();
  row.sample_count = sample_count;
  row.static_side_count = sample_count;
  row.dynamic_side_count = 0U;
  row.skip_reason = row.valid ? "OK" : "INVALID_THRESHOLD";
  return row;
}

struct ActiveWindow {
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> up_medians;
  std::vector<double> up_lows;
  std::vector<double> up_highs;
  std::size_t feature_count = 0;
};

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
  row.vz_sigma_mps = config.late_static_vz_sigma_mps;
  row.up_sigma_m = config.late_static_up_sigma_m;
  row.valid = row.duration_s + kTimeEpsilonS >= config.late_static_min_duration_s &&
              std::isfinite(row.rtk_median_up_m);
  row.skip_reason = row.valid ? "OK" : "SHORT_OR_INVALID";
  return row;
}

}  // namespace

LateStaticFeatureExtractor::LateStaticFeatureExtractor(LateStaticDetectionRequest request)
    : request_(std::move(request)) {}

std::vector<LateStaticFeatureDiagnosticRow> LateStaticFeatureExtractor::Extract() const {
  if (request_.config == nullptr || request_.imu_samples == nullptr ||
      request_.gnss_samples == nullptr || !request_.corrected_time_s) {
    throw std::runtime_error("LateStaticFeatureExtractor received an incomplete request");
  }

  std::vector<LateStaticFeatureDiagnosticRow> rows;
  if (!request_.config->enable_late_static_detection) {
    return rows;
  }

  const double start_time_s = DetectionStartTime(request_);
  const double end_time_s = DetectionEndTime(request_);
  if (end_time_s <= start_time_s + request_.config->late_static_window_s) {
    return rows;
  }

  std::size_t window_index = 0U;
  for (double window_start = start_time_s;
       window_start + request_.config->late_static_window_s <= end_time_s + kTimeEpsilonS;
       window_start += request_.config->late_static_stride_s) {
    LateStaticFeatureDiagnosticRow row;
    row.window_index = window_index++;
    row.window_start_time_s = window_start;
    row.window_end_time_s = window_start + request_.config->late_static_window_s;
    row.window_center_time_s = 0.5 * (row.window_start_time_s + row.window_end_time_s);
    row.overlaps_initial_static =
      OverlapsInterval(
        row.window_start_time_s,
        row.window_end_time_s,
        request_.alignment_start_time_s,
        request_.alignment_end_time_s);
    row.overlaps_rtk_outage =
      OverlapsAnyOutage(
        row.window_start_time_s,
        row.window_end_time_s,
        request_.rtk_outage_windows);
    row.excluded_from_detection =
      (request_.config->late_static_exclude_initial_static && row.overlaps_initial_static) ||
      (request_.config->late_static_exclude_rtk_outage && row.overlaps_rtk_outage);

    std::vector<std::pair<double, Eigen::Vector3d>> rtk_positions;
    for (const auto &sample : *request_.gnss_samples) {
      const double time_s = request_.corrected_time_s(sample);
      if (time_s + kTimeEpsilonS < row.window_start_time_s ||
          time_s > row.window_end_time_s + kTimeEpsilonS ||
          !IsRtkFixSample(sample, request_)) {
        continue;
      }
      rtk_positions.push_back({time_s, sample.enu_position_m});
    }
    std::sort(
      rtk_positions.begin(),
      rtk_positions.end(),
      [](const auto &lhs, const auto &rhs) { return lhs.first < rhs.first; });
    row.rtkfix_sample_count = rtk_positions.size();
    if (row.rtkfix_sample_count <
        static_cast<std::size_t>(request_.config->late_static_min_rtkfix_samples)) {
      row.skip_reason = "INSUFFICIENT_RTKFIX";
      rows.push_back(row);
      continue;
    }

    std::vector<double> horizontal_speeds;
    horizontal_speeds.reserve(rtk_positions.size() - 1U);
    std::vector<double> ups;
    ups.reserve(rtk_positions.size());
    double min_e = std::numeric_limits<double>::infinity();
    double max_e = -std::numeric_limits<double>::infinity();
    double min_n = std::numeric_limits<double>::infinity();
    double max_n = -std::numeric_limits<double>::infinity();
    double min_u = std::numeric_limits<double>::infinity();
    double max_u = -std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < rtk_positions.size(); ++i) {
      const Eigen::Vector3d &p = rtk_positions[i].second;
      min_e = std::min(min_e, p.x());
      max_e = std::max(max_e, p.x());
      min_n = std::min(min_n, p.y());
      max_n = std::max(max_n, p.y());
      min_u = std::min(min_u, p.z());
      max_u = std::max(max_u, p.z());
      ups.push_back(p.z());
      if (i == 0U) {
        continue;
      }
      const double dt = rtk_positions[i].first - rtk_positions[i - 1U].first;
      if (dt <= kTimeEpsilonS) {
        continue;
      }
      const Eigen::Vector2d delta =
        (p - rtk_positions[i - 1U].second).head<2>();
      horizontal_speeds.push_back(delta.norm() / dt);
    }
    if (horizontal_speeds.empty()) {
      row.skip_reason = "INSUFFICIENT_RTK_SPEED_PAIRS";
      rows.push_back(row);
      continue;
    }
    row.rtk_horizontal_speed_rms_mps = Rms(horizontal_speeds);
    row.rtk_horizontal_range_m =
      std::hypot(max_e - min_e, max_n - min_n);
    row.rtk_up_median_m = Median(std::move(ups));
    row.rtk_up_range_m = max_u - min_u;

    std::vector<double> gyro_norms;
    std::vector<double> acc_norms;
    for (const auto &sample : *request_.imu_samples) {
      if (sample.time_s + kTimeEpsilonS < row.window_start_time_s ||
          sample.time_s > row.window_end_time_s + kTimeEpsilonS) {
        continue;
      }
      gyro_norms.push_back(sample.gyro_radps.norm());
      acc_norms.push_back(sample.accel_mps2.norm());
    }
    row.imu_sample_count = gyro_norms.size();
    if (gyro_norms.empty() || acc_norms.empty()) {
      row.skip_reason = "INSUFFICIENT_IMU";
      rows.push_back(row);
      continue;
    }
    row.imu_gyro_norm_rms_radps = Rms(gyro_norms);
    row.imu_gyro_norm_p95_radps = Percentile(gyro_norms, 0.95);
    row.imu_acc_norm_mean_mps2 = Mean(acc_norms);
    row.imu_acc_norm_std_mps2 = StdDevPopulation(acc_norms);
    row.log_rtk_horizontal_speed_rms = SafeLog10(row.rtk_horizontal_speed_rms_mps);
    row.log_rtk_horizontal_range = SafeLog10(row.rtk_horizontal_range_m);
    row.log_imu_gyro_norm_rms = SafeLog10(row.imu_gyro_norm_rms_radps);
    row.log_imu_gyro_norm_p95 = SafeLog10(row.imu_gyro_norm_p95_radps);
    row.valid_features = true;
    row.skip_reason = row.excluded_from_detection ? "EXCLUDED_FROM_DETECTION" : "OK";
    rows.push_back(row);
  }
  return rows;
}

DataDrivenStaticThresholdEstimator::DataDrivenStaticThresholdEstimator(
  const OfflineRunnerConfig &config)
    : config_(config) {}

LateStaticThresholdSet DataDrivenStaticThresholdEstimator::Estimate(
  const std::vector<LateStaticFeatureDiagnosticRow> &features) const {
  LateStaticThresholdSet thresholds;
  thresholds.diagnostics.reserve(5U);
  if (!config_.enable_late_static_detection) {
    return thresholds;
  }

  std::vector<double> rtk_speeds;
  std::vector<double> rtk_ranges;
  std::vector<double> gyro_rms_values;
  std::vector<double> gyro_p95_values;
  std::vector<double> acc_norm_std_values;
  for (const auto &row : features) {
    if (!row.valid_features || row.overlaps_rtk_outage) {
      continue;
    }
    rtk_speeds.push_back(row.rtk_horizontal_speed_rms_mps);
    rtk_ranges.push_back(row.rtk_horizontal_range_m);
    gyro_rms_values.push_back(row.imu_gyro_norm_rms_radps);
    gyro_p95_values.push_back(row.imu_gyro_norm_p95_radps);
    acc_norm_std_values.push_back(row.imu_acc_norm_std_mps2);
  }

  const std::string method = Lowercase(config_.late_static_threshold_method);
  const OtsuResult rtk_speed = EstimateLogOtsuThreshold(std::move(rtk_speeds));
  const OtsuResult rtk_range = EstimateLogOtsuThreshold(std::move(rtk_ranges));
  const OtsuResult gyro_rms =
    ApplyThresholdScale(
      EstimateLogOtsuThreshold(std::move(gyro_rms_values)),
      config_.late_static_gyro_threshold_scale);
  const OtsuResult gyro_p95 =
    ApplyThresholdScale(
      EstimateLogOtsuThreshold(std::move(gyro_p95_values)),
      config_.late_static_gyro_threshold_scale);
  thresholds.diagnostics.push_back(
    MakeThresholdRow("rtk_horizontal_speed_rms_mps", method, rtk_speed));
  thresholds.diagnostics.push_back(
    MakeThresholdRow("rtk_horizontal_range_m", method, rtk_range));
  thresholds.diagnostics.push_back(
    MakeThresholdRow("imu_gyro_norm_rms_radps", method, gyro_rms));
  thresholds.diagnostics.push_back(
    MakeThresholdRow("imu_gyro_norm_p95_radps", method, gyro_p95));
  thresholds.diagnostics.push_back(
    MakeFixedThresholdRow(
      "imu_acc_norm_std_mps2",
      config_.late_static_acc_norm_std_threshold_mps2,
      acc_norm_std_values.size()));
  thresholds.valid =
    rtk_speed.valid && rtk_range.valid && gyro_rms.valid && gyro_p95.valid &&
    thresholds.diagnostics.back().valid;
  thresholds.rtk_speed_rms_threshold_mps = rtk_speed.value_threshold;
  thresholds.rtk_horizontal_range_threshold_m = rtk_range.value_threshold;
  thresholds.gyro_rms_threshold_radps = gyro_rms.value_threshold;
  thresholds.gyro_p95_threshold_radps = gyro_p95.value_threshold;
  thresholds.acc_norm_std_threshold_mps2 =
    config_.late_static_acc_norm_std_threshold_mps2;
  return thresholds;
}

LateStaticWindowDetector::LateStaticWindowDetector(const OfflineRunnerConfig &config)
    : config_(config) {}

std::vector<LateStaticWindowRow> LateStaticWindowDetector::Detect(
  const LateStaticThresholdSet &thresholds,
  std::vector<LateStaticFeatureDiagnosticRow> *features) const {
  if (features == nullptr) {
    throw std::runtime_error("LateStaticWindowDetector received null feature diagnostics");
  }
  std::vector<LateStaticWindowRow> windows;
  if (!config_.enable_late_static_detection || !thresholds.valid) {
    return windows;
  }

  ActiveWindow active;
  bool has_active = false;
  auto close_active = [&]() {
    if (!has_active) {
      return;
    }
    LateStaticWindowRow row =
      MakeWindowRow(active, config_, windows.size());
    if (row.valid) {
      windows.push_back(std::move(row));
    }
    active = ActiveWindow{};
    has_active = false;
  };

  bool suppress_initial_static_prefix = false;
  for (auto &row : *features) {
    if (!row.valid_features) {
      row.pass_all = false;
      continue;
    }
    row.pass_rtk_speed_rms =
      row.rtk_horizontal_speed_rms_mps <= thresholds.rtk_speed_rms_threshold_mps;
    row.pass_rtk_range =
      row.rtk_horizontal_range_m <= thresholds.rtk_horizontal_range_threshold_m;
    row.pass_gyro_rms =
      row.imu_gyro_norm_rms_radps <= thresholds.gyro_rms_threshold_radps;
    row.pass_gyro_p95 =
      row.imu_gyro_norm_p95_radps <= thresholds.gyro_p95_threshold_radps;
    row.pass_acc_std =
      row.imu_acc_norm_std_mps2 <= thresholds.acc_norm_std_threshold_mps2;
    row.pass_all =
      row.pass_rtk_speed_rms && row.pass_rtk_range &&
      row.pass_gyro_rms && row.pass_gyro_p95 && row.pass_acc_std;

    const bool excluded_initial =
      config_.late_static_exclude_initial_static && row.overlaps_initial_static;
    const bool excluded_outage =
      config_.late_static_exclude_rtk_outage && row.overlaps_rtk_outage;

    if (excluded_initial) {
      suppress_initial_static_prefix = true;
      row.excluded_from_detection = true;
      row.pass_all = false;
      row.skip_reason = "EXCLUDED_FROM_DETECTION";
      close_active();
      continue;
    }

    if (excluded_outage) {
      suppress_initial_static_prefix = false;
      row.excluded_from_detection = true;
      row.pass_all = false;
      row.skip_reason = "EXCLUDED_FROM_DETECTION";
      close_active();
      continue;
    }

    if (suppress_initial_static_prefix) {
      if (row.pass_all) {
        row.excluded_from_detection = true;
        row.pass_all = false;
        row.skip_reason = "INITIAL_STATIC_PREFIX";
        close_active();
        continue;
      }
      suppress_initial_static_prefix = false;
    }

    if (row.excluded_from_detection) {
      row.pass_all = false;
      close_active();
      continue;
    }

    if (!row.pass_all) {
      close_active();
      continue;
    }

    if (!has_active ||
        row.window_start_time_s > active.end_time_s + config_.late_static_merge_gap_s) {
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

}  // namespace offline_lc_minimal
