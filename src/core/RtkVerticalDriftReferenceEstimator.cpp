#include "offline_lc_minimal/core/RtkVerticalDriftReferenceEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/NavState.h>

#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kInterpolatorQcVariance = 10000.0;
constexpr double kTiny = 1.0e-12;

template <typename T>
T Clamp(const T value, const T low, const T high) {
  return std::min(std::max(value, low), high);
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t mid = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  double median = values[mid];
  if ((values.size() % 2U) == 0U) {
    const auto max_lower =
      std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
    median = 0.5 * (median + *max_lower);
  }
  return median;
}

double StdDev(const std::vector<double> &values) {
  if (values.size() < 2U) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) /
                      static_cast<double>(values.size());
  double sum_sq = 0.0;
  for (const double value : values) {
    const double delta = value - mean;
    sum_sq += delta * delta;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size() - 1U));
}

struct Candidate {
  std::size_t row_index = 0;
  double time_s = 0.0;
  double residual_m = 0.0;
  bool static_window = false;
};

std::optional<double> NavReferenceUpM(
  const RtkVerticalDriftReferenceEstimateRequest &request,
  const double corrected_time_s,
  const bool static_window,
  std::string *skip_reason) {
  if (static_window) {
    return request.static_reference_up_m;
  }
  if (request.optimized_values == nullptr) {
    *skip_reason = "missing_optimized_values";
    return std::nullopt;
  }
  if (!request.is_within_imu_coverage || !request.is_within_imu_coverage(corrected_time_s)) {
    *skip_reason = "out_of_imu_coverage";
    return std::nullopt;
  }
  if (!request.find_state_for_time_s) {
    *skip_reason = "missing_state_sync";
    return std::nullopt;
  }
  const StateMeasSyncResult sync = request.find_state_for_time_s(corrected_time_s);
  if (sync.status == StateMeasSyncStatus::kSynchronizedI ||
      sync.status == StateMeasSyncStatus::kSynchronizedJ) {
    const std::size_t state_index =
      sync.status == StateMeasSyncStatus::kSynchronizedI ? sync.key_index_i : sync.key_index_j;
    const gtsam::Key x_key = symbol::X(state_index);
    if (!request.optimized_values->exists(x_key)) {
      *skip_reason = "missing_pose_key";
      return std::nullopt;
    }
    return request.optimized_values->at<gtsam::Pose3>(x_key).translation().z();
  }
  if (sync.status == StateMeasSyncStatus::kInterpolated) {
    const gtsam::Key xi_key = symbol::X(sync.key_index_i);
    const gtsam::Key vi_key = symbol::V(sync.key_index_i);
    const gtsam::Key wi_key = symbol::W(sync.key_index_i);
    const gtsam::Key xj_key = symbol::X(sync.key_index_j);
    const gtsam::Key vj_key = symbol::V(sync.key_index_j);
    const gtsam::Key wj_key = symbol::W(sync.key_index_j);
    if (!request.optimized_values->exists(xi_key) ||
        !request.optimized_values->exists(vi_key) ||
        !request.optimized_values->exists(wi_key) ||
        !request.optimized_values->exists(xj_key) ||
        !request.optimized_values->exists(vj_key) ||
        !request.optimized_values->exists(wj_key)) {
      *skip_reason = "missing_interpolation_key";
      return std::nullopt;
    }
    const auto qc_model =
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance));
    gp::GPWNOJInterpolator interpolator(
      qc_model,
      sync.timestamp_j_s - sync.timestamp_i_s,
      sync.duration_from_state_i_s);
    const gtsam::Pose3 pose = interpolator.InterpolatePose(
      request.optimized_values->at<gtsam::Pose3>(xi_key),
      request.optimized_values->at<gtsam::Vector3>(vi_key),
      request.optimized_values->at<gtsam::Vector3>(wi_key),
      request.optimized_values->at<gtsam::Pose3>(xj_key),
      request.optimized_values->at<gtsam::Vector3>(vj_key),
      request.optimized_values->at<gtsam::Vector3>(wj_key));
    return pose.translation().z();
  }
  *skip_reason = "unsupported_sync_status";
  return std::nullopt;
}

void EstimateOuDrift(
  const OfflineRunnerConfig &config,
  const std::vector<Candidate> &candidates,
  const double constant_bias_m,
  std::vector<RtkVerticalDriftReferenceDiagnosticRow> *profile) {
  const std::size_t count = candidates.size();
  if (count == 0U) {
    return;
  }

  std::vector<double> x_pred(count, 0.0);
  std::vector<double> p_pred(count, config.rtk_vertical_drift_sigma_m * config.rtk_vertical_drift_sigma_m);
  std::vector<double> x_filt(count, 0.0);
  std::vector<double> p_filt(count, config.rtk_vertical_drift_sigma_m * config.rtk_vertical_drift_sigma_m);
  std::vector<double> phi_to_next(count, 0.0);

  double prev_x = 0.0;
  double prev_p = config.rtk_vertical_drift_sigma_m * config.rtk_vertical_drift_sigma_m;
  const double process_var_stationary =
    config.rtk_vertical_drift_sigma_m * config.rtk_vertical_drift_sigma_m;
  const double meas_var =
    std::max(config.rtk_vertical_white_noise_sigma_m * config.rtk_vertical_white_noise_sigma_m, kTiny);

  for (std::size_t i = 0; i < count; ++i) {
    double phi = 0.0;
    double q = process_var_stationary;
    if (i > 0U) {
      const double dt = std::max(0.0, candidates[i].time_s - candidates[i - 1U].time_s);
      phi = std::exp(-dt / config.rtk_vertical_drift_correlation_time_s);
      q = process_var_stationary * (1.0 - phi * phi);
      x_pred[i] = phi * prev_x;
      p_pred[i] = phi * phi * prev_p + q;
      phi_to_next[i - 1U] = phi;
    } else {
      x_pred[i] = 0.0;
      p_pred[i] = process_var_stationary;
    }

    const double observation = candidates[i].residual_m - constant_bias_m;
    const double innovation =
      Clamp(observation - x_pred[i],
            -config.rtk_vertical_drift_huber_sigma_m,
            config.rtk_vertical_drift_huber_sigma_m);
    const double innovation_var = p_pred[i] + meas_var;
    const double kalman_gain = p_pred[i] / std::max(innovation_var, kTiny);
    x_filt[i] = x_pred[i] + kalman_gain * innovation;
    p_filt[i] = std::max((1.0 - kalman_gain) * p_pred[i], kTiny);
    prev_x = x_filt[i];
    prev_p = p_filt[i];
  }

  std::vector<double> x_smooth = x_filt;
  std::vector<double> p_smooth = p_filt;
  for (std::size_t reverse = count - 1U; reverse > 0U; --reverse) {
    const std::size_t i = reverse - 1U;
    const double transition = phi_to_next[i];
    const double gain = p_filt[i] * transition / std::max(p_pred[i + 1U], kTiny);
    x_smooth[i] = x_filt[i] + gain * (x_smooth[i + 1U] - x_pred[i + 1U]);
    p_smooth[i] = p_filt[i] + gain * gain * (p_smooth[i + 1U] - p_pred[i + 1U]);
  }

  for (std::size_t i = 0; i < count; ++i) {
    auto &row = (*profile)[candidates[i].row_index];
    const double clipped_drift =
      Clamp(x_smooth[i],
            -config.rtk_vertical_drift_max_abs_correction_m,
            config.rtk_vertical_drift_max_abs_correction_m);
    row.constant_bias_m = constant_bias_m;
    row.drift_estimate_m = clipped_drift;
    row.corrected_center_up_m = row.raw_rtk_up_m - clipped_drift;
    row.white_residual_m = row.residual_m - constant_bias_m - clipped_drift;
    row.valid = true;
    row.skip_reason = "OK";
  }
}

}  // namespace

RtkVerticalDriftReferenceEstimator::RtkVerticalDriftReferenceEstimator(
  RtkVerticalDriftReferenceEstimateRequest request)
    : request_(std::move(request)) {}

RtkVerticalDriftReferenceEstimateResult RtkVerticalDriftReferenceEstimator::Estimate(
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> *previous_profile) const {
  if (request_.config == nullptr || request_.gnss_samples == nullptr ||
      !request_.should_use_sample || !request_.corrected_time_s) {
    throw std::runtime_error("RtkVerticalDriftReferenceEstimator received an incomplete request");
  }

  RtkVerticalDriftReferenceEstimateResult result;
  result.profile.resize(request_.gnss_samples->size());
  std::vector<Candidate> candidates;
  std::vector<double> residuals;
  std::vector<double> static_residuals;

  for (std::size_t sample_index = 0; sample_index < request_.gnss_samples->size(); ++sample_index) {
    const GnssSolutionSample &sample = (*request_.gnss_samples)[sample_index];
    const double corrected_time_s = request_.corrected_time_s(sample);
    auto &row = result.profile[sample_index];
    row.sample_index = sample_index;
    row.time_s = corrected_time_s;
    row.raw_rtk_up_m = sample.enu_position_m.z();
    row.drift_sigma_m = request_.config->rtk_vertical_drift_sigma_m;
    row.white_sigma_m = request_.config->rtk_vertical_white_noise_sigma_m;
    row.tau_s = request_.config->rtk_vertical_drift_correlation_time_s;
    row.static_window_flag =
      corrected_time_s >= request_.alignment_start_time_s &&
      corrected_time_s <= request_.alignment_end_time_s;

    if (!sample.has_enu_position || !std::isfinite(sample.enu_position_m.z())) {
      row.skip_reason = "invalid_up";
      continue;
    }
    if (!request_.should_use_sample(sample)) {
      row.skip_reason = "filtered_gnss_sample";
      continue;
    }

    std::string skip_reason;
    const std::optional<double> nav_up_m =
      NavReferenceUpM(request_, corrected_time_s, row.static_window_flag, &skip_reason);
    if (!nav_up_m.has_value() || !std::isfinite(*nav_up_m)) {
      row.skip_reason = skip_reason.empty() ? "missing_nav_reference" : skip_reason;
      continue;
    }

    row.nav_reference_up_m = *nav_up_m;
    row.residual_m = row.raw_rtk_up_m - row.nav_reference_up_m;
    candidates.push_back(Candidate{sample_index, corrected_time_s, row.residual_m, row.static_window_flag});
    residuals.push_back(row.residual_m);
    if (row.static_window_flag) {
      static_residuals.push_back(row.residual_m);
    }
  }

  const double constant_bias_m =
    !static_residuals.empty() ? Median(std::move(static_residuals)) : Median(std::move(residuals));
  if (std::isfinite(constant_bias_m)) {
    EstimateOuDrift(*request_.config, candidates, constant_bias_m, &result.profile);
  }

  if (previous_profile == nullptr || previous_profile->empty()) {
    result.max_abs_profile_delta_m = std::numeric_limits<double>::infinity();
  } else {
    for (std::size_t i = 0; i < result.profile.size() && i < previous_profile->size(); ++i) {
      const auto &row = result.profile[i];
      const auto &prev = (*previous_profile)[i];
      if (!row.valid || !prev.valid ||
          !std::isfinite(row.drift_estimate_m) ||
          !std::isfinite(prev.drift_estimate_m)) {
        continue;
      }
      result.max_abs_profile_delta_m =
        std::max(result.max_abs_profile_delta_m, std::abs(row.drift_estimate_m - prev.drift_estimate_m));
    }
  }
  return result;
}

void PopulateRtkVerticalDriftReferenceSummary(
  const OfflineRunnerConfig &config,
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> &profile,
  RunSummary &run_summary) {
  run_summary.rtk_vertical_drift_reference_enabled = config.enable_rtk_vertical_drift_reference;
  std::vector<double> static_drifts;
  std::vector<double> white_residuals;
  std::vector<double> first20_corrections;
  double max_abs_correction_m = 0.0;
  std::size_t valid_count = 0;
  for (const auto &row : profile) {
    if (!row.valid || !std::isfinite(row.drift_estimate_m)) {
      continue;
    }
    ++valid_count;
    max_abs_correction_m = std::max(max_abs_correction_m, std::abs(row.drift_estimate_m));
    if (row.static_window_flag) {
      static_drifts.push_back(row.drift_estimate_m);
    }
    if (std::isfinite(row.white_residual_m)) {
      white_residuals.push_back(row.white_residual_m);
    }
    const double rel_dynamic_time_s = row.time_s - run_summary.dynamic_start_time_s;
    if (rel_dynamic_time_s >= 0.0 && rel_dynamic_time_s <= 20.0) {
      first20_corrections.push_back(row.drift_estimate_m);
    }
  }
  run_summary.rtk_vertical_drift_reference_valid_count = valid_count;
  run_summary.rtk_vertical_drift_max_abs_correction_m =
    valid_count > 0U ? max_abs_correction_m : std::numeric_limits<double>::quiet_NaN();
  if (!static_drifts.empty()) {
    const auto [min_it, max_it] = std::minmax_element(static_drifts.begin(), static_drifts.end());
    run_summary.rtk_vertical_drift_static_range_m = *max_it - *min_it;
    run_summary.rtk_vertical_drift_static_std_m = StdDev(static_drifts);
  }
  run_summary.rtk_vertical_drift_white_residual_std_m = StdDev(white_residuals);
  if (!first20_corrections.empty()) {
    const double sum = std::accumulate(first20_corrections.begin(), first20_corrections.end(), 0.0);
    run_summary.rtk_vertical_drift_first20_mean_correction_m =
      sum / static_cast<double>(first20_corrections.size());
    double max_abs = 0.0;
    for (const double correction : first20_corrections) {
      max_abs = std::max(max_abs, std::abs(correction));
    }
    run_summary.rtk_vertical_drift_first20_max_abs_correction_m = max_abs;
  }
}

}  // namespace offline_lc_minimal
