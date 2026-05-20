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
#include "offline_lc_minimal/core/RtkVerticalDriftGateWeighting.h"
#include "offline_lc_minimal/core/RtkVerticalLowpassReferenceFilter.h"

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
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  bool static_window = false;
};

struct DriftOutageWindow {
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
};

struct NavReferenceResult {
  std::optional<double> up_m;
  std::string source = "UNSET";
  double causal_up_m = std::numeric_limits<double>::quiet_NaN();
  double full_up_m = std::numeric_limits<double>::quiet_NaN();
  double full_minus_causal_m = std::numeric_limits<double>::quiet_NaN();
  std::string skip_reason;
};

struct StaticReferenceResult {
  bool valid = false;
  double up_m = std::numeric_limits<double>::quiet_NaN();
  std::string source = "NONE";
};

std::optional<double> FullOptimizedNavReferenceUpM(
  const RtkVerticalDriftReferenceEstimateRequest &request,
  const double corrected_time_s,
  std::string *skip_reason) {
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

bool HasActiveCausalReferenceBoundary(
  const RtkVerticalDriftReferenceEstimateRequest &request) {
  return request.config != nullptr &&
         request.config->enable_rtk_outage_causal_drift_reference &&
         request.causal_nav_reference_profile != nullptr &&
         std::isfinite(request.causal_nav_reference_end_time_s);
}

NavReferenceResult NavReferenceUpM(
  const RtkVerticalDriftReferenceEstimateRequest &request,
  const std::size_t sample_index,
  const double corrected_time_s,
  const StaticReferenceResult &static_reference) {
  NavReferenceResult result;
  if (static_reference.valid) {
    result.up_m = static_reference.up_m;
    result.source =
      static_reference.source == "LATE_STATIC" ? "LATE_STATIC_RTK_REFERENCE" :
                                                  "STATIC_REFERENCE";
    result.skip_reason = "OK";
    return result;
  }

  if (HasActiveCausalReferenceBoundary(request) &&
      corrected_time_s <= request.causal_nav_reference_end_time_s + kTiny) {
    if (sample_index >= request.causal_nav_reference_profile->size()) {
      result.skip_reason = "missing_causal_nav_reference";
      return result;
    }
    const auto &causal_row = (*request.causal_nav_reference_profile)[sample_index];
    if (!causal_row.valid || !std::isfinite(causal_row.causal_nav_reference_up_m)) {
      result.skip_reason = "missing_causal_nav_reference";
      return result;
    }
    result.up_m = causal_row.causal_nav_reference_up_m;
    result.source = "CAUSAL_PREFIX";
    result.causal_up_m = causal_row.causal_nav_reference_up_m;
    std::string full_skip_reason;
    const std::optional<double> full_up_m =
      FullOptimizedNavReferenceUpM(request, corrected_time_s, &full_skip_reason);
    if (full_up_m.has_value() && std::isfinite(*full_up_m)) {
      result.full_up_m = *full_up_m;
      result.full_minus_causal_m = result.full_up_m - result.causal_up_m;
    }
    result.skip_reason = "OK";
    return result;
  }

  std::string skip_reason;
  const std::optional<double> full_up_m =
    FullOptimizedNavReferenceUpM(request, corrected_time_s, &skip_reason);
  if (!full_up_m.has_value() || !std::isfinite(*full_up_m)) {
    result.skip_reason = skip_reason.empty() ? "missing_nav_reference" : skip_reason;
    return result;
  }
  result.up_m = *full_up_m;
  result.source = "FULL_OPTIMIZED";
  result.full_up_m = *full_up_m;
  result.skip_reason = "OK";
  return result;
}

StaticReferenceResult StaticReferenceForTime(
  const RtkVerticalDriftReferenceEstimateRequest &request,
  const double corrected_time_s) {
  StaticReferenceResult result;
  if (corrected_time_s >= request.alignment_start_time_s &&
      corrected_time_s <= request.alignment_end_time_s) {
    result.valid = true;
    result.up_m = request.static_reference_up_m;
    result.source = "INITIAL_STATIC";
    return result;
  }
  if (request.late_static_windows == nullptr) {
    return result;
  }
  for (const auto &window : *request.late_static_windows) {
    if (!window.valid || !std::isfinite(window.rtk_median_up_m)) {
      continue;
    }
    if (corrected_time_s >= window.start_time_s - kTiny &&
        corrected_time_s <= window.end_time_s + kTiny) {
      result.valid = true;
      result.up_m = window.rtk_median_up_m;
      result.source = "LATE_STATIC";
      return result;
    }
  }
  return result;
}

std::vector<DriftOutageWindow> NormalizedDriftOutageWindows(
  const RtkVerticalDriftReferenceEstimateRequest &request) {
  std::vector<DriftOutageWindow> windows;
  if (request.config == nullptr ||
      !request.config->enable_rtk_vertical_drift_outage_segmentation ||
      request.rtk_outage_windows == nullptr) {
    return windows;
  }
  windows.reserve(request.rtk_outage_windows->size());
  for (const auto &window : *request.rtk_outage_windows) {
    if (!std::isfinite(window.start_time_s) ||
        !std::isfinite(window.end_time_s) ||
        window.end_time_s <= window.start_time_s) {
      continue;
    }
    windows.push_back(DriftOutageWindow{window.start_time_s, window.end_time_s});
  }
  std::sort(windows.begin(), windows.end(), [](const auto &lhs, const auto &rhs) {
    if (lhs.start_time_s == rhs.start_time_s) {
      return lhs.end_time_s < rhs.end_time_s;
    }
    return lhs.start_time_s < rhs.start_time_s;
  });
  std::vector<DriftOutageWindow> merged;
  for (const auto &window : windows) {
    if (merged.empty() || window.start_time_s > merged.back().end_time_s) {
      merged.push_back(window);
      continue;
    }
    merged.back().end_time_s = std::max(merged.back().end_time_s, window.end_time_s);
  }
  return merged;
}

int SegmentIndexForTime(
  const double time_s,
  const std::vector<DriftOutageWindow> &outage_windows) {
  int segment_index = 0;
  for (const auto &window : outage_windows) {
    if (time_s > window.start_time_s && time_s < window.end_time_s) {
      return -1;
    }
    if (time_s >= window.end_time_s) {
      ++segment_index;
      continue;
    }
    break;
  }
  return segment_index;
}

std::string SegmentRole(const int segment_index, const std::size_t possible_segment_count) {
  if (segment_index < 0) {
    return "IN_OUTAGE";
  }
  if (possible_segment_count <= 1U) {
    return "UNSEGMENTED";
  }
  if (segment_index == 0) {
    return "PRE_OUTAGE";
  }
  if (static_cast<std::size_t>(segment_index + 1) >= possible_segment_count) {
    return "POST_OUTAGE";
  }
  return "BETWEEN_OUTAGES";
}

void MarkOutageBoundaryRows(
  const std::vector<std::vector<Candidate>> &candidate_segments,
  std::vector<RtkVerticalDriftReferenceDiagnosticRow> *profile) {
  if (profile == nullptr || candidate_segments.size() <= 1U) {
    return;
  }
  bool has_previous_segment = false;
  std::size_t previous_last_row_index = 0U;
  for (const auto &segment : candidate_segments) {
    if (segment.empty()) {
      continue;
    }
    if (has_previous_segment) {
      (*profile)[previous_last_row_index].outage_boundary_blocked = true;
      (*profile)[segment.front().row_index].outage_boundary_blocked = true;
    }
    previous_last_row_index = segment.back().row_index;
    has_previous_segment = true;
  }
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
    const RtkVerticalDriftGateWeightingResult gate_weighting =
      ComputeRtkVerticalDriftGateWeighting(
        config,
        observation,
        candidates[i].sigma_u_m);
    auto &row = (*profile)[candidates[i].row_index];
    row.gate_half_width_m = gate_weighting.gate_half_width_m;
    row.gate_observation_m = gate_weighting.gate_observation_m;
    row.gate_violation_m = gate_weighting.gate_violation_m;
    row.gate_weight = gate_weighting.gate_weight;
    row.effective_white_sigma_m = gate_weighting.effective_white_sigma_m;

    const double innovation =
      Clamp(observation - x_pred[i],
            -config.rtk_vertical_drift_huber_sigma_m,
            config.rtk_vertical_drift_huber_sigma_m);
    const double effective_meas_var =
      std::isfinite(gate_weighting.effective_white_sigma_m) &&
      gate_weighting.effective_white_sigma_m > 0.0
        ? gate_weighting.effective_white_sigma_m * gate_weighting.effective_white_sigma_m
        : meas_var;
    const double innovation_var = p_pred[i] + std::max(effective_meas_var, kTiny);
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
  const std::vector<DriftOutageWindow> outage_windows =
    NormalizedDriftOutageWindows(request_);
  const bool outage_segmentation_enabled = !outage_windows.empty();
  const std::size_t possible_segment_count =
    outage_segmentation_enabled ? outage_windows.size() + 1U : 1U;
  std::vector<std::vector<Candidate>> candidate_segments(possible_segment_count);
  std::vector<std::vector<double>> segment_residuals(possible_segment_count);
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
    row.effective_white_sigma_m = request_.config->rtk_vertical_white_noise_sigma_m;
    row.tau_s = request_.config->rtk_vertical_drift_correlation_time_s;
    const StaticReferenceResult static_reference =
      StaticReferenceForTime(request_, corrected_time_s);
    row.static_window_flag = static_reference.valid;
    row.static_window_source = static_reference.source;

    if (!sample.has_enu_position || !std::isfinite(sample.enu_position_m.z())) {
      row.skip_reason = "invalid_up";
      continue;
    }
    if (!request_.should_use_sample(sample)) {
      row.skip_reason = "filtered_gnss_sample";
      continue;
    }

    const NavReferenceResult nav_reference =
      NavReferenceUpM(request_, sample_index, corrected_time_s, static_reference);
    row.nav_reference_source = nav_reference.source;
    row.causal_reference_up_m = nav_reference.causal_up_m;
    row.full_reference_up_m = nav_reference.full_up_m;
    row.full_minus_causal_nav_reference_m = nav_reference.full_minus_causal_m;
    row.causal_reference_boundary_time_s = request_.causal_nav_reference_end_time_s;
    if (!nav_reference.up_m.has_value() || !std::isfinite(*nav_reference.up_m)) {
      row.skip_reason =
        nav_reference.skip_reason.empty() ? "missing_nav_reference" : nav_reference.skip_reason;
      continue;
    }

    double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
    if (request_.clamped_sigma_m) {
      sigma_u_m = request_.clamped_sigma_m(sample).z();
    } else {
      sigma_u_m = sample.sigma_h_m;
    }

    row.nav_reference_up_m = *nav_reference.up_m;
    row.residual_m = row.raw_rtk_up_m - row.nav_reference_up_m;
    const int segment_index =
      outage_segmentation_enabled ? SegmentIndexForTime(corrected_time_s, outage_windows) : 0;
    row.drift_segment_index = segment_index;
    row.drift_segment_role = SegmentRole(segment_index, possible_segment_count);
    if (segment_index < 0) {
      row.skip_reason = "inside_rtk_outage";
      continue;
    }
    candidate_segments[static_cast<std::size_t>(segment_index)].push_back(
      Candidate{
        sample_index,
        corrected_time_s,
        row.residual_m,
        sigma_u_m,
        row.static_window_flag});
    segment_residuals[static_cast<std::size_t>(segment_index)].push_back(row.residual_m);
    if (row.static_window_flag) {
      static_residuals.push_back(row.residual_m);
    }
  }

  const std::optional<double> static_constant_bias_m =
    !static_residuals.empty()
      ? std::optional<double>(Median(std::move(static_residuals)))
      : std::nullopt;
  bool estimated_any_segment = false;
  for (std::size_t segment_index = 0; segment_index < candidate_segments.size(); ++segment_index) {
    const double constant_bias_m =
      static_constant_bias_m.has_value()
        ? *static_constant_bias_m
        : Median(segment_residuals[segment_index]);
    if (std::isfinite(constant_bias_m)) {
      EstimateOuDrift(
        *request_.config,
        candidate_segments[segment_index],
        constant_bias_m,
        &result.profile);
      estimated_any_segment = true;
    }
  }
  if (estimated_any_segment) {
    MarkOutageBoundaryRows(candidate_segments, &result.profile);
    const RtkVerticalLowpassReferenceFilterSummary lowpass_summary =
      ApplyRtkVerticalLowpassReferenceFilter(*request_.config, &result.profile);
    (void)lowpass_summary;
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
  run_summary.rtk_vertical_drift_gate_weighting_enabled =
    config.enable_rtk_vertical_drift_gate_weighting;
  run_summary.rtk_vertical_lowpass_reference_enabled =
    config.enable_rtk_vertical_lowpass_reference;
  run_summary.rtk_vertical_lowpass_reference_cutoff_hz =
    config.rtk_vertical_lowpass_reference_cutoff_hz;
  std::vector<double> static_drifts;
  std::vector<double> white_residuals;
  std::vector<double> first20_corrections;
  std::vector<double> gate_weights;
  std::vector<int> valid_segment_indices;
  double max_abs_correction_m = 0.0;
  double max_abs_lowpass_delta_m = 0.0;
  double max_gate_violation_m = 0.0;
  std::size_t valid_count = 0;
  std::size_t lowpass_valid_count = 0;
  std::size_t downweighted_count = 0;
  std::size_t outage_boundary_count = 0;
  std::size_t causal_reference_count = 0;
  bool outage_segmentation_enabled = false;
  double max_abs_full_minus_causal_m = 0.0;
  bool has_previous_segment = false;
  int previous_segment_index = -1;
  for (const auto &row : profile) {
    if (!row.valid || !std::isfinite(row.drift_estimate_m)) {
      continue;
    }
    ++valid_count;
    if (row.drift_segment_index >= 0) {
      if (std::find(valid_segment_indices.begin(),
                    valid_segment_indices.end(),
                    row.drift_segment_index) == valid_segment_indices.end()) {
        valid_segment_indices.push_back(row.drift_segment_index);
      }
      if (has_previous_segment &&
          previous_segment_index >= 0 &&
          previous_segment_index != row.drift_segment_index) {
        ++outage_boundary_count;
      }
      previous_segment_index = row.drift_segment_index;
      has_previous_segment = true;
    }
    if (row.drift_segment_role != "UNSEGMENTED") {
      outage_segmentation_enabled = true;
    }
    if (row.nav_reference_source == "CAUSAL_PREFIX") {
      ++causal_reference_count;
      if (std::isfinite(row.full_minus_causal_nav_reference_m)) {
        max_abs_full_minus_causal_m =
          std::max(max_abs_full_minus_causal_m, std::abs(row.full_minus_causal_nav_reference_m));
      }
    }
    max_abs_correction_m = std::max(max_abs_correction_m, std::abs(row.drift_estimate_m));
    if (row.static_window_flag) {
      static_drifts.push_back(row.drift_estimate_m);
    }
    if (std::isfinite(row.white_residual_m)) {
      white_residuals.push_back(row.white_residual_m);
    }
    if (std::isfinite(row.gate_weight)) {
      gate_weights.push_back(row.gate_weight);
      if (row.gate_weight < 1.0 - 1.0e-12) {
        ++downweighted_count;
      }
    }
    if (std::isfinite(row.gate_violation_m)) {
      max_gate_violation_m = std::max(max_gate_violation_m, row.gate_violation_m);
    }
    if (row.lowpass_applied && std::isfinite(row.lowpass_delta_m)) {
      ++lowpass_valid_count;
      max_abs_lowpass_delta_m =
        std::max(max_abs_lowpass_delta_m, std::abs(row.lowpass_delta_m));
    }
    const double rel_dynamic_time_s = row.time_s - run_summary.dynamic_start_time_s;
    if (rel_dynamic_time_s >= 0.0 && rel_dynamic_time_s <= 20.0) {
      first20_corrections.push_back(row.drift_estimate_m);
    }
  }
  run_summary.rtk_vertical_drift_reference_valid_count = valid_count;
  run_summary.rtk_vertical_drift_max_abs_correction_m =
    valid_count > 0U ? max_abs_correction_m : std::numeric_limits<double>::quiet_NaN();
  run_summary.rtk_vertical_drift_outage_segmentation_enabled =
    config.enable_rtk_vertical_drift_outage_segmentation && outage_segmentation_enabled;
  run_summary.rtk_vertical_drift_segment_count = valid_segment_indices.size();
  run_summary.rtk_vertical_drift_outage_boundary_count = outage_boundary_count;
  run_summary.rtk_vertical_drift_cross_outage_lowpass_blocked =
    config.enable_rtk_vertical_lowpass_reference && outage_boundary_count > 0U;
  run_summary.rtk_vertical_drift_gate_downweighted_count = downweighted_count;
  run_summary.rtk_vertical_drift_gate_max_violation_m =
    valid_count > 0U ? max_gate_violation_m : std::numeric_limits<double>::quiet_NaN();
  run_summary.rtk_vertical_drift_causal_reference_enabled =
    config.enable_rtk_outage_causal_drift_reference && causal_reference_count > 0U;
  run_summary.rtk_vertical_drift_causal_reference_sample_count = causal_reference_count;
  run_summary.rtk_vertical_drift_causal_reference_max_full_delta_m =
    causal_reference_count > 0U ? max_abs_full_minus_causal_m : std::numeric_limits<double>::quiet_NaN();
  run_summary.rtk_vertical_lowpass_reference_valid_count = lowpass_valid_count;
  run_summary.rtk_vertical_lowpass_reference_max_abs_delta_m =
    lowpass_valid_count > 0U ? max_abs_lowpass_delta_m : std::numeric_limits<double>::quiet_NaN();
  if (!static_drifts.empty()) {
    const auto [min_it, max_it] = std::minmax_element(static_drifts.begin(), static_drifts.end());
    run_summary.rtk_vertical_drift_static_range_m = *max_it - *min_it;
    run_summary.rtk_vertical_drift_static_std_m = StdDev(static_drifts);
  }
  run_summary.rtk_vertical_drift_white_residual_std_m = StdDev(white_residuals);
  if (!gate_weights.empty()) {
    run_summary.rtk_vertical_drift_gate_weight_mean =
      std::accumulate(gate_weights.begin(), gate_weights.end(), 0.0) /
      static_cast<double>(gate_weights.size());
    run_summary.rtk_vertical_drift_gate_weight_min =
      *std::min_element(gate_weights.begin(), gate_weights.end());
  }
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
