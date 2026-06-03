#include "offline_lc_minimal/core/BodyZNHCConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/BodyZHorizontalLeakageEstimator.h"
#include "offline_lc_minimal/core/BodyZNHCWeightPlanner.h"
#include "offline_lc_minimal/factor/BodyZLeakageCorrectedVelocityFactor.h"
#include "offline_lc_minimal/factor/BodyZLeakageCorrectedWindowDisplacementFactor.h"
#include "offline_lc_minimal/factor/BodyZVelocityZeroFactor.h"
#include "offline_lc_minimal/factor/BodyZWindowDisplacementZeroFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kStrictWindowBoundaryPaddingS = 1.0e-6;

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

struct BodyZNHCMetrics {
  double mean_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double max_abs_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double body_z_displacement_m = std::numeric_limits<double>::quiet_NaN();
};

bool HasReferenceStates(
  const std::vector<ReferenceNodeState> *reference_states,
  const std::vector<double> &state_timestamps) {
  return reference_states != nullptr && reference_states->size() == state_timestamps.size();
}

std::optional<BodyZNHCMetrics> ComputeMetrics(
  const gtsam::Values &values,
  const std::vector<double> &state_timestamps,
  const std::vector<std::size_t> &state_indices,
  const std::vector<gtsam::Vector3> &body_z_axes_nav) {
  if (state_indices.size() < 2U) {
    return std::nullopt;
  }
  if (body_z_axes_nav.size() != state_indices.size()) {
    return std::nullopt;
  }

  std::vector<double> body_z_velocities_mps;
  body_z_velocities_mps.reserve(state_indices.size());
  for (std::size_t offset = 0U; offset < state_indices.size(); ++offset) {
    const std::size_t state_index = state_indices[offset];
    if (state_index >= state_timestamps.size()) {
      return std::nullopt;
    }
    const auto velocity = values.at<gtsam::Vector3>(symbol::V(state_index));
    const double body_z_velocity_mps =
      factor::BodyZVelocityMps(body_z_axes_nav[offset], velocity);
    if (!std::isfinite(body_z_velocity_mps)) {
      return std::nullopt;
    }
    body_z_velocities_mps.push_back(body_z_velocity_mps);
  }

  double abs_sum = 0.0;
  double max_abs = 0.0;
  for (const double body_z_velocity_mps : body_z_velocities_mps) {
    const double abs_velocity = std::abs(body_z_velocity_mps);
    abs_sum += abs_velocity;
    max_abs = std::max(max_abs, abs_velocity);
  }

  double displacement_m = 0.0;
  for (std::size_t index = 1U; index < state_indices.size(); ++index) {
    const double prev_time_s = state_timestamps[state_indices[index - 1U]];
    const double current_time_s = state_timestamps[state_indices[index]];
    const double dt_s = current_time_s - prev_time_s;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      return std::nullopt;
    }
    displacement_m +=
      0.5 * dt_s * (body_z_velocities_mps[index - 1U] + body_z_velocities_mps[index]);
  }

  BodyZNHCMetrics metrics;
  metrics.mean_abs_body_z_velocity_mps =
    abs_sum / static_cast<double>(body_z_velocities_mps.size());
  metrics.max_abs_body_z_velocity_mps = max_abs;
  metrics.body_z_displacement_m = displacement_m;
  return metrics;
}

std::optional<BodyZNHCMetrics> ComputeCorrectedMetrics(
  const gtsam::Values &values,
  const std::vector<double> &state_timestamps,
  const std::vector<std::size_t> &state_indices,
  const std::vector<factor::BodyFrameAxesNav> &body_axes_nav,
  const factor::BodyZHorizontalLeakageModel &leakage) {
  if (state_indices.size() < 2U) {
    return std::nullopt;
  }
  if (body_axes_nav.size() != state_indices.size()) {
    return std::nullopt;
  }

  std::vector<double> corrected_velocities_mps;
  corrected_velocities_mps.reserve(state_indices.size());
  for (std::size_t offset = 0U; offset < state_indices.size(); ++offset) {
    const std::size_t state_index = state_indices[offset];
    if (state_index >= state_timestamps.size()) {
      return std::nullopt;
    }
    const auto velocity = values.at<gtsam::Vector3>(symbol::V(state_index));
    const double corrected_velocity_mps =
      factor::BodyZLeakageCorrectedVelocityMps(body_axes_nav[offset], leakage, velocity);
    if (!std::isfinite(corrected_velocity_mps)) {
      return std::nullopt;
    }
    corrected_velocities_mps.push_back(corrected_velocity_mps);
  }

  double abs_sum = 0.0;
  double max_abs = 0.0;
  for (const double corrected_velocity_mps : corrected_velocities_mps) {
    const double abs_velocity = std::abs(corrected_velocity_mps);
    abs_sum += abs_velocity;
    max_abs = std::max(max_abs, abs_velocity);
  }

  double displacement_m = 0.0;
  for (std::size_t index = 1U; index < state_indices.size(); ++index) {
    const double prev_time_s = state_timestamps[state_indices[index - 1U]];
    const double current_time_s = state_timestamps[state_indices[index]];
    const double dt_s = current_time_s - prev_time_s;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      return std::nullopt;
    }
    displacement_m +=
      0.5 * dt_s * (corrected_velocities_mps[index - 1U] + corrected_velocities_mps[index]);
  }

  BodyZNHCMetrics metrics;
  metrics.mean_abs_body_z_velocity_mps =
    abs_sum / static_cast<double>(corrected_velocities_mps.size());
  metrics.max_abs_body_z_velocity_mps = max_abs;
  metrics.body_z_displacement_m = displacement_m;
  return metrics;
}

std::optional<BodyZNHCMetrics> ComputePoseBodyZMetrics(
  const gtsam::Values &values,
  const std::vector<double> &state_timestamps,
  const std::vector<std::size_t> &state_indices) {
  std::vector<gtsam::Vector3> optimized_axes_nav;
  optimized_axes_nav.reserve(state_indices.size());
  for (const std::size_t state_index : state_indices) {
    try {
      const auto pose = values.at<gtsam::Pose3>(symbol::X(state_index));
      optimized_axes_nav.push_back(factor::BodyZAxisNavFromPose(pose));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return ComputeMetrics(values, state_timestamps, state_indices, optimized_axes_nav);
}

std::optional<std::vector<gtsam::Vector3>> BodyZAxesNavFromInitialValues(
  const gtsam::Values &initial_values,
  const std::vector<std::size_t> &state_indices) {
  std::vector<gtsam::Vector3> body_z_axes_nav;
  body_z_axes_nav.reserve(state_indices.size());
  for (const std::size_t state_index : state_indices) {
    try {
      const auto pose = initial_values.at<gtsam::Pose3>(symbol::X(state_index));
      body_z_axes_nav.push_back(factor::BodyZAxisNavFromPose(pose));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return body_z_axes_nav;
}

std::optional<std::vector<factor::BodyFrameAxesNav>> BodyFrameAxesNavFromStateSource(
  const gtsam::Values &initial_values,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> *reference_states,
  const std::vector<std::size_t> &state_indices,
  const bool prefer_reference_states) {
  std::vector<factor::BodyFrameAxesNav> body_axes_nav;
  body_axes_nav.reserve(state_indices.size());
  const bool use_reference_states =
    prefer_reference_states && HasReferenceStates(reference_states, state_timestamps);
  for (const std::size_t state_index : state_indices) {
    try {
      const gtsam::Pose3 pose = use_reference_states
        ? (*reference_states)[state_index].pose
        : initial_values.at<gtsam::Pose3>(symbol::X(state_index));
      body_axes_nav.push_back(factor::BodyFrameAxesNavFromPose(pose));
    } catch (const std::exception &) {
      return std::nullopt;
    }
  }
  return body_axes_nav;
}

std::vector<gtsam::Vector3> BodyZAxesFromBodyFrameAxes(
  const std::vector<factor::BodyFrameAxesNav> &body_axes_nav) {
  std::vector<gtsam::Vector3> body_z_axes_nav;
  body_z_axes_nav.reserve(body_axes_nav.size());
  for (const auto &axes : body_axes_nav) {
    body_z_axes_nav.push_back(axes.body_z_axis_nav);
  }
  return body_z_axes_nav;
}

void PopulateAttitudeRanges(
  const gtsam::Values &values,
  const std::vector<std::size_t> &state_indices,
  BodyZNHCDiagnosticRow &row) {
  if (state_indices.size() < 2U) {
    return;
  }
  double min_pitch_rad = std::numeric_limits<double>::infinity();
  double max_pitch_rad = -std::numeric_limits<double>::infinity();
  double min_roll_rad = std::numeric_limits<double>::infinity();
  double max_roll_rad = -std::numeric_limits<double>::infinity();
  for (const std::size_t state_index : state_indices) {
    try {
      const auto pose = values.at<gtsam::Pose3>(symbol::X(state_index));
      const auto ypr = pose.rotation().ypr();
      min_pitch_rad = std::min(min_pitch_rad, ypr.y());
      max_pitch_rad = std::max(max_pitch_rad, ypr.y());
      min_roll_rad = std::min(min_roll_rad, ypr.z());
      max_roll_rad = std::max(max_roll_rad, ypr.z());
    } catch (const std::exception &) {
      return;
    }
  }
  if (std::isfinite(min_pitch_rad) && std::isfinite(max_pitch_rad)) {
    row.optimized_pitch_range_rad = max_pitch_rad - min_pitch_rad;
  }
  if (std::isfinite(min_roll_rad) && std::isfinite(max_roll_rad)) {
    row.optimized_roll_range_rad = max_roll_rad - min_roll_rad;
  }
}

}  // namespace

BodyZNHCConstraintBuilder::BodyZNHCConstraintBuilder(BodyZNHCConstraintBuildRequest request)
    : request_(std::move(request)) {}

void BodyZNHCConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.jump_windows == nullptr || request_.initial_values == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.diagnostics == nullptr) {
    throw std::runtime_error("BodyZNHCConstraintBuilder received an incomplete request");
  }

  if (!request_.config->enable_body_z_nhc_constraint) {
    return;
  }
  request_.run_summary->body_z_nhc_strict_effective_weighting_enabled =
    request_.config->enable_body_z_nhc_strict_effective_weighting;

  std::vector<BodyZJumpConstraintWindow> windows = BuildJumpWindows();
  BodyZHorizontalLeakageEstimate leakage_estimate;
  if (request_.config->enable_body_z_nhc_horizontal_leakage_correction) {
    BodyZHorizontalLeakageEstimateRequest leakage_request;
    leakage_request.config = request_.config;
    leakage_request.state_timestamps = request_.state_timestamps;
    leakage_request.excluded_windows = &windows;
    leakage_request.initial_values = request_.initial_values;
    leakage_request.reference_states = request_.reference_states;
    leakage_request.dynamic_start_index = request_.dynamic_start_index;
    leakage_estimate = BodyZHorizontalLeakageEstimator(std::move(leakage_request)).Estimate();
    if (request_.horizontal_leakage_diagnostics != nullptr) {
      request_.horizontal_leakage_diagnostics->push_back(leakage_estimate.diagnostic);
    }
    request_.run_summary->body_z_nhc_horizontal_leakage_correction_enabled = true;
    request_.run_summary->body_z_nhc_horizontal_leakage_estimate_valid = leakage_estimate.valid;
    request_.run_summary->body_z_nhc_horizontal_leakage_sample_count =
      leakage_estimate.diagnostic.used_sample_count;
    request_.run_summary->body_z_nhc_horizontal_leakage_skipped_window_count =
      leakage_estimate.diagnostic.skipped_window_count;
    request_.run_summary->body_z_nhc_horizontal_leakage_skipped_low_speed_count =
      leakage_estimate.diagnostic.skipped_low_speed_count;
    request_.run_summary->body_z_nhc_horizontal_leakage_skipped_invalid_count =
      leakage_estimate.diagnostic.skipped_invalid_count;
    request_.run_summary->body_z_nhc_horizontal_leakage_x_rad =
      leakage_estimate.diagnostic.leak_x_rad;
    request_.run_summary->body_z_nhc_horizontal_leakage_y_rad =
      leakage_estimate.diagnostic.leak_y_rad;
  }
  const bool use_leakage_correction = leakage_estimate.valid;
  const std::vector<BodyZJumpConstraintWindow> global_windows = BuildGlobalWindows(windows);
  windows.insert(windows.end(), global_windows.begin(), global_windows.end());
  request_.diagnostics->reserve(request_.diagnostics->size() + windows.size());

  const std::size_t window_index_offset = request_.diagnostics->size();
  BodyZNHCWeightPlanner weight_planner;
  for (std::size_t window_index = 0U; window_index < windows.size(); ++window_index) {
    const BodyZJumpConstraintWindow &window = windows[window_index];
    const double velocity_sigma_mps = window.source_window_count > 0U
      ? request_.config->body_z_nhc_jump_velocity_sigma_mps
      : request_.config->body_z_nhc_global_velocity_sigma_mps;
    const double displacement_sigma_m = window.source_window_count > 0U
      ? request_.config->body_z_nhc_jump_displacement_sigma_m
      : request_.config->body_z_nhc_global_displacement_sigma_m;
    const std::vector<std::size_t> state_indices =
      StateIndicesInWindow(window.start_time_s, window.end_time_s);
    BodyZNHCDiagnosticRow row =
      MakeDiagnosticRow(
        window_index_offset + window_index,
        window,
        state_indices,
        velocity_sigma_mps,
        displacement_sigma_m);

    if (state_indices.size() < 2U) {
      row.skip_reason = "INSUFFICIENT_STATES";
      ++request_.run_summary->body_z_nhc_skipped_short_window_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (!std::isfinite(row.actual_state_span_s) ||
        row.actual_state_span_s < request_.config->body_z_nhc_min_window_s) {
      row.skip_reason = "SHORT_WINDOW";
      ++request_.run_summary->body_z_nhc_skipped_short_window_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const auto body_z_axes_nav =
      BodyZAxesNavFromInitialValues(*request_.initial_values, state_indices);
    if (!body_z_axes_nav.has_value()) {
      row.skip_reason = "INVALID_FIXED_AXIS";
      ++request_.run_summary->body_z_nhc_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    const auto body_axes_nav = BodyFrameAxesNavFromStateSource(
      *request_.initial_values,
      *request_.state_timestamps,
      request_.reference_states,
      state_indices,
      use_leakage_correction);
    if (!body_axes_nav.has_value()) {
      row.skip_reason = "INVALID_FIXED_AXES";
      ++request_.run_summary->body_z_nhc_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    const std::vector<gtsam::Vector3> effective_body_z_axes_nav =
      BodyZAxesFromBodyFrameAxes(*body_axes_nav);

    const auto initial_metrics =
      ComputeMetrics(
        *request_.initial_values,
        *request_.state_timestamps,
        state_indices,
        effective_body_z_axes_nav);
    if (!initial_metrics.has_value()) {
      row.skip_reason = "INVALID_METRICS";
      ++request_.run_summary->body_z_nhc_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    row.initial_mean_abs_body_z_velocity_mps = initial_metrics->mean_abs_body_z_velocity_mps;
    row.initial_max_abs_body_z_velocity_mps = initial_metrics->max_abs_body_z_velocity_mps;
    row.initial_body_z_displacement_m = initial_metrics->body_z_displacement_m;
    row.horizontal_leakage_correction_enabled = use_leakage_correction;
    if (use_leakage_correction) {
      row.horizontal_leakage_x_rad = leakage_estimate.model.leak_x_rad;
      row.horizontal_leakage_y_rad = leakage_estimate.model.leak_y_rad;
      const auto initial_corrected_metrics = ComputeCorrectedMetrics(
        *request_.initial_values,
        *request_.state_timestamps,
        state_indices,
        *body_axes_nav,
        leakage_estimate.model);
      if (!initial_corrected_metrics.has_value()) {
        row.skip_reason = "INVALID_CORRECTED_METRICS";
        ++request_.run_summary->body_z_nhc_skipped_invalid_count;
        request_.diagnostics->push_back(row);
        continue;
      }
      row.initial_mean_abs_corrected_body_z_velocity_mps =
        initial_corrected_metrics->mean_abs_body_z_velocity_mps;
      row.initial_max_abs_corrected_body_z_velocity_mps =
        initial_corrected_metrics->max_abs_body_z_velocity_mps;
      row.initial_corrected_body_z_displacement_m =
        initial_corrected_metrics->body_z_displacement_m;
    }

    BodyZNHCWeightPlannerWindowRequest weight_request;
    weight_request.strict_effective_weighting =
      request_.config->enable_body_z_nhc_strict_effective_weighting;
    weight_request.state_indices = state_indices;
    weight_request.start_time_s = window.start_time_s;
    weight_request.end_time_s = window.end_time_s;
    weight_request.target_velocity_sigma_mps = velocity_sigma_mps;
    weight_request.target_displacement_sigma_m = displacement_sigma_m;
    const BodyZNHCWindowWeightPlan weight_plan =
      weight_planner.PlanWindow(weight_request);
    row.velocity_state_duplicate_count =
      weight_plan.velocity_state_duplicate_count;
    row.interval_overlap_count = weight_plan.interval_overlap_count;
    row.applied_velocity_sigma_mps = weight_plan.applied_velocity_sigma_mps;
    row.applied_displacement_sigma_m = weight_plan.applied_displacement_sigma_m;

    std::vector<gtsam::Key> velocity_keys;
    std::vector<double> state_times_s;
    velocity_keys.reserve(state_indices.size());
    state_times_s.reserve(state_indices.size());
    for (std::size_t offset = 0U; offset < state_indices.size(); ++offset) {
      const std::size_t state_index = state_indices[offset];
      velocity_keys.push_back(symbol::V(state_index));
      state_times_s.push_back((*request_.state_timestamps)[state_index]);
      if (!weight_plan.velocity_factor_used[offset]) {
        continue;
      }
      if (use_leakage_correction) {
        request_.graph->add(factor::BodyZLeakageCorrectedVelocityZeroFactor(
          symbol::V(state_index),
          (*body_axes_nav)[offset],
          leakage_estimate.model,
          gtsam::noiseModel::Isotropic::Sigma(1, weight_plan.applied_velocity_sigma_mps)));
        ++request_.run_summary->body_z_nhc_leakage_corrected_velocity_factor_count;
      } else {
        request_.graph->add(factor::BodyZVelocityZeroFactor(
          symbol::V(state_index),
          effective_body_z_axes_nav[offset],
          gtsam::noiseModel::Isotropic::Sigma(1, weight_plan.applied_velocity_sigma_mps)));
      }
      row.velocity_factor_state_indices.push_back(state_index);
      ++row.velocity_factor_count;
    }

    if (use_leakage_correction) {
      request_.graph->add(factor::BodyZLeakageCorrectedWindowDisplacementZeroFactor(
        velocity_keys,
        *body_axes_nav,
        leakage_estimate.model,
        state_times_s,
        gtsam::noiseModel::Isotropic::Sigma(1, weight_plan.applied_displacement_sigma_m)));
      ++request_.run_summary->body_z_nhc_leakage_corrected_displacement_factor_count;
    } else {
      request_.graph->add(factor::BodyZWindowDisplacementZeroFactor(
        velocity_keys,
        effective_body_z_axes_nav,
        state_times_s,
        gtsam::noiseModel::Isotropic::Sigma(1, weight_plan.applied_displacement_sigma_m)));
    }
    row.displacement_factor_count = 1U;
    row.factor_added = true;
    row.skip_reason = "ADDED";

    ++request_.run_summary->body_z_nhc_window_count;
    request_.run_summary->body_z_nhc_velocity_factor_count += row.velocity_factor_count;
    request_.run_summary->body_z_nhc_displacement_factor_count += row.displacement_factor_count;
    if (row.from_jump_window) {
      request_.run_summary->body_z_nhc_jump_velocity_factor_count += row.velocity_factor_count;
      request_.run_summary->body_z_nhc_jump_displacement_factor_count += row.displacement_factor_count;
    } else {
      request_.run_summary->body_z_nhc_global_velocity_factor_count += row.velocity_factor_count;
      request_.run_summary->body_z_nhc_global_displacement_factor_count += row.displacement_factor_count;
    }
    request_.diagnostics->push_back(row);
  }
  request_.run_summary->body_z_nhc_unique_velocity_factor_count =
    weight_planner.uniqueVelocityFactorStateCount();
  request_.run_summary->body_z_nhc_velocity_duplicate_state_count =
    weight_planner.duplicateVelocityStateCount();
  request_.run_summary->body_z_nhc_interval_overlap_count =
    weight_planner.intervalOverlapCount();
}

std::vector<BodyZJumpConstraintWindow> BodyZNHCConstraintBuilder::BuildJumpWindows() const {
  for (std::size_t index = 0U; index < request_.jump_windows->size(); ++index) {
    const auto &jump_window = (*request_.jump_windows)[index];
    if (!std::isfinite(jump_window.start_time_s) ||
        !std::isfinite(jump_window.end_time_s) ||
        jump_window.end_time_s < jump_window.start_time_s) {
      BodyZNHCDiagnosticRow row;
      row.window_index = request_.diagnostics->size();
      row.window_type = "JUMP";
      row.from_jump_window = true;
      row.source_window_index = index;
      row.source_window_count = 1U;
      row.start_time_s = jump_window.start_time_s;
      row.end_time_s = jump_window.end_time_s;
      row.duration_s = jump_window.end_time_s - jump_window.start_time_s;
      row.actual_state_span_s = std::numeric_limits<double>::quiet_NaN();
      row.skip_reason = "INVALID_WINDOW";
      ++request_.run_summary->body_z_nhc_skipped_invalid_count;
      request_.diagnostics->push_back(row);
    }
  }
  return BuildBodyZJumpConstraintWindows(
    *request_.jump_windows,
    BodyZNHCJumpConstraintWindowOptions(*request_.config));
}

std::vector<BodyZJumpConstraintWindow> BodyZNHCConstraintBuilder::BuildGlobalWindows(
  const std::vector<BodyZJumpConstraintWindow> &jump_windows) const {
  std::vector<BodyZJumpConstraintWindow> windows;
  if (!request_.config->enable_body_z_nhc_global_weak_constraint ||
      request_.state_timestamps->empty() ||
      request_.dynamic_start_index >= request_.state_timestamps->size()) {
    return windows;
  }

  const double dynamic_start_time_s = (*request_.state_timestamps)[request_.dynamic_start_index];
  const double final_time_s = request_.state_timestamps->back();
  const bool strict_weighting =
    request_.config->enable_body_z_nhc_strict_effective_weighting;
  const double boundary_padding_s =
    strict_weighting ? kStrictWindowBoundaryPaddingS : 0.0;
  double segment_start_s = dynamic_start_time_s;
  for (const auto &jump_window : jump_windows) {
    const double segment_end_s = std::clamp(
      jump_window.start_time_s - boundary_padding_s,
      dynamic_start_time_s,
      final_time_s);
    double start_time_s = segment_start_s;
    while (start_time_s + request_.config->body_z_nhc_min_window_s <= segment_end_s + kTimeEpsilonS) {
      const double end_time_s =
        std::min(start_time_s + request_.config->body_z_nhc_global_window_s, segment_end_s);
      BodyZJumpConstraintWindow window;
      window.source_window_index = 0U;
      window.source_window_count = 0U;
      window.start_time_s = start_time_s;
      window.end_time_s = end_time_s;
      windows.push_back(window);
      start_time_s += request_.config->body_z_nhc_global_stride_s;
      if (strict_weighting) {
        start_time_s += kStrictWindowBoundaryPaddingS;
      }
    }
    segment_start_s = std::max(
      segment_start_s,
      jump_window.end_time_s + boundary_padding_s);
  }
  double start_time_s = segment_start_s;
  while (start_time_s + request_.config->body_z_nhc_min_window_s <= final_time_s + kTimeEpsilonS) {
    const double end_time_s =
      std::min(start_time_s + request_.config->body_z_nhc_global_window_s, final_time_s);
    BodyZJumpConstraintWindow window;
    window.source_window_index = 0U;
    window.source_window_count = 0U;
    window.start_time_s = start_time_s;
    window.end_time_s = end_time_s;
    windows.push_back(window);
    start_time_s += request_.config->body_z_nhc_global_stride_s;
    if (strict_weighting) {
      start_time_s += kStrictWindowBoundaryPaddingS;
    }
  }
  return windows;
}

std::vector<std::size_t> BodyZNHCConstraintBuilder::StateIndicesInWindow(
  const double start_time_s,
  const double end_time_s) const {
  std::vector<std::size_t> state_indices;
  if (request_.state_timestamps == nullptr || request_.state_timestamps->empty()) {
    return state_indices;
  }
  const std::size_t start_index =
    std::min(request_.dynamic_start_index, request_.state_timestamps->size());
  for (std::size_t state_index = start_index; state_index < request_.state_timestamps->size(); ++state_index) {
    const double state_time_s = (*request_.state_timestamps)[state_index];
    if (state_time_s + kTimeEpsilonS < start_time_s) {
      continue;
    }
    if (state_time_s > end_time_s + kTimeEpsilonS) {
      break;
    }
    state_indices.push_back(state_index);
  }
  return state_indices;
}

bool BodyZNHCConstraintBuilder::OverlapsAnyWindow(
  const double start_time_s,
  const double end_time_s,
  const std::vector<BodyZJumpConstraintWindow> &windows) const {
  for (const auto &window : windows) {
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      return true;
    }
  }
  return false;
}

BodyZNHCDiagnosticRow BodyZNHCConstraintBuilder::MakeDiagnosticRow(
  const std::size_t window_index,
  const BodyZJumpConstraintWindow &window,
  const std::vector<std::size_t> &state_indices,
  const double velocity_sigma_mps,
  const double displacement_sigma_m) const {
  BodyZNHCDiagnosticRow row;
  row.window_index = window_index;
  row.window_type = window.source_window_count > 0U ? "JUMP" : "GLOBAL";
  row.from_jump_window = window.source_window_count > 0U;
  row.source_window_index = window.source_window_index;
  row.source_window_count = window.source_window_count;
  row.start_time_s = window.start_time_s;
  row.end_time_s = window.end_time_s;
  row.duration_s = window.end_time_s - window.start_time_s;
  row.state_count = state_indices.size();
  row.start_state_index = state_indices.empty() ? 0U : state_indices.front();
  row.end_state_index = state_indices.empty() ? 0U : state_indices.back();
  if (state_indices.size() >= 2U) {
    row.actual_state_span_s =
      (*request_.state_timestamps)[state_indices.back()] -
      (*request_.state_timestamps)[state_indices.front()];
  }
  row.velocity_sigma_mps = velocity_sigma_mps;
  row.displacement_sigma_m = displacement_sigma_m;
  row.strict_weighting_enabled = request_.config->enable_body_z_nhc_strict_effective_weighting;
  row.target_velocity_sigma_mps = velocity_sigma_mps;
  row.applied_velocity_sigma_mps = velocity_sigma_mps;
  row.target_displacement_sigma_m = displacement_sigma_m;
  row.applied_displacement_sigma_m = displacement_sigma_m;
  return row;
}

void PopulateBodyZNHCDiagnostics(
  const gtsam::Values &initial_values,
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps,
  std::vector<BodyZNHCDiagnosticRow> &diagnostics,
  std::vector<BodyZNHCStateDiagnosticRow> *state_diagnostics,
  const std::vector<ReferenceNodeState> *reference_states,
  RunSummary *run_summary) {
  if (state_diagnostics != nullptr) {
    state_diagnostics->clear();
  }
  for (auto &row : diagnostics) {
    if (!row.factor_added || row.end_state_index < row.start_state_index) {
      continue;
    }
    std::vector<std::size_t> state_indices;
    state_indices.reserve(row.end_state_index - row.start_state_index + 1U);
    for (std::size_t state_index = row.start_state_index; state_index <= row.end_state_index; ++state_index) {
      state_indices.push_back(state_index);
    }
    const auto body_z_axes_nav = BodyZAxesNavFromInitialValues(initial_values, state_indices);
    if (!body_z_axes_nav.has_value()) {
      continue;
    }
    const auto body_axes_nav = BodyFrameAxesNavFromStateSource(
      initial_values,
      state_timestamps,
      reference_states,
      state_indices,
      row.horizontal_leakage_correction_enabled);
    if (!body_axes_nav.has_value()) {
      continue;
    }
    const std::vector<gtsam::Vector3> effective_body_z_axes_nav =
      BodyZAxesFromBodyFrameAxes(*body_axes_nav);
    if (state_diagnostics != nullptr) {
      state_diagnostics->reserve(state_diagnostics->size() + state_indices.size());
      for (std::size_t offset = 0U; offset < state_indices.size(); ++offset) {
        const std::size_t state_index = state_indices[offset];
        const auto velocity = optimized_values.at<gtsam::Vector3>(symbol::V(state_index));
        const factor::BodyFrameAxesNav fixed_body_axes = (*body_axes_nav)[offset];
        const gtsam::Vector3 fixed_axis = fixed_body_axes.body_z_axis_nav;
        const auto optimized_pose = optimized_values.at<gtsam::Pose3>(symbol::X(state_index));
        const gtsam::Vector3 optimized_pose_axis =
          factor::BodyZAxisNavFromPose(optimized_pose);
        factor::BodyZHorizontalLeakageModel leakage;
        if (row.horizontal_leakage_correction_enabled) {
          leakage.leak_x_rad = row.horizontal_leakage_x_rad;
          leakage.leak_y_rad = row.horizontal_leakage_y_rad;
        }

        BodyZNHCStateDiagnosticRow state_row;
        state_row.window_index = row.window_index;
        state_row.state_index = state_index;
        state_row.time_s = state_timestamps[state_index];
        state_row.nhc_region_type = row.window_type;
        state_row.velocity_factor_used =
          std::find(
            row.velocity_factor_state_indices.begin(),
            row.velocity_factor_state_indices.end(),
            state_index) != row.velocity_factor_state_indices.end();
        state_row.effective_velocity_sigma_mps =
          state_row.velocity_factor_used ? row.applied_velocity_sigma_mps :
                                           std::numeric_limits<double>::quiet_NaN();
        state_row.fixed_axis_x = fixed_axis.x();
        state_row.fixed_axis_y = fixed_axis.y();
        state_row.fixed_axis_z = fixed_axis.z();
        state_row.vx_mps = velocity.x();
        state_row.vy_mps = velocity.y();
        state_row.vz_mps = velocity.z();
        state_row.horizontal_speed_mps =
          std::hypot(velocity.x(), velocity.y());
        state_row.v_body_x_mps = factor::BodyXVelocityMps(fixed_body_axes, velocity);
        state_row.v_body_y_mps = factor::BodyYVelocityMps(fixed_body_axes, velocity);
        state_row.raw_v_body_z_mps = factor::BodyZRawVelocityMps(fixed_body_axes, velocity);
        state_row.horizontal_leakage_x_rad = leakage.leak_x_rad;
        state_row.horizontal_leakage_y_rad = leakage.leak_y_rad;
        state_row.leakage_correction_mps =
          factor::BodyZLeakageCorrectionMps(fixed_body_axes, leakage, velocity);
        state_row.corrected_v_body_z_mps =
          factor::BodyZLeakageCorrectedVelocityMps(fixed_body_axes, leakage, velocity);
        state_row.fixed_horizontal_projection_mps =
          fixed_axis.x() * velocity.x() + fixed_axis.y() * velocity.y();
        state_row.fixed_vertical_projection_mps =
          fixed_axis.z() * velocity.z();
        state_row.fixed_body_z_velocity_mps =
          state_row.fixed_horizontal_projection_mps +
          state_row.fixed_vertical_projection_mps;
        state_row.optimized_pose_axis_x = optimized_pose_axis.x();
        state_row.optimized_pose_axis_y = optimized_pose_axis.y();
        state_row.optimized_pose_axis_z = optimized_pose_axis.z();
        state_row.optimized_pose_horizontal_projection_mps =
          optimized_pose_axis.x() * velocity.x() +
          optimized_pose_axis.y() * velocity.y();
        state_row.optimized_pose_vertical_projection_mps =
          optimized_pose_axis.z() * velocity.z();
        state_row.optimized_pose_body_z_velocity_mps =
          state_row.optimized_pose_horizontal_projection_mps +
          state_row.optimized_pose_vertical_projection_mps;
        state_diagnostics->push_back(state_row);
      }
    }
    const auto metrics =
      ComputeMetrics(optimized_values, state_timestamps, state_indices, effective_body_z_axes_nav);
    if (!metrics.has_value()) {
      continue;
    }
    row.optimized_mean_abs_body_z_velocity_mps = metrics->mean_abs_body_z_velocity_mps;
    row.optimized_max_abs_body_z_velocity_mps = metrics->max_abs_body_z_velocity_mps;
    row.optimized_body_z_displacement_m = metrics->body_z_displacement_m;
    row.max_velocity_residual_mps = metrics->max_abs_body_z_velocity_mps;
    row.displacement_residual_m = metrics->body_z_displacement_m;

    if (row.horizontal_leakage_correction_enabled) {
      factor::BodyZHorizontalLeakageModel leakage;
      leakage.leak_x_rad = row.horizontal_leakage_x_rad;
      leakage.leak_y_rad = row.horizontal_leakage_y_rad;
      const auto corrected_metrics =
        ComputeCorrectedMetrics(optimized_values, state_timestamps, state_indices, *body_axes_nav, leakage);
      if (corrected_metrics.has_value()) {
        row.optimized_mean_abs_corrected_body_z_velocity_mps =
          corrected_metrics->mean_abs_body_z_velocity_mps;
        row.optimized_max_abs_corrected_body_z_velocity_mps =
          corrected_metrics->max_abs_body_z_velocity_mps;
        row.optimized_corrected_body_z_displacement_m =
          corrected_metrics->body_z_displacement_m;
        row.max_velocity_residual_mps =
          corrected_metrics->max_abs_body_z_velocity_mps;
        row.displacement_residual_m =
          corrected_metrics->body_z_displacement_m;
        if (run_summary != nullptr) {
          if (!std::isfinite(run_summary->body_z_nhc_corrected_max_velocity_residual_mps)) {
            run_summary->body_z_nhc_corrected_max_velocity_residual_mps = 0.0;
          }
          if (!std::isfinite(run_summary->body_z_nhc_corrected_max_abs_displacement_residual_m)) {
            run_summary->body_z_nhc_corrected_max_abs_displacement_residual_m = 0.0;
          }
          run_summary->body_z_nhc_corrected_max_velocity_residual_mps = std::max(
            run_summary->body_z_nhc_corrected_max_velocity_residual_mps,
            corrected_metrics->max_abs_body_z_velocity_mps);
          run_summary->body_z_nhc_corrected_max_abs_displacement_residual_m = std::max(
            run_summary->body_z_nhc_corrected_max_abs_displacement_residual_m,
            std::abs(corrected_metrics->body_z_displacement_m));
        }
      }
    }

    const auto pose_metrics = ComputePoseBodyZMetrics(optimized_values, state_timestamps, state_indices);
    if (pose_metrics.has_value()) {
      row.optimized_pose_mean_abs_body_z_velocity_mps = pose_metrics->mean_abs_body_z_velocity_mps;
      row.optimized_pose_max_abs_body_z_velocity_mps = pose_metrics->max_abs_body_z_velocity_mps;
      row.optimized_pose_body_z_displacement_m = pose_metrics->body_z_displacement_m;
    }
    PopulateAttitudeRanges(optimized_values, state_indices, row);
  }
}

}  // namespace offline_lc_minimal
