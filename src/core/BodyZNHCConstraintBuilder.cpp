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

#include "offline_lc_minimal/factor/BodyZVelocityZeroFactor.h"
#include "offline_lc_minimal/factor/BodyZWindowDisplacementZeroFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

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

  std::vector<BodyZJumpConstraintWindow> windows = BuildJumpWindows();
  const std::vector<BodyZJumpConstraintWindow> global_windows = BuildGlobalWindows(windows);
  windows.insert(windows.end(), global_windows.begin(), global_windows.end());
  request_.diagnostics->reserve(request_.diagnostics->size() + windows.size());

  const std::size_t window_index_offset = request_.diagnostics->size();
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

    const auto initial_metrics =
      ComputeMetrics(
        *request_.initial_values,
        *request_.state_timestamps,
        state_indices,
        *body_z_axes_nav);
    if (!initial_metrics.has_value()) {
      row.skip_reason = "INVALID_METRICS";
      ++request_.run_summary->body_z_nhc_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    row.initial_mean_abs_body_z_velocity_mps = initial_metrics->mean_abs_body_z_velocity_mps;
    row.initial_max_abs_body_z_velocity_mps = initial_metrics->max_abs_body_z_velocity_mps;
    row.initial_body_z_displacement_m = initial_metrics->body_z_displacement_m;

    std::vector<gtsam::Key> velocity_keys;
    std::vector<double> state_times_s;
    velocity_keys.reserve(state_indices.size());
    state_times_s.reserve(state_indices.size());
    for (std::size_t offset = 0U; offset < state_indices.size(); ++offset) {
      const std::size_t state_index = state_indices[offset];
      velocity_keys.push_back(symbol::V(state_index));
      state_times_s.push_back((*request_.state_timestamps)[state_index]);
      request_.graph->add(factor::BodyZVelocityZeroFactor(
        symbol::V(state_index),
        (*body_z_axes_nav)[offset],
        gtsam::noiseModel::Isotropic::Sigma(1, velocity_sigma_mps)));
      ++row.velocity_factor_count;
    }

    request_.graph->add(factor::BodyZWindowDisplacementZeroFactor(
      velocity_keys,
      *body_z_axes_nav,
      state_times_s,
      gtsam::noiseModel::Isotropic::Sigma(1, displacement_sigma_m)));
    row.displacement_factor_count = 1U;
    row.factor_added = true;
    row.skip_reason = "ADDED";

    ++request_.run_summary->body_z_nhc_window_count;
    request_.run_summary->body_z_nhc_velocity_factor_count += row.velocity_factor_count;
    request_.run_summary->body_z_nhc_displacement_factor_count += row.displacement_factor_count;
    request_.diagnostics->push_back(row);
  }
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

  double start_time_s = (*request_.state_timestamps)[request_.dynamic_start_index];
  const double final_time_s = request_.state_timestamps->back();
  while (start_time_s + request_.config->body_z_nhc_min_window_s <= final_time_s + kTimeEpsilonS) {
    const double end_time_s =
      std::min(start_time_s + request_.config->body_z_nhc_global_window_s, final_time_s);
    if (!OverlapsAnyWindow(start_time_s, end_time_s, jump_windows)) {
      BodyZJumpConstraintWindow window;
      window.source_window_index = 0U;
      window.source_window_count = 0U;
      window.start_time_s = start_time_s;
      window.end_time_s = end_time_s;
      windows.push_back(window);
    }
    start_time_s += request_.config->body_z_nhc_global_stride_s;
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
  return row;
}

void PopulateBodyZNHCDiagnostics(
  const gtsam::Values &initial_values,
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps,
  std::vector<BodyZNHCDiagnosticRow> &diagnostics,
  std::vector<BodyZNHCStateDiagnosticRow> *state_diagnostics) {
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
    if (state_diagnostics != nullptr) {
      state_diagnostics->reserve(state_diagnostics->size() + state_indices.size());
      for (std::size_t offset = 0U; offset < state_indices.size(); ++offset) {
        const std::size_t state_index = state_indices[offset];
        const auto velocity = optimized_values.at<gtsam::Vector3>(symbol::V(state_index));
        const gtsam::Vector3 fixed_axis = (*body_z_axes_nav)[offset];
        const auto optimized_pose = optimized_values.at<gtsam::Pose3>(symbol::X(state_index));
        const gtsam::Vector3 optimized_pose_axis =
          factor::BodyZAxisNavFromPose(optimized_pose);

        BodyZNHCStateDiagnosticRow state_row;
        state_row.window_index = row.window_index;
        state_row.state_index = state_index;
        state_row.time_s = state_timestamps[state_index];
        state_row.fixed_axis_x = fixed_axis.x();
        state_row.fixed_axis_y = fixed_axis.y();
        state_row.fixed_axis_z = fixed_axis.z();
        state_row.vx_mps = velocity.x();
        state_row.vy_mps = velocity.y();
        state_row.vz_mps = velocity.z();
        state_row.horizontal_speed_mps =
          std::hypot(velocity.x(), velocity.y());
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
    const auto metrics = ComputeMetrics(optimized_values, state_timestamps, state_indices, *body_z_axes_nav);
    if (!metrics.has_value()) {
      continue;
    }
    row.optimized_mean_abs_body_z_velocity_mps = metrics->mean_abs_body_z_velocity_mps;
    row.optimized_max_abs_body_z_velocity_mps = metrics->max_abs_body_z_velocity_mps;
    row.optimized_body_z_displacement_m = metrics->body_z_displacement_m;
    row.max_velocity_residual_mps = metrics->max_abs_body_z_velocity_mps;
    row.displacement_residual_m = metrics->body_z_displacement_m;

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
