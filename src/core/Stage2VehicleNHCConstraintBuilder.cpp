#include "offline_lc_minimal/core/Stage2VehicleNHCConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <unordered_set>
#include <utility>

#include <Eigen/Dense>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/BodyZNHCWeightPlanner.h"
#include "offline_lc_minimal/factor/VehicleVelocityNHCFactor.h"
#include "offline_lc_minimal/factor/VehicleZFixedForwardNHCFactor.h"

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

bool OverlapsAnyWindow(
  const double start_time_s,
  const double end_time_s,
  const std::vector<BodyZJumpConstraintWindow> &windows) {
  for (const auto &window : windows) {
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      return true;
    }
  }
  return false;
}

bool HasReferenceStates(
  const std::vector<ReferenceNodeState> *reference_states,
  const std::vector<double> &state_timestamps) {
  return reference_states != nullptr && reference_states->size() == state_timestamps.size();
}

factor::BodyFrameAxesNav BodyAxesFromReference(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t state_index) {
  return factor::BodyFrameAxesNavFromPose(reference_states[state_index].pose);
}

double ReferenceBodyXMps(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t state_index) {
  const auto axes = BodyAxesFromReference(reference_states, state_index);
  return factor::BodyXVelocityMps(axes, reference_states[state_index].velocity);
}

gtsam::Vector3 ReferenceNavVelocity(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t state_index) {
  return reference_states[state_index].velocity;
}

struct VelocityStats {
  std::size_t count = 0;
  double rms = std::numeric_limits<double>::quiet_NaN();
  double max_abs = std::numeric_limits<double>::quiet_NaN();
};

struct VehicleVelocityStats {
  VelocityStats raw_y;
  VelocityStats vehicle_y;
  VelocityStats raw_z;
  VelocityStats vehicle_z;
};

void AccumulateValue(
  const double value,
  double &sum_squares,
  double &max_abs) {
  sum_squares += value * value;
  max_abs = std::max(max_abs, std::abs(value));
}

VelocityStats FinalizeStats(
  const std::size_t count,
  const double sum_squares,
  const double max_abs) {
  VelocityStats stats;
  stats.count = count;
  if (count == 0U) {
    return stats;
  }
  stats.rms = std::sqrt(sum_squares / static_cast<double>(count));
  stats.max_abs = max_abs;
  return stats;
}

VehicleVelocityStats ComputeVehicleVelocityStats(
  const gtsam::Values &values,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<std::size_t> &state_indices,
  const factor::VehicleMountLeakageModel &mount) {
  (void)state_timestamps;
  double raw_y_sum_squares = 0.0;
  double vehicle_y_sum_squares = 0.0;
  double raw_z_sum_squares = 0.0;
  double vehicle_z_sum_squares = 0.0;
  double raw_y_max_abs = 0.0;
  double vehicle_y_max_abs = 0.0;
  double raw_z_max_abs = 0.0;
  double vehicle_z_max_abs = 0.0;
  std::size_t count = 0U;
  for (const std::size_t state_index : state_indices) {
    const auto velocity = values.at<gtsam::Vector3>(symbol::V(state_index));
    const auto axes = BodyAxesFromReference(reference_states, state_index);
    const double body_y = factor::BodyYVelocityMps(axes, velocity);
    const double body_z = factor::BodyZRawVelocityMps(axes, velocity);
    const double reference_body_x = ReferenceBodyXMps(reference_states, state_index);
    const double vehicle_y = body_y;
    const double vehicle_z =
      factor::VehicleZVelocityMpsWithFixedForward(
        axes,
        mount,
        velocity,
        reference_body_x,
        ReferenceNavVelocity(reference_states, state_index));
    if (!std::isfinite(body_y) || !std::isfinite(body_z) ||
        !std::isfinite(vehicle_y) || !std::isfinite(vehicle_z)) {
      continue;
    }
    AccumulateValue(body_y, raw_y_sum_squares, raw_y_max_abs);
    AccumulateValue(vehicle_y, vehicle_y_sum_squares, vehicle_y_max_abs);
    AccumulateValue(body_z, raw_z_sum_squares, raw_z_max_abs);
    AccumulateValue(vehicle_z, vehicle_z_sum_squares, vehicle_z_max_abs);
    ++count;
  }

  VehicleVelocityStats stats;
  stats.raw_y = FinalizeStats(count, raw_y_sum_squares, raw_y_max_abs);
  stats.vehicle_y = FinalizeStats(count, vehicle_y_sum_squares, vehicle_y_max_abs);
  stats.raw_z = FinalizeStats(count, raw_z_sum_squares, raw_z_max_abs);
  stats.vehicle_z = FinalizeStats(count, vehicle_z_sum_squares, vehicle_z_max_abs);
  return stats;
}

std::vector<std::size_t> DynamicStateIndicesOutsideWindows(
  const std::vector<double> &state_timestamps,
  const std::size_t dynamic_start_index,
  const std::vector<BodyZJumpConstraintWindow> &excluded_windows) {
  std::vector<std::size_t> state_indices;
  for (std::size_t state_index = dynamic_start_index; state_index < state_timestamps.size(); ++state_index) {
    const double time_s = state_timestamps[state_index];
    if (OverlapsAnyWindow(time_s, time_s, excluded_windows)) {
      continue;
    }
    state_indices.push_back(state_index);
  }
  return state_indices;
}

struct MountLeakageInitialEstimate {
  bool valid = false;
  std::string skip_reason = "UNSET";
  std::size_t used_sample_count = 0U;
  factor::VehicleMountLeakageModel model;
};

MountLeakageInitialEstimate EstimateMountLeakage(
  const OfflineRunnerConfig &config,
  const gtsam::Values &values,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t dynamic_start_index,
  const std::vector<BodyZJumpConstraintWindow> &excluded_windows) {
  MountLeakageInitialEstimate estimate;
  const std::vector<std::size_t> state_indices =
    DynamicStateIndicesOutsideWindows(
      state_timestamps,
      dynamic_start_index,
      excluded_windows);
  double zx_normal = 0.0;
  double zx_rhs = 0.0;
  const double min_speed_mps =
    std::max(0.0, config.body_z_nhc_horizontal_leakage_min_speed_mps);
  for (const std::size_t state_index : state_indices) {
    const auto velocity = values.at<gtsam::Vector3>(symbol::V(state_index));
    const auto axes = BodyAxesFromReference(reference_states, state_index);
    const double reference_body_x = ReferenceBodyXMps(reference_states, state_index);
    const double body_z = factor::BodyZRawVelocityMps(axes, velocity);
    if (!std::isfinite(reference_body_x) || !std::isfinite(body_z)) {
      continue;
    }
    if (std::abs(reference_body_x) < min_speed_mps) {
      continue;
    }
    zx_normal += reference_body_x * reference_body_x;
    zx_rhs += reference_body_x * body_z;
    ++estimate.used_sample_count;
  }

  if (estimate.used_sample_count <
      static_cast<std::size_t>(std::max(1, config.body_z_nhc_horizontal_leakage_min_sample_count))) {
    estimate.skip_reason = "INSUFFICIENT_SAMPLES";
    return estimate;
  }
  if (zx_normal <= 1.0e-12) {
    estimate.skip_reason = "RANK_DEFICIENT";
    return estimate;
  }
  const double max_abs_coeff =
    std::max(0.0, config.body_z_nhc_horizontal_leakage_max_abs_coeff_rad);
  estimate.model = factor::VehicleMountLeakageModel{
    std::clamp(zx_rhs / zx_normal, -max_abs_coeff, max_abs_coeff),
    0.0,
    0.0};
  estimate.valid = true;
  estimate.skip_reason = "ESTIMATED";
  return estimate;
}

void SetMountLeakageInitialValue(
  gtsam::Values &values,
  const gtsam::Key key,
  const factor::VehicleMountLeakageModel &model) {
  const gtsam::Vector3 vector = factor::VehicleMountLeakageVector(model);
  if (values.exists(key)) {
    values.update(key, vector);
  } else {
    values.insert(key, vector);
  }
}

std::vector<factor::BodyFrameAxesNav> BodyAxesForStates(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<std::size_t> &state_indices) {
  std::vector<factor::BodyFrameAxesNav> axes;
  axes.reserve(state_indices.size());
  for (const std::size_t state_index : state_indices) {
    axes.push_back(BodyAxesFromReference(reference_states, state_index));
  }
  return axes;
}

std::vector<double> ReferenceBodyXForStates(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<std::size_t> &state_indices) {
  std::vector<double> reference_body_x_mps;
  reference_body_x_mps.reserve(state_indices.size());
  for (const std::size_t state_index : state_indices) {
    reference_body_x_mps.push_back(ReferenceBodyXMps(reference_states, state_index));
  }
  return reference_body_x_mps;
}

std::vector<gtsam::Vector3> ReferenceNavVelocitiesForStates(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<std::size_t> &state_indices) {
  std::vector<gtsam::Vector3> reference_nav_velocities;
  reference_nav_velocities.reserve(state_indices.size());
  for (const std::size_t state_index : state_indices) {
    reference_nav_velocities.push_back(ReferenceNavVelocity(reference_states, state_index));
  }
  return reference_nav_velocities;
}

std::vector<std::size_t> UniqueStateIndices(
  const std::vector<Stage2VehicleNHCStateDiagnosticRow> &diagnostics) {
  std::vector<std::size_t> state_indices;
  std::unordered_set<std::size_t> seen;
  for (const auto &row : diagnostics) {
    if (seen.insert(row.state_index).second) {
      state_indices.push_back(row.state_index);
    }
  }
  std::sort(state_indices.begin(), state_indices.end());
  return state_indices;
}

void PopulateMountStats(
  const gtsam::Values &values,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<std::size_t> &state_indices,
  const factor::VehicleMountLeakageModel &mount,
  const bool optimized,
  Stage2MountLeakageDiagnosticRow &row) {
  const VehicleVelocityStats stats =
    ComputeVehicleVelocityStats(values, state_timestamps, reference_states, state_indices, mount);
  if (optimized) {
    row.optimized_raw_y_rms_mps = stats.raw_y.rms;
    row.optimized_raw_y_max_abs_mps = stats.raw_y.max_abs;
    row.optimized_vehicle_y_rms_mps = stats.vehicle_y.rms;
    row.optimized_vehicle_y_max_abs_mps = stats.vehicle_y.max_abs;
    row.optimized_raw_z_rms_mps = stats.raw_z.rms;
    row.optimized_raw_z_max_abs_mps = stats.raw_z.max_abs;
    row.optimized_vehicle_z_rms_mps = stats.vehicle_z.rms;
    row.optimized_vehicle_z_max_abs_mps = stats.vehicle_z.max_abs;
  } else {
    row.initial_raw_y_rms_mps = stats.raw_y.rms;
    row.initial_raw_y_max_abs_mps = stats.raw_y.max_abs;
    row.initial_vehicle_y_rms_mps = stats.vehicle_y.rms;
    row.initial_vehicle_y_max_abs_mps = stats.vehicle_y.max_abs;
    row.initial_raw_z_rms_mps = stats.raw_z.rms;
    row.initial_raw_z_max_abs_mps = stats.raw_z.max_abs;
    row.initial_vehicle_z_rms_mps = stats.vehicle_z.rms;
    row.initial_vehicle_z_max_abs_mps = stats.vehicle_z.max_abs;
  }
}

}  // namespace

Stage2VehicleNHCConstraintBuilder::Stage2VehicleNHCConstraintBuilder(
  Stage2VehicleNHCConstraintBuildRequest request)
    : request_(std::move(request)) {}

void Stage2VehicleNHCConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.jump_windows == nullptr || request_.initial_values == nullptr ||
      request_.reference_states == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.mount_diagnostics == nullptr ||
      request_.state_diagnostics == nullptr) {
    throw std::runtime_error("Stage2VehicleNHCConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_stage2_velocity_optimization ||
      !request_.config->enable_stage2_vehicle_nhc_constraint) {
    return;
  }
  if (!HasReferenceStates(request_.reference_states, *request_.state_timestamps)) {
    throw std::runtime_error("stage2 vehicle NHC requires one fixed reference state per graph state");
  }

  request_.run_summary->stage2_velocity_optimization_enabled = true;

  std::vector<BodyZJumpConstraintWindow> jump_windows = BuildJumpWindows();
  const MountLeakageInitialEstimate mount_estimate =
    EstimateMountLeakage(
      *request_.config,
      *request_.initial_values,
      *request_.state_timestamps,
      *request_.reference_states,
      request_.dynamic_start_index,
      jump_windows);
  const factor::VehicleMountLeakageModel initial_mount =
    mount_estimate.valid ? mount_estimate.model : factor::VehicleMountLeakageModel{};
  SetMountLeakageInitialValue(
    *request_.initial_values,
    request_.mount_leakage_key,
    initial_mount);
  request_.graph->add(gtsam::PriorFactor<gtsam::Vector3>(
    request_.mount_leakage_key,
    factor::VehicleMountLeakageVector(initial_mount),
    gtsam::noiseModel::Isotropic::Sigma(
      3,
      request_.config->stage2_mount_leakage_prior_sigma_rad)));
  request_.run_summary->stage2_mount_initial_k_zx_rad = initial_mount.k_zx_rad;
  request_.run_summary->stage2_mount_initial_k_zy_rad = initial_mount.k_zy_rad;
  request_.run_summary->stage2_mount_initial_k_yx_rad = initial_mount.k_yx_rad;

  request_.mount_diagnostics->clear();
  Stage2MountLeakageDiagnosticRow mount_row;
  mount_row.enabled = true;
  mount_row.estimate_valid = mount_estimate.valid;
  mount_row.skip_reason = mount_estimate.skip_reason;
  mount_row.used_sample_count = mount_estimate.used_sample_count;
  mount_row.prior_sigma_rad = request_.config->stage2_mount_leakage_prior_sigma_rad;
  mount_row.initial_k_zx_rad = initial_mount.k_zx_rad;
  mount_row.initial_k_zy_rad = initial_mount.k_zy_rad;
  mount_row.initial_k_yx_rad = initial_mount.k_yx_rad;
  const std::vector<std::size_t> initial_metric_states =
    DynamicStateIndicesOutsideWindows(
      *request_.state_timestamps,
      request_.dynamic_start_index,
      jump_windows);
  PopulateMountStats(
    *request_.initial_values,
    *request_.state_timestamps,
    *request_.reference_states,
    initial_metric_states,
    initial_mount,
    false,
    mount_row);
  request_.mount_diagnostics->push_back(mount_row);

  const std::vector<BodyZJumpConstraintWindow> global_windows = BuildGlobalWindows(jump_windows);
  std::vector<BodyZJumpConstraintWindow> windows = jump_windows;
  windows.insert(windows.end(), global_windows.begin(), global_windows.end());

  BodyZNHCWeightPlanner weight_planner;
  const std::size_t window_index_offset = 0U;
  request_.state_diagnostics->clear();
  for (std::size_t window_index = 0U; window_index < windows.size(); ++window_index) {
    const BodyZJumpConstraintWindow &window = windows[window_index];
    const std::vector<std::size_t> state_indices =
      StateIndicesInWindow(window.start_time_s, window.end_time_s);
    if (state_indices.size() < 2U) {
      ++request_.run_summary->stage2_vehicle_nhc_skipped_short_window_count;
      continue;
    }
    const double actual_span_s =
      (*request_.state_timestamps)[state_indices.back()] -
      (*request_.state_timestamps)[state_indices.front()];
    if (!std::isfinite(actual_span_s) ||
        actual_span_s < request_.config->body_z_nhc_min_window_s) {
      ++request_.run_summary->stage2_vehicle_nhc_skipped_short_window_count;
      continue;
    }

    const double z_velocity_sigma_mps = window.source_window_count > 0U
      ? request_.config->body_z_nhc_jump_velocity_sigma_mps
      : request_.config->body_z_nhc_global_velocity_sigma_mps;
    const double z_displacement_sigma_m = window.source_window_count > 0U
      ? request_.config->body_z_nhc_jump_displacement_sigma_m
      : request_.config->body_z_nhc_global_displacement_sigma_m;
    BodyZNHCWeightPlannerWindowRequest weight_request;
    weight_request.strict_effective_weighting =
      request_.config->enable_body_z_nhc_strict_effective_weighting;
    weight_request.state_indices = state_indices;
    weight_request.start_time_s = window.start_time_s;
    weight_request.end_time_s = window.end_time_s;
    weight_request.target_velocity_sigma_mps = z_velocity_sigma_mps;
    weight_request.target_displacement_sigma_m = z_displacement_sigma_m;
    const BodyZNHCWindowWeightPlan weight_plan =
      weight_planner.PlanWindow(weight_request);

    std::vector<gtsam::Key> velocity_keys;
    std::vector<double> state_times_s;
    const auto body_axes_nav = BodyAxesForStates(*request_.reference_states, state_indices);
    const auto reference_body_x_mps =
      ReferenceBodyXForStates(*request_.reference_states, state_indices);
    const auto reference_nav_velocities =
      ReferenceNavVelocitiesForStates(*request_.reference_states, state_indices);
    velocity_keys.reserve(state_indices.size());
    state_times_s.reserve(state_indices.size());
    for (std::size_t offset = 0U; offset < state_indices.size(); ++offset) {
      const std::size_t state_index = state_indices[offset];
      velocity_keys.push_back(symbol::V(state_index));
      state_times_s.push_back((*request_.state_timestamps)[state_index]);
      if (weight_plan.velocity_factor_used[offset]) {
        request_.graph->add(factor::VehicleZVelocityZeroFixedForwardFactor(
          symbol::V(state_index),
          request_.mount_leakage_key,
          body_axes_nav[offset],
          reference_body_x_mps[offset],
          reference_nav_velocities[offset],
          gtsam::noiseModel::Isotropic::Sigma(1, weight_plan.applied_velocity_sigma_mps)));
        ++request_.run_summary->stage2_vehicle_z_nhc_velocity_factor_count;
      }

      Stage2VehicleNHCStateDiagnosticRow state_row;
      state_row.window_index = window_index_offset + window_index;
      state_row.state_index = state_index;
      state_row.time_s = (*request_.state_timestamps)[state_index];
      state_row.nhc_region_type = window.source_window_count > 0U ? "JUMP" : "GLOBAL";
      state_row.velocity_factor_used = weight_plan.velocity_factor_used[offset];
      state_row.effective_vehicle_z_sigma_mps =
        state_row.velocity_factor_used ? weight_plan.applied_velocity_sigma_mps :
                                         std::numeric_limits<double>::quiet_NaN();
      request_.state_diagnostics->push_back(state_row);
    }

    request_.graph->add(factor::VehicleZWindowDisplacementZeroFixedForwardFactor(
      velocity_keys,
      request_.mount_leakage_key,
      body_axes_nav,
      reference_body_x_mps,
      reference_nav_velocities,
      state_times_s,
      gtsam::noiseModel::Isotropic::Sigma(1, weight_plan.applied_displacement_sigma_m)));
    ++request_.run_summary->stage2_vehicle_z_nhc_displacement_factor_count;
    ++request_.run_summary->stage2_vehicle_nhc_window_count;
    request_.run_summary->stage2_vehicle_nhc_velocity_duplicate_state_count +=
      weight_plan.velocity_state_duplicate_count;
    request_.run_summary->stage2_vehicle_nhc_interval_overlap_count +=
      weight_plan.interval_overlap_count;
  }

  request_.run_summary->stage2_vehicle_nhc_unique_velocity_factor_count =
    weight_planner.uniqueVelocityFactorStateCount();
}

std::vector<BodyZJumpConstraintWindow> Stage2VehicleNHCConstraintBuilder::BuildJumpWindows() const {
  for (const auto &jump_window : *request_.jump_windows) {
    if (!std::isfinite(jump_window.start_time_s) ||
        !std::isfinite(jump_window.end_time_s) ||
        jump_window.end_time_s < jump_window.start_time_s) {
      ++request_.run_summary->stage2_vehicle_nhc_skipped_invalid_count;
    }
  }
  return BuildBodyZJumpConstraintWindows(
    *request_.jump_windows,
    BodyZNHCJumpConstraintWindowOptions(*request_.config));
}

std::vector<BodyZJumpConstraintWindow> Stage2VehicleNHCConstraintBuilder::BuildGlobalWindows(
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
    while (start_time_s + request_.config->body_z_nhc_min_window_s <=
           segment_end_s + kTimeEpsilonS) {
      BodyZJumpConstraintWindow window;
      window.start_time_s = start_time_s;
      window.end_time_s =
        std::min(start_time_s + request_.config->body_z_nhc_global_window_s, segment_end_s);
      windows.push_back(window);
      start_time_s += request_.config->body_z_nhc_global_stride_s;
      if (strict_weighting) {
        start_time_s += kStrictWindowBoundaryPaddingS;
      }
    }
    segment_start_s =
      std::max(segment_start_s, jump_window.end_time_s + boundary_padding_s);
  }

  double start_time_s = segment_start_s;
  while (start_time_s + request_.config->body_z_nhc_min_window_s <= final_time_s + kTimeEpsilonS) {
    BodyZJumpConstraintWindow window;
    window.start_time_s = start_time_s;
    window.end_time_s =
      std::min(start_time_s + request_.config->body_z_nhc_global_window_s, final_time_s);
    windows.push_back(window);
    start_time_s += request_.config->body_z_nhc_global_stride_s;
    if (strict_weighting) {
      start_time_s += kStrictWindowBoundaryPaddingS;
    }
  }
  return windows;
}

std::vector<std::size_t> Stage2VehicleNHCConstraintBuilder::StateIndicesInWindow(
  const double start_time_s,
  const double end_time_s) const {
  std::vector<std::size_t> state_indices;
  const std::size_t start_index =
    std::min(request_.dynamic_start_index, request_.state_timestamps->size());
  for (std::size_t state_index = start_index; state_index < request_.state_timestamps->size(); ++state_index) {
    const double time_s = (*request_.state_timestamps)[state_index];
    if (time_s + kTimeEpsilonS < start_time_s) {
      continue;
    }
    if (time_s > end_time_s + kTimeEpsilonS) {
      break;
    }
    state_indices.push_back(state_index);
  }
  return state_indices;
}

void PopulateStage2VehicleNHCDiagnostics(
  const gtsam::Values &initial_values,
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const gtsam::Key mount_leakage_key,
  std::vector<Stage2MountLeakageDiagnosticRow> &mount_diagnostics,
  std::vector<Stage2VehicleNHCStateDiagnosticRow> &state_diagnostics,
  RunSummary &run_summary) {
  if (state_diagnostics.empty()) {
    return;
  }
  const auto initial_mount_vector = initial_values.at<gtsam::Vector3>(mount_leakage_key);
  const auto optimized_mount_vector = optimized_values.at<gtsam::Vector3>(mount_leakage_key);
  const auto initial_mount = factor::VehicleMountLeakageModelFromVector(initial_mount_vector);
  const auto optimized_mount = factor::VehicleMountLeakageModelFromVector(optimized_mount_vector);
  run_summary.stage2_mount_k_zx_rad = optimized_mount.k_zx_rad;
  run_summary.stage2_mount_k_zy_rad = optimized_mount.k_zy_rad;
  run_summary.stage2_mount_k_yx_rad = optimized_mount.k_yx_rad;

  for (auto &row : state_diagnostics) {
    if (row.state_index >= state_timestamps.size() ||
        row.state_index >= reference_states.size()) {
      continue;
    }
    const auto velocity = optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index));
    const auto axes = BodyAxesFromReference(reference_states, row.state_index);
    const double body_x = factor::BodyXVelocityMps(axes, velocity);
    const double body_y = factor::BodyYVelocityMps(axes, velocity);
    const double body_z = factor::BodyZRawVelocityMps(axes, velocity);
    const double reference_body_x = ReferenceBodyXMps(reference_states, row.state_index);
    row.vx_mps = velocity.x();
    row.vy_mps = velocity.y();
    row.vz_mps = velocity.z();
    row.v_body_x_mps = body_x;
    row.v_body_y_mps = body_y;
    row.v_body_z_mps = body_z;
    row.k_zx_rad = optimized_mount.k_zx_rad;
    row.k_zy_rad = optimized_mount.k_zy_rad;
    row.k_yx_rad = optimized_mount.k_yx_rad;
    row.vehicle_y_correction_mps = 0.0;
    row.vehicle_z_correction_from_x_mps = optimized_mount.k_zx_rad * reference_body_x;
    row.vehicle_z_correction_from_y_mps = optimized_mount.k_zy_rad * body_y;
    row.v_vehicle_y_mps = body_y;
    row.v_vehicle_z_mps =
      factor::VehicleZVelocityMpsWithFixedForward(
        axes,
        optimized_mount,
        velocity,
        reference_body_x,
        ReferenceNavVelocity(reference_states, row.state_index));
  }

  if (!mount_diagnostics.empty()) {
    auto &mount_row = mount_diagnostics.front();
    mount_row.optimized_k_zx_rad = optimized_mount.k_zx_rad;
    mount_row.optimized_k_zy_rad = optimized_mount.k_zy_rad;
    mount_row.optimized_k_yx_rad = optimized_mount.k_yx_rad;
    const gtsam::Vector3 prior_residual =
      optimized_mount_vector - initial_mount_vector;
    mount_row.prior_residual_norm =
      prior_residual.norm() / std::max(mount_row.prior_sigma_rad, 1.0e-12);
    const std::vector<std::size_t> state_indices = UniqueStateIndices(state_diagnostics);
    PopulateMountStats(
      initial_values,
      state_timestamps,
      reference_states,
      state_indices,
      initial_mount,
      false,
      mount_row);
    PopulateMountStats(
      optimized_values,
      state_timestamps,
      reference_states,
      state_indices,
      optimized_mount,
      true,
      mount_row);
  }
}

}  // namespace offline_lc_minimal
