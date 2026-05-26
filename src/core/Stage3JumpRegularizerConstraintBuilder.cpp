#include "offline_lc_minimal/core/Stage3JumpRegularizerConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaDeadbandFactor.h"

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

const BodyZJumpConstraintWindow *FindWindowForTime(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double time_s) {
  for (const auto &window : windows) {
    if (time_s + kTimeEpsilonS >= window.start_time_s &&
        time_s <= window.end_time_s + kTimeEpsilonS) {
      return &window;
    }
  }
  return nullptr;
}

const BodyZJumpConstraintWindow *FindWindowForInterval(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double start_time_s,
  const double end_time_s) {
  for (const auto &window : windows) {
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      return &window;
    }
  }
  return nullptr;
}

void InitializeSummary(const OfflineRunnerConfig &config, RunSummary &summary) {
  summary.stage3_jump_velocity_smoothness_regularizer_enabled =
    config.enable_stage3_jump_velocity_smoothness_regularizer;
  summary.stage3_jump_height_highfreq_deadband_enabled =
    config.enable_stage3_jump_height_highfreq_deadband;
  summary.stage3_jump_velocity_smoothness_factor_count = 0U;
  summary.stage3_jump_velocity_smoothness_skipped_count = 0U;
  summary.stage3_jump_velocity_smoothness_max_abs_residual_mps =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_height_highfreq_deadband_factor_count = 0U;
  summary.stage3_jump_height_highfreq_deadband_skipped_count = 0U;
  summary.stage3_jump_height_highfreq_deadband_max_abs_raw_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_height_highfreq_deadband_max_abs_overflow_residual_m =
    std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

Stage3JumpRegularizerConstraintBuilder::Stage3JumpRegularizerConstraintBuilder(
  Stage3JumpRegularizerConstraintBuildRequest request)
    : request_(std::move(request)) {}

void Stage3JumpRegularizerConstraintBuilder::Validate() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.reference == nullptr || request_.jump_windows == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.diagnostics == nullptr) {
    throw std::runtime_error(
      "Stage3JumpRegularizerConstraintBuilder received an incomplete request");
  }
  if (request_.reference->rows.size() != request_.state_timestamps->size()) {
    throw std::runtime_error(
      "Stage3 jump regularizer reference size does not match the graph timeline");
  }
}

void Stage3JumpRegularizerConstraintBuilder::Build() const {
  Validate();
  InitializeSummary(*request_.config, *request_.run_summary);
  if (!request_.config->enable_stage3_jump_velocity_smoothness_regularizer &&
      !request_.config->enable_stage3_jump_height_highfreq_deadband) {
    return;
  }

  const std::vector<BodyZJumpConstraintWindow> windows =
    BuildBodyZJumpConstraintWindows(
      *request_.jump_windows,
      VerticalVelocityDeltaJumpConstraintWindowOptions(*request_.config));
  if (windows.empty()) {
    return;
  }

  const auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->stage3_jump_velocity_smoothness_sigma_mps);
  const auto height_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->stage3_jump_height_highfreq_sigma_m);

  if (request_.config->enable_stage3_jump_velocity_smoothness_regularizer) {
    for (std::size_t state_i = request_.dynamic_start_index;
         state_i + 1U < request_.state_timestamps->size();
         ++state_i) {
      const std::size_t state_j = state_i + 1U;
      const double start_time_s = (*request_.state_timestamps)[state_i];
      const double end_time_s = (*request_.state_timestamps)[state_j];
      const BodyZJumpConstraintWindow *window =
        FindWindowForInterval(windows, start_time_s, end_time_s);
      if (window == nullptr) {
        continue;
      }
      Stage3JumpRegularizerDiagnosticRow row;
      row.constraint_type = "velocity_smoothness";
      row.window_index = window->source_window_index;
      row.source_window_count = window->source_window_count;
      row.state_index_i = state_i;
      row.state_index_j = state_j;
      row.start_time_s = start_time_s;
      row.end_time_s = end_time_s;
      row.dt_s = end_time_s - start_time_s;
      row.deadband = request_.config->stage3_jump_velocity_smoothness_deadband_mps;
      row.sigma = request_.config->stage3_jump_velocity_smoothness_sigma_mps;
      if (row.dt_s <= 0.0 || !std::isfinite(row.dt_s)) {
        row.factor_added = false;
        row.skip_reason = "INVALID_INTERVAL";
        ++request_.run_summary->stage3_jump_velocity_smoothness_skipped_count;
        request_.diagnostics->push_back(row);
        continue;
      }
      request_.graph->add(factor::VerticalVelocityDeltaDeadbandFactor(
        symbol::V(state_i),
        symbol::V(state_j),
        request_.config->stage3_jump_velocity_smoothness_deadband_mps,
        velocity_noise));
      row.factor_added = true;
      row.skip_reason = "ADDED";
      ++request_.run_summary->stage3_jump_velocity_smoothness_factor_count;
      request_.diagnostics->push_back(row);
    }
  }

  if (request_.config->enable_stage3_jump_height_highfreq_deadband) {
    for (std::size_t state_index = request_.dynamic_start_index;
         state_index < request_.state_timestamps->size();
         ++state_index) {
      const double time_s = (*request_.state_timestamps)[state_index];
      const BodyZJumpConstraintWindow *window = FindWindowForTime(windows, time_s);
      if (window == nullptr) {
        continue;
      }
      Stage3JumpRegularizerDiagnosticRow row;
      row.constraint_type = "height_highfreq_deadband";
      row.window_index = window->source_window_index;
      row.source_window_count = window->source_window_count;
      row.state_index_i = state_index;
      row.state_index_j = state_index;
      row.start_time_s = time_s;
      row.end_time_s = time_s;
      row.dt_s = 0.0;
      row.reference_up_m = request_.reference->rows[state_index].stage2_lowpass_up_m;
      row.deadband = request_.config->stage3_jump_height_highfreq_deadband_m;
      row.sigma = request_.config->stage3_jump_height_highfreq_sigma_m;
      if (!std::isfinite(row.reference_up_m)) {
        row.factor_added = false;
        row.skip_reason = "LOWPASS_UNAVAILABLE";
        ++request_.run_summary->stage3_jump_height_highfreq_deadband_skipped_count;
        request_.diagnostics->push_back(row);
        continue;
      }
      request_.graph->add(factor::VerticalEnvelopeFactor(
        symbol::X(state_index),
        row.reference_up_m,
        request_.config->stage3_jump_height_highfreq_deadband_m,
        height_noise));
      row.factor_added = true;
      row.skip_reason = "ADDED";
      ++request_.run_summary->stage3_jump_height_highfreq_deadband_factor_count;
      request_.diagnostics->push_back(row);
    }
  }
}

void PopulateStage3JumpRegularizerDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage3JumpRegularizerDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  double max_abs_velocity_residual_mps = 0.0;
  double max_abs_height_raw_residual_m = 0.0;
  double max_abs_height_overflow_residual_m = 0.0;
  bool has_velocity_residual = false;
  bool has_height_residual = false;

  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    if (row.constraint_type == "velocity_smoothness") {
      const gtsam::Key velocity_i_key = symbol::V(row.state_index_i);
      const gtsam::Key velocity_j_key = symbol::V(row.state_index_j);
      if (!optimized_values.exists(velocity_i_key) ||
          !optimized_values.exists(velocity_j_key)) {
        continue;
      }
      row.optimized_delta_vz_mps =
        optimized_values.at<gtsam::Vector3>(velocity_j_key).z() -
        optimized_values.at<gtsam::Vector3>(velocity_i_key).z();
      row.residual =
        factor::SignedDeadbandResidual(row.optimized_delta_vz_mps, row.deadband);
      max_abs_velocity_residual_mps =
        std::max(max_abs_velocity_residual_mps, std::abs(row.residual));
      has_velocity_residual = true;
      continue;
    }
    if (row.constraint_type == "height_highfreq_deadband") {
      const gtsam::Key pose_key = symbol::X(row.state_index_i);
      if (!optimized_values.exists(pose_key) || !std::isfinite(row.reference_up_m)) {
        continue;
      }
      row.optimized_up_m =
        optimized_values.at<gtsam::Pose3>(pose_key).translation().z();
      row.raw_residual = row.optimized_up_m - row.reference_up_m;
      row.residual = factor::VerticalEnvelopeResidual(row.raw_residual, row.deadband);
      max_abs_height_raw_residual_m =
        std::max(max_abs_height_raw_residual_m, std::abs(row.raw_residual));
      max_abs_height_overflow_residual_m =
        std::max(max_abs_height_overflow_residual_m, std::abs(row.residual));
      has_height_residual = true;
    }
  }

  run_summary.stage3_jump_velocity_smoothness_factor_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3JumpRegularizerDiagnosticRow &row) {
          return row.factor_added && row.constraint_type == "velocity_smoothness";
        }));
  run_summary.stage3_jump_height_highfreq_deadband_factor_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3JumpRegularizerDiagnosticRow &row) {
          return row.factor_added && row.constraint_type == "height_highfreq_deadband";
        }));
  if (has_velocity_residual) {
    run_summary.stage3_jump_velocity_smoothness_max_abs_residual_mps =
      max_abs_velocity_residual_mps;
  }
  if (has_height_residual) {
    run_summary.stage3_jump_height_highfreq_deadband_max_abs_raw_residual_m =
      max_abs_height_raw_residual_m;
    run_summary.stage3_jump_height_highfreq_deadband_max_abs_overflow_residual_m =
      max_abs_height_overflow_residual_m;
  }
}

}  // namespace offline_lc_minimal
