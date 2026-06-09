#include "offline_lc_minimal/core/Stage3Stage2JumpShapeHoldConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/core/Stage3Stage2HeightReferenceUtils.h"
#include "offline_lc_minimal/factor/Stage2VerticalIncrementHoldFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

void InitializeSummary(
  const OfflineRunnerConfig &config,
  RunSummary &summary) {
  summary.stage3_stage2_jump_shape_hold_enabled =
    config.enable_stage3_stage2_jump_shape_hold;
  summary.stage3_stage2_jump_shape_factor_count = 0U;
  summary.stage3_stage2_jump_shape_skipped_count = 0U;
  summary.stage3_stage2_jump_shape_sigma_m =
    config.stage3_stage2_jump_shape_sigma_m;
  summary.stage3_stage2_jump_shape_mean_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_stage2_jump_shape_max_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
}

std::vector<std::size_t> StateIndicesInsideWindow(
  const std::vector<double> &state_timestamps,
  const std::size_t dynamic_start_index,
  const BodyZJumpConstraintWindow &window) {
  std::vector<std::size_t> indices;
  for (std::size_t state_index = dynamic_start_index;
       state_index < state_timestamps.size();
       ++state_index) {
    const double time_s = state_timestamps[state_index];
    if (!std::isfinite(time_s)) {
      continue;
    }
    if (time_s + 1.0e-9 < window.start_time_s) {
      continue;
    }
    if (time_s > window.end_time_s + 1.0e-9) {
      break;
    }
    indices.push_back(state_index);
  }
  return indices;
}

Stage3Stage2JumpShapeHoldDiagnosticRow MakeBaseRow(
  const BodyZJumpConstraintWindow &window,
  const std::size_t anchor_state_index,
  const std::size_t state_index,
  const double anchor_time_s,
  const double time_s,
  const double sigma_m) {
  Stage3Stage2JumpShapeHoldDiagnosticRow row;
  row.window_index = window.source_window_index;
  row.source_window_count = window.source_window_count;
  row.anchor_state_index = anchor_state_index;
  row.state_index = state_index;
  row.anchor_time_s = anchor_time_s;
  row.time_s = time_s;
  row.dt_s = time_s - anchor_time_s;
  row.sigma_m = sigma_m;
  return row;
}

}  // namespace

Stage3Stage2JumpShapeHoldConstraintBuilder::
  Stage3Stage2JumpShapeHoldConstraintBuilder(
    Stage3Stage2JumpShapeHoldConstraintBuildRequest request)
    : request_(std::move(request)) {}

void Stage3Stage2JumpShapeHoldConstraintBuilder::Validate() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.stage2_reference == nullptr || request_.jump_windows == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.diagnostics == nullptr) {
    throw std::runtime_error(
      "Stage3Stage2JumpShapeHoldConstraintBuilder received an incomplete request");
  }
}

void Stage3Stage2JumpShapeHoldConstraintBuilder::Build() const {
  Validate();
  InitializeSummary(*request_.config, *request_.run_summary);
  if (!request_.config->enable_stage3_stage2_jump_shape_hold) {
    return;
  }
  if (request_.state_timestamps->size() < 2U) {
    return;
  }

  const std::vector<TimedStage2UpSample> stage2_samples =
    BuildSortedStage2UpSamples(*request_.stage2_reference);
  if (stage2_samples.empty()) {
    ++request_.run_summary->stage3_stage2_jump_shape_skipped_count;
    return;
  }

  const std::vector<BodyZJumpConstraintWindow> windows =
    BuildBodyZJumpConstraintWindows(
      *request_.jump_windows,
      VerticalVelocityDeltaJumpConstraintWindowOptions(*request_.config));
  if (windows.empty()) {
    return;
  }

  const auto noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->stage3_stage2_jump_shape_sigma_m);

  for (const auto &window : windows) {
    const std::vector<std::size_t> indices = StateIndicesInsideWindow(
      *request_.state_timestamps,
      request_.dynamic_start_index,
      window);
    if (indices.size() < 2U) {
      Stage3Stage2JumpShapeHoldDiagnosticRow row;
      row.window_index = window.source_window_index;
      row.source_window_count = window.source_window_count;
      row.sigma_m = request_.config->stage3_stage2_jump_shape_sigma_m;
      row.skip_reason = "INSUFFICIENT_WINDOW_STATES";
      request_.diagnostics->push_back(row);
      ++request_.run_summary->stage3_stage2_jump_shape_skipped_count;
      continue;
    }

    const std::size_t anchor_state_index = indices.front();
    const double anchor_time_s = (*request_.state_timestamps)[anchor_state_index];
    const double stage2_anchor_up_m =
      InterpolateStage2UpAt(stage2_samples, anchor_time_s);
    if (!std::isfinite(stage2_anchor_up_m)) {
      Stage3Stage2JumpShapeHoldDiagnosticRow row = MakeBaseRow(
        window,
        anchor_state_index,
        anchor_state_index,
        anchor_time_s,
        anchor_time_s,
        request_.config->stage3_stage2_jump_shape_sigma_m);
      row.skip_reason = "STAGE2_ANCHOR_UNAVAILABLE";
      request_.diagnostics->push_back(row);
      ++request_.run_summary->stage3_stage2_jump_shape_skipped_count;
      continue;
    }

    for (std::size_t index_offset = 1U; index_offset < indices.size(); ++index_offset) {
      const std::size_t state_index = indices[index_offset];
      const double time_s = (*request_.state_timestamps)[state_index];
      Stage3Stage2JumpShapeHoldDiagnosticRow row = MakeBaseRow(
        window,
        anchor_state_index,
        state_index,
        anchor_time_s,
        time_s,
        request_.config->stage3_stage2_jump_shape_sigma_m);
      row.stage2_anchor_up_m = stage2_anchor_up_m;
      row.stage2_up_m = InterpolateStage2UpAt(stage2_samples, time_s);
      if (row.dt_s <= 0.0 || !std::isfinite(row.dt_s)) {
        row.skip_reason = "INVALID_INTERVAL";
        request_.diagnostics->push_back(row);
        ++request_.run_summary->stage3_stage2_jump_shape_skipped_count;
        continue;
      }
      if (!std::isfinite(row.stage2_up_m)) {
        row.skip_reason = "STAGE2_REFERENCE_UNAVAILABLE";
        request_.diagnostics->push_back(row);
        ++request_.run_summary->stage3_stage2_jump_shape_skipped_count;
        continue;
      }

      row.reference_relative_z_m = row.stage2_up_m - row.stage2_anchor_up_m;
      request_.graph->add(factor::Stage2VerticalIncrementHoldFactor(
        symbol::X(anchor_state_index),
        symbol::X(state_index),
        row.reference_relative_z_m,
        noise));
      row.factor_added = true;
      row.skip_reason = "ADDED";
      request_.diagnostics->push_back(row);
      ++request_.run_summary->stage3_stage2_jump_shape_factor_count;
    }
  }
}

void PopulateStage3Stage2JumpShapeHoldDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage3Stage2JumpShapeHoldDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  double abs_residual_sum_m = 0.0;
  double max_abs_residual_m = 0.0;
  std::size_t residual_count = 0U;

  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const gtsam::Key anchor_pose_key = symbol::X(row.anchor_state_index);
    const gtsam::Key pose_key = symbol::X(row.state_index);
    if (!optimized_values.exists(anchor_pose_key) || !optimized_values.exists(pose_key)) {
      continue;
    }
    row.optimized_relative_z_m =
      optimized_values.at<gtsam::Pose3>(pose_key).translation().z() -
      optimized_values.at<gtsam::Pose3>(anchor_pose_key).translation().z();
    row.residual_m = row.optimized_relative_z_m - row.reference_relative_z_m;
    if (std::isfinite(row.sigma_m) && row.sigma_m > 0.0) {
      row.normalized_residual = row.residual_m / row.sigma_m;
    }
    if (std::isfinite(row.residual_m)) {
      const double abs_residual_m = std::abs(row.residual_m);
      abs_residual_sum_m += abs_residual_m;
      max_abs_residual_m = std::max(max_abs_residual_m, abs_residual_m);
      ++residual_count;
    }
  }

  run_summary.stage3_stage2_jump_shape_factor_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3Stage2JumpShapeHoldDiagnosticRow &row) {
          return row.factor_added;
        }));
  run_summary.stage3_stage2_jump_shape_skipped_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3Stage2JumpShapeHoldDiagnosticRow &row) {
          return !row.factor_added;
        }));
  if (residual_count > 0U) {
    run_summary.stage3_stage2_jump_shape_mean_abs_residual_m =
      abs_residual_sum_m / static_cast<double>(residual_count);
    run_summary.stage3_stage2_jump_shape_max_abs_residual_m =
      max_abs_residual_m;
  }
}

}  // namespace offline_lc_minimal
