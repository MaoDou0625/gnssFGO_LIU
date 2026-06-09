#include "offline_lc_minimal/core/Stage3Stage2IncrementHoldConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/factor/Stage2VerticalIncrementHoldFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

struct TimedUpSample {
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double up_m = std::numeric_limits<double>::quiet_NaN();
};

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

std::vector<TimedUpSample> BuildSortedStage2UpSamples(
  const Stage2VelocityReference &reference) {
  std::vector<TimedUpSample> samples;
  samples.reserve(reference.trajectory.size());
  for (const auto &row : reference.trajectory) {
    const double up_m = row.enu_position_m.z();
    if (!std::isfinite(row.time_s) || !std::isfinite(up_m)) {
      continue;
    }
    samples.push_back(TimedUpSample{row.time_s, up_m});
  }
  std::sort(
    samples.begin(),
    samples.end(),
    [](const TimedUpSample &lhs, const TimedUpSample &rhs) {
      return lhs.time_s < rhs.time_s;
    });
  samples.erase(
    std::unique(
      samples.begin(),
      samples.end(),
      [](const TimedUpSample &lhs, const TimedUpSample &rhs) {
        return std::abs(lhs.time_s - rhs.time_s) <= kTimeEpsilonS;
      }),
    samples.end());
  return samples;
}

bool HasCoverage(
  const std::vector<TimedUpSample> &samples,
  const double time_s) {
  return !samples.empty() &&
         time_s + kTimeEpsilonS >= samples.front().time_s &&
         time_s <= samples.back().time_s + kTimeEpsilonS;
}

double InterpolateUpAt(
  const std::vector<TimedUpSample> &samples,
  const double time_s) {
  if (!HasCoverage(samples, time_s)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (samples.size() == 1U) {
    return samples.front().up_m;
  }
  const auto upper = std::lower_bound(
    samples.begin(),
    samples.end(),
    time_s,
    [](const TimedUpSample &sample, const double target_time_s) {
      return sample.time_s < target_time_s;
    });
  if (upper == samples.begin()) {
    return samples.front().up_m;
  }
  if (upper == samples.end()) {
    return samples.back().up_m;
  }
  const auto &rhs = *upper;
  const auto &lhs = *(upper - 1);
  const double dt_s = rhs.time_s - lhs.time_s;
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double alpha = std::clamp((time_s - lhs.time_s) / dt_s, 0.0, 1.0);
  return (1.0 - alpha) * lhs.up_m + alpha * rhs.up_m;
}

bool IntervalOverlapsJumpWindow(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double start_time_s,
  const double end_time_s) {
  return std::any_of(
    windows.begin(),
    windows.end(),
    [start_time_s, end_time_s](const BodyZJumpConstraintWindow &window) {
      return IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s);
    });
}

void InitializeSummary(
  const OfflineRunnerConfig &config,
  RunSummary &summary) {
  summary.stage3_stage2_vertical_increment_hold_enabled =
    config.enable_stage3_stage2_vertical_increment_hold;
  summary.stage3_stage2_vertical_increment_factor_count = 0U;
  summary.stage3_stage2_vertical_increment_skipped_count = 0U;
  summary.stage3_stage2_vertical_increment_sigma_m =
    config.stage3_stage2_vertical_increment_sigma_m;
  summary.stage3_stage2_vertical_increment_jump_sigma_m =
    config.stage3_stage2_vertical_increment_jump_sigma_m;
  summary.stage3_stage2_vertical_increment_mean_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_stage2_vertical_increment_max_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

Stage3Stage2IncrementHoldConstraintBuilder::Stage3Stage2IncrementHoldConstraintBuilder(
  Stage3Stage2IncrementHoldConstraintBuildRequest request)
    : request_(std::move(request)) {}

void Stage3Stage2IncrementHoldConstraintBuilder::Validate() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.stage2_reference == nullptr || request_.jump_windows == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.diagnostics == nullptr) {
    throw std::runtime_error(
      "Stage3Stage2IncrementHoldConstraintBuilder received an incomplete request");
  }
}

void Stage3Stage2IncrementHoldConstraintBuilder::Build() const {
  Validate();
  InitializeSummary(*request_.config, *request_.run_summary);
  if (!request_.config->enable_stage3_stage2_vertical_increment_hold) {
    return;
  }
  if (request_.state_timestamps->size() < 2U) {
    return;
  }

  const std::vector<TimedUpSample> stage2_samples =
    BuildSortedStage2UpSamples(*request_.stage2_reference);
  if (stage2_samples.empty()) {
    ++request_.run_summary->stage3_stage2_vertical_increment_skipped_count;
    return;
  }
  const std::vector<BodyZJumpConstraintWindow> jump_windows =
    BuildBodyZJumpConstraintWindows(
      *request_.jump_windows,
      VerticalVelocityDeltaJumpConstraintWindowOptions(*request_.config));
  const auto normal_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->stage3_stage2_vertical_increment_sigma_m);
  const auto jump_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->stage3_stage2_vertical_increment_jump_sigma_m);

  for (std::size_t state_i = request_.dynamic_start_index;
       state_i + 1U < request_.state_timestamps->size();
       ++state_i) {
    const std::size_t state_j = state_i + 1U;
    const double start_time_s = (*request_.state_timestamps)[state_i];
    const double end_time_s = (*request_.state_timestamps)[state_j];
    const bool in_jump_padding =
      IntervalOverlapsJumpWindow(jump_windows, start_time_s, end_time_s);
    const double sigma_m =
      in_jump_padding
        ? request_.config->stage3_stage2_vertical_increment_jump_sigma_m
        : request_.config->stage3_stage2_vertical_increment_sigma_m;

    Stage3Stage2IncrementHoldDiagnosticRow row;
    row.state_index_i = state_i;
    row.state_index_j = state_j;
    row.start_time_s = start_time_s;
    row.end_time_s = end_time_s;
    row.dt_s = end_time_s - start_time_s;
    row.sigma_m = sigma_m;
    row.in_jump_padding = in_jump_padding;

    if (row.dt_s <= 0.0 || !std::isfinite(row.dt_s)) {
      row.skip_reason = "INVALID_INTERVAL";
      ++request_.run_summary->stage3_stage2_vertical_increment_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    row.stage2_up_i_m = InterpolateUpAt(stage2_samples, start_time_s);
    row.stage2_up_j_m = InterpolateUpAt(stage2_samples, end_time_s);
    if (!std::isfinite(row.stage2_up_i_m) || !std::isfinite(row.stage2_up_j_m)) {
      row.skip_reason = "STAGE2_REFERENCE_UNAVAILABLE";
      ++request_.run_summary->stage3_stage2_vertical_increment_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    row.reference_delta_z_m = row.stage2_up_j_m - row.stage2_up_i_m;
    request_.graph->add(factor::Stage2VerticalIncrementHoldFactor(
      symbol::X(state_i),
      symbol::X(state_j),
      row.reference_delta_z_m,
      in_jump_padding ? jump_noise : normal_noise));
    row.factor_added = true;
    row.skip_reason = "ADDED";
    ++request_.run_summary->stage3_stage2_vertical_increment_factor_count;
    request_.diagnostics->push_back(row);
  }
}

void PopulateStage3Stage2IncrementHoldDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage3Stage2IncrementHoldDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  double abs_residual_sum_m = 0.0;
  double max_abs_residual_m = 0.0;
  std::size_t residual_count = 0U;

  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const gtsam::Key pose_i_key = symbol::X(row.state_index_i);
    const gtsam::Key pose_j_key = symbol::X(row.state_index_j);
    if (!optimized_values.exists(pose_i_key) || !optimized_values.exists(pose_j_key)) {
      continue;
    }
    row.optimized_delta_z_m =
      optimized_values.at<gtsam::Pose3>(pose_j_key).translation().z() -
      optimized_values.at<gtsam::Pose3>(pose_i_key).translation().z();
    row.residual_m = row.optimized_delta_z_m - row.reference_delta_z_m;
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

  run_summary.stage3_stage2_vertical_increment_factor_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3Stage2IncrementHoldDiagnosticRow &row) {
          return row.factor_added;
        }));
  run_summary.stage3_stage2_vertical_increment_skipped_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3Stage2IncrementHoldDiagnosticRow &row) {
          return !row.factor_added;
        }));
  if (residual_count > 0U) {
    run_summary.stage3_stage2_vertical_increment_mean_abs_residual_m =
      abs_residual_sum_m / static_cast<double>(residual_count);
    run_summary.stage3_stage2_vertical_increment_max_abs_residual_m =
      max_abs_residual_m;
  }
}

}  // namespace offline_lc_minimal
