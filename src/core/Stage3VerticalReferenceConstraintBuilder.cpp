#include "offline_lc_minimal/core/Stage3VerticalReferenceConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalRtkFactors.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimestampToleranceS = 1.0e-6;

bool UsesEnvelopeMode(const OfflineRunnerConfig &config) {
  return config.stage3_vertical_reference_constraint_mode ==
         Stage3VerticalReferenceConstraintMode::kEnvelope;
}

gtsam::SharedNoiseModel MakePrimaryNoiseModel(const OfflineRunnerConfig &config) {
  const double sigma_m =
    UsesEnvelopeMode(config)
      ? config.stage3_vertical_envelope_sigma_m
      : config.stage3_vertical_anchor_sigma_m;
  return gtsam::noiseModel::Isotropic::Sigma(1, sigma_m);
}

gtsam::SharedNoiseModel MakeCenterPullNoiseModel(const OfflineRunnerConfig &config) {
  return gtsam::noiseModel::Isotropic::Sigma(
    1,
    config.stage3_vertical_envelope_center_sigma_m);
}

void InitializeStage3Summary(const OfflineRunnerConfig &config, RunSummary &summary) {
  summary.stage3_vertical_reference_optimization_enabled = true;
  summary.stage3_vertical_reference_lowpass_cutoff_hz =
    config.stage3_vertical_reference_lowpass_cutoff_hz;
  summary.stage3_vertical_anchor_sigma_m = config.stage3_vertical_anchor_sigma_m;
  summary.stage3_vertical_reference_constraint_mode =
    ToString(config.stage3_vertical_reference_constraint_mode);
  summary.stage3_vertical_reference_factor_count = 0U;
  summary.stage3_vertical_reference_center_pull_factor_count = 0U;
  summary.stage3_vertical_reference_total_factor_count = 0U;
  summary.stage3_vertical_reference_skipped_count = 0U;
  summary.stage3_vertical_reference_mean_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_reference_max_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_reference_max_abs_lowpass_delta_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_envelope_half_width_m =
    UsesEnvelopeMode(config) ? config.stage3_vertical_envelope_half_width_m
                             : std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_envelope_sigma_m =
    UsesEnvelopeMode(config) ? config.stage3_vertical_envelope_sigma_m
                             : std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_envelope_center_sigma_m =
    UsesEnvelopeMode(config) && config.enable_stage3_vertical_envelope_center_pull
      ? config.stage3_vertical_envelope_center_sigma_m
      : std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_envelope_center_deadband_m =
    UsesEnvelopeMode(config) && config.enable_stage3_vertical_envelope_center_pull
      ? config.stage3_vertical_envelope_center_deadband_m
      : std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_envelope_outside_gate_count = 0U;
  summary.stage3_vertical_envelope_max_abs_overflow_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_vertical_envelope_max_abs_center_pull_residual_m =
    std::numeric_limits<double>::quiet_NaN();
}

void PopulateRowConfig(
  const OfflineRunnerConfig &config,
  Stage3VerticalReferenceDiagnosticRow &row) {
  row.constraint_mode = ToString(config.stage3_vertical_reference_constraint_mode);
  row.reference_up_m = row.stage2_lowpass_up_m;
  row.sigma_m =
    UsesEnvelopeMode(config) ? config.stage3_vertical_envelope_sigma_m
                             : config.stage3_vertical_anchor_sigma_m;
  if (!UsesEnvelopeMode(config)) {
    return;
  }
  row.envelope_half_width_m = config.stage3_vertical_envelope_half_width_m;
  row.envelope_sigma_m = config.stage3_vertical_envelope_sigma_m;
  if (config.enable_stage3_vertical_envelope_center_pull) {
    row.center_pull_sigma_m = config.stage3_vertical_envelope_center_sigma_m;
    row.center_pull_deadband_m = config.stage3_vertical_envelope_center_deadband_m;
  }
}

void AddStage3Factor(
  const OfflineRunnerConfig &config,
  const std::size_t state_index,
  const double reference_up_m,
  const gtsam::SharedNoiseModel &primary_noise,
  gtsam::NonlinearFactorGraph &graph,
  Stage3VerticalReferenceDiagnosticRow &row,
  RunSummary &summary) {
  if (UsesEnvelopeMode(config)) {
    graph.add(factor::VerticalEnvelopeFactor(
      symbol::X(state_index),
      reference_up_m,
      config.stage3_vertical_envelope_half_width_m,
      primary_noise));
    if (config.enable_stage3_vertical_envelope_center_pull) {
      graph.add(factor::VerticalEnvelopeCenterPullFactor(
        symbol::X(state_index),
        reference_up_m,
        config.stage3_vertical_envelope_half_width_m,
        config.stage3_vertical_envelope_center_deadband_m,
        MakeCenterPullNoiseModel(config)));
      row.center_pull_factor_added = true;
      ++summary.stage3_vertical_reference_center_pull_factor_count;
      ++summary.stage3_vertical_reference_total_factor_count;
    }
  } else {
    graph.add(factor::VerticalPositionFactor(
      symbol::X(state_index),
      reference_up_m,
      primary_noise));
  }
  row.factor_added = true;
  row.skip_reason = "ADDED";
  ++summary.stage3_vertical_reference_factor_count;
  ++summary.stage3_vertical_reference_total_factor_count;
}

}  // namespace

Stage3VerticalReferenceConstraintBuilder::Stage3VerticalReferenceConstraintBuilder(
  Stage3VerticalReferenceConstraintBuildRequest request)
    : request_(std::move(request)) {}

void Stage3VerticalReferenceConstraintBuilder::Validate() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.reference == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr) {
    throw std::runtime_error(
      "Stage3VerticalReferenceConstraintBuilder received an incomplete request");
  }
  if (request_.reference->rows.size() != request_.state_timestamps->size()) {
    throw std::runtime_error(
      "Stage3 vertical reference size does not match the graph timeline");
  }
}

void Stage3VerticalReferenceConstraintBuilder::Build() const {
  Validate();

  request_.diagnostics->clear();
  request_.diagnostics->reserve(request_.reference->rows.size());
  InitializeStage3Summary(*request_.config, *request_.run_summary);

  const auto primary_noise = MakePrimaryNoiseModel(*request_.config);
  double max_abs_lowpass_delta_m = 0.0;
  bool has_lowpass_delta = false;
  for (std::size_t state_index = 0; state_index < request_.reference->rows.size(); ++state_index) {
    Stage3VerticalReferenceDiagnosticRow row = request_.reference->rows[state_index];
    row.state_index = state_index;
    PopulateRowConfig(*request_.config, row);
    const double state_time_s = (*request_.state_timestamps)[state_index];
    if (!std::isfinite(row.time_s) ||
        std::abs(row.time_s - state_time_s) > kTimestampToleranceS) {
      row.factor_added = false;
      row.skip_reason = "TIMESTAMP_MISMATCH";
      ++request_.run_summary->stage3_vertical_reference_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (state_index < request_.dynamic_start_index) {
      row.factor_added = false;
      row.skip_reason = "INITIAL_STATIC";
      ++request_.run_summary->stage3_vertical_reference_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (!std::isfinite(row.stage2_lowpass_up_m)) {
      row.factor_added = false;
      row.skip_reason = "LOWPASS_UNAVAILABLE";
      ++request_.run_summary->stage3_vertical_reference_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    AddStage3Factor(
      *request_.config,
      state_index,
      row.stage2_lowpass_up_m,
      primary_noise,
      *request_.graph,
      row,
      *request_.run_summary);
    if (std::isfinite(row.lowpass_delta_m)) {
      max_abs_lowpass_delta_m =
        std::max(max_abs_lowpass_delta_m, std::abs(row.lowpass_delta_m));
      has_lowpass_delta = true;
    }
    request_.diagnostics->push_back(row);
  }

  if (has_lowpass_delta) {
    request_.run_summary->stage3_vertical_reference_max_abs_lowpass_delta_m =
      max_abs_lowpass_delta_m;
  }
}

void PopulateStage3VerticalReferenceDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage3VerticalReferenceDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  double abs_residual_sum_m = 0.0;
  double max_abs_residual_m = 0.0;
  double max_abs_overflow_residual_m = 0.0;
  double max_abs_center_pull_residual_m = 0.0;
  std::size_t residual_count = 0;
  std::size_t outside_gate_count = 0;
  bool has_overflow_residual = false;
  bool has_center_pull_residual = false;
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const gtsam::Key pose_key = symbol::X(row.state_index);
    if (!optimized_values.exists(pose_key) || !std::isfinite(row.stage2_lowpass_up_m)) {
      continue;
    }
    const auto pose = optimized_values.at<gtsam::Pose3>(pose_key);
    row.optimized_up_m = pose.translation().z();
    row.residual_m = row.optimized_up_m - row.stage2_lowpass_up_m;
    if (std::isfinite(row.residual_m)) {
      const double abs_residual_m = std::abs(row.residual_m);
      abs_residual_sum_m += abs_residual_m;
      max_abs_residual_m = std::max(max_abs_residual_m, abs_residual_m);
      ++residual_count;
    }
    if (row.constraint_mode == "envelope" &&
        std::isfinite(row.envelope_half_width_m) &&
        std::isfinite(row.residual_m)) {
      row.envelope_overflow_residual_m =
        factor::VerticalEnvelopeResidual(row.residual_m, row.envelope_half_width_m);
      row.outside_gate = std::abs(row.residual_m) > row.envelope_half_width_m;
      if (row.outside_gate) {
        ++outside_gate_count;
      }
      if (std::isfinite(row.envelope_overflow_residual_m)) {
        max_abs_overflow_residual_m =
          std::max(max_abs_overflow_residual_m, std::abs(row.envelope_overflow_residual_m));
        has_overflow_residual = true;
      }
      if (row.center_pull_factor_added &&
          std::isfinite(row.center_pull_deadband_m)) {
        row.center_pull_residual_m =
          factor::VerticalEnvelopeCenterResidual(
            row.residual_m,
            row.envelope_half_width_m,
            row.center_pull_deadband_m);
        if (std::isfinite(row.center_pull_residual_m)) {
          max_abs_center_pull_residual_m =
            std::max(max_abs_center_pull_residual_m, std::abs(row.center_pull_residual_m));
          has_center_pull_residual = true;
        }
      }
    }
  }
  run_summary.stage3_vertical_reference_factor_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3VerticalReferenceDiagnosticRow &row) {
          return row.factor_added;
        }));
  run_summary.stage3_vertical_reference_center_pull_factor_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3VerticalReferenceDiagnosticRow &row) {
          return row.center_pull_factor_added;
        }));
  run_summary.stage3_vertical_reference_total_factor_count =
    run_summary.stage3_vertical_reference_factor_count +
    run_summary.stage3_vertical_reference_center_pull_factor_count;
  run_summary.stage3_vertical_reference_skipped_count =
    diagnostics.size() - run_summary.stage3_vertical_reference_factor_count;
  run_summary.stage3_vertical_envelope_outside_gate_count = outside_gate_count;
  if (residual_count > 0U) {
    run_summary.stage3_vertical_reference_mean_abs_residual_m =
      abs_residual_sum_m / static_cast<double>(residual_count);
    run_summary.stage3_vertical_reference_max_abs_residual_m =
      max_abs_residual_m;
  }
  if (has_overflow_residual) {
    run_summary.stage3_vertical_envelope_max_abs_overflow_residual_m =
      max_abs_overflow_residual_m;
  }
  if (has_center_pull_residual) {
    run_summary.stage3_vertical_envelope_max_abs_center_pull_residual_m =
      max_abs_center_pull_residual_m;
  }
}

}  // namespace offline_lc_minimal
