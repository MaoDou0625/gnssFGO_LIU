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

struct WindowMatch {
  std::size_t index = 0U;
  const BodyZJumpConstraintWindow *window = nullptr;
};

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

WindowMatch FindWindowForTime(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double time_s) {
  for (std::size_t index = 0U; index < windows.size(); ++index) {
    const auto &window = windows[index];
    if (time_s + kTimeEpsilonS >= window.start_time_s &&
        time_s <= window.end_time_s + kTimeEpsilonS) {
      return WindowMatch{index, &window};
    }
  }
  return WindowMatch{};
}

WindowMatch FindWindowForInterval(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double start_time_s,
  const double end_time_s) {
  for (std::size_t index = 0U; index < windows.size(); ++index) {
    const auto &window = windows[index];
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      return WindowMatch{index, &window};
    }
  }
  return WindowMatch{};
}

const Stage3JumpContextEnvelopeProfileRow *FindProfile(
  const std::vector<Stage3JumpContextEnvelopeProfileRow> &profiles,
  const std::size_t profile_index) {
  if (profile_index >= profiles.size()) {
    return nullptr;
  }
  return &profiles[profile_index];
}

double LowpassVzAt(
  const std::vector<double> &state_timestamps,
  const Stage3VerticalReference &reference,
  const std::size_t state_index) {
  if (reference.rows.size() != state_timestamps.size() ||
      state_index >= state_timestamps.size()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::size_t left_index = state_index;
  std::size_t right_index = state_index;
  if (state_index > 0U) {
    left_index = state_index - 1U;
  }
  if (state_index + 1U < state_timestamps.size()) {
    right_index = state_index + 1U;
  }
  if (left_index == right_index) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double dt_s = state_timestamps[right_index] - state_timestamps[left_index];
  if (!std::isfinite(dt_s) || dt_s <= 0.0 ||
      !std::isfinite(reference.rows[left_index].stage2_lowpass_up_m) ||
      !std::isfinite(reference.rows[right_index].stage2_lowpass_up_m)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return (reference.rows[right_index].stage2_lowpass_up_m -
          reference.rows[left_index].stage2_lowpass_up_m) /
         dt_s;
}

void PopulateContextProfileSummary(
  const std::vector<Stage3JumpContextEnvelopeProfileRow> &profiles,
  RunSummary &summary) {
  summary.stage3_jump_context_envelope_profile_count = profiles.size();
  summary.stage3_jump_context_envelope_fallback_count =
    static_cast<std::size_t>(
      std::count_if(
        profiles.begin(),
        profiles.end(),
        [](const Stage3JumpContextEnvelopeProfileRow &row) {
          return row.velocity_fallback || row.velocity_delta_fallback ||
                 row.height_fallback;
        }));

  bool has_velocity_deadband = false;
  bool has_velocity_delta_deadband = false;
  bool has_height_deadband = false;
  double velocity_min = std::numeric_limits<double>::infinity();
  double velocity_max = 0.0;
  double velocity_delta_min = std::numeric_limits<double>::infinity();
  double velocity_delta_max = 0.0;
  double height_min = std::numeric_limits<double>::infinity();
  double height_max = 0.0;
  for (const auto &row : profiles) {
    if (std::isfinite(row.velocity_deadband_mps)) {
      velocity_min = std::min(velocity_min, row.velocity_deadband_mps);
      velocity_max = std::max(velocity_max, row.velocity_deadband_mps);
      has_velocity_deadband = true;
    }
    if (std::isfinite(row.velocity_delta_deadband_mps)) {
      velocity_delta_min = std::min(velocity_delta_min, row.velocity_delta_deadband_mps);
      velocity_delta_max = std::max(velocity_delta_max, row.velocity_delta_deadband_mps);
      has_velocity_delta_deadband = true;
    }
    if (std::isfinite(row.height_deadband_m)) {
      height_min = std::min(height_min, row.height_deadband_m);
      height_max = std::max(height_max, row.height_deadband_m);
      has_height_deadband = true;
    }
  }
  if (has_velocity_deadband) {
    summary.stage3_jump_context_velocity_deadband_min_mps = velocity_min;
    summary.stage3_jump_context_velocity_deadband_max_mps = velocity_max;
  }
  if (has_velocity_delta_deadband) {
    summary.stage3_jump_context_velocity_delta_deadband_min_mps =
      velocity_delta_min;
    summary.stage3_jump_context_velocity_delta_deadband_max_mps =
      velocity_delta_max;
  }
  if (has_height_deadband) {
    summary.stage3_jump_context_height_deadband_min_m = height_min;
    summary.stage3_jump_context_height_deadband_max_m = height_max;
  }
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
  summary.stage3_jump_adaptive_context_envelope_enabled =
    config.enable_stage3_jump_adaptive_context_envelope;
  summary.stage3_jump_context_envelope_profile_count = 0U;
  summary.stage3_jump_context_envelope_fallback_count = 0U;
  summary.stage3_jump_velocity_context_envelope_factor_count = 0U;
  summary.stage3_jump_velocity_context_envelope_skipped_count = 0U;
  summary.stage3_jump_velocity_context_envelope_max_abs_overflow_residual_mps =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_context_velocity_deadband_min_mps =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_context_velocity_deadband_max_mps =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_context_velocity_delta_deadband_min_mps =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_context_velocity_delta_deadband_max_mps =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_context_height_deadband_min_m =
    std::numeric_limits<double>::quiet_NaN();
  summary.stage3_jump_context_height_deadband_max_m =
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
  if (request_.config->enable_stage3_jump_adaptive_context_envelope &&
      (request_.initial_values == nullptr || request_.context_profiles == nullptr)) {
    throw std::runtime_error(
      "Stage3 adaptive jump context envelope requires initial values and profile output");
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

  std::vector<Stage3JumpContextEnvelopeProfileRow> context_profiles;
  if (request_.config->enable_stage3_jump_adaptive_context_envelope) {
    Stage3JumpContextEnvelopePlanRequest plan_request;
    plan_request.config = request_.config;
    plan_request.state_timestamps = request_.state_timestamps;
    plan_request.reference = request_.reference;
    plan_request.windows = &windows;
    plan_request.initial_values = request_.initial_values;
    plan_request.dynamic_start_index = request_.dynamic_start_index;
    context_profiles =
      Stage3JumpContextEnvelopePlanner(std::move(plan_request)).Plan().profiles;
    *request_.context_profiles = context_profiles;
    PopulateContextProfileSummary(context_profiles, *request_.run_summary);
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
      const WindowMatch match =
        FindWindowForInterval(windows, start_time_s, end_time_s);
      if (match.window == nullptr) {
        continue;
      }
      const Stage3JumpContextEnvelopeProfileRow *profile =
        FindProfile(context_profiles, match.index);
      const double deadband_mps =
        profile == nullptr
          ? request_.config->stage3_jump_velocity_smoothness_deadband_mps
          : profile->velocity_delta_deadband_mps;
      Stage3JumpRegularizerDiagnosticRow row;
      row.constraint_type = "velocity_smoothness";
      row.window_index = match.window->source_window_index;
      row.source_window_count = match.window->source_window_count;
      row.state_index_i = state_i;
      row.state_index_j = state_j;
      row.start_time_s = start_time_s;
      row.end_time_s = end_time_s;
      row.dt_s = end_time_s - start_time_s;
      row.deadband = deadband_mps;
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
        deadband_mps,
        velocity_noise));
      row.factor_added = true;
      row.skip_reason = "ADDED";
      ++request_.run_summary->stage3_jump_velocity_smoothness_factor_count;
      request_.diagnostics->push_back(row);
    }

    if (request_.config->enable_stage3_jump_adaptive_context_envelope) {
      for (std::size_t state_index = request_.dynamic_start_index;
           state_index < request_.state_timestamps->size();
           ++state_index) {
        const double time_s = (*request_.state_timestamps)[state_index];
        const WindowMatch match = FindWindowForTime(windows, time_s);
        if (match.window == nullptr) {
          continue;
        }
        const Stage3JumpContextEnvelopeProfileRow *profile =
          FindProfile(context_profiles, match.index);
        Stage3JumpRegularizerDiagnosticRow row;
        row.constraint_type = "velocity_context_envelope";
        row.window_index = match.window->source_window_index;
        row.source_window_count = match.window->source_window_count;
        row.state_index_i = state_index;
        row.state_index_j = state_index;
        row.start_time_s = time_s;
        row.end_time_s = time_s;
        row.dt_s = 0.0;
        row.reference_vz_mps =
          profile == nullptr
            ? std::numeric_limits<double>::quiet_NaN()
            : LowpassVzAt(*request_.state_timestamps, *request_.reference, state_index) +
                profile->velocity_reference_offset_mps;
        row.deadband =
          profile == nullptr
            ? request_.config->stage3_jump_velocity_smoothness_deadband_mps
            : profile->velocity_deadband_mps;
        row.sigma = request_.config->stage3_jump_velocity_smoothness_sigma_mps;
        if (profile == nullptr || profile->velocity_fallback ||
            !std::isfinite(row.reference_vz_mps) || !std::isfinite(row.deadband)) {
          row.factor_added = false;
          row.skip_reason =
            profile != nullptr && profile->velocity_fallback
              ? "INSUFFICIENT_CONTEXT"
              : "CONTEXT_UNAVAILABLE";
          ++request_.run_summary->stage3_jump_velocity_context_envelope_skipped_count;
          request_.diagnostics->push_back(row);
          continue;
        }
        request_.graph->add(factor::VerticalVelocityDeadbandFactor(
          symbol::V(state_index),
          row.reference_vz_mps,
          row.deadband,
          velocity_noise));
        row.factor_added = true;
        row.skip_reason = "ADDED";
        ++request_.run_summary->stage3_jump_velocity_context_envelope_factor_count;
        request_.diagnostics->push_back(row);
      }
    }
  }

  if (request_.config->enable_stage3_jump_height_highfreq_deadband) {
    for (std::size_t state_index = request_.dynamic_start_index;
         state_index < request_.state_timestamps->size();
         ++state_index) {
      const double time_s = (*request_.state_timestamps)[state_index];
      const WindowMatch match = FindWindowForTime(windows, time_s);
      if (match.window == nullptr) {
        continue;
      }
      const Stage3JumpContextEnvelopeProfileRow *profile =
        FindProfile(context_profiles, match.index);
      const double deadband_m =
        profile == nullptr
          ? request_.config->stage3_jump_height_highfreq_deadband_m
          : profile->height_deadband_m;
      Stage3JumpRegularizerDiagnosticRow row;
      row.constraint_type = "height_highfreq_deadband";
      row.window_index = match.window->source_window_index;
      row.source_window_count = match.window->source_window_count;
      row.state_index_i = state_index;
      row.state_index_j = state_index;
      row.start_time_s = time_s;
      row.end_time_s = time_s;
      row.dt_s = 0.0;
      const double height_reference_offset_m =
        profile == nullptr || profile->height_fallback ||
            !std::isfinite(profile->height_reference_offset_m)
          ? 0.0
          : profile->height_reference_offset_m;
      row.reference_up_m =
        request_.reference->rows[state_index].stage2_lowpass_up_m +
        height_reference_offset_m;
      row.deadband = deadband_m;
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
        deadband_m,
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
  double max_abs_velocity_context_residual_mps = 0.0;
  double max_abs_height_raw_residual_m = 0.0;
  double max_abs_height_overflow_residual_m = 0.0;
  bool has_velocity_residual = false;
  bool has_velocity_context_residual = false;
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
    if (row.constraint_type == "velocity_context_envelope") {
      const gtsam::Key velocity_key = symbol::V(row.state_index_i);
      if (!optimized_values.exists(velocity_key) ||
          !std::isfinite(row.reference_vz_mps)) {
        continue;
      }
      row.optimized_vz_mps =
        optimized_values.at<gtsam::Vector3>(velocity_key).z();
      row.raw_residual = row.optimized_vz_mps - row.reference_vz_mps;
      row.residual =
        factor::SignedDeadbandResidual(row.raw_residual, row.deadband);
      max_abs_velocity_context_residual_mps =
        std::max(max_abs_velocity_context_residual_mps, std::abs(row.residual));
      has_velocity_context_residual = true;
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
  run_summary.stage3_jump_velocity_context_envelope_factor_count =
    static_cast<std::size_t>(
      std::count_if(
        diagnostics.begin(),
        diagnostics.end(),
        [](const Stage3JumpRegularizerDiagnosticRow &row) {
          return row.factor_added && row.constraint_type == "velocity_context_envelope";
        }));
  if (has_velocity_residual) {
    run_summary.stage3_jump_velocity_smoothness_max_abs_residual_mps =
      max_abs_velocity_residual_mps;
  }
  if (has_velocity_context_residual) {
    run_summary.stage3_jump_velocity_context_envelope_max_abs_overflow_residual_mps =
      max_abs_velocity_context_residual_mps;
  }
  if (has_height_residual) {
    run_summary.stage3_jump_height_highfreq_deadband_max_abs_raw_residual_m =
      max_abs_height_raw_residual_m;
    run_summary.stage3_jump_height_highfreq_deadband_max_abs_overflow_residual_m =
      max_abs_height_overflow_residual_m;
  }
}

}  // namespace offline_lc_minimal
