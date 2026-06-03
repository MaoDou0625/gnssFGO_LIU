#include "offline_lc_minimal/core/Stage3JumpContextEnvelopePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

#include <gtsam/inference/Symbol.h>

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

[[nodiscard]] bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

[[nodiscard]] bool ContainsTime(
  const BodyZJumpConstraintWindow &window,
  const double time_s) {
  return time_s + kTimeEpsilonS >= window.start_time_s &&
         time_s <= window.end_time_s + kTimeEpsilonS;
}

[[nodiscard]] bool InsideAnyWindow(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double time_s) {
  return std::any_of(
    windows.begin(),
    windows.end(),
    [time_s](const BodyZJumpConstraintWindow &window) {
      return ContainsTime(window, time_s);
    });
}

[[nodiscard]] bool IntervalOverlapsAnyWindow(
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

[[nodiscard]] bool InContextWindow(
  const BodyZJumpConstraintWindow &window,
  const double context_window_s,
  const double time_s) {
  const bool in_pre =
    time_s + kTimeEpsilonS >= window.start_time_s - context_window_s &&
    time_s < window.start_time_s - kTimeEpsilonS;
  const bool in_post =
    time_s > window.end_time_s + kTimeEpsilonS &&
    time_s <= window.end_time_s + context_window_s + kTimeEpsilonS;
  return in_pre || in_post;
}

[[nodiscard]] double Median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 1U) {
    return values[middle];
  }
  return 0.5 * (values[middle - 1U] + values[middle]);
}

[[nodiscard]] double Quantile(std::vector<double> values, const double quantile) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double scaled =
    std::clamp(quantile, 0.0, 1.0) * static_cast<double>(values.size() - 1U);
  const auto lower_index = static_cast<std::size_t>(std::floor(scaled));
  const auto upper_index = static_cast<std::size_t>(std::ceil(scaled));
  const double alpha = scaled - static_cast<double>(lower_index);
  return values[lower_index] * (1.0 - alpha) + values[upper_index] * alpha;
}

[[nodiscard]] std::vector<double> CenterAbs(
  const std::vector<double> &values,
  const double center) {
  std::vector<double> centered;
  centered.reserve(values.size());
  if (!std::isfinite(center)) {
    return centered;
  }
  for (const double value : values) {
    if (std::isfinite(value)) {
      centered.push_back(std::abs(value - center));
    }
  }
  return centered;
}

[[nodiscard]] double ResolveDeadband(
  const double width,
  const double multiplier,
  const double floor,
  const double cap,
  const double fallback_deadband,
  const bool fallback) {
  if (fallback || !std::isfinite(width)) {
    return fallback_deadband;
  }
  const double context_deadband = std::clamp(width * multiplier, floor, cap);
  return std::min(fallback_deadband, context_deadband);
}

[[nodiscard]] std::string JoinFallback(
  const bool velocity_fallback,
  const bool velocity_delta_fallback,
  const bool height_fallback) {
  std::string reason;
  if (velocity_fallback) {
    reason += "VELOCITY";
  }
  if (velocity_delta_fallback) {
    if (!reason.empty()) {
      reason += "|";
    }
    reason += "VELOCITY_DELTA";
  }
  if (height_fallback) {
    if (!reason.empty()) {
      reason += "|";
    }
    reason += "HEIGHT";
  }
  return reason.empty() ? "NONE" : reason;
}

[[nodiscard]] double LowpassVzAt(
  const std::vector<double> &state_timestamps,
  const Stage3VerticalReference &reference,
  const std::size_t state_index) {
  if (state_timestamps.size() != reference.rows.size() ||
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

}  // namespace

Stage3JumpContextEnvelopePlanner::Stage3JumpContextEnvelopePlanner(
  Stage3JumpContextEnvelopePlanRequest request)
    : request_(std::move(request)) {}

void Stage3JumpContextEnvelopePlanner::Validate() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.reference == nullptr || request_.windows == nullptr ||
      request_.initial_values == nullptr) {
    throw std::runtime_error(
      "Stage3JumpContextEnvelopePlanner received an incomplete request");
  }
  if (request_.reference->rows.size() != request_.state_timestamps->size()) {
    throw std::runtime_error(
      "Stage3 jump context envelope reference size does not match timeline");
  }
}

Stage3JumpContextEnvelopePlan Stage3JumpContextEnvelopePlanner::Plan() const {
  Validate();
  Stage3JumpContextEnvelopePlan plan;
  plan.profiles.reserve(request_.windows->size());

  for (std::size_t window_index = 0U; window_index < request_.windows->size(); ++window_index) {
    const BodyZJumpConstraintWindow &window = (*request_.windows)[window_index];
    Stage3JumpContextEnvelopeProfileRow row;
    row.profile_index = window_index;
    row.window_index = window.source_window_index;
    row.source_window_count = window.source_window_count;
    row.window_start_time_s = window.start_time_s;
    row.window_end_time_s = window.end_time_s;
    row.pre_context_start_time_s =
      window.start_time_s - request_.config->stage3_jump_context_window_s;
    row.pre_context_end_time_s = window.start_time_s;
    row.post_context_start_time_s = window.end_time_s;
    row.post_context_end_time_s =
      window.end_time_s + request_.config->stage3_jump_context_window_s;

    std::vector<double> context_lowpass_vz_mps;
    std::vector<double> context_vz_residual_mps;
    std::vector<double> context_delta_vz_mps;
    std::vector<double> context_height_residual_m;

    for (std::size_t state_index = request_.dynamic_start_index;
         state_index < request_.state_timestamps->size();
         ++state_index) {
      const double time_s = (*request_.state_timestamps)[state_index];
      if (!InContextWindow(window, request_.config->stage3_jump_context_window_s, time_s) ||
          InsideAnyWindow(*request_.windows, time_s)) {
        continue;
      }
      const gtsam::Key velocity_key = symbol::V(state_index);
      const double lowpass_vz_mps =
        LowpassVzAt(*request_.state_timestamps, *request_.reference, state_index);
      if (std::isfinite(lowpass_vz_mps)) {
        context_lowpass_vz_mps.push_back(lowpass_vz_mps);
      }
      if (request_.initial_values->exists(velocity_key)) {
        const double vz_mps =
          request_.initial_values->at<gtsam::Vector3>(velocity_key).z();
        if (std::isfinite(vz_mps) && std::isfinite(lowpass_vz_mps)) {
          context_vz_residual_mps.push_back(vz_mps - lowpass_vz_mps);
        }
      }
      const auto &reference_row = request_.reference->rows[state_index];
      const double height_residual_m =
        reference_row.stage2_up_m - reference_row.stage2_lowpass_up_m;
      if (std::isfinite(height_residual_m)) {
        context_height_residual_m.push_back(height_residual_m);
      }
    }

    for (std::size_t state_i = request_.dynamic_start_index;
         state_i + 1U < request_.state_timestamps->size();
         ++state_i) {
      const std::size_t state_j = state_i + 1U;
      const double start_time_s = (*request_.state_timestamps)[state_i];
      const double end_time_s = (*request_.state_timestamps)[state_j];
      const double midpoint_time_s = 0.5 * (start_time_s + end_time_s);
      if (!InContextWindow(window, request_.config->stage3_jump_context_window_s, midpoint_time_s) ||
          IntervalOverlapsAnyWindow(*request_.windows, start_time_s, end_time_s)) {
        continue;
      }
      const gtsam::Key velocity_i_key = symbol::V(state_i);
      const gtsam::Key velocity_j_key = symbol::V(state_j);
      if (!request_.initial_values->exists(velocity_i_key) ||
          !request_.initial_values->exists(velocity_j_key)) {
        continue;
      }
      const double vz_i_mps =
        request_.initial_values->at<gtsam::Vector3>(velocity_i_key).z();
      const double vz_j_mps =
        request_.initial_values->at<gtsam::Vector3>(velocity_j_key).z();
      if (std::isfinite(vz_i_mps) && std::isfinite(vz_j_mps)) {
        context_delta_vz_mps.push_back(std::abs(vz_j_mps - vz_i_mps));
      }
    }

    row.velocity_sample_count = context_vz_residual_mps.size();
    row.velocity_delta_sample_count = context_delta_vz_mps.size();
    row.height_sample_count = context_height_residual_m.size();

    const double context_lowpass_vz_median_mps = Median(context_lowpass_vz_mps);
    row.context_vz_residual_median_mps = Median(context_vz_residual_mps);
    row.velocity_reference_offset_mps =
      request_.config->stage3_jump_context_preserve_local_center &&
          std::isfinite(row.context_vz_residual_median_mps)
        ? std::clamp(
            row.context_vz_residual_median_mps,
            -request_.config->stage3_jump_context_velocity_cap_mps,
            request_.config->stage3_jump_context_velocity_cap_mps)
        : 0.0;
    row.context_vz_median_mps =
      std::isfinite(context_lowpass_vz_median_mps) &&
          std::isfinite(row.velocity_reference_offset_mps)
        ? context_lowpass_vz_median_mps + row.velocity_reference_offset_mps
        : std::numeric_limits<double>::quiet_NaN();
    row.context_height_median_residual_m = Median(context_height_residual_m);
    row.height_reference_offset_m =
      request_.config->stage3_jump_context_preserve_local_center &&
          std::isfinite(row.context_height_median_residual_m)
        ? std::clamp(
            row.context_height_median_residual_m,
            -request_.config->stage3_jump_context_height_cap_m,
            request_.config->stage3_jump_context_height_cap_m)
        : 0.0;
    row.context_vz_p95_abs_centered_mps =
      Quantile(
        CenterAbs(context_vz_residual_mps, row.context_vz_residual_median_mps),
        request_.config->stage3_jump_context_quantile);
    row.context_delta_vz_p95_abs_mps =
      Quantile(context_delta_vz_mps, request_.config->stage3_jump_context_quantile);
    row.context_height_p95_abs_centered_m =
      Quantile(
        CenterAbs(context_height_residual_m, row.context_height_median_residual_m),
        request_.config->stage3_jump_context_quantile);

    const auto min_count =
      static_cast<std::size_t>(request_.config->stage3_jump_context_min_sample_count);
    row.velocity_fallback = row.velocity_sample_count < min_count;
    row.velocity_delta_fallback = row.velocity_delta_sample_count < min_count;
    row.height_fallback = row.height_sample_count < min_count;
    row.velocity_deadband_mps =
      ResolveDeadband(
        row.context_vz_p95_abs_centered_mps,
        request_.config->stage3_jump_context_velocity_multiplier,
        request_.config->stage3_jump_context_velocity_floor_mps,
        request_.config->stage3_jump_context_velocity_cap_mps,
        request_.config->stage3_jump_velocity_smoothness_deadband_mps,
        row.velocity_fallback);
    row.velocity_delta_deadband_mps =
      ResolveDeadband(
        row.context_delta_vz_p95_abs_mps,
        request_.config->stage3_jump_context_velocity_multiplier,
        request_.config->stage3_jump_context_velocity_floor_mps,
        request_.config->stage3_jump_context_velocity_cap_mps,
        request_.config->stage3_jump_velocity_smoothness_deadband_mps,
        row.velocity_delta_fallback);
    row.height_deadband_m =
      ResolveDeadband(
        row.context_height_p95_abs_centered_m,
        request_.config->stage3_jump_context_height_multiplier,
        request_.config->stage3_jump_context_height_floor_m,
        request_.config->stage3_jump_context_height_cap_m,
        request_.config->stage3_jump_height_highfreq_deadband_m,
        row.height_fallback);
    row.fallback_reason =
      JoinFallback(row.velocity_fallback, row.velocity_delta_fallback, row.height_fallback);
    plan.profiles.push_back(row);
  }

  return plan;
}

}  // namespace offline_lc_minimal
