#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace offline_lc_minimal {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool CanFilterRow(const TrajectoryRow &row) {
  return std::isfinite(row.time_s) && std::isfinite(row.enu_position_m.z());
}

std::vector<double> BuildRawFilterInputUp(
  const std::vector<TrajectoryRow> &trajectory) {
  std::vector<double> input_up_m(
    trajectory.size(),
    std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    if (CanFilterRow(trajectory[index])) {
      input_up_m[index] = trajectory[index].enu_position_m.z();
    }
  }
  return input_up_m;
}

double LowpassAlpha(const double dt_s, const double tau_s) {
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return 0.0;
  }
  return 1.0 - std::exp(-dt_s / tau_s);
}

void FilterSegment(
  const OfflineRunnerConfig &config,
  const std::vector<std::size_t> &indices,
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &input_up_m,
  std::vector<double> &lowpass_up_m) {
  if (indices.empty()) {
    return;
  }

  const double tau_s = 1.0 / (kTwoPi * config.stage3_vertical_reference_lowpass_cutoff_hz);
  std::vector<double> forward(indices.size(), 0.0);
  forward.front() = input_up_m[indices.front()];

  for (std::size_t i = 1; i < indices.size(); ++i) {
    const auto &prev_row = trajectory[indices[i - 1U]];
    const auto &row = trajectory[indices[i]];
    const double alpha = LowpassAlpha(row.time_s - prev_row.time_s, tau_s);
    forward[i] = forward[i - 1U] + alpha * (input_up_m[indices[i]] - forward[i - 1U]);
  }

  std::vector<double> zero_phase = forward;
  for (std::size_t reverse = indices.size() - 1U; reverse > 0U; --reverse) {
    const std::size_t i = reverse - 1U;
    const auto &row = trajectory[indices[i]];
    const auto &next_row = trajectory[indices[i + 1U]];
    const double alpha = LowpassAlpha(next_row.time_s - row.time_s, tau_s);
    zero_phase[i] = zero_phase[i + 1U] + alpha * (forward[i] - zero_phase[i + 1U]);
  }

  for (std::size_t i = 0; i < indices.size(); ++i) {
    lowpass_up_m[indices[i]] = zero_phase[i];
  }
}

std::vector<double> BuildLowpassProfile(
  const OfflineRunnerConfig &config,
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &input_up_m) {
  std::vector<double> lowpass_up_m(
    trajectory.size(),
    std::numeric_limits<double>::quiet_NaN());
  std::vector<std::size_t> segment;
  segment.reserve(trajectory.size());
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    if (!CanFilterRow(trajectory[index]) || !std::isfinite(input_up_m[index])) {
      FilterSegment(config, segment, trajectory, input_up_m, lowpass_up_m);
      segment.clear();
      continue;
    }
    if (!segment.empty()) {
      const auto &prev_row = trajectory[segment.back()];
      const auto &row = trajectory[index];
      if (row.time_s <= prev_row.time_s) {
        FilterSegment(config, segment, trajectory, input_up_m, lowpass_up_m);
        segment.clear();
      }
    }
    segment.push_back(index);
  }
  FilterSegment(config, segment, trajectory, input_up_m, lowpass_up_m);
  return lowpass_up_m;
}

bool FindInitialDynamicStaticReferenceUp(
  const std::vector<TrajectoryRow> &trajectory,
  const std::size_t dynamic_start_index,
  const double dynamic_start_time_s,
  double &reference_up_m) {
  if (dynamic_start_index < trajectory.size()) {
    const std::size_t first_candidate =
      dynamic_start_index > 0U ? dynamic_start_index - 1U : dynamic_start_index;
    for (std::size_t reverse = first_candidate + 1U; reverse > 0U; --reverse) {
      const std::size_t index = reverse - 1U;
      if (CanFilterRow(trajectory[index])) {
        reference_up_m = trajectory[index].enu_position_m.z();
        return true;
      }
    }
    for (std::size_t index = dynamic_start_index; index < trajectory.size(); ++index) {
      if (CanFilterRow(trajectory[index])) {
        reference_up_m = trajectory[index].enu_position_m.z();
        return true;
      }
    }
    return false;
  }

  bool found = false;
  double best_time_s = -std::numeric_limits<double>::infinity();
  for (const auto &row : trajectory) {
    if (!CanFilterRow(row) || row.time_s > dynamic_start_time_s) {
      continue;
    }
    if (row.time_s >= best_time_s) {
      best_time_s = row.time_s;
      reference_up_m = row.enu_position_m.z();
      found = true;
    }
  }
  if (found) {
    return true;
  }
  for (const auto &row : trajectory) {
    if (!CanFilterRow(row) || row.time_s < dynamic_start_time_s) {
      continue;
    }
    reference_up_m = row.enu_position_m.z();
    return true;
  }
  return false;
}

void ApplyInitialDynamicStaticHoldToProfile(
  const OfflineRunnerConfig &config,
  const std::size_t dynamic_start_index,
  const double dynamic_start_time_s,
  const std::vector<TrajectoryRow> &trajectory,
  std::vector<double> &up_profile_m) {
  if (!config.enable_stage3_initial_dynamic_static_reference_hold ||
      config.stage3_initial_dynamic_static_reference_hold_duration_s <= 0.0) {
    return;
  }
  const bool has_dynamic_start_index = dynamic_start_index < trajectory.size();
  if (!has_dynamic_start_index && !std::isfinite(dynamic_start_time_s)) {
    return;
  }

  double static_reference_up_m = std::numeric_limits<double>::quiet_NaN();
  if (!FindInitialDynamicStaticReferenceUp(
        trajectory,
        dynamic_start_index,
        dynamic_start_time_s,
        static_reference_up_m)) {
    return;
  }

  const double hold_start_time_s =
    std::isfinite(dynamic_start_time_s)
      ? dynamic_start_time_s
      : trajectory[dynamic_start_index].time_s;
  const double hold_end_time_s =
    hold_start_time_s +
    config.stage3_initial_dynamic_static_reference_hold_duration_s;
  const double blend_duration_s =
    std::max(0.0, config.stage3_initial_dynamic_static_reference_hold_blend_s);
  const double blend_end_time_s = hold_end_time_s + blend_duration_s;

  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    const auto &row = trajectory[index];
    if (!std::isfinite(row.time_s) || !std::isfinite(up_profile_m[index]) ||
        (has_dynamic_start_index
           ? index < dynamic_start_index
           : row.time_s < hold_start_time_s)) {
      continue;
    }
    if (row.time_s <= hold_end_time_s) {
      up_profile_m[index] = static_reference_up_m;
      continue;
    }
    if (blend_duration_s > 0.0 && row.time_s < blend_end_time_s) {
      const double blend_fraction =
        (row.time_s - hold_end_time_s) / blend_duration_s;
      up_profile_m[index] =
        (1.0 - blend_fraction) * static_reference_up_m +
        blend_fraction * up_profile_m[index];
    }
  }
}

}  // namespace

Stage3VerticalReferenceProfilePlanner::Stage3VerticalReferenceProfilePlanner(
  Stage3VerticalReferenceProfilePlanRequest request)
    : request_(std::move(request)) {}

Stage3VerticalReference Stage3VerticalReferenceProfilePlanner::Plan() const {
  if (request_.config == nullptr || request_.stage2_trajectory == nullptr) {
    throw std::runtime_error(
      "Stage3VerticalReferenceProfilePlanner received an incomplete request");
  }
  if (request_.stage2_trajectory->empty()) {
    throw std::runtime_error("Stage3 vertical reference requires a non-empty Stage2 trajectory");
  }

  std::vector<double> filter_input_up_m =
    BuildRawFilterInputUp(*request_.stage2_trajectory);
  ApplyInitialDynamicStaticHoldToProfile(
    *request_.config,
    request_.dynamic_start_index,
    request_.dynamic_start_time_s,
    *request_.stage2_trajectory,
    filter_input_up_m);
  std::vector<double> lowpass_up_m =
    BuildLowpassProfile(
      *request_.config,
      *request_.stage2_trajectory,
      filter_input_up_m);
  ApplyInitialDynamicStaticHoldToProfile(
    *request_.config,
    request_.dynamic_start_index,
    request_.dynamic_start_time_s,
    *request_.stage2_trajectory,
    lowpass_up_m);

  Stage3VerticalReference reference;
  reference.source_config =
    std::make_shared<OfflineRunnerConfig>(*request_.config);
  reference.rows.reserve(request_.stage2_trajectory->size());
  for (std::size_t index = 0; index < request_.stage2_trajectory->size(); ++index) {
    const auto &trajectory_row = (*request_.stage2_trajectory)[index];
    Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = trajectory_row.time_s;
    row.stage2_up_m = trajectory_row.enu_position_m.z();
    row.stage2_lowpass_up_m = lowpass_up_m[index];
    row.lowpass_delta_m = row.stage2_lowpass_up_m - row.stage2_up_m;
    row.sigma_m = request_.config->stage3_vertical_anchor_sigma_m;
    row.factor_added = false;
    row.skip_reason = std::isfinite(row.stage2_lowpass_up_m) ? "PLANNED" : "LOWPASS_UNAVAILABLE";
    reference.rows.push_back(row);
  }
  return reference;
}

}  // namespace offline_lc_minimal
