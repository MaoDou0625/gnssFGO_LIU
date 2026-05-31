#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

#include "offline_lc_minimal/core/Stage3VerticalReferenceSmoother.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace offline_lc_minimal {
namespace {

bool CanFilterRow(const TrajectoryRow &row) {
  return std::isfinite(row.time_s) && std::isfinite(row.enu_position_m.z());
}

bool IsTerminalStaticCandidate(
  const OfflineRunnerConfig &config,
  const TrajectoryRow &row) {
  if (!std::isfinite(row.time_s) || !row.enu_velocity_mps.allFinite()) {
    return false;
  }
  const double horizontal_speed_mps =
    std::hypot(row.enu_velocity_mps.x(), row.enu_velocity_mps.y());
  return horizontal_speed_mps <=
           config.stage3_vertical_reference_terminal_static_speed_threshold_mps &&
         std::abs(row.enu_velocity_mps.z()) <=
           config.stage3_vertical_reference_terminal_static_vz_threshold_mps;
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

std::size_t FirstLowpassFilterIndex(
  const std::size_t dynamic_start_index,
  const std::size_t trajectory_size) {
  if (dynamic_start_index >= trajectory_size) {
    return 0U;
  }
  return dynamic_start_index;
}

std::size_t OnePastLastLowpassFilterIndex(
  const OfflineRunnerConfig &config,
  const std::vector<TrajectoryRow> &trajectory,
  const std::size_t dynamic_start_index) {
  if (!config.enable_stage3_vertical_reference_terminal_static_exclusion ||
      dynamic_start_index >= trajectory.size() || trajectory.empty()) {
    return trajectory.size();
  }

  std::size_t first_terminal_static_index = trajectory.size();
  for (std::size_t reverse = trajectory.size(); reverse > dynamic_start_index; --reverse) {
    const std::size_t index = reverse - 1U;
    if (!IsTerminalStaticCandidate(config, trajectory[index])) {
      break;
    }
    first_terminal_static_index = index;
  }
  if (first_terminal_static_index == trajectory.size()) {
    return trajectory.size();
  }
  const double terminal_static_duration_s =
    trajectory.back().time_s - trajectory[first_terminal_static_index].time_s;
  if (!std::isfinite(terminal_static_duration_s) ||
      terminal_static_duration_s + 1.0e-9 <
        config.stage3_vertical_reference_terminal_static_min_duration_s) {
    return trajectory.size();
  }
  return first_terminal_static_index;
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

bool TimeInWindow(const double time_s, const LateStaticWindowRow &window) {
  return std::isfinite(time_s) &&
         time_s + 1.0e-9 >= window.start_time_s &&
         time_s <= window.end_time_s + 1.0e-9;
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const std::size_t mid = values.size() / 2U;
  std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid), values.end());
  double median = values[mid];
  if ((values.size() % 2U) == 0U) {
    const auto lower_max =
      std::max_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(mid));
    median = 0.5 * (median + *lower_max);
  }
  return median;
}

bool ApplyInitialDynamicStaticWindowsToProfile(
  const OfflineRunnerConfig &config,
  const std::vector<LateStaticWindowRow> *windows,
  const std::vector<TrajectoryRow> &trajectory,
  std::vector<double> &up_profile_m,
  const bool protect_output) {
  if (!config.enable_initial_dynamic_static_lowpass_protection ||
      windows == nullptr || windows->empty()) {
    return false;
  }

  bool applied_any = false;
  for (const auto &window : *windows) {
    if (!window.valid) {
      continue;
    }
    std::vector<double> source_up_values;
    source_up_values.reserve(trajectory.size());
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
      if (TimeInWindow(trajectory[index].time_s, window) &&
          CanFilterRow(trajectory[index])) {
        source_up_values.push_back(trajectory[index].enu_position_m.z());
      }
    }
    const double reference_up_m = Median(std::move(source_up_values));
    if (!std::isfinite(reference_up_m)) {
      continue;
    }

    const double blend_duration_s =
      std::max(0.0, config.initial_dynamic_static_lowpass_blend_s);
    const double blend_end_time_s = window.end_time_s + blend_duration_s;
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
      const auto &row = trajectory[index];
      if (!std::isfinite(row.time_s) || !std::isfinite(up_profile_m[index])) {
        continue;
      }
      if (TimeInWindow(row.time_s, window)) {
        up_profile_m[index] = reference_up_m;
        applied_any = true;
        continue;
      }
      if (!protect_output) {
        continue;
      }
      if (blend_duration_s > 0.0 &&
          row.time_s > window.end_time_s &&
          row.time_s < blend_end_time_s) {
        const double blend_fraction =
          (row.time_s - window.end_time_s) / blend_duration_s;
        up_profile_m[index] =
          (1.0 - blend_fraction) * reference_up_m +
          blend_fraction * up_profile_m[index];
        applied_any = true;
      }
    }
  }
  return applied_any;
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
  const bool applied_detected_static_windows_to_input =
    ApplyInitialDynamicStaticWindowsToProfile(
      *request_.config,
      request_.initial_dynamic_static_windows,
      *request_.stage2_trajectory,
      filter_input_up_m,
      false);
  if (!applied_detected_static_windows_to_input) {
    ApplyInitialDynamicStaticHoldToProfile(
      *request_.config,
      request_.dynamic_start_index,
      request_.dynamic_start_time_s,
      *request_.stage2_trajectory,
      filter_input_up_m);
  }
  const std::size_t first_filter_index =
    FirstLowpassFilterIndex(
      request_.dynamic_start_index,
      request_.stage2_trajectory->size());
  const std::size_t one_past_last_filter_index =
    OnePastLastLowpassFilterIndex(
      *request_.config,
      *request_.stage2_trajectory,
      request_.dynamic_start_index);
  std::vector<double> lowpass_up_m =
    BuildStage3VerticalReferenceSmoothedProfile(
      *request_.config,
      *request_.stage2_trajectory,
      filter_input_up_m,
      first_filter_index,
      one_past_last_filter_index);
  const bool applied_detected_static_windows_to_output =
    ApplyInitialDynamicStaticWindowsToProfile(
      *request_.config,
      request_.initial_dynamic_static_windows,
      *request_.stage2_trajectory,
      lowpass_up_m,
      true);
  if (!applied_detected_static_windows_to_input &&
      !applied_detected_static_windows_to_output) {
    ApplyInitialDynamicStaticHoldToProfile(
      *request_.config,
      request_.dynamic_start_index,
      request_.dynamic_start_time_s,
      *request_.stage2_trajectory,
      lowpass_up_m);
  }

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
    row.skip_reason =
      index >= one_past_last_filter_index
        ? "TERMINAL_STATIC"
        : (std::isfinite(row.stage2_lowpass_up_m) ? "PLANNED" : "LOWPASS_UNAVAILABLE");
    reference.rows.push_back(row);
  }
  return reference;
}

}  // namespace offline_lc_minimal
