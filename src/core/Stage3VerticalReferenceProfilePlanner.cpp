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
  std::vector<double> &lowpass_up_m) {
  if (indices.empty()) {
    return;
  }

  const double tau_s = 1.0 / (kTwoPi * config.stage3_vertical_reference_lowpass_cutoff_hz);
  std::vector<double> forward(indices.size(), 0.0);
  forward.front() = trajectory[indices.front()].enu_position_m.z();

  for (std::size_t i = 1; i < indices.size(); ++i) {
    const auto &prev_row = trajectory[indices[i - 1U]];
    const auto &row = trajectory[indices[i]];
    const double alpha = LowpassAlpha(row.time_s - prev_row.time_s, tau_s);
    forward[i] = forward[i - 1U] + alpha * (row.enu_position_m.z() - forward[i - 1U]);
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
  const std::vector<TrajectoryRow> &trajectory) {
  std::vector<double> lowpass_up_m(
    trajectory.size(),
    std::numeric_limits<double>::quiet_NaN());
  std::vector<std::size_t> segment;
  segment.reserve(trajectory.size());
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    if (!CanFilterRow(trajectory[index])) {
      FilterSegment(config, segment, trajectory, lowpass_up_m);
      segment.clear();
      continue;
    }
    if (!segment.empty()) {
      const auto &prev_row = trajectory[segment.back()];
      const auto &row = trajectory[index];
      if (row.time_s <= prev_row.time_s) {
        FilterSegment(config, segment, trajectory, lowpass_up_m);
        segment.clear();
      }
    }
    segment.push_back(index);
  }
  FilterSegment(config, segment, trajectory, lowpass_up_m);
  return lowpass_up_m;
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

  const std::vector<double> lowpass_up_m =
    BuildLowpassProfile(*request_.config, *request_.stage2_trajectory);

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
