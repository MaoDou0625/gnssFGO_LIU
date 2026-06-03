#include "offline_lc_minimal/core/Stage3VerticalReferenceTimelineAligner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "offline_lc_minimal/core/StageAttitudeReference.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimestampToleranceS = 1.0e-9;
constexpr double kMaxBridgeGapS = 1.0;

template <typename Row>
[[nodiscard]] bool HasFiniteTime(const Row &row) {
  return std::isfinite(row.time_s);
}

template <typename Row>
[[nodiscard]] std::vector<Row> SortedFiniteRows(std::vector<Row> rows) {
  rows.erase(
    std::remove_if(
      rows.begin(),
      rows.end(),
      [](const Row &row) { return !HasFiniteTime(row); }),
    rows.end());
  std::stable_sort(
    rows.begin(),
    rows.end(),
    [](const Row &lhs, const Row &rhs) {
      return lhs.time_s < rhs.time_s;
    });
  rows.erase(
    std::unique(
      rows.begin(),
      rows.end(),
      [](const Row &lhs, const Row &rhs) {
        return std::abs(lhs.time_s - rhs.time_s) <= kTimestampToleranceS;
      }),
    rows.end());
  return rows;
}

template <typename Row>
[[nodiscard]] std::size_t LowerBracketIndex(
  const std::vector<Row> &rows,
  const double time_s) {
  if (rows.size() < 2U || time_s <= rows.front().time_s) {
    return 0U;
  }
  if (time_s >= rows.back().time_s) {
    return rows.size() - 2U;
  }
  const auto upper =
    std::upper_bound(
      rows.begin(),
      rows.end(),
      time_s,
      [](const double value, const Row &row) {
        return value < row.time_s;
      });
  const auto lower = std::prev(upper);
  return static_cast<std::size_t>(std::distance(rows.begin(), lower));
}

template <typename Row>
[[nodiscard]] bool HasReferenceCoverage(
  const std::vector<Row> &rows,
  const double time_s) {
  if (rows.empty()) {
    return false;
  }
  if (rows.size() == 1U) {
    return std::abs(time_s - rows.front().time_s) <= kTimestampToleranceS;
  }
  if (time_s < rows.front().time_s - kTimestampToleranceS ||
      time_s > rows.back().time_s + kTimestampToleranceS) {
    return false;
  }
  const std::size_t lower_index = LowerBracketIndex(rows, time_s);
  return rows[lower_index + 1U].time_s - rows[lower_index].time_s <= kMaxBridgeGapS;
}

[[nodiscard]] double BlendScalar(
  const double lhs,
  const double rhs,
  const double alpha) {
  if (!std::isfinite(lhs) || !std::isfinite(rhs)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return lhs + alpha * (rhs - lhs);
}

[[nodiscard]] double BlendFiniteOrNearest(
  const double lhs,
  const double rhs,
  const double alpha) {
  if (std::isfinite(lhs) && std::isfinite(rhs)) {
    return lhs + alpha * (rhs - lhs);
  }
  if (alpha < 0.5 && std::isfinite(lhs)) {
    return lhs;
  }
  if (std::isfinite(rhs)) {
    return rhs;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

[[nodiscard]] std::string BlendSkipReason(
  const Stage3VerticalReferenceDiagnosticRow &lhs,
  const Stage3VerticalReferenceDiagnosticRow &rhs,
  const double alpha,
  const double stage2_lowpass_up_m) {
  const auto &nearest = alpha < 0.5 ? lhs : rhs;
  if (nearest.skip_reason == "TERMINAL_STATIC") {
    return "TERMINAL_STATIC";
  }
  return std::isfinite(stage2_lowpass_up_m) ? "PLANNED" : "LOWPASS_UNAVAILABLE";
}

[[nodiscard]] double InterpolationAlpha(
  const double time_s,
  const double lhs_time_s,
  const double rhs_time_s) {
  const double dt_s = rhs_time_s - lhs_time_s;
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return 0.0;
  }
  return std::clamp((time_s - lhs_time_s) / dt_s, 0.0, 1.0);
}

[[nodiscard]] Stage3VerticalReferenceDiagnosticRow InterpolateStage3Row(
  const std::vector<Stage3VerticalReferenceDiagnosticRow> &rows,
  const double target_time_s,
  const std::size_t state_index) {
  Stage3VerticalReferenceDiagnosticRow row;
  row.state_index = state_index;
  row.time_s = target_time_s;
  row.factor_added = false;

  if (rows.empty()) {
    row.skip_reason = "LOWPASS_UNAVAILABLE";
    return row;
  }
  if (!HasReferenceCoverage(rows, target_time_s)) {
    row.skip_reason = "LOWPASS_UNAVAILABLE";
    return row;
  }
  if (rows.size() == 1U) {
    row = rows.front();
    row.state_index = state_index;
    row.time_s = target_time_s;
    row.factor_added = false;
    row.optimized_up_m = std::numeric_limits<double>::quiet_NaN();
    row.residual_m = std::numeric_limits<double>::quiet_NaN();
    if (row.skip_reason != "TERMINAL_STATIC") {
      row.skip_reason =
        std::isfinite(row.stage2_lowpass_up_m) ? "PLANNED" : "LOWPASS_UNAVAILABLE";
    }
    return row;
  }

  const std::size_t lower_index = LowerBracketIndex(rows, target_time_s);
  const auto &lhs = rows[lower_index];
  const auto &rhs = rows[lower_index + 1U];
  const double alpha = InterpolationAlpha(target_time_s, lhs.time_s, rhs.time_s);
  row.stage2_up_m = BlendScalar(lhs.stage2_up_m, rhs.stage2_up_m, alpha);
  row.stage2_lowpass_up_m =
    BlendFiniteOrNearest(lhs.stage2_lowpass_up_m, rhs.stage2_lowpass_up_m, alpha);
  row.lowpass_delta_m = row.stage2_lowpass_up_m - row.stage2_up_m;
  row.sigma_m = BlendFiniteOrNearest(lhs.sigma_m, rhs.sigma_m, alpha);
  row.optimized_up_m = std::numeric_limits<double>::quiet_NaN();
  row.residual_m = std::numeric_limits<double>::quiet_NaN();
  row.skip_reason = BlendSkipReason(lhs, rhs, alpha, row.stage2_lowpass_up_m);
  return row;
}

}  // namespace

Stage3VerticalReferenceTimelineAlignResult AlignStage3VerticalReferencesToTimeline(
  const Stage2VelocityReference &stage2_reference,
  const Stage3VerticalReference &stage3_reference,
  const std::vector<double> &target_timestamps_s) {
  const std::vector<ReferenceNodeState> stage2_reference_states =
    SortedFiniteReferenceStates(BuildStage2ReferenceStates(stage2_reference));
  const std::vector<Stage3VerticalReferenceDiagnosticRow> stage3_rows =
    SortedFiniteRows(stage3_reference.rows);
  if (stage2_reference_states.empty()) {
    throw std::runtime_error("cannot align Stage3 references from an empty Stage2 trajectory");
  }
  if (target_timestamps_s.empty()) {
    throw std::runtime_error("cannot align Stage3 references to an empty graph timeline");
  }

  Stage3VerticalReferenceTimelineAlignResult result;
  result.stage2_reference.boundary_references = stage2_reference.boundary_references;
  result.stage2_reference.source_config = stage2_reference.source_config;
  result.stage2_reference.trajectory.reserve(target_timestamps_s.size());
  result.stage2_reference.reference_states.reserve(target_timestamps_s.size());
  result.stage3_reference.source_config = stage3_reference.source_config;
  result.stage3_reference.rows.reserve(target_timestamps_s.size());

  for (std::size_t state_index = 0; state_index < target_timestamps_s.size(); ++state_index) {
    const double target_time_s = target_timestamps_s[state_index];
    if (!HasReferenceStateCoverage(stage2_reference_states, target_time_s, kMaxBridgeGapS)) {
      throw std::runtime_error("Stage2 trajectory does not cover the Stage3 graph timeline");
    }
    const ReferenceNodeState aligned_state =
      InterpolateStageReferenceState(stage2_reference_states, target_time_s);
    result.stage2_reference.reference_states.push_back(aligned_state);
    result.stage2_reference.trajectory.push_back(
      TrajectoryRowFromReferenceState(aligned_state));
    result.stage3_reference.rows.push_back(
      InterpolateStage3Row(stage3_rows, target_time_s, state_index));
  }
  return result;
}

}  // namespace offline_lc_minimal
