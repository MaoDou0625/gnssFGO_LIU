#include "offline_lc_minimal/core/Stage1YawBranchResolver.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <gtsam/geometry/Rot3.h>

#include "offline_lc_minimal/core/RtkHeadingAlignmentEstimator.h"
#include "offline_lc_minimal/core/StageAttitudeReference.h"

namespace offline_lc_minimal {
namespace {

constexpr double kCycleYawReturnToleranceRad = 0.05;
constexpr double kCycleBranchSeparationRad = M_PI / 4.0;
constexpr double kContinuityWeight = 10.0;
constexpr double kImuMismatchWeight = 5.0;
constexpr double kScoreTieEpsilon = 1.0e-6;

[[nodiscard]] double FiniteOrPenalty(const double value, const double penalty) {
  return std::isfinite(value) ? value : penalty;
}

[[nodiscard]] double ContinuityMaxRotationRad(const OfflineRunResult &result) {
  const std::vector<ReferenceNodeState> states =
    !result.optimized_reference_states.empty()
      ? result.optimized_reference_states
      : BuildReferenceStatesFromTrajectoryRows(result.trajectory);
  return MaxAdjacentRotationStepRad(states);
}

[[nodiscard]] double ImuRotationMismatchMaxRad(const OfflineRunResult &result) {
  const std::vector<ReferenceNodeState> candidate_states =
    SortedFiniteReferenceStates(
      !result.optimized_reference_states.empty()
        ? result.optimized_reference_states
        : BuildReferenceStatesFromTrajectoryRows(result.trajectory));
  const std::vector<ReferenceNodeState> imu_states =
    SortedFiniteReferenceStates(result.imu_propagated_reference_states);
  if (candidate_states.empty() || imu_states.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }

  double max_mismatch_rad = 0.0;
  std::size_t compared_count = 0U;
  for (const auto &candidate_state : candidate_states) {
    if (!HasReferenceStateCoverage(
          imu_states,
          candidate_state.time_s,
          std::numeric_limits<double>::infinity())) {
      continue;
    }
    const ReferenceNodeState imu_state =
      InterpolateStageReferenceState(imu_states, candidate_state.time_s);
    const double mismatch_rad =
      gtsam::Rot3::Logmap(
        imu_state.pose.rotation().between(candidate_state.pose.rotation())).norm();
    if (std::isfinite(mismatch_rad)) {
      max_mismatch_rad = std::max(max_mismatch_rad, mismatch_rad);
      ++compared_count;
    }
  }
  return compared_count > 0U
    ? max_mismatch_rad
    : std::numeric_limits<double>::quiet_NaN();
}

[[nodiscard]] double CandidateScore(const Stage1YawRefinementDiagnosticRow &row) {
  const double continuity =
    FiniteOrPenalty(row.branch_continuity_max_rot_rad, M_PI);
  const double imu_mismatch =
    FiniteOrPenalty(row.imu_rotation_mismatch_max_rad, M_PI);
  const double heading_error =
    FiniteOrPenalty(std::abs(row.median_error_rad), M_PI);
  const double final_error =
    FiniteOrPenalty(std::abs(row.final_error), 0.0);
  return kContinuityWeight * continuity +
         kImuMismatchWeight * imu_mismatch +
         heading_error +
         1.0e-9 * final_error;
}

[[nodiscard]] bool DetectTwoCycle(
  const std::vector<Stage1YawRefinementDiagnosticRow> &diagnostics) {
  if (diagnostics.size() < 3U) {
    return false;
  }
  for (std::size_t index = 2U; index < diagnostics.size(); ++index) {
    const double return_delta = std::abs(NormalizeHeadingAngleRad(
      diagnostics[index].input_yaw_rad - diagnostics[index - 2U].input_yaw_rad));
    const double branch_delta = std::abs(NormalizeHeadingAngleRad(
      diagnostics[index].input_yaw_rad - diagnostics[index - 1U].input_yaw_rad));
    if (return_delta <= kCycleYawReturnToleranceRad &&
        branch_delta >= kCycleBranchSeparationRad) {
      return true;
    }
  }
  return false;
}

[[nodiscard]] double MinOpposingBranchScore(
  const std::vector<Stage1YawRefinementDiagnosticRow> &diagnostics,
  const Stage1YawRefinementDiagnosticRow &selected) {
  double min_score = std::numeric_limits<double>::infinity();
  for (const auto &row : diagnostics) {
    const double branch_delta = std::abs(NormalizeHeadingAngleRad(
      row.input_yaw_rad - selected.input_yaw_rad));
    if (branch_delta < kCycleBranchSeparationRad ||
        !std::isfinite(row.branch_score)) {
      continue;
    }
    min_score = std::min(min_score, row.branch_score);
  }
  return min_score;
}

}  // namespace

Stage1YawBranchResolution Stage1YawBranchResolver::Resolve(
  std::vector<Stage1YawBranchCandidate> candidates) const {
  Stage1YawBranchResolution resolution;
  resolution.diagnostics.reserve(candidates.size());
  if (candidates.empty()) {
    resolution.selection_reason = "no_candidates";
    return resolution;
  }

  for (auto &candidate : candidates) {
    candidate.diagnostic.branch_continuity_max_rot_rad =
      ContinuityMaxRotationRad(candidate.result);
    candidate.diagnostic.imu_rotation_mismatch_max_rad =
      ImuRotationMismatchMaxRad(candidate.result);
    candidate.diagnostic.branch_score = CandidateScore(candidate.diagnostic);
    resolution.diagnostics.push_back(candidate.diagnostic);
  }
  resolution.cycle_detected = DetectTwoCycle(resolution.diagnostics);

  const auto selected_it = std::min_element(
    resolution.diagnostics.begin(),
    resolution.diagnostics.end(),
    [](const Stage1YawRefinementDiagnosticRow &lhs,
       const Stage1YawRefinementDiagnosticRow &rhs) {
      if (lhs.branch_score != rhs.branch_score) {
        return lhs.branch_score < rhs.branch_score;
      }
      return lhs.iteration < rhs.iteration;
    });
  if (selected_it == resolution.diagnostics.end()) {
    resolution.selection_reason = "no_selectable_branch";
    return resolution;
  }

  resolution.has_selection = true;
  resolution.selected_index =
    static_cast<std::size_t>(std::distance(resolution.diagnostics.begin(), selected_it));
  resolution.selected_iteration = selected_it->iteration;
  resolution.selected_branch_score = selected_it->branch_score;
  if (resolution.cycle_detected) {
    const double opposing_branch_score =
      MinOpposingBranchScore(resolution.diagnostics, *selected_it);
    resolution.reference_valid_for_strong_hold =
      std::isfinite(resolution.selected_branch_score) &&
      resolution.selected_branch_score + kScoreTieEpsilon < opposing_branch_score;
    resolution.stop_reason =
      resolution.reference_valid_for_strong_hold
        ? "cycle_detected_selected_branch"
        : "cycle_detected_unresolved";
    resolution.selection_reason =
      resolution.reference_valid_for_strong_hold
        ? "selected_imu_continuous_rtk_consistent_branch_from_two_cycle"
        : "cycle_detected_ambiguous_branch_reference_invalid";
  } else {
    resolution.reference_valid_for_strong_hold = false;
    resolution.stop_reason = "max_iterations";
    resolution.selection_reason = "max_iterations_reference_invalid";
  }

  for (auto &row : resolution.diagnostics) {
    row.cycle_detected = resolution.cycle_detected;
    row.selected_branch = row.iteration == resolution.selected_iteration;
    row.reference_valid_for_strong_hold =
      row.selected_branch && resolution.reference_valid_for_strong_hold;
    row.selection_reason =
      row.selected_branch ? resolution.selection_reason : "not_selected";
    if (row.selected_branch) {
      row.stop_reason = resolution.stop_reason;
    }
  }
  return resolution;
}

}  // namespace offline_lc_minimal
