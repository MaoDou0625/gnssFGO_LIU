#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <unordered_set>
#include <vector>

#include <Eigen/Core>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"
#include "offline_lc_minimal/core/SparseVerticalJumpPlanner.h"
#include "offline_lc_minimal/core/VerticalInsideBiasAdapter.h"

namespace offline_lc_minimal {

struct VerticalHoldWindowSpec {
  std::size_t sample_index = 0;
  double corrected_time_s = 0.0;
  Eigen::Vector3d measurement_enu_m = Eigen::Vector3d::Zero();
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t reference_state_index = 0;
  bool interpolated = false;
};

struct VerticalHoldWindowEvaluation {
  bool current_inside = false;
  bool hold_window_passed = false;
  double gate_excess_cost = std::numeric_limits<double>::infinity();
  double current_local_postfit_u_m = std::numeric_limits<double>::quiet_NaN();
  double max_up_from_vz_error_m = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalFutureTrendEvaluation {
  bool valid = false;
  std::size_t fix_count = 0;
  double residual_mean_m = std::numeric_limits<double>::quiet_NaN();
  double residual_slope_mps = std::numeric_limits<double>::quiet_NaN();
  double cost = 0.0;
};

struct VerticalLocalRecoveryResult {
  ReferenceNodeState recovered_anchor_state;
  double local_postfit_u_m = std::numeric_limits<double>::quiet_NaN();
  double required_up_anchor_correction_m = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_applied_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_up_anchor_applied_m = std::numeric_limits<double>::quiet_NaN();
  double delta_roll_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_pitch_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_baz_applied_mps2 = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_state_index = -1;
  double selected_jump_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double jump_candidate_score = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_window_start_state_index = -1;
  long long selected_jump_window_center_state_index = -1;
  long long selected_jump_window_end_state_index = -1;
  double selected_jump_window_duration_s = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_window_point_count = 0;
  double selected_jump_delta_vz_tail_mps = std::numeric_limits<double>::quiet_NaN();
  double window_velocity_smooth_cost = std::numeric_limits<double>::quiet_NaN();
  double window_height_integral_delta_m = std::numeric_limits<double>::quiet_NaN();
  double future_trend_residual_mean_m = std::numeric_limits<double>::quiet_NaN();
  double future_trend_residual_slope_mps = std::numeric_limits<double>::quiet_NaN();
  double future_trend_cost = std::numeric_limits<double>::quiet_NaN();
  long long future_trend_fix_count = 0;
  std::string recovery_mode = "NONE";
  bool hold_window_passed = false;
};

struct VerticalVelocityWindowCorrection {
  std::size_t start_state_index = 0;
  std::size_t center_state_index = 0;
  std::size_t end_state_index = 0;
  double target_tail_up_m = std::numeric_limits<double>::quiet_NaN();
  double target_tail_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_up_tail_m = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_tail_mps = std::numeric_limits<double>::quiet_NaN();
  double velocity_smooth_cost = std::numeric_limits<double>::quiet_NaN();
  double height_integral_delta_m = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> corrected_up_m;
  std::vector<double> corrected_vz_mps;
};

[[nodiscard]] gtsam::Rot3 InterpolateRotation(
  const gtsam::Rot3 &left,
  const gtsam::Rot3 &right,
  double alpha);
[[nodiscard]] ReferenceNodeState InterpolateReferenceState(
  const std::vector<ReferenceNodeState> &states,
  double timestamp_s);
[[nodiscard]] Eigen::Vector3d ComputePositionResidualEnu(
  const gtsam::Pose3 &pose,
  const Eigen::Vector3d &measurement_enu_m);
[[nodiscard]] double ComputeVerticalNis(double residual_u_m, double sigma_u_m);
[[nodiscard]] double ComputeUpFromVzConsistencyError(
  const std::vector<ReferenceNodeState> &reference_states,
  std::size_t start_index,
  std::size_t end_index);

[[nodiscard]] ReferenceNodeState ResolveReferenceStateForHoldWindowSpec(
  const std::vector<ReferenceNodeState> &reference_states,
  const VerticalHoldWindowSpec &spec);
[[nodiscard]] std::vector<VerticalHoldWindowSpec> FilterVerticalHoldWindowSpecsAfterState(
  const std::vector<VerticalHoldWindowSpec> &hold_window_specs,
  std::size_t state_index);
[[nodiscard]] VerticalHoldWindowEvaluation EvaluateVerticalHoldWindow(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalHoldWindowSpec> &hold_window_specs,
  std::size_t up_anchor_state_index,
  double vertical_gate_nis_threshold);
[[nodiscard]] VerticalFutureTrendEvaluation EvaluateVerticalFutureTrend(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalHoldWindowSpec> &future_trend_specs,
  int minimum_fix_count,
  double mean_weight,
  double slope_weight);
[[nodiscard]] std::optional<double> EstimateIntervalVelocityFeedbackDelta(
  const OfflineRunnerConfig &config,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalHoldWindowSpec> &interval_specs,
  double active_window_end_time_s);

void UpsertReferenceStateInitialValues(
  std::size_t state_index,
  const ReferenceNodeState &state,
  gtsam::Values *values);
[[nodiscard]] ReferenceNodeState ApplyVerticalUpAnchorCorrection(
  const ReferenceNodeState &anchor_state,
  double delta_up_anchor_m);
[[nodiscard]] ReferenceNodeState ApplyInsideLowFrequencyStateCorrection(
  const ReferenceNodeState &anchor_state,
  const VerticalInsideBiasUpdate &update);
[[nodiscard]] double ComputeRequiredUpAnchorCorrectionM(
  const gtsam::Pose3 &propagated_pose,
  const Eigen::Vector3d &measurement_enu_m);
[[nodiscard]] std::size_t ResolvePrefitReferenceRightIndex(
  const StateMeasSyncResult &sync_result);

void PushUniqueCandidateValue(std::vector<double> *values, double value);
[[nodiscard]] double MedianFinite(std::vector<double> values);
[[nodiscard]] double BodyZWindowCurrentTailOffsetM(
  const std::vector<ReferenceNodeState> &reference_states,
  const SparseVerticalJumpWindowCandidate &window_candidate);
[[nodiscard]] std::vector<double> BuildBodyZTailVelocityTargetsMps(
  const std::vector<ReferenceNodeState> &reference_states,
  const SparseVerticalJumpWindowCandidate &window_candidate,
  bool velocity_already_corrected,
  bool velocity_feedback_requested_for_window,
  double velocity_feedback_delta_mps,
  double stored_tail_velocity_target_mps);
[[nodiscard]] std::vector<double> BuildBodyZTailPositionOffsetsM(
  const std::vector<ReferenceNodeState> &reference_states,
  const SparseVerticalJumpWindowCandidate &window_candidate,
  bool position_offset_already_corrected);
[[nodiscard]] std::optional<VerticalVelocityWindowCorrection> BuildVerticalVelocityWindowCorrection(
  const OfflineRunnerConfig &config,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const SparseVerticalJumpWindowCandidate &window_candidate,
  std::size_t segment_end_state_index,
  double tail_delta_scale = 1.0,
  double forced_tail_delta_mps = std::numeric_limits<double>::quiet_NaN(),
  double forced_tail_delta_up_m = std::numeric_limits<double>::quiet_NaN());
[[nodiscard]] std::optional<SparseVerticalJumpWindowCandidate> BuildBodyZSeedSparseWindowCandidate(
  const BodyZJumpWindowCandidate &body_z_window,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  std::size_t dynamic_start_index,
  std::size_t feedback_anchor_state_index,
  const std::optional<std::size_t> &required_center_state_index,
  const std::unordered_set<std::size_t> &nhc_supported_state_indices);
[[nodiscard]] std::optional<SparseVerticalJumpWindowCandidate> FindLatestEndedBodyZSeedWindowCandidate(
  const std::vector<BodyZJumpWindowCandidate> &body_z_windows,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  std::size_t dynamic_start_index,
  std::size_t feedback_anchor_state_index);
[[nodiscard]] std::optional<SparseVerticalJumpWindowCandidate> FindCoveringBodyZSeedWindowCandidate(
  const std::vector<BodyZJumpWindowCandidate> &body_z_windows,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  std::size_t dynamic_start_index,
  std::size_t feedback_anchor_state_index,
  double corrected_time_s,
  double sync_upper_bound_s);
void ApplyVerticalVelocityWindowCorrection(
  const VerticalVelocityWindowCorrection &correction,
  std::vector<ReferenceNodeState> *reference_states,
  gtsam::Values *initial_values);

}  // namespace offline_lc_minimal
