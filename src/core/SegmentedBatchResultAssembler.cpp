#include "offline_lc_minimal/core/SegmentedBatchResultAssembler.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool InSegment(
  const double time_s,
  const RtkOutageBatchSegmentRow &segment,
  const bool include_start) {
  if (!std::isfinite(time_s)) {
    return false;
  }
  const bool after_start = include_start
    ? time_s >= segment.start_time_s - kTimeEpsilonS
    : time_s > segment.start_time_s + kTimeEpsilonS;
  return after_start && time_s <= segment.end_time_s + kTimeEpsilonS;
}

template <typename Row, typename TimeFn>
void AppendRowsInSegment(
  std::vector<Row> &target,
  const std::vector<Row> &source,
  const RtkOutageBatchSegmentRow &segment,
  const bool include_start,
  TimeFn time_fn) {
  for (const auto &row : source) {
    if (InSegment(time_fn(row), segment, include_start)) {
      target.push_back(row);
    }
  }
}

template <typename Row>
void AppendAll(std::vector<Row> &target, const std::vector<Row> &source) {
  target.insert(target.end(), source.begin(), source.end());
}

std::vector<RtkOutageBatchSegmentRow> CollectSegments(
  const std::vector<SegmentedBatchResultPiece> &pieces) {
  std::vector<RtkOutageBatchSegmentRow> segments;
  segments.reserve(pieces.size());
  for (const auto &piece : pieces) {
    segments.push_back(piece.segment);
  }
  return segments;
}

}  // namespace

SegmentedBatchResultAssembler::SegmentedBatchResultAssembler(
  SegmentedBatchResultAssemblerRequest request)
    : request_(std::move(request)) {}

OfflineRunResult SegmentedBatchResultAssembler::Assemble() const {
  if (request_.pieces.empty()) {
    throw std::runtime_error("SegmentedBatchResultAssembler requires at least one segment result");
  }

  OfflineRunResult assembled = request_.pieces.front().result;
  assembled.trajectory.clear();
  assembled.reference_node_trajectory.clear();
  assembled.gnss_factor_records.clear();
  assembled.gnss_consistency_records.clear();
  assembled.vertical_envelope_diagnostics.clear();
  assembled.rtk_vertical_drift_reference_diagnostics.clear();
  assembled.rtk_outage_causal_nav_reference_diagnostics.clear();
  assembled.rtk_outage_attitude_hold_diagnostics.clear();
  assembled.rtk_outage_velocity_delta_3d_diagnostics.clear();
  assembled.rtk_velocity_diagnostics.clear();
  assembled.vertical_velocity_delta_diagnostics.clear();
  assembled.vertical_motion_adaptive_reweighting_diagnostics.clear();
  assembled.vertical_position_velocity_consistency_diagnostics.clear();
  assembled.vertical_state_corrections.clear();
  assembled.body_z_bias_reestimate_segments.clear();
  assembled.rtk_outage_windows = request_.outage_windows;
  assembled.rtk_outage_batch_segments = CollectSegments(request_.pieces);

  const auto &first_piece = request_.pieces.front();
  AppendRowsInSegment(
    assembled.trajectory,
    first_piece.result.trajectory,
    RtkOutageBatchSegmentRow{
      0U,
      "INITIAL_STATIC",
      -1,
      -std::numeric_limits<double>::infinity(),
      first_piece.segment.start_time_s - 2.0 * kTimeEpsilonS,
      std::numeric_limits<double>::quiet_NaN(),
      true,
      false,
      "RUN_START",
      "DYNAMIC_START",
      "PLANNED"},
    true,
    [](const TrajectoryRow &row) { return row.time_s; });
  AppendRowsInSegment(
    assembled.reference_node_trajectory,
    first_piece.result.reference_node_trajectory,
    RtkOutageBatchSegmentRow{
      0U,
      "INITIAL_STATIC",
      -1,
      -std::numeric_limits<double>::infinity(),
      first_piece.segment.start_time_s - 2.0 * kTimeEpsilonS,
      std::numeric_limits<double>::quiet_NaN(),
      true,
      false,
      "RUN_START",
      "DYNAMIC_START",
      "PLANNED"},
    true,
    [](const ReferenceNodeRow &row) { return row.time_s; });

  for (std::size_t piece_index = 0; piece_index < request_.pieces.size(); ++piece_index) {
    const auto &piece = request_.pieces[piece_index];
    const bool include_start = piece_index == 0U;
    AppendRowsInSegment(
      assembled.trajectory,
      piece.result.trajectory,
      piece.segment,
      include_start,
      [](const TrajectoryRow &row) { return row.time_s; });
    AppendRowsInSegment(
      assembled.reference_node_trajectory,
      piece.result.reference_node_trajectory,
      piece.segment,
      include_start,
      [](const ReferenceNodeRow &row) { return row.time_s; });
    AppendRowsInSegment(
      assembled.gnss_factor_records,
      piece.result.gnss_factor_records,
      piece.segment,
      include_start,
      [](const GnssFactorRecord &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.gnss_consistency_records,
      piece.result.gnss_consistency_records,
      piece.segment,
      include_start,
      [](const GnssConsistencyRecord &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.vertical_envelope_diagnostics,
      piece.result.vertical_envelope_diagnostics,
      piece.segment,
      include_start,
      [](const VerticalEnvelopeDiagnosticRow &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.rtk_vertical_drift_reference_diagnostics,
      piece.result.rtk_vertical_drift_reference_diagnostics,
      piece.segment,
      include_start,
      [](const RtkVerticalDriftReferenceDiagnosticRow &row) { return row.time_s; });
    AppendRowsInSegment(
      assembled.rtk_outage_causal_nav_reference_diagnostics,
      piece.result.rtk_outage_causal_nav_reference_diagnostics,
      piece.segment,
      include_start,
      [](const RtkOutageCausalNavReferenceRow &row) { return row.time_s; });
    AppendRowsInSegment(
      assembled.rtk_velocity_diagnostics,
      piece.result.rtk_velocity_diagnostics,
      piece.segment,
      include_start,
      [](const RtkVelocityDiagnosticRow &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.vertical_velocity_delta_diagnostics,
      piece.result.vertical_velocity_delta_diagnostics,
      piece.segment,
      include_start,
      [](const VerticalVelocityDeltaDiagnosticRow &row) { return row.start_time_s; });
    AppendRowsInSegment(
      assembled.vertical_motion_adaptive_reweighting_diagnostics,
      piece.result.vertical_motion_adaptive_reweighting_diagnostics,
      piece.segment,
      include_start,
      [](const VerticalMotionAdaptiveReweightingDiagnosticRow &row) {
        return row.start_time_s;
      });
    AppendRowsInSegment(
      assembled.vertical_position_velocity_consistency_diagnostics,
      piece.result.vertical_position_velocity_consistency_diagnostics,
      piece.segment,
      include_start,
      [](const VerticalPositionVelocityConsistencyDiagnosticRow &row) {
        return row.start_time_s;
      });
    AppendRowsInSegment(
      assembled.vertical_state_corrections,
      piece.result.vertical_state_corrections,
      piece.segment,
      include_start,
      [](const VerticalStateCorrectionRow &row) { return row.corrected_time_s; });

    AppendAll(assembled.body_z_bias_reestimate_segments, piece.result.body_z_bias_reestimate_segments);
    AppendAll(assembled.rtk_outage_attitude_hold_diagnostics, piece.result.rtk_outage_attitude_hold_diagnostics);
    AppendAll(assembled.rtk_outage_velocity_delta_3d_diagnostics, piece.result.rtk_outage_velocity_delta_3d_diagnostics);
  }

  assembled.run_summary.rtk_outage_segmented_batch_enabled = true;
  assembled.run_summary.rtk_outage_batch_segment_count =
    assembled.rtk_outage_batch_segments.size();
  assembled.run_summary.rtk_outage_segmented_batch_run_count =
    request_.pieces.size();
  assembled.run_summary.rtk_outage_segmented_batch_vertical_boundary_jump_allowed =
    request_.vertical_boundary_jump_allowed;
  assembled.run_summary.rtk_outage_window_count = request_.outage_windows.size();
  assembled.run_summary.processing_start_time_s = request_.processing_start_time_s;
  assembled.run_summary.processing_end_time_s = request_.processing_end_time_s;
  assembled.run_summary.state_count = assembled.trajectory.size();
  assembled.run_summary.gnss_factor_count = assembled.gnss_factor_records.size();
  assembled.run_summary.gnss_synced_factor_count = std::count_if(
    assembled.gnss_factor_records.begin(),
    assembled.gnss_factor_records.end(),
    [](const GnssFactorRecord &row) { return row.factor_used; });
  return assembled;
}

}  // namespace offline_lc_minimal
