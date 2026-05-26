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
  const bool include_start,
  const bool include_end) {
  if (!std::isfinite(time_s)) {
    return false;
  }
  const bool after_start = include_start
    ? time_s >= segment.start_time_s - kTimeEpsilonS
    : time_s > segment.start_time_s + kTimeEpsilonS;
  const bool before_end = include_end
    ? time_s <= segment.end_time_s + kTimeEpsilonS
    : time_s < segment.end_time_s - kTimeEpsilonS;
  return after_start && before_end;
}

template <typename Row, typename TimeFn>
void AppendRowsInSegment(
  std::vector<Row> &target,
  const std::vector<Row> &source,
  const RtkOutageBatchSegmentRow &segment,
  const bool include_start,
  const bool include_end,
  TimeFn time_fn) {
  for (const auto &row : source) {
    if (InSegment(time_fn(row), segment, include_start, include_end)) {
      target.push_back(row);
    }
  }
}

template <typename Row>
void AppendAll(std::vector<Row> &target, const std::vector<Row> &source) {
  target.insert(target.end(), source.begin(), source.end());
}

template <typename Row>
void LabelSegment(Row &row, const RtkOutageBatchSegmentRow &segment) {
  row.batch_segment_index = static_cast<long long>(segment.segment_index);
  row.batch_segment_role = segment.segment_role;
}

template <typename Row>
void AppendAllWithSegmentLabel(
  std::vector<Row> &target,
  const std::vector<Row> &source,
  const RtkOutageBatchSegmentRow &segment) {
  for (auto row : source) {
    LabelSegment(row, segment);
    target.push_back(std::move(row));
  }
}

template <typename Row, typename TimeFn>
void AppendRowsInSegmentWithSegmentLabel(
  std::vector<Row> &target,
  const std::vector<Row> &source,
  const RtkOutageBatchSegmentRow &segment,
  const bool include_start,
  const bool include_end,
  TimeFn time_fn) {
  for (auto row : source) {
    if (InSegment(time_fn(row), segment, include_start, include_end)) {
      LabelSegment(row, segment);
      target.push_back(std::move(row));
    }
  }
}

const RtkOutageWindowRow *FindSourceOutage(
  const std::vector<RtkOutageWindowRow> &outage_windows,
  const RtkOutageBatchSegmentRow &segment) {
  if (segment.source_outage_window_index < 0) {
    return nullptr;
  }
  const auto it = std::find_if(
    outage_windows.begin(),
    outage_windows.end(),
    [&](const RtkOutageWindowRow &row) {
      return static_cast<long long>(row.window_index) ==
             segment.source_outage_window_index;
    });
  return it == outage_windows.end() ? nullptr : &(*it);
}

RtkOutageBatchSegmentRow EffectiveSpliceSegment(
  const RtkOutageBatchSegmentRow &segment,
  const std::vector<RtkOutageWindowRow> &outage_windows) {
  RtkOutageBatchSegmentRow effective_segment = segment;
  const RtkOutageWindowRow *source_outage =
    FindSourceOutage(outage_windows, segment);
  if (source_outage == nullptr) {
    return effective_segment;
  }
  if (segment.segment_role == "POST_RTK_VALID") {
    effective_segment.start_time_s = source_outage->end_time_s;
  } else if (segment.segment_role == "RTK_OUTAGE") {
    effective_segment.end_time_s = source_outage->end_time_s;
  }
  return effective_segment;
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

bool IncludeSegmentStart(
  const RtkOutageBatchSegmentRow &segment,
  const std::size_t piece_index) {
  if (segment.segment_role == "POST_RTK_VALID") {
    return true;
  }
  if (segment.segment_role == "RTK_OUTAGE") {
    return false;
  }
  return piece_index == 0U;
}

bool IncludeSegmentEnd(const RtkOutageBatchSegmentRow &segment) {
  return segment.segment_role != "RTK_OUTAGE";
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
  assembled.late_static_feature_diagnostics.clear();
  assembled.late_static_threshold_diagnostics.clear();
  assembled.late_static_windows.clear();
  assembled.initial_dynamic_static_feature_diagnostics.clear();
  assembled.initial_dynamic_static_threshold_diagnostics.clear();
  assembled.initial_dynamic_static_windows.clear();
  assembled.rtk_outage_causal_nav_reference_diagnostics.clear();
  assembled.rtk_outage_recovery_references.clear();
  assembled.rtk_outage_bias_continuity_policy.clear();
  assembled.rtk_outage_boundary_diagnostics.clear();
  assembled.rtk_outage_attitude_hold_diagnostics.clear();
  assembled.rtk_outage_velocity_delta_3d_diagnostics.clear();
  assembled.rtk_velocity_diagnostics.clear();
  assembled.vertical_velocity_delta_diagnostics.clear();
  assembled.vertical_motion_adaptive_reweighting_diagnostics.clear();
  assembled.vertical_position_velocity_consistency_diagnostics.clear();
  assembled.vertical_state_corrections.clear();
  assembled.body_z_bias_reestimate_segments.clear();
  assembled.stage2_mount_leakage_diagnostics.clear();
  assembled.stage2_vehicle_nhc_state_diagnostics.clear();
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
    true,
    [](const ReferenceNodeRow &row) { return row.time_s; });

  for (std::size_t piece_index = 0; piece_index < request_.pieces.size(); ++piece_index) {
    const auto &piece = request_.pieces[piece_index];
    const RtkOutageBatchSegmentRow splice_segment =
      EffectiveSpliceSegment(piece.segment, request_.outage_windows);
    const bool include_start = IncludeSegmentStart(piece.segment, piece_index);
    const bool include_end = IncludeSegmentEnd(piece.segment);
    AppendRowsInSegment(
      assembled.trajectory,
      piece.result.trajectory,
      splice_segment,
      include_start,
      include_end,
      [](const TrajectoryRow &row) { return row.time_s; });
    AppendRowsInSegment(
      assembled.reference_node_trajectory,
      piece.result.reference_node_trajectory,
      splice_segment,
      include_start,
      include_end,
      [](const ReferenceNodeRow &row) { return row.time_s; });
    AppendRowsInSegment(
      assembled.gnss_factor_records,
      piece.result.gnss_factor_records,
      splice_segment,
      include_start,
      include_end,
      [](const GnssFactorRecord &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.gnss_consistency_records,
      piece.result.gnss_consistency_records,
      splice_segment,
      include_start,
      include_end,
      [](const GnssConsistencyRecord &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.vertical_envelope_diagnostics,
      piece.result.vertical_envelope_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const VerticalEnvelopeDiagnosticRow &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.rtk_vertical_drift_reference_diagnostics,
      piece.result.rtk_vertical_drift_reference_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const RtkVerticalDriftReferenceDiagnosticRow &row) { return row.time_s; });
    if (piece.segment.segment_role != "RTK_OUTAGE") {
      AppendRowsInSegment(
        assembled.late_static_feature_diagnostics,
        piece.result.late_static_feature_diagnostics,
        splice_segment,
        include_start,
        include_end,
        [](const LateStaticFeatureDiagnosticRow &row) { return row.window_center_time_s; });
      AppendAll(assembled.late_static_threshold_diagnostics, piece.result.late_static_threshold_diagnostics);
      AppendAll(assembled.late_static_windows, piece.result.late_static_windows);
      if (piece.segment.segment_role == "PRE_RTK_VALID") {
        AppendRowsInSegment(
          assembled.initial_dynamic_static_feature_diagnostics,
          piece.result.initial_dynamic_static_feature_diagnostics,
          splice_segment,
          include_start,
          include_end,
          [](const LateStaticFeatureDiagnosticRow &row) { return row.window_center_time_s; });
        AppendAll(
          assembled.initial_dynamic_static_threshold_diagnostics,
          piece.result.initial_dynamic_static_threshold_diagnostics);
        AppendAll(
          assembled.initial_dynamic_static_windows,
          piece.result.initial_dynamic_static_windows);
      }
    }
    AppendRowsInSegment(
      assembled.rtk_outage_causal_nav_reference_diagnostics,
      piece.result.rtk_outage_causal_nav_reference_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const RtkOutageCausalNavReferenceRow &row) { return row.time_s; });
    AppendRowsInSegment(
      assembled.rtk_velocity_diagnostics,
      piece.result.rtk_velocity_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const RtkVelocityDiagnosticRow &row) { return row.corrected_time_s; });
    AppendRowsInSegment(
      assembled.vertical_velocity_delta_diagnostics,
      piece.result.vertical_velocity_delta_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const VerticalVelocityDeltaDiagnosticRow &row) { return row.start_time_s; });
    AppendRowsInSegment(
      assembled.vertical_motion_adaptive_reweighting_diagnostics,
      piece.result.vertical_motion_adaptive_reweighting_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const VerticalMotionAdaptiveReweightingDiagnosticRow &row) {
        return row.start_time_s;
      });
    AppendRowsInSegment(
      assembled.vertical_position_velocity_consistency_diagnostics,
      piece.result.vertical_position_velocity_consistency_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const VerticalPositionVelocityConsistencyDiagnosticRow &row) {
        return row.start_time_s;
      });
    AppendRowsInSegment(
      assembled.vertical_state_corrections,
      piece.result.vertical_state_corrections,
      splice_segment,
      include_start,
      include_end,
      [](const VerticalStateCorrectionRow &row) { return row.corrected_time_s; });
    AppendRowsInSegmentWithSegmentLabel(
      assembled.stage2_vehicle_nhc_state_diagnostics,
      piece.result.stage2_vehicle_nhc_state_diagnostics,
      splice_segment,
      include_start,
      include_end,
      [](const Stage2VehicleNHCStateDiagnosticRow &row) { return row.time_s; });

    AppendAll(assembled.body_z_bias_reestimate_segments, piece.result.body_z_bias_reestimate_segments);
    AppendAllWithSegmentLabel(
      assembled.stage2_mount_leakage_diagnostics,
      piece.result.stage2_mount_leakage_diagnostics,
      piece.segment);
    AppendAll(assembled.rtk_outage_boundary_diagnostics, piece.result.rtk_outage_boundary_diagnostics);
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
  assembled.run_summary.rtk_outage_boundary_constraints_enabled =
    std::any_of(
      request_.pieces.begin(),
      request_.pieces.end(),
      [](const SegmentedBatchResultPiece &piece) {
        return piece.result.run_summary.rtk_outage_boundary_constraints_enabled;
      });
  assembled.run_summary.rtk_outage_boundary_reference_count = 0;
  assembled.run_summary.rtk_outage_boundary_up_factor_count = 0;
  assembled.run_summary.rtk_outage_boundary_vz_factor_count = 0;
  assembled.run_summary.rtk_outage_boundary_baz_factor_count = 0;
  assembled.run_summary.late_static_detection_enabled =
    std::any_of(
      request_.pieces.begin(),
      request_.pieces.end(),
      [](const SegmentedBatchResultPiece &piece) {
        return piece.result.run_summary.late_static_detection_enabled;
      });
  assembled.run_summary.late_static_feature_window_count =
    assembled.late_static_feature_diagnostics.size();
  assembled.run_summary.late_static_valid_feature_window_count =
    static_cast<std::size_t>(
      std::count_if(
        assembled.late_static_feature_diagnostics.begin(),
        assembled.late_static_feature_diagnostics.end(),
        [](const LateStaticFeatureDiagnosticRow &row) { return row.valid_features; }));
  assembled.run_summary.late_static_window_count = 0;
  assembled.run_summary.late_static_vz_factor_count = 0;
  assembled.run_summary.late_static_up_factor_count = 0;
  assembled.run_summary.late_static_height_hold_factor_count = 0;
  assembled.run_summary.initial_dynamic_static_detection_enabled =
    std::any_of(
      request_.pieces.begin(),
      request_.pieces.end(),
      [](const SegmentedBatchResultPiece &piece) {
        return piece.result.run_summary.initial_dynamic_static_detection_enabled;
      });
  assembled.run_summary.initial_dynamic_static_lowpass_protection_enabled =
    std::any_of(
      request_.pieces.begin(),
      request_.pieces.end(),
      [](const SegmentedBatchResultPiece &piece) {
        return piece.result.run_summary.initial_dynamic_static_lowpass_protection_enabled;
      });
  assembled.run_summary.initial_dynamic_static_vz_constraint_enabled =
    std::any_of(
      request_.pieces.begin(),
      request_.pieces.end(),
      [](const SegmentedBatchResultPiece &piece) {
        return piece.result.run_summary.initial_dynamic_static_vz_constraint_enabled;
      });
  assembled.run_summary.initial_dynamic_static_feature_window_count =
    assembled.initial_dynamic_static_feature_diagnostics.size();
  assembled.run_summary.initial_dynamic_static_valid_feature_window_count =
    static_cast<std::size_t>(
      std::count_if(
        assembled.initial_dynamic_static_feature_diagnostics.begin(),
        assembled.initial_dynamic_static_feature_diagnostics.end(),
        [](const LateStaticFeatureDiagnosticRow &row) { return row.valid_features; }));
  assembled.run_summary.initial_dynamic_static_window_count = 0;
  assembled.run_summary.initial_dynamic_static_vz_factor_count = 0;
  assembled.run_summary.stage2_velocity_optimization_enabled =
    std::any_of(
      request_.pieces.begin(),
      request_.pieces.end(),
      [](const SegmentedBatchResultPiece &piece) {
        return piece.result.run_summary.stage2_velocity_optimization_enabled;
      });
  assembled.run_summary.stage2_attitude_hold_factor_count = 0;
  assembled.run_summary.stage2_horizontal_position_hold_factor_count = 0;
  assembled.run_summary.stage2_horizontal_velocity_hold_factor_count = 0;
  assembled.run_summary.stage2_vehicle_y_nhc_velocity_factor_count = 0;
  assembled.run_summary.stage2_vehicle_y_nhc_displacement_factor_count = 0;
  assembled.run_summary.stage2_vehicle_z_nhc_velocity_factor_count = 0;
  assembled.run_summary.stage2_vehicle_z_nhc_displacement_factor_count = 0;
  assembled.run_summary.stage2_vehicle_nhc_window_count = 0;
  assembled.run_summary.stage2_vehicle_nhc_skipped_short_window_count = 0;
  assembled.run_summary.stage2_vehicle_nhc_skipped_invalid_count = 0;
  assembled.run_summary.stage2_vehicle_nhc_unique_velocity_factor_count = 0;
  assembled.run_summary.stage2_vehicle_nhc_velocity_duplicate_state_count = 0;
  assembled.run_summary.stage2_vehicle_nhc_interval_overlap_count = 0;
  for (const auto &piece : request_.pieces) {
    assembled.run_summary.rtk_outage_boundary_reference_count +=
      piece.result.run_summary.rtk_outage_boundary_reference_count;
    assembled.run_summary.rtk_outage_boundary_up_factor_count +=
      piece.result.run_summary.rtk_outage_boundary_up_factor_count;
    assembled.run_summary.rtk_outage_boundary_vz_factor_count +=
      piece.result.run_summary.rtk_outage_boundary_vz_factor_count;
    assembled.run_summary.rtk_outage_boundary_baz_factor_count +=
      piece.result.run_summary.rtk_outage_boundary_baz_factor_count;
    if (piece.segment.segment_role != "RTK_OUTAGE") {
      assembled.run_summary.late_static_window_count +=
        piece.result.run_summary.late_static_window_count;
      assembled.run_summary.late_static_vz_factor_count +=
        piece.result.run_summary.late_static_vz_factor_count;
      assembled.run_summary.late_static_up_factor_count +=
        piece.result.run_summary.late_static_up_factor_count;
      assembled.run_summary.late_static_height_hold_factor_count +=
        piece.result.run_summary.late_static_height_hold_factor_count;
      if (!std::isfinite(assembled.run_summary.late_static_rtk_speed_threshold_mps) &&
          std::isfinite(piece.result.run_summary.late_static_rtk_speed_threshold_mps)) {
        assembled.run_summary.late_static_rtk_speed_threshold_mps =
          piece.result.run_summary.late_static_rtk_speed_threshold_mps;
      }
      if (!std::isfinite(assembled.run_summary.late_static_gyro_rms_threshold_radps) &&
          std::isfinite(piece.result.run_summary.late_static_gyro_rms_threshold_radps)) {
        assembled.run_summary.late_static_gyro_rms_threshold_radps =
          piece.result.run_summary.late_static_gyro_rms_threshold_radps;
      }
      if (piece.segment.segment_role == "PRE_RTK_VALID") {
        assembled.run_summary.initial_dynamic_static_window_count +=
          piece.result.run_summary.initial_dynamic_static_window_count;
        assembled.run_summary.initial_dynamic_static_vz_factor_count +=
          piece.result.run_summary.initial_dynamic_static_vz_factor_count;
        if (!std::isfinite(assembled.run_summary.initial_dynamic_static_rtk_speed_threshold_mps) &&
            std::isfinite(piece.result.run_summary.initial_dynamic_static_rtk_speed_threshold_mps)) {
          assembled.run_summary.initial_dynamic_static_rtk_speed_threshold_mps =
            piece.result.run_summary.initial_dynamic_static_rtk_speed_threshold_mps;
        }
        if (!std::isfinite(assembled.run_summary.initial_dynamic_static_gyro_rms_threshold_radps) &&
            std::isfinite(piece.result.run_summary.initial_dynamic_static_gyro_rms_threshold_radps)) {
          assembled.run_summary.initial_dynamic_static_gyro_rms_threshold_radps =
            piece.result.run_summary.initial_dynamic_static_gyro_rms_threshold_radps;
        }
        if (!std::isfinite(assembled.run_summary.initial_dynamic_static_acc_std_threshold_mps2) &&
            std::isfinite(piece.result.run_summary.initial_dynamic_static_acc_std_threshold_mps2)) {
          assembled.run_summary.initial_dynamic_static_acc_std_threshold_mps2 =
            piece.result.run_summary.initial_dynamic_static_acc_std_threshold_mps2;
        }
      }
    }
    assembled.run_summary.stage2_attitude_hold_factor_count +=
      piece.result.run_summary.stage2_attitude_hold_factor_count;
    assembled.run_summary.stage2_horizontal_position_hold_factor_count +=
      piece.result.run_summary.stage2_horizontal_position_hold_factor_count;
    assembled.run_summary.stage2_horizontal_velocity_hold_factor_count +=
      piece.result.run_summary.stage2_horizontal_velocity_hold_factor_count;
    assembled.run_summary.stage2_vehicle_y_nhc_velocity_factor_count +=
      piece.result.run_summary.stage2_vehicle_y_nhc_velocity_factor_count;
    assembled.run_summary.stage2_vehicle_y_nhc_displacement_factor_count +=
      piece.result.run_summary.stage2_vehicle_y_nhc_displacement_factor_count;
    assembled.run_summary.stage2_vehicle_z_nhc_velocity_factor_count +=
      piece.result.run_summary.stage2_vehicle_z_nhc_velocity_factor_count;
    assembled.run_summary.stage2_vehicle_z_nhc_displacement_factor_count +=
      piece.result.run_summary.stage2_vehicle_z_nhc_displacement_factor_count;
    assembled.run_summary.stage2_vehicle_nhc_window_count +=
      piece.result.run_summary.stage2_vehicle_nhc_window_count;
    assembled.run_summary.stage2_vehicle_nhc_skipped_short_window_count +=
      piece.result.run_summary.stage2_vehicle_nhc_skipped_short_window_count;
    assembled.run_summary.stage2_vehicle_nhc_skipped_invalid_count +=
      piece.result.run_summary.stage2_vehicle_nhc_skipped_invalid_count;
    assembled.run_summary.stage2_vehicle_nhc_unique_velocity_factor_count +=
      piece.result.run_summary.stage2_vehicle_nhc_unique_velocity_factor_count;
    assembled.run_summary.stage2_vehicle_nhc_velocity_duplicate_state_count +=
      piece.result.run_summary.stage2_vehicle_nhc_velocity_duplicate_state_count;
    assembled.run_summary.stage2_vehicle_nhc_interval_overlap_count +=
      piece.result.run_summary.stage2_vehicle_nhc_interval_overlap_count;
  }
  for (const auto &row : assembled.late_static_threshold_diagnostics) {
    if (!row.valid || !std::isfinite(row.threshold_value)) {
      continue;
    }
    if (row.feature_name == "rtk_horizontal_speed_rms_mps" &&
        !std::isfinite(assembled.run_summary.late_static_rtk_speed_threshold_mps)) {
      assembled.run_summary.late_static_rtk_speed_threshold_mps = row.threshold_value;
    } else if (row.feature_name == "imu_gyro_norm_rms_radps" &&
               !std::isfinite(assembled.run_summary.late_static_gyro_rms_threshold_radps)) {
      assembled.run_summary.late_static_gyro_rms_threshold_radps = row.threshold_value;
    }
  }
  assembled.run_summary.processing_start_time_s = request_.processing_start_time_s;
  assembled.run_summary.processing_end_time_s = request_.processing_end_time_s;
  assembled.run_summary.state_count = assembled.trajectory.size();
  assembled.run_summary.gnss_factor_count = assembled.gnss_factor_records.size();
  assembled.run_summary.gnss_synced_factor_count = std::count_if(
    assembled.gnss_factor_records.begin(),
    assembled.gnss_factor_records.end(),
    [](const GnssFactorRecord &row) { return row.factor_used; });
  assembled.run_summary.gnss_vertical_reference_source =
    request_.pieces.back().result.run_summary.gnss_vertical_reference_source;
  assembled.run_summary.gnss_vertical_reference_selected_count =
    static_cast<std::size_t>(
      std::count_if(
        assembled.gnss_factor_records.begin(),
        assembled.gnss_factor_records.end(),
        [](const GnssFactorRecord &row) {
          return row.vertical_reference_skip_reason == "OK";
        }));
  assembled.run_summary.gnss_vertical_reference_skipped_count =
    static_cast<std::size_t>(
      std::count_if(
        assembled.gnss_factor_records.begin(),
        assembled.gnss_factor_records.end(),
        [](const GnssFactorRecord &row) {
          return row.vertical_reference_skip_reason != "OK" &&
                 row.vertical_reference_skip_reason != "UNSET";
        }));
  return assembled;
}

}  // namespace offline_lc_minimal
