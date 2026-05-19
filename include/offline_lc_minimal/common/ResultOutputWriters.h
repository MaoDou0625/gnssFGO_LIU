#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

void WriteTextFile(const std::filesystem::path &path, const std::string &content);

void WriteTrajectoryCsv(
  const std::filesystem::path &path,
  const std::vector<TrajectoryRow> &rows,
  const GeoReference &geo_reference);

void WriteReferenceNodeCsv(
  const std::filesystem::path &path,
  const std::vector<ReferenceNodeRow> &rows,
  const GeoReference &geo_reference);

void WriteSeedBodyZAccDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZSeedImuDiagnosticRow> &rows);

void WriteBodyZSeedJumpWindowCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZSeedJumpWindowRow> &rows);

void WriteBodyZBiasReestimateSegmentsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZBiasReestimateSegmentRow> &rows);

void WriteErrorStateCsv(const std::filesystem::path &path, const std::vector<ErrorStateRow> &rows);

void WriteSegmentErrorCsv(
  const std::filesystem::path &path,
  const std::vector<SegmentErrorDiagnostic> &rows);

[[nodiscard]] std::string BuildSegmentErrorSummaryText(
  const std::vector<SegmentErrorDiagnostic> &rows);

void WriteGnssConsistencyCsv(
  const std::filesystem::path &path,
  const std::vector<GnssConsistencyRecord> &records);

void WriteVerticalEnvelopeDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalEnvelopeDiagnosticRow> &rows);

void WriteRtkVerticalDriftReferenceDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> &rows);

void WriteRtkOutageCausalNavReferenceCsv(
  const std::filesystem::path &path,
  const std::vector<RtkOutageCausalNavReferenceRow> &rows);

void WriteRtkOutageWindowsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkOutageWindowRow> &rows);

void WriteRtkOutageBatchSegmentsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkOutageBatchSegmentRow> &rows);

void WriteRtkOutageAttitudeHoldDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkOutageAttitudeHoldDiagnosticRow> &rows);

void WriteRtkOutageVelocityDelta3dDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkOutageVelocityDelta3dDiagnosticRow> &rows);

void WriteRtkVelocityDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RtkVelocityDiagnosticRow> &rows);

void WriteStage1YawRefinementDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<Stage1YawRefinementDiagnosticRow> &rows);

void WriteStaticAlignmentValidationCsv(
  const std::filesystem::path &path,
  const std::vector<StaticAlignmentValidationRow> &rows);

void WriteVerticalVelocityDeltaDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalVelocityDeltaDiagnosticRow> &rows);

void WriteVerticalMotionAdaptiveReweightingDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalMotionAdaptiveReweightingDiagnosticRow> &rows);

void WriteVerticalPositionVelocityConsistencyDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalPositionVelocityConsistencyDiagnosticRow> &rows);

void WriteAttitudeReferenceDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<AttitudeReferenceDiagnosticRow> &rows);

void WriteRelativeYawReferenceDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<RelativeYawReferenceDiagnosticRow> &rows);

void WriteBodyZHorizontalLeakageDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZHorizontalLeakageDiagnosticRow> &rows);

void WriteBodyZNHCDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZNHCDiagnosticRow> &rows);

void WriteBodyZNHCStateDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<BodyZNHCStateDiagnosticRow> &rows);

void WriteStage2MountLeakageDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<Stage2MountLeakageDiagnosticRow> &rows);

void WriteStage2VehicleNHCStateDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<Stage2VehicleNHCStateDiagnosticRow> &rows);

void WriteVerticalJumpMaskedImuDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpMaskedImuDiagnosticRow> &rows);

void WriteVerticalJumpImpulseDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpImpulseDiagnosticRow> &rows);

void WriteVerticalJumpBiasDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpBiasDiagnosticRow> &rows);

void WriteVerticalJumpVelocityRampDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpVelocityRampDiagnosticRow> &rows);

void WriteVerticalJumpContinuityDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalJumpContinuityDiagnosticRow> &rows);

void WriteVerticalStateCorrectionCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalStateCorrectionRow> &rows);

void WriteInitialDynamicConsistencyCsv(
  const std::filesystem::path &path,
  const std::vector<TrajectoryRow> &rows,
  const RunSummary &summary);

[[nodiscard]] std::vector<TrajectoryRow> FilterDynamicTrajectoryRows(
  const std::vector<TrajectoryRow> &rows,
  double dynamic_start_time_s);

}  // namespace offline_lc_minimal
