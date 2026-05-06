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

void WriteVerticalVelocityDeltaDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<VerticalVelocityDeltaDiagnosticRow> &rows);

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
