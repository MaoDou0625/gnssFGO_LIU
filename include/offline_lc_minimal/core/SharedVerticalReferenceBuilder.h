#pragma once

#include <filesystem>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/io/TrajectoryCsvReader.h"

namespace offline_lc_minimal {

struct SharedVerticalReferenceMember {
  std::string member_id;
  OfflineRunnerConfig config;
  std::vector<TrajectoryCsvRow> trajectory;
  std::vector<GnssSolutionSample> gnss_samples;
};

struct SharedVerticalReferenceBuildRequest {
  std::vector<SharedVerticalReferenceMember> members;
  double grid_spacing_m = 1.0;
  double sigma_m = 0.015;
};

struct SharedReferenceLinePoint {
  double s_m = 0.0;
  double east_m = 0.0;
  double north_m = 0.0;
  double origin_lat_rad = std::numeric_limits<double>::quiet_NaN();
  double origin_lon_rad = std::numeric_limits<double>::quiet_NaN();
  double origin_h_m = std::numeric_limits<double>::quiet_NaN();
};

struct SharedReferenceProjection {
  bool valid = false;
  double s_m = 0.0;
  double lateral_offset_m = 0.0;
  std::size_t segment_index = 0;
};

struct SharedVerticalReferenceRow {
  double s_m = 0.0;
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_m = std::numeric_limits<double>::quiet_NaN();
  std::string source = "UNSET";
  double rtk_weight = 0.0;
  double nav_bridge_weight = 0.0;
  std::size_t sample_count = 0;
};

struct SharedVerticalReferenceProjectionDiagnosticRow {
  std::string member_id;
  std::string sample_kind;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double s_m = std::numeric_limits<double>::quiet_NaN();
  double lateral_offset_m = std::numeric_limits<double>::quiet_NaN();
  double raw_up_m = std::numeric_limits<double>::quiet_NaN();
  double corrected_up_m = std::numeric_limits<double>::quiet_NaN();
  bool used = false;
  std::string source = "UNSET";
};

struct SharedVerticalReference {
  std::vector<SharedVerticalReferenceRow> rows;
  std::vector<SharedReferenceLinePoint> reference_line;
  std::vector<SharedVerticalReferenceProjectionDiagnosticRow> projection_diagnostics;
  std::string reference_member_id;
};

[[nodiscard]] SharedReferenceProjection ProjectPointToSharedReferenceLine(
  const Eigen::Vector2d &point_m,
  const std::vector<SharedReferenceLinePoint> &reference_line);

[[nodiscard]] SharedVerticalReference BuildSharedVerticalReference(
  SharedVerticalReferenceBuildRequest request);

void WriteSharedVerticalReferenceCsv(
  const std::filesystem::path &path,
  const std::vector<SharedVerticalReferenceRow> &rows);

void WriteSharedReferenceLineCsv(
  const std::filesystem::path &path,
  const std::vector<SharedReferenceLinePoint> &rows);

void WriteSharedVerticalReferenceProjectionDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<SharedVerticalReferenceProjectionDiagnosticRow> &rows);

[[nodiscard]] std::vector<SharedVerticalReferenceRow> ReadSharedVerticalReferenceCsv(
  const std::filesystem::path &path);

[[nodiscard]] std::vector<SharedReferenceLinePoint> ReadSharedReferenceLineCsv(
  const std::filesystem::path &path);

}  // namespace offline_lc_minimal
