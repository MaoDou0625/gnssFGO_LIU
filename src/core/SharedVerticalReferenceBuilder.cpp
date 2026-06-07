#include "offline_lc_minimal/core/SharedVerticalReferenceBuilder.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <utility>

#include "offline_lc_minimal/common/GeoUtils.h"

namespace offline_lc_minimal {
namespace {

constexpr double kDistanceEpsilonM = 1.0e-6;
constexpr double kTimeEpsilonS = 1.0e-9;

struct CommonTrajectoryPoint {
  double time_s = 0.0;
  Eigen::Vector2d xy_m = Eigen::Vector2d::Zero();
  double up_m = std::numeric_limits<double>::quiet_NaN();
};

struct CommonMember {
  std::string member_id;
  OfflineRunnerConfig config;
  std::vector<CommonTrajectoryPoint> trajectory;
  std::vector<GnssSolutionSample> gnss_samples;
  bool direction_flipped = false;
};

std::string Trim(std::string value) {
  auto not_space = [](const unsigned char c) { return !std::isspace(c); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
}

std::vector<std::string> SplitCsvLine(const std::string &line) {
  std::vector<std::string> fields;
  std::string current;
  bool in_quotes = false;
  for (std::size_t index = 0; index < line.size(); ++index) {
    const char ch = line[index];
    if (ch == '"') {
      if (in_quotes && index + 1U < line.size() && line[index + 1U] == '"') {
        current.push_back('"');
        ++index;
      } else {
        in_quotes = !in_quotes;
      }
      continue;
    }
    if (ch == ',' && !in_quotes) {
      fields.push_back(Trim(std::move(current)));
      current.clear();
      continue;
    }
    current.push_back(ch);
  }
  fields.push_back(Trim(std::move(current)));
  return fields;
}

std::unordered_map<std::string, std::size_t> HeaderIndex(
  const std::vector<std::string> &header) {
  std::unordered_map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size(); ++index) {
    columns.emplace(header[index], index);
  }
  return columns;
}

std::size_t RequiredColumn(
  const std::unordered_map<std::string, std::size_t> &columns,
  const std::string &name) {
  const auto it = columns.find(name);
  if (it == columns.end()) {
    throw std::runtime_error("CSV missing required column: " + name);
  }
  return it->second;
}

double ParseDouble(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name) {
  if (column >= fields.size()) {
    throw std::runtime_error("CSV row missing field: " + name);
  }
  try {
    std::size_t consumed = 0U;
    const double value = std::stod(fields[column], &consumed);
    if (consumed != fields[column].size()) {
      throw std::runtime_error("trailing characters");
    }
    return value;
  } catch (const std::exception &) {
    throw std::runtime_error("CSV row has invalid numeric field: " + name);
  }
}

std::size_t ParseSize(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name) {
  const double value = ParseDouble(fields, column, name);
  if (!std::isfinite(value) || value < 0.0) {
    throw std::runtime_error("CSV row has invalid non-negative count: " + name);
  }
  return static_cast<std::size_t>(std::llround(value));
}

void WriteCsvString(std::ostream &stream, const std::string &value) {
  stream << '"';
  for (const char ch : value) {
    if (ch == '"') {
      stream << "\"\"";
    } else {
      stream << ch;
    }
  }
  stream << '"';
}

double Median(std::vector<double> values) {
  values.erase(
    std::remove_if(
      values.begin(),
      values.end(),
      [](const double value) { return !std::isfinite(value); }),
    values.end());
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

GeoReference MakeCommonGeoReference(
  const std::vector<SharedVerticalReferenceMember> &members) {
  for (const auto &member : members) {
    for (const auto &row : member.trajectory) {
      if (row.has_geodetic) {
        return GeoReference(row.lat_rad, row.lon_rad, row.h_m);
      }
    }
  }
  throw std::runtime_error("shared vertical reference requires geodetic trajectory columns");
}

Eigen::Vector2d TrackVector(const std::vector<CommonTrajectoryPoint> &points) {
  if (points.size() < 2U) {
    return Eigen::Vector2d::Zero();
  }
  const Eigen::Vector2d delta = points.back().xy_m - points.front().xy_m;
  const double norm = delta.norm();
  if (norm <= 0.0) {
    return Eigen::Vector2d::Zero();
  }
  return delta / norm;
}

std::vector<CommonMember> BuildCommonMembers(
  const SharedVerticalReferenceBuildRequest &request,
  const GeoReference &geo_reference) {
  std::vector<CommonMember> members;
  members.reserve(request.members.size());
  for (const auto &input_member : request.members) {
    CommonMember member;
    member.member_id = input_member.member_id;
    member.config = input_member.config;
    member.gnss_samples = input_member.gnss_samples;
    member.trajectory.reserve(input_member.trajectory.size());
    for (const auto &row : input_member.trajectory) {
      if (!row.has_geodetic) {
        throw std::runtime_error(
          "member " + input_member.member_id +
          " trajectory missing lat_rad/lon_rad/h_m columns");
      }
      const Eigen::Vector3d enu =
        geo_reference.Forward(row.lat_rad, row.lon_rad, row.h_m);
      CommonTrajectoryPoint point;
      point.time_s = row.trajectory.time_s;
      point.xy_m = enu.head<2>();
      point.up_m = row.h_m;
      member.trajectory.push_back(point);
    }
    if (member.trajectory.size() < 2U) {
      throw std::runtime_error("shared vertical reference member requires at least two trajectory rows");
    }
    members.push_back(std::move(member));
  }

  const Eigen::Vector2d first_direction = TrackVector(members.front().trajectory);
  for (auto &member : members) {
    const double dot = TrackVector(member.trajectory).dot(first_direction);
    if (dot < 0.0) {
      std::reverse(member.trajectory.begin(), member.trajectory.end());
      member.direction_flipped = true;
    }
  }
  return members;
}

double TrajectoryLength(const std::vector<CommonTrajectoryPoint> &points) {
  double length_m = 0.0;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    length_m += (points[index].xy_m - points[index - 1U].xy_m).norm();
  }
  return length_m;
}

std::size_t LongestMemberIndex(const std::vector<CommonMember> &members) {
  std::size_t best_index = 0U;
  double best_length_m = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < members.size(); ++index) {
    const double length_m = TrajectoryLength(members[index].trajectory);
    if (length_m > best_length_m) {
      best_length_m = length_m;
      best_index = index;
    }
  }
  return best_index;
}

std::vector<SharedReferenceLinePoint> BuildReferenceLine(
  const std::vector<CommonTrajectoryPoint> &points,
  const GeoReference &geo_reference) {
  std::vector<SharedReferenceLinePoint> line;
  line.reserve(points.size());
  SharedReferenceLinePoint first;
  first.s_m = 0.0;
  first.east_m = points.front().xy_m.x();
  first.north_m = points.front().xy_m.y();
  first.origin_lat_rad = geo_reference.origin_lat_rad();
  first.origin_lon_rad = geo_reference.origin_lon_rad();
  first.origin_h_m = geo_reference.origin_h_m();
  line.push_back(first);

  double s_m = 0.0;
  Eigen::Vector2d previous = points.front().xy_m;
  for (std::size_t index = 1U; index < points.size(); ++index) {
    const double ds_m = (points[index].xy_m - previous).norm();
    if (ds_m <= kDistanceEpsilonM) {
      continue;
    }
    s_m += ds_m;
    SharedReferenceLinePoint row;
    row.s_m = s_m;
    row.east_m = points[index].xy_m.x();
    row.north_m = points[index].xy_m.y();
    row.origin_lat_rad = geo_reference.origin_lat_rad();
    row.origin_lon_rad = geo_reference.origin_lon_rad();
    row.origin_h_m = geo_reference.origin_h_m();
    line.push_back(row);
    previous = points[index].xy_m;
  }
  if (line.size() < 2U) {
    throw std::runtime_error("shared reference line requires at least two unique points");
  }
  return line;
}

std::size_t GridIndex(const double s_m, const double grid_spacing_m) {
  return static_cast<std::size_t>(std::llround(s_m / grid_spacing_m));
}

bool TimeWithinProcessingRange(const OfflineRunnerConfig &config, const double corrected_time_s) {
  if (config.processing_start_time_s > 0.0 &&
      corrected_time_s + kTimeEpsilonS < config.processing_start_time_s) {
    return false;
  }
  if (config.processing_end_time_s > 0.0 &&
      corrected_time_s > config.processing_end_time_s + kTimeEpsilonS) {
    return false;
  }
  return true;
}

bool IsUsableRtkFixForSharedReference(
  const OfflineRunnerConfig &config,
  const GnssSolutionSample &sample) {
  if (!sample.has_valid_position() || sample.fix_type() != GnssFixType::kRtkFix) {
    return false;
  }
  if (config.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config.required_best_sol_status_code) {
    return false;
  }
  if (config.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return TimeWithinProcessingRange(config, sample.time_s - config.gnss_time_offset_s);
}

void FillMissingRowsByInterpolation(std::vector<SharedVerticalReferenceRow> &rows) {
  std::vector<std::size_t> finite_indices;
  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (std::isfinite(rows[index].reference_up_m)) {
      finite_indices.push_back(index);
    }
  }
  if (finite_indices.empty()) {
    throw std::runtime_error("shared vertical reference has no finite observations");
  }
  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (std::isfinite(rows[index].reference_up_m)) {
      continue;
    }
    const auto right_it =
      std::lower_bound(finite_indices.begin(), finite_indices.end(), index);
    if (right_it == finite_indices.begin()) {
      rows[index].reference_up_m = rows[*right_it].reference_up_m;
    } else if (right_it == finite_indices.end()) {
      rows[index].reference_up_m = rows[finite_indices.back()].reference_up_m;
    } else {
      const std::size_t right = *right_it;
      const std::size_t left = *(right_it - 1);
      const double alpha =
        (rows[index].s_m - rows[left].s_m) /
        std::max(rows[right].s_m - rows[left].s_m, kDistanceEpsilonM);
      rows[index].reference_up_m =
        (1.0 - alpha) * rows[left].reference_up_m +
        alpha * rows[right].reference_up_m;
    }
    rows[index].source = "INTERPOLATED";
    rows[index].sample_count = 0U;
  }
}

}  // namespace

SharedReferenceProjection ProjectPointToSharedReferenceLine(
  const Eigen::Vector2d &point_m,
  const std::vector<SharedReferenceLinePoint> &reference_line) {
  SharedReferenceProjection result;
  if (reference_line.size() < 2U) {
    return result;
  }

  double best_distance2_m2 = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index + 1U < reference_line.size(); ++index) {
    const Eigen::Vector2d a(reference_line[index].east_m, reference_line[index].north_m);
    const Eigen::Vector2d b(reference_line[index + 1U].east_m, reference_line[index + 1U].north_m);
    const Eigen::Vector2d segment = b - a;
    const double segment_len2 = segment.squaredNorm();
    if (segment_len2 <= 0.0) {
      continue;
    }
    const double t =
      std::clamp((point_m - a).dot(segment) / segment_len2, 0.0, 1.0);
    const Eigen::Vector2d projection = a + t * segment;
    const double distance2_m2 = (point_m - projection).squaredNorm();
    if (distance2_m2 < best_distance2_m2) {
      best_distance2_m2 = distance2_m2;
      result.valid = true;
      result.segment_index = index;
      const double segment_len = std::sqrt(segment_len2);
      result.s_m = reference_line[index].s_m + t * segment_len;
      const double cross =
        segment.x() * (point_m.y() - a.y()) -
        segment.y() * (point_m.x() - a.x());
      result.lateral_offset_m =
        (cross < 0.0 ? -1.0 : 1.0) * std::sqrt(distance2_m2);
    }
  }
  return result;
}

SharedVerticalReference BuildSharedVerticalReference(
  SharedVerticalReferenceBuildRequest request) {
  if (request.members.size() < 2U) {
    throw std::runtime_error("shared vertical reference requires at least two members");
  }
  if (!std::isfinite(request.grid_spacing_m) || request.grid_spacing_m <= 0.0 ||
      !std::isfinite(request.sigma_m) || request.sigma_m <= 0.0) {
    throw std::runtime_error("shared vertical reference grid spacing and sigma must be positive");
  }

  const GeoReference geo_reference = MakeCommonGeoReference(request.members);
  const std::vector<CommonMember> members = BuildCommonMembers(request, geo_reference);
  const std::size_t reference_index = LongestMemberIndex(members);
  SharedVerticalReference result;
  result.reference_member_id = members[reference_index].member_id;
  result.reference_line = BuildReferenceLine(members[reference_index].trajectory, geo_reference);
  const double max_s_m = result.reference_line.back().s_m;
  const std::size_t grid_count =
    static_cast<std::size_t>(std::floor(max_s_m / request.grid_spacing_m)) + 1U;
  std::vector<std::vector<double>> rtk_up_by_bin(grid_count);
  std::vector<std::vector<double>> nav_up_by_bin(grid_count);

  for (const auto &member : members) {
    for (const auto &sample : member.gnss_samples) {
      SharedVerticalReferenceProjectionDiagnosticRow diagnostic;
      diagnostic.member_id = member.member_id;
      diagnostic.sample_kind = "RTKFIX";
      diagnostic.time_s = sample.time_s - member.config.gnss_time_offset_s;
      const Eigen::Vector3d enu =
        geo_reference.Forward(sample.lat_rad, sample.lon_rad, sample.h_m);
      diagnostic.raw_up_m = sample.h_m;
      if (!IsUsableRtkFixForSharedReference(member.config, sample)) {
        diagnostic.used = false;
        diagnostic.source = "RTK_REJECTED";
        result.projection_diagnostics.push_back(diagnostic);
        continue;
      }
      const SharedReferenceProjection projection =
        ProjectPointToSharedReferenceLine(enu.head<2>(), result.reference_line);
      diagnostic.s_m = projection.s_m;
      diagnostic.lateral_offset_m = projection.lateral_offset_m;
      if (!projection.valid || projection.s_m < -0.5 * request.grid_spacing_m ||
          projection.s_m > max_s_m + 0.5 * request.grid_spacing_m) {
        diagnostic.source = "OUTSIDE_REFERENCE_LINE";
        result.projection_diagnostics.push_back(diagnostic);
        continue;
      }
      const std::size_t bin = GridIndex(projection.s_m, request.grid_spacing_m);
      if (bin < rtk_up_by_bin.size()) {
        rtk_up_by_bin[bin].push_back(sample.h_m);
        diagnostic.used = true;
        diagnostic.corrected_up_m = sample.h_m;
        diagnostic.source = "RTK";
      }
      result.projection_diagnostics.push_back(diagnostic);
    }
  }

  std::vector<double> rtk_median_by_bin(grid_count, std::numeric_limits<double>::quiet_NaN());
  for (std::size_t bin = 0; bin < grid_count; ++bin) {
    rtk_median_by_bin[bin] = Median(rtk_up_by_bin[bin]);
  }

  for (const auto &member : members) {
    std::vector<double> offset_samples;
    for (const auto &point : member.trajectory) {
      const SharedReferenceProjection projection =
        ProjectPointToSharedReferenceLine(point.xy_m, result.reference_line);
      if (!projection.valid || projection.s_m < -0.5 * request.grid_spacing_m ||
          projection.s_m > max_s_m + 0.5 * request.grid_spacing_m) {
        continue;
      }
      const std::size_t bin = GridIndex(projection.s_m, request.grid_spacing_m);
      if (bin < rtk_median_by_bin.size() && std::isfinite(rtk_median_by_bin[bin])) {
        offset_samples.push_back(rtk_median_by_bin[bin] - point.up_m);
      }
    }
    const double nav_offset_m = std::isfinite(Median(offset_samples))
      ? Median(offset_samples)
      : 0.0;

    for (const auto &point : member.trajectory) {
      SharedVerticalReferenceProjectionDiagnosticRow diagnostic;
      diagnostic.member_id = member.member_id;
      diagnostic.sample_kind = "STAGE2_NAV";
      diagnostic.time_s = point.time_s;
      diagnostic.raw_up_m = point.up_m;
      diagnostic.corrected_up_m = point.up_m + nav_offset_m;
      const SharedReferenceProjection projection =
        ProjectPointToSharedReferenceLine(point.xy_m, result.reference_line);
      diagnostic.s_m = projection.s_m;
      diagnostic.lateral_offset_m = projection.lateral_offset_m;
      if (!projection.valid || projection.s_m < -0.5 * request.grid_spacing_m ||
          projection.s_m > max_s_m + 0.5 * request.grid_spacing_m) {
        diagnostic.source = "OUTSIDE_REFERENCE_LINE";
        result.projection_diagnostics.push_back(diagnostic);
        continue;
      }
      const std::size_t bin = GridIndex(projection.s_m, request.grid_spacing_m);
      if (bin < nav_up_by_bin.size()) {
        nav_up_by_bin[bin].push_back(diagnostic.corrected_up_m);
        diagnostic.used = true;
        diagnostic.source = "NAV_BRIDGE";
      }
      result.projection_diagnostics.push_back(diagnostic);
    }
  }

  result.rows.reserve(grid_count);
  for (std::size_t bin = 0; bin < grid_count; ++bin) {
    SharedVerticalReferenceRow row;
    row.s_m = static_cast<double>(bin) * request.grid_spacing_m;
    row.sigma_m = request.sigma_m;
    if (std::isfinite(rtk_median_by_bin[bin])) {
      row.reference_up_m = rtk_median_by_bin[bin];
      row.source = "RTK";
      row.rtk_weight = 1.0;
      row.sample_count = rtk_up_by_bin[bin].size();
    } else {
      row.reference_up_m = Median(nav_up_by_bin[bin]);
      row.source = std::isfinite(row.reference_up_m) ? "NAV_BRIDGE" : "UNOBSERVED";
      row.nav_bridge_weight = std::isfinite(row.reference_up_m) ? 1.0 : 0.0;
      row.sample_count = nav_up_by_bin[bin].size();
    }
    result.rows.push_back(row);
  }
  FillMissingRowsByInterpolation(result.rows);
  return result;
}

void WriteSharedVerticalReferenceCsv(
  const std::filesystem::path &path,
  const std::vector<SharedVerticalReferenceRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write shared vertical reference: " + path.string());
  }
  stream << std::setprecision(17);
  stream << "s_m,reference_up_m,sigma_m,source,rtk_weight,nav_bridge_weight,sample_count\n";
  for (const auto &row : rows) {
    stream << row.s_m << ',' << row.reference_up_m << ',' << row.sigma_m << ',';
    WriteCsvString(stream, row.source);
    stream << ',' << row.rtk_weight << ',' << row.nav_bridge_weight << ',' << row.sample_count << '\n';
  }
}

void WriteSharedReferenceLineCsv(
  const std::filesystem::path &path,
  const std::vector<SharedReferenceLinePoint> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write shared reference line: " + path.string());
  }
  stream << std::setprecision(17);
  stream << "s_m,east_m,north_m,origin_lat_rad,origin_lon_rad,origin_h_m\n";
  for (const auto &row : rows) {
    stream << row.s_m << ',' << row.east_m << ',' << row.north_m << ','
           << row.origin_lat_rad << ',' << row.origin_lon_rad << ','
           << row.origin_h_m << '\n';
  }
}

void WriteSharedVerticalReferenceProjectionDiagnosticsCsv(
  const std::filesystem::path &path,
  const std::vector<SharedVerticalReferenceProjectionDiagnosticRow> &rows) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write shared reference projection diagnostics: " + path.string());
  }
  stream << std::setprecision(17);
  stream
    << "member_id,sample_kind,time_s,s_m,lateral_offset_m,raw_up_m,corrected_up_m,used,source\n";
  for (const auto &row : rows) {
    WriteCsvString(stream, row.member_id);
    stream << ',';
    WriteCsvString(stream, row.sample_kind);
    stream << ',' << row.time_s << ',' << row.s_m << ',' << row.lateral_offset_m << ','
           << row.raw_up_m << ',' << row.corrected_up_m << ',' << (row.used ? 1 : 0) << ',';
    WriteCsvString(stream, row.source);
    stream << '\n';
  }
}

std::vector<SharedVerticalReferenceRow> ReadSharedVerticalReferenceCsv(
  const std::filesystem::path &path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open shared vertical reference: " + path.string());
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("shared vertical reference CSV is empty: " + path.string());
  }
  const auto columns = HeaderIndex(SplitCsvLine(line));
  const std::size_t s_col = RequiredColumn(columns, "s_m");
  const std::size_t up_col = RequiredColumn(columns, "reference_up_m");
  const std::size_t sigma_col = RequiredColumn(columns, "sigma_m");
  const std::size_t source_col = RequiredColumn(columns, "source");
  const std::size_t rtk_weight_col = RequiredColumn(columns, "rtk_weight");
  const std::size_t nav_weight_col = RequiredColumn(columns, "nav_bridge_weight");
  const std::size_t sample_count_col = RequiredColumn(columns, "sample_count");
  std::vector<SharedVerticalReferenceRow> rows;
  while (std::getline(stream, line)) {
    if (Trim(line).empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    SharedVerticalReferenceRow row;
    row.s_m = ParseDouble(fields, s_col, "s_m");
    row.reference_up_m = ParseDouble(fields, up_col, "reference_up_m");
    row.sigma_m = ParseDouble(fields, sigma_col, "sigma_m");
    row.source = source_col < fields.size() ? fields[source_col] : "UNSET";
    row.rtk_weight = ParseDouble(fields, rtk_weight_col, "rtk_weight");
    row.nav_bridge_weight = ParseDouble(fields, nav_weight_col, "nav_bridge_weight");
    row.sample_count = ParseSize(fields, sample_count_col, "sample_count");
    rows.push_back(row);
  }
  if (rows.empty()) {
    throw std::runtime_error("shared vertical reference CSV contains no rows");
  }
  return rows;
}

std::vector<SharedReferenceLinePoint> ReadSharedReferenceLineCsv(
  const std::filesystem::path &path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open shared reference line: " + path.string());
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("shared reference line CSV is empty: " + path.string());
  }
  const auto columns = HeaderIndex(SplitCsvLine(line));
  const std::size_t s_col = RequiredColumn(columns, "s_m");
  const std::size_t east_col = RequiredColumn(columns, "east_m");
  const std::size_t north_col = RequiredColumn(columns, "north_m");
  const auto origin_lat_it = columns.find("origin_lat_rad");
  const auto origin_lon_it = columns.find("origin_lon_rad");
  const auto origin_h_it = columns.find("origin_h_m");
  std::vector<SharedReferenceLinePoint> rows;
  while (std::getline(stream, line)) {
    if (Trim(line).empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    SharedReferenceLinePoint row;
    row.s_m = ParseDouble(fields, s_col, "s_m");
    row.east_m = ParseDouble(fields, east_col, "east_m");
    row.north_m = ParseDouble(fields, north_col, "north_m");
    if (origin_lat_it != columns.end() &&
        origin_lon_it != columns.end() &&
        origin_h_it != columns.end()) {
      row.origin_lat_rad = ParseDouble(fields, origin_lat_it->second, "origin_lat_rad");
      row.origin_lon_rad = ParseDouble(fields, origin_lon_it->second, "origin_lon_rad");
      row.origin_h_m = ParseDouble(fields, origin_h_it->second, "origin_h_m");
    }
    rows.push_back(row);
  }
  if (rows.size() < 2U) {
    throw std::runtime_error("shared reference line CSV requires at least two rows");
  }
  return rows;
}

}  // namespace offline_lc_minimal
