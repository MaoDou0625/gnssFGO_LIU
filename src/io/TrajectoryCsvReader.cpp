#include "offline_lc_minimal/io/TrajectoryCsvReader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>

namespace offline_lc_minimal {
namespace {

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
  if (in_quotes) {
    throw std::runtime_error("unterminated quoted CSV field");
  }
  fields.push_back(Trim(std::move(current)));
  return fields;
}

std::unordered_map<std::string, std::size_t> BuildHeaderIndex(
  const std::vector<std::string> &header) {
  std::unordered_map<std::string, std::size_t> index_by_name;
  for (std::size_t index = 0; index < header.size(); ++index) {
    if (header[index].empty()) {
      continue;
    }
    index_by_name.emplace(header[index], index);
  }
  return index_by_name;
}

std::size_t RequiredColumn(
  const std::unordered_map<std::string, std::size_t> &columns,
  const std::string &name) {
  const auto it = columns.find(name);
  if (it == columns.end()) {
    throw std::runtime_error("trajectory CSV missing required column: " + name);
  }
  return it->second;
}

double ParseDoubleField(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name,
  const std::size_t line_number) {
  if (column >= fields.size()) {
    throw std::runtime_error(
      "trajectory CSV line " + std::to_string(line_number) +
      " missing field for column: " + name);
  }
  try {
    std::size_t consumed = 0U;
    const double value = std::stod(fields[column], &consumed);
    if (consumed != fields[column].size()) {
      throw std::runtime_error("trailing characters");
    }
    return value;
  } catch (const std::exception &) {
    throw std::runtime_error(
      "trajectory CSV line " + std::to_string(line_number) +
      " has invalid numeric value for column: " + name);
  }
}

bool ParseBoolField(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::size_t line_number) {
  if (column >= fields.size()) {
    return false;
  }
  const std::string value = fields[column];
  if (value == "1" || value == "true" || value == "TRUE") {
    return true;
  }
  if (value == "0" || value == "false" || value == "FALSE" || value.empty()) {
    return false;
  }
  throw std::runtime_error(
    "trajectory CSV line " + std::to_string(line_number) +
    " has invalid gnss_factor_used value");
}

GnssFixType ParseGnssFixTypeField(const std::string &value) {
  if (value == "RTKFIX") {
    return GnssFixType::kRtkFix;
  }
  if (value == "RTKFLOAT") {
    return GnssFixType::kRtkFloat;
  }
  if (value == "SINGLE") {
    return GnssFixType::kSingle;
  }
  return GnssFixType::kNoSolution;
}

std::size_t OptionalColumn(
  const std::unordered_map<std::string, std::size_t> &columns,
  const std::string &name) {
  const auto it = columns.find(name);
  return it == columns.end() ? std::numeric_limits<std::size_t>::max() : it->second;
}

bool HasColumn(const std::size_t column) {
  return column != std::numeric_limits<std::size_t>::max();
}

}  // namespace

std::vector<TrajectoryCsvRow> ReadTrajectoryCsvRows(
  const std::filesystem::path &path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open trajectory CSV: " + path.string());
  }

  std::string line;
  std::size_t line_number = 0U;
  while (std::getline(stream, line)) {
    ++line_number;
    if (!Trim(line).empty()) {
      break;
    }
  }
  if (line.empty()) {
    throw std::runtime_error("trajectory CSV is empty: " + path.string());
  }

  const std::vector<std::string> header = SplitCsvLine(line);
  const auto columns = BuildHeaderIndex(header);
  const std::size_t time_col = RequiredColumn(columns, "time_s");
  const std::size_t east_col = RequiredColumn(columns, "east_m");
  const std::size_t north_col = RequiredColumn(columns, "north_m");
  const std::size_t up_col = RequiredColumn(columns, "up_m");
  const std::size_t vx_col = RequiredColumn(columns, "vx_mps");
  const std::size_t vy_col = RequiredColumn(columns, "vy_mps");
  const std::size_t vz_col = RequiredColumn(columns, "vz_mps");
  const std::size_t yaw_col = RequiredColumn(columns, "yaw_rad");
  const std::size_t pitch_col = RequiredColumn(columns, "pitch_rad");
  const std::size_t roll_col = RequiredColumn(columns, "roll_rad");
  const std::size_t bax_col = RequiredColumn(columns, "bax");
  const std::size_t bay_col = RequiredColumn(columns, "bay");
  const std::size_t baz_col = RequiredColumn(columns, "baz");
  const std::size_t bgx_col = RequiredColumn(columns, "bgx");
  const std::size_t bgy_col = RequiredColumn(columns, "bgy");
  const std::size_t bgz_col = RequiredColumn(columns, "bgz");
  const std::size_t gnss_used_col = OptionalColumn(columns, "gnss_factor_used");
  const std::size_t gnss_fix_col = OptionalColumn(columns, "gnss_fix_type");
  const std::size_t gnss_residual_col = OptionalColumn(columns, "gnss_residual_m");
  const std::size_t lat_col = OptionalColumn(columns, "lat_rad");
  const std::size_t lon_col = OptionalColumn(columns, "lon_rad");
  const std::size_t h_col = OptionalColumn(columns, "h_m");

  std::vector<TrajectoryCsvRow> rows;
  double previous_time_s = -std::numeric_limits<double>::infinity();
  while (std::getline(stream, line)) {
    ++line_number;
    if (Trim(line).empty()) {
      continue;
    }
    const std::vector<std::string> fields = SplitCsvLine(line);
    TrajectoryCsvRow csv_row;
    TrajectoryRow &row = csv_row.trajectory;
    row.time_s = ParseDoubleField(fields, time_col, "time_s", line_number);
    row.enu_position_m = Eigen::Vector3d(
      ParseDoubleField(fields, east_col, "east_m", line_number),
      ParseDoubleField(fields, north_col, "north_m", line_number),
      ParseDoubleField(fields, up_col, "up_m", line_number));
    row.enu_velocity_mps = Eigen::Vector3d(
      ParseDoubleField(fields, vx_col, "vx_mps", line_number),
      ParseDoubleField(fields, vy_col, "vy_mps", line_number),
      ParseDoubleField(fields, vz_col, "vz_mps", line_number));
    row.ypr_rad = Eigen::Vector3d(
      ParseDoubleField(fields, yaw_col, "yaw_rad", line_number),
      ParseDoubleField(fields, pitch_col, "pitch_rad", line_number),
      ParseDoubleField(fields, roll_col, "roll_rad", line_number));
    row.bias_acc = Eigen::Vector3d(
      ParseDoubleField(fields, bax_col, "bax", line_number),
      ParseDoubleField(fields, bay_col, "bay", line_number),
      ParseDoubleField(fields, baz_col, "baz", line_number));
    row.bias_gyro = Eigen::Vector3d(
      ParseDoubleField(fields, bgx_col, "bgx", line_number),
      ParseDoubleField(fields, bgy_col, "bgy", line_number),
      ParseDoubleField(fields, bgz_col, "bgz", line_number));
    if (HasColumn(gnss_used_col)) {
      row.gnss_factor_used = ParseBoolField(fields, gnss_used_col, line_number);
    }
    if (HasColumn(gnss_fix_col) && gnss_fix_col < fields.size()) {
      row.gnss_fix_type = ParseGnssFixTypeField(fields[gnss_fix_col]);
    }
    if (HasColumn(gnss_residual_col) && gnss_residual_col < fields.size()) {
      row.gnss_residual_m =
        ParseDoubleField(fields, gnss_residual_col, "gnss_residual_m", line_number);
    }
    if (HasColumn(lat_col) && HasColumn(lon_col) && HasColumn(h_col)) {
      csv_row.lat_rad = ParseDoubleField(fields, lat_col, "lat_rad", line_number);
      csv_row.lon_rad = ParseDoubleField(fields, lon_col, "lon_rad", line_number);
      csv_row.h_m = ParseDoubleField(fields, h_col, "h_m", line_number);
      csv_row.has_geodetic =
        std::isfinite(csv_row.lat_rad) &&
        std::isfinite(csv_row.lon_rad) &&
        std::isfinite(csv_row.h_m);
    }
    if (!(row.time_s > previous_time_s)) {
      throw std::runtime_error(
        "trajectory CSV times must be strictly increasing at line " +
        std::to_string(line_number));
    }
    previous_time_s = row.time_s;
    rows.push_back(csv_row);
  }

  if (rows.empty()) {
    throw std::runtime_error("trajectory CSV contains no trajectory rows: " + path.string());
  }
  return rows;
}

std::vector<TrajectoryRow> ReadTrajectoryCsv(
  const std::filesystem::path &path) {
  const std::vector<TrajectoryCsvRow> csv_rows = ReadTrajectoryCsvRows(path);
  std::vector<TrajectoryRow> rows;
  rows.reserve(csv_rows.size());
  for (const auto &csv_row : csv_rows) {
    rows.push_back(csv_row.trajectory);
  }
  return rows;
}

Stage2VelocityReference ReadStage2VelocityReferenceCsv(
  const std::filesystem::path &path,
  std::shared_ptr<const OfflineRunnerConfig> source_config) {
  Stage2VelocityReference reference;
  reference.trajectory = ReadTrajectoryCsv(path);
  reference.reference_states =
    BuildStage2ReferenceStatesFromTrajectory(reference.trajectory);
  reference.source_config = std::move(source_config);
  return reference;
}

}  // namespace offline_lc_minimal
