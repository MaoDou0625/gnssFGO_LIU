#include "offline_lc_minimal/io/BodyZBiasReestimateCsvReader.h"

#include <algorithm>
#include <cctype>
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
    if (!header[index].empty()) {
      index_by_name.emplace(header[index], index);
    }
  }
  return index_by_name;
}

std::size_t RequiredColumn(
  const std::unordered_map<std::string, std::size_t> &columns,
  const std::string &name) {
  const auto it = columns.find(name);
  if (it == columns.end()) {
    throw std::runtime_error(
      "body-z bias reestimate CSV missing required column: " + name);
  }
  return it->second;
}

std::string StringField(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name,
  const std::size_t line_number) {
  if (column >= fields.size()) {
    throw std::runtime_error(
      "body-z bias reestimate CSV line " + std::to_string(line_number) +
      " missing field for column: " + name);
  }
  return fields[column];
}

double DoubleField(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name,
  const std::size_t line_number) {
  const std::string value = StringField(fields, column, name, line_number);
  try {
    std::size_t consumed = 0U;
    const double parsed = std::stod(value, &consumed);
    if (consumed != value.size()) {
      throw std::runtime_error("trailing characters");
    }
    return parsed;
  } catch (const std::exception &) {
    throw std::runtime_error(
      "body-z bias reestimate CSV line " + std::to_string(line_number) +
      " has invalid numeric value for column: " + name);
  }
}

long long IntegerField(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name,
  const std::size_t line_number) {
  const std::string value = StringField(fields, column, name, line_number);
  try {
    std::size_t consumed = 0U;
    const long long parsed = std::stoll(value, &consumed);
    if (consumed != value.size()) {
      throw std::runtime_error("trailing characters");
    }
    return parsed;
  } catch (const std::exception &) {
    throw std::runtime_error(
      "body-z bias reestimate CSV line " + std::to_string(line_number) +
      " has invalid integer value for column: " + name);
  }
}

std::size_t SizeField(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name,
  const std::size_t line_number) {
  const long long parsed = IntegerField(fields, column, name, line_number);
  if (parsed < 0) {
    throw std::runtime_error(
      "body-z CSV line " + std::to_string(line_number) +
      " has negative value for column: " + name);
  }
  return static_cast<std::size_t>(parsed);
}

bool BoolField(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name,
  const std::size_t line_number) {
  const std::string value = StringField(fields, column, name, line_number);
  if (value == "1" || value == "true" || value == "TRUE") {
    return true;
  }
  if (value == "0" || value == "false" || value == "FALSE") {
    return false;
  }
  throw std::runtime_error(
    "body-z CSV line " + std::to_string(line_number) +
    " has invalid boolean value for column: " + name);
}

}  // namespace

std::vector<BodyZBiasReestimateSegmentRow> ReadBodyZBiasReestimateSegmentCsv(
  const std::filesystem::path &path) {
  std::ifstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error(
      "failed to open body-z bias reestimate segment CSV: " + path.string());
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
    return {};
  }

  const auto columns = BuildHeaderIndex(SplitCsvLine(line));
  const std::size_t segment_col = RequiredColumn(columns, "segment_index");
  const std::size_t source_type_col = RequiredColumn(columns, "source_type");
  const std::size_t source_bias_col =
    RequiredColumn(columns, "source_bias_window_index");
  const std::size_t source_outage_col =
    RequiredColumn(columns, "source_outage_window_index");
  const std::size_t start_state_col = RequiredColumn(columns, "start_state_index");
  const std::size_t end_state_col = RequiredColumn(columns, "end_state_index");
  const std::size_t anchor_state_col = RequiredColumn(columns, "anchor_state_index");
  const std::size_t bias_start_col =
    RequiredColumn(columns, "bias_window_start_time_s");
  const std::size_t bias_end_col =
    RequiredColumn(columns, "bias_window_end_time_s");
  const std::size_t start_time_col = RequiredColumn(columns, "start_time_s");
  const std::size_t end_time_col = RequiredColumn(columns, "end_time_s");
  const std::size_t duration_col = RequiredColumn(columns, "duration_s");
  const std::size_t detected_delta_col =
    RequiredColumn(columns, "detected_bias_delta_mps2");
  const std::size_t reference_baz_col =
    RequiredColumn(columns, "reference_ba_z_mps2");
  const std::size_t prior_target_col =
    RequiredColumn(columns, "prior_target_ba_z_mps2");
  const std::size_t prior_sigma_col =
    RequiredColumn(columns, "prior_sigma_mps2");
  const std::size_t initialized_count_col =
    RequiredColumn(columns, "initialized_state_count");
  const std::size_t prior_added_col =
    RequiredColumn(columns, "prior_factor_added");
  const std::size_t skip_reason_col = RequiredColumn(columns, "skip_reason");

  std::vector<BodyZBiasReestimateSegmentRow> rows;
  double previous_start_time_s = -std::numeric_limits<double>::infinity();
  while (std::getline(stream, line)) {
    ++line_number;
    if (Trim(line).empty()) {
      continue;
    }
    const std::vector<std::string> fields = SplitCsvLine(line);
    BodyZBiasReestimateSegmentRow row;
    row.segment_index = SizeField(fields, segment_col, "segment_index", line_number);
    row.source_type = StringField(fields, source_type_col, "source_type", line_number);
    row.source_bias_window_index =
      SizeField(fields, source_bias_col, "source_bias_window_index", line_number);
    row.source_outage_window_index =
      IntegerField(fields, source_outage_col, "source_outage_window_index", line_number);
    row.start_state_index =
      IntegerField(fields, start_state_col, "start_state_index", line_number);
    row.end_state_index =
      IntegerField(fields, end_state_col, "end_state_index", line_number);
    row.anchor_state_index =
      IntegerField(fields, anchor_state_col, "anchor_state_index", line_number);
    row.bias_window_start_time_s =
      DoubleField(fields, bias_start_col, "bias_window_start_time_s", line_number);
    row.bias_window_end_time_s =
      DoubleField(fields, bias_end_col, "bias_window_end_time_s", line_number);
    row.start_time_s = DoubleField(fields, start_time_col, "start_time_s", line_number);
    row.end_time_s = DoubleField(fields, end_time_col, "end_time_s", line_number);
    row.duration_s = DoubleField(fields, duration_col, "duration_s", line_number);
    row.detected_bias_delta_mps2 =
      DoubleField(fields, detected_delta_col, "detected_bias_delta_mps2", line_number);
    row.reference_ba_z_mps2 =
      DoubleField(fields, reference_baz_col, "reference_ba_z_mps2", line_number);
    row.prior_target_ba_z_mps2 =
      DoubleField(fields, prior_target_col, "prior_target_ba_z_mps2", line_number);
    row.prior_sigma_mps2 =
      DoubleField(fields, prior_sigma_col, "prior_sigma_mps2", line_number);
    row.initialized_state_count =
      SizeField(fields, initialized_count_col, "initialized_state_count", line_number);
    row.prior_factor_added =
      BoolField(fields, prior_added_col, "prior_factor_added", line_number);
    row.skip_reason = StringField(fields, skip_reason_col, "skip_reason", line_number);
    if (!(row.start_time_s >= previous_start_time_s)) {
      throw std::runtime_error(
        "body-z bias reestimate segment CSV start_time_s must be nondecreasing at line " +
        std::to_string(line_number));
    }
    if (!(row.end_time_s >= row.start_time_s)) {
      throw std::runtime_error(
        "body-z bias reestimate segment CSV end_time_s must be after start_time_s at line " +
        std::to_string(line_number));
    }
    previous_start_time_s = row.start_time_s;
    rows.push_back(row);
  }
  return rows;
}

}  // namespace offline_lc_minimal
