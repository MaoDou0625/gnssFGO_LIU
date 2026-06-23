#include <algorithm>
#include <cctype>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/ResultOutputWriters.h"
#include "offline_lc_minimal/core/SharedVerticalReferenceBuilder.h"
#include "offline_lc_minimal/io/TrajectoryCsvReader.h"

namespace offline_lc_minimal {
namespace {

struct ManifestRow {
  std::string member_id;
  std::filesystem::path config_path;
  std::filesystem::path stage2_trajectory_path;
  std::filesystem::path gnss_path;
};

void PrintUsage() {
  std::cout
    << "offline_lc_shared_vertical_reference_builder --manifest <csv> "
       "--output-dir <dir> [--grid-spacing-m <m>] [--sigma-m <m>]\n";
}

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
    throw std::runtime_error("manifest missing required column: " + name);
  }
  return it->second;
}

std::size_t OptionalColumn(
  const std::unordered_map<std::string, std::size_t> &columns,
  const std::string &name) {
  const auto it = columns.find(name);
  return it == columns.end() ? std::numeric_limits<std::size_t>::max() : it->second;
}

std::string Field(
  const std::vector<std::string> &fields,
  const std::size_t column,
  const std::string &name) {
  if (column >= fields.size()) {
    throw std::runtime_error("manifest row missing field: " + name);
  }
  return fields[column];
}

std::filesystem::path ResolvePath(
  const std::filesystem::path &base_dir,
  const std::string &value) {
  if (value.empty()) {
    return {};
  }
  std::filesystem::path path(value);
  if (path.is_relative()) {
    path = base_dir / path;
  }
  return path.lexically_normal();
}

std::vector<ManifestRow> ReadManifest(const std::filesystem::path &manifest_path) {
  std::ifstream stream(manifest_path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to open manifest: " + manifest_path.string());
  }
  std::string line;
  if (!std::getline(stream, line)) {
    throw std::runtime_error("manifest is empty: " + manifest_path.string());
  }
  const auto columns = HeaderIndex(SplitCsvLine(line));
  const std::size_t member_col = RequiredColumn(columns, "member_id");
  const std::size_t config_col = RequiredColumn(columns, "config_path");
  const std::size_t trajectory_col = RequiredColumn(columns, "stage2_trajectory_path");
  const std::size_t gnss_col = OptionalColumn(columns, "gnss_path");
  const std::filesystem::path base_dir = manifest_path.parent_path();

  std::vector<ManifestRow> rows;
  while (std::getline(stream, line)) {
    if (Trim(line).empty()) {
      continue;
    }
    const auto fields = SplitCsvLine(line);
    ManifestRow row;
    row.member_id = Field(fields, member_col, "member_id");
    row.config_path = ResolvePath(base_dir, Field(fields, config_col, "config_path"));
    row.stage2_trajectory_path =
      ResolvePath(base_dir, Field(fields, trajectory_col, "stage2_trajectory_path"));
    if (gnss_col != std::numeric_limits<std::size_t>::max() && gnss_col < fields.size()) {
      row.gnss_path = ResolvePath(base_dir, fields[gnss_col]);
    }
    if (row.member_id.empty()) {
      throw std::runtime_error("manifest member_id cannot be empty");
    }
    rows.push_back(std::move(row));
  }
  if (rows.size() < 2U) {
    throw std::runtime_error("manifest requires at least two members");
  }
  return rows;
}

std::vector<SharedVerticalReferenceMember> LoadMembers(
  const std::vector<ManifestRow> &manifest_rows) {
  std::vector<SharedVerticalReferenceMember> members;
  members.reserve(manifest_rows.size());
  for (const auto &manifest_row : manifest_rows) {
    OfflineRunnerConfig config = LoadConfigFile(manifest_row.config_path.string(), DefaultConfig());
    if (!manifest_row.gnss_path.empty()) {
      config.gnss_path = manifest_row.gnss_path.string();
    }
    ValidateConfig(config);

    SharedVerticalReferenceMember member;
    member.member_id = manifest_row.member_id;
    member.config = config;
    member.trajectory = ReadTrajectoryCsvRows(manifest_row.stage2_trajectory_path);
    members.push_back(std::move(member));
  }
  return members;
}

}  // namespace
}  // namespace offline_lc_minimal

int main(int argc, char **argv) {
  using namespace offline_lc_minimal;

  try {
    std::filesystem::path manifest_path;
    std::filesystem::path output_dir;
    double grid_spacing_m = 1.0;
    double sigma_m = 0.015;

    for (int index = 1; index < argc; ++index) {
      const std::string arg = argv[index];
      if (arg == "--help" || arg == "-h") {
        PrintUsage();
        return 0;
      }
      if (arg == "--manifest" && index + 1 < argc) {
        manifest_path = argv[++index];
      } else if (arg == "--output-dir" && index + 1 < argc) {
        output_dir = argv[++index];
      } else if (arg == "--grid-spacing-m" && index + 1 < argc) {
        grid_spacing_m = std::stod(argv[++index]);
      } else if (arg == "--sigma-m" && index + 1 < argc) {
        sigma_m = std::stod(argv[++index]);
      } else {
        throw std::runtime_error("unknown or incomplete argument: " + arg);
      }
    }

    if (manifest_path.empty() || output_dir.empty()) {
      PrintUsage();
      throw std::runtime_error("--manifest and --output-dir are required");
    }

    SharedVerticalReferenceBuildRequest request;
    request.grid_spacing_m = grid_spacing_m;
    request.sigma_m = sigma_m;
    request.members = LoadMembers(ReadManifest(manifest_path));

    const SharedVerticalReference reference =
      BuildSharedVerticalReference(std::move(request));
    std::filesystem::create_directories(output_dir);
    WriteSharedVerticalReferenceCsv(
      output_dir / "shared_vertical_reference.csv",
      reference.rows);
    WriteSharedReferenceLineCsv(
      output_dir / "shared_reference_line.csv",
      reference.reference_line);
    WriteSharedVerticalReferenceProjectionDiagnosticsCsv(
      output_dir / "shared_vertical_reference_projection_diagnostics.csv",
      reference.projection_diagnostics);
    WriteTextFile(
      output_dir / "shared_vertical_reference_summary.txt",
      "reference_member_id=" + reference.reference_member_id + "\n" +
      "reference_row_count=" + std::to_string(reference.rows.size()) + "\n" +
      "projection_diagnostic_count=" +
        std::to_string(reference.projection_diagnostics.size()) + "\n");

    std::cout << "offline_lc_shared_vertical_reference_builder completed.\n"
              << "output_dir=" << std::filesystem::absolute(output_dir).string() << '\n'
              << "reference_member_id=" << reference.reference_member_id << '\n'
              << "reference_row_count=" << reference.rows.size() << '\n';
    return 0;
  } catch (const std::exception &exception) {
    std::cerr << "offline_lc_shared_vertical_reference_builder failed: "
              << exception.what() << '\n';
    return 1;
  }
}
