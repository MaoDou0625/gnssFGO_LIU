#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "offline_lc_minimal/io/BodyZBiasReestimateCsvReader.h"

namespace {

template <typename Function>
void RunTest(const std::string &name, Function &&function) {
  try {
    function();
  } catch (const std::exception &exception) {
    throw std::runtime_error(name + ": " + exception.what());
  }
}

void ExpectTrue(const bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void ExpectNear(
  const double actual,
  const double expected,
  const double tolerance,
  const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) +
      " expected=" + std::to_string(expected));
  }
}

std::filesystem::path TempCsvPath(const std::string &name) {
  return std::filesystem::temp_directory_path() / name;
}

void WriteFile(const std::filesystem::path &path, const std::string &content) {
  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("failed to write temp file");
  }
  stream << content;
}

std::string ValidBiasReestimateSegmentCsv() {
  return
    "segment_index,source_type,source_bias_window_index,source_outage_window_index,"
    "start_state_index,end_state_index,anchor_state_index,bias_window_start_time_s,"
    "bias_window_end_time_s,start_time_s,end_time_s,duration_s,"
    "detected_bias_delta_mps2,detected_bias_delta_ug,reference_ba_z_mps2,"
    "reference_ba_z_ug,prior_target_ba_z_mps2,prior_target_ba_z_ug,"
    "prior_sigma_mps2,prior_sigma_ug,initialized_state_count,"
    "prior_factor_added,skip_reason\n"
    "0,ROAD_HIGH_NOISE,1,-1,10,20,10,100.0,110.0,101.0,109.0,8.0,"
    "0.001,101.971621,-0.004,-407.886484,-0.003,-305.914863,"
    "0.05,5098.581064,11,1,ADDED\n";
}

void TestReadBodyZBiasReestimateSegmentCsvParsesRows() {
  const auto path = TempCsvPath("offline_lc_body_z_bias_reestimate_reader_valid.csv");
  WriteFile(path, ValidBiasReestimateSegmentCsv());

  const auto rows = offline_lc_minimal::ReadBodyZBiasReestimateSegmentCsv(path);

  ExpectTrue(rows.size() == 1U, "reader should parse one segment");
  ExpectTrue(rows[0].segment_index == 0U, "segment index should parse");
  ExpectTrue(rows[0].source_type == "ROAD_HIGH_NOISE", "source type should parse");
  ExpectTrue(rows[0].source_outage_window_index == -1, "outage index should parse");
  ExpectNear(rows[0].start_time_s, 101.0, 1.0e-12, "segment start should parse");
  ExpectNear(rows[0].end_time_s, 109.0, 1.0e-12, "segment end should parse");
  ExpectNear(
    rows[0].detected_bias_delta_mps2,
    0.001,
    1.0e-12,
    "detected delta should parse");
  ExpectNear(
    rows[0].reference_ba_z_mps2,
    -0.004,
    1.0e-12,
    "reference ba_z should parse");
  ExpectNear(
    rows[0].prior_target_ba_z_mps2,
    -0.003,
    1.0e-12,
    "prior target should parse");
  ExpectTrue(rows[0].prior_factor_added, "prior factor flag should parse");
  ExpectTrue(rows[0].skip_reason == "ADDED", "skip reason should parse");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestReadBodyZBiasReestimateSegmentCsvParsesRows",
      TestReadBodyZBiasReestimateSegmentCsvParsesRows);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
