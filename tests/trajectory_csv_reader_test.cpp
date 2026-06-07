#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include "offline_lc_minimal/io/TrajectoryCsvReader.h"

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

std::string ValidCsv() {
  return
    "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,"
    "bax,bay,baz,bgx,bgy,bgz,gnss_factor_used,gnss_fix_type,gnss_residual_m\n"
    "1,2,3,4,0.1,0.2,0.3,0.01,0.02,0.03,1e-3,2e-3,3e-3,4e-4,5e-4,6e-4,1,RTKFIX,0.12\n"
    "2,5,6,7,0.4,0.5,0.6,0.04,0.05,0.06,4e-3,5e-3,6e-3,7e-4,8e-4,9e-4,0,NO_SOLUTION,nan\n";
}

void TestReadTrajectoryCsvParsesRowsByHeader() {
  const auto path = TempCsvPath("offline_lc_trajectory_reader_valid.csv");
  WriteFile(path, ValidCsv());

  const auto rows = offline_lc_minimal::ReadTrajectoryCsv(path);

  ExpectTrue(rows.size() == 2U, "reader should parse two rows");
  ExpectNear(rows[0].time_s, 1.0, 1e-12, "time should parse");
  ExpectNear(rows[0].enu_position_m.z(), 4.0, 1e-12, "up should parse");
  ExpectNear(rows[0].enu_velocity_mps.y(), 0.2, 1e-12, "velocity should parse");
  ExpectNear(rows[0].ypr_rad.x(), 0.01, 1e-12, "yaw should parse");
  ExpectNear(rows[0].bias_acc.z(), 3e-3, 1e-15, "accel bias should parse");
  ExpectNear(rows[0].bias_gyro.y(), 5e-4, 1e-15, "gyro bias should parse");
  ExpectTrue(rows[0].gnss_factor_used, "GNSS factor flag should parse");
  ExpectTrue(
    rows[0].gnss_fix_type == offline_lc_minimal::GnssFixType::kRtkFix,
    "GNSS fix type should parse");
  ExpectNear(rows[0].gnss_residual_m, 0.12, 1e-12, "GNSS residual should parse");
  ExpectTrue(std::isnan(rows[1].gnss_residual_m), "nan GNSS residual should parse");
}

void TestReadStage2VelocityReferenceBuildsReferenceStates() {
  const auto path = TempCsvPath("offline_lc_trajectory_reader_stage2.csv");
  WriteFile(path, ValidCsv());
  auto config = std::make_shared<offline_lc_minimal::OfflineRunnerConfig>(
    offline_lc_minimal::DefaultConfig());

  const auto reference =
    offline_lc_minimal::ReadStage2VelocityReferenceCsv(path, config);

  ExpectTrue(reference.trajectory.size() == 2U, "Stage2 reference should keep trajectory");
  ExpectTrue(reference.reference_states.size() == 2U, "Stage2 reference should build native states");
  ExpectTrue(reference.source_config == config, "Stage2 reference should keep source config");
  ExpectNear(
    reference.reference_states[1].pose.translation().z(),
    7.0,
    1e-12,
    "reference state should preserve height");
}

void TestReadTrajectoryCsvRejectsMissingRequiredColumn() {
  const auto path = TempCsvPath("offline_lc_trajectory_reader_missing.csv");
  WriteFile(
    path,
    "time_s,east_m,north_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz\n"
    "1,2,3,0.1,0.2,0.3,0.01,0.02,0.03,1e-3,2e-3,3e-3,4e-4,5e-4,6e-4\n");

  bool threw = false;
  try {
    (void)offline_lc_minimal::ReadTrajectoryCsv(path);
  } catch (const std::exception &exception) {
    threw = std::string(exception.what()).find("up_m") != std::string::npos;
  }
  ExpectTrue(threw, "reader should reject missing up_m");
}

void TestReadTrajectoryCsvRejectsNonIncreasingTime() {
  const auto path = TempCsvPath("offline_lc_trajectory_reader_time.csv");
  WriteFile(
    path,
    "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz\n"
    "1,2,3,4,0.1,0.2,0.3,0.01,0.02,0.03,1e-3,2e-3,3e-3,4e-4,5e-4,6e-4\n"
    "1,5,6,7,0.4,0.5,0.6,0.04,0.05,0.06,4e-3,5e-3,6e-3,7e-4,8e-4,9e-4\n");

  bool threw = false;
  try {
    (void)offline_lc_minimal::ReadTrajectoryCsv(path);
  } catch (const std::exception &exception) {
    threw = std::string(exception.what()).find("strictly increasing") != std::string::npos;
  }
  ExpectTrue(threw, "reader should reject non-increasing time");
}

}  // namespace

int main() {
  try {
    RunTest("TestReadTrajectoryCsvParsesRowsByHeader", TestReadTrajectoryCsvParsesRowsByHeader);
    RunTest(
      "TestReadStage2VelocityReferenceBuildsReferenceStates",
      TestReadStage2VelocityReferenceBuildsReferenceStates);
    RunTest(
      "TestReadTrajectoryCsvRejectsMissingRequiredColumn",
      TestReadTrajectoryCsvRejectsMissingRequiredColumn);
    RunTest(
      "TestReadTrajectoryCsvRejectsNonIncreasingTime",
      TestReadTrajectoryCsvRejectsNonIncreasingTime);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
