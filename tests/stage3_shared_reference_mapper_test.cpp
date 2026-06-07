#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_lc_minimal/core/Stage3SharedReferenceMapper.h"

namespace {

constexpr double kLat0Rad = 0.5;
constexpr double kLon0Rad = 1.0;
constexpr double kEarthRadiusM = 6378137.0;

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

double LonAtDistance(const double s_m) {
  return kLon0Rad + s_m / (kEarthRadiusM * std::cos(kLat0Rad));
}

offline_lc_minimal::TrajectoryCsvRow MakeTrajectoryRow(
  const double s_m,
  const double up_m) {
  offline_lc_minimal::TrajectoryCsvRow row;
  row.trajectory.time_s = 10.0 + s_m;
  row.trajectory.enu_position_m = Eigen::Vector3d(s_m, 0.0, up_m);
  row.trajectory.enu_velocity_mps = Eigen::Vector3d(1.0, 0.0, 0.0);
  row.lat_rad = kLat0Rad;
  row.lon_rad = LonAtDistance(s_m);
  row.h_m = 100.0 + up_m;
  row.has_geodetic = true;
  return row;
}

void TestBuildStage3ReferenceFromSharedVerticalReference() {
  const offline_lc_minimal::OfflineRunnerConfig config =
    offline_lc_minimal::DefaultConfig();
  std::vector<offline_lc_minimal::TrajectoryCsvRow> stage2{
    MakeTrajectoryRow(0.0, 50.0),
    MakeTrajectoryRow(0.5, 51.0),
    MakeTrajectoryRow(1.0, 52.0),
  };
  std::vector<offline_lc_minimal::SharedReferenceLinePoint> line{
    {0.0, 0.0, 0.0, kLat0Rad, kLon0Rad, 100.0},
    {1.0, 1.0, 0.0, kLat0Rad, kLon0Rad, 100.0},
  };
  std::vector<offline_lc_minimal::SharedVerticalReferenceRow> shared{
    {0.0, 100.0, 0.02, "RTK", 1.0, 0.0, 1U},
    {1.0, 102.0, 0.04, "NAV_BRIDGE", 0.0, 1.0, 2U},
  };

  offline_lc_minimal::Stage3SharedReferenceMapRequest request;
  request.config = &config;
  request.stage2_trajectory = &stage2;
  request.shared_reference = &shared;
  request.shared_reference_line = &line;
  const auto reference =
    offline_lc_minimal::BuildStage3ReferenceFromSharedVerticalReference(request);

  ExpectTrue(reference.rows.size() == stage2.size(), "mapper should create one row per Stage2 state");
  ExpectNear(reference.rows[0].stage2_lowpass_up_m, 100.0, 1e-9, "first reference should match shared start");
  ExpectNear(reference.rows[1].stage2_lowpass_up_m, 101.0, 0.02, "middle reference should interpolate by s");
  ExpectNear(reference.rows[2].stage2_lowpass_up_m, 102.0, 0.02, "last reference should match shared end");
  ExpectNear(reference.rows[1].lowpass_delta_m, 50.0, 0.02, "delta should be shared minus Stage2 up");
}

void TestMapperRejectsMissingOrigin() {
  const offline_lc_minimal::OfflineRunnerConfig config =
    offline_lc_minimal::DefaultConfig();
  std::vector<offline_lc_minimal::TrajectoryCsvRow> stage2{
    MakeTrajectoryRow(0.0, 50.0),
  };
  std::vector<offline_lc_minimal::SharedReferenceLinePoint> line{
    {0.0, 0.0, 0.0},
    {1.0, 1.0, 0.0},
  };
  std::vector<offline_lc_minimal::SharedVerticalReferenceRow> shared{
    {0.0, 100.0, 0.02, "RTK", 1.0, 0.0, 1U},
    {1.0, 102.0, 0.02, "RTK", 1.0, 0.0, 1U},
  };

  offline_lc_minimal::Stage3SharedReferenceMapRequest request;
  request.config = &config;
  request.stage2_trajectory = &stage2;
  request.shared_reference = &shared;
  request.shared_reference_line = &line;
  bool threw = false;
  try {
    (void)offline_lc_minimal::BuildStage3ReferenceFromSharedVerticalReference(request);
  } catch (const std::exception &exception) {
    threw = std::string(exception.what()).find("origin_lat_rad") != std::string::npos;
  }
  ExpectTrue(threw, "mapper should reject shared reference lines without origin metadata");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestBuildStage3ReferenceFromSharedVerticalReference",
      TestBuildStage3ReferenceFromSharedVerticalReference);
    RunTest("TestMapperRejectsMissingOrigin", TestMapperRejectsMissingOrigin);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
