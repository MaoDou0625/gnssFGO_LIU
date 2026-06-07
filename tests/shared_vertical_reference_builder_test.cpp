#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_lc_minimal/core/SharedVerticalReferenceBuilder.h"

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

offline_lc_minimal::TrajectoryCsvRow MakeTrajectoryCsvRow(
  const double s_m,
  const double up_m,
  const double lateral_m = 0.0) {
  offline_lc_minimal::TrajectoryCsvRow row;
  row.trajectory.time_s = s_m;
  row.trajectory.enu_position_m = Eigen::Vector3d(s_m, lateral_m, up_m);
  row.trajectory.enu_velocity_mps = Eigen::Vector3d(1.0, 0.0, 0.0);
  row.lat_rad = kLat0Rad + lateral_m / kEarthRadiusM;
  row.lon_rad = LonAtDistance(s_m);
  row.h_m = 100.0 + up_m;
  row.has_geodetic = true;
  return row;
}

offline_lc_minimal::GnssSolutionSample MakeRtkSample(
  const double s_m,
  const double height_m,
  const double time_s) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = kLat0Rad;
  sample.lon_rad = LonAtDistance(s_m);
  sample.h_m = height_m;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.01;
  sample.best_sol_status_code = 1;
  sample.gnssfgo_type_code = 1;
  return sample;
}

offline_lc_minimal::SharedVerticalReferenceMember MakeMember(
  const std::string &id,
  const double height_bias_m,
  const double lateral_m) {
  offline_lc_minimal::SharedVerticalReferenceMember member;
  member.member_id = id;
  member.config = offline_lc_minimal::DefaultConfig();
  member.config.required_best_sol_status_code = 1;
  member.config.drop_nonfinite_sigma = true;
  for (int s = 0; s <= 4; ++s) {
    const double true_height_m = 100.0 + static_cast<double>(s);
    member.trajectory.push_back(
      MakeTrajectoryCsvRow(
        static_cast<double>(s),
        true_height_m + height_bias_m,
        lateral_m));
  }
  member.gnss_samples.push_back(MakeRtkSample(0.0, 100.0, 0.0));
  member.gnss_samples.push_back(MakeRtkSample(4.0, 104.0, 4.0));
  return member;
}

void TestProjectPointToSharedReferenceLine() {
  std::vector<offline_lc_minimal::SharedReferenceLinePoint> line{
    {0.0, 0.0, 0.0},
    {10.0, 10.0, 0.0},
  };

  const auto projection =
    offline_lc_minimal::ProjectPointToSharedReferenceLine(
      Eigen::Vector2d(4.0, 3.0),
      line);

  ExpectTrue(projection.valid, "projection should be valid");
  ExpectNear(projection.s_m, 4.0, 1e-12, "projected distance should use segment length");
  ExpectNear(projection.lateral_offset_m, 3.0, 1e-12, "lateral offset should be signed distance");
}

void TestSharedReferenceKeepsRtkAndUsesDebiasedBridge() {
  offline_lc_minimal::SharedVerticalReferenceBuildRequest request;
  request.grid_spacing_m = 1.0;
  request.sigma_m = 0.02;
  request.members.push_back(MakeMember("A", 5.0, 0.0));
  request.members.push_back(MakeMember("B", -3.0, 0.2));

  const auto reference =
    offline_lc_minimal::BuildSharedVerticalReference(std::move(request));

  ExpectTrue(reference.rows.size() >= 5U, "shared reference should cover the longest member");
  ExpectNear(reference.rows[0].reference_up_m, 100.0, 0.02, "start should remain RTK dominated");
  ExpectNear(reference.rows[4].reference_up_m, 104.0, 0.02, "end should remain RTK dominated");
  ExpectTrue(reference.rows[0].source == "RTK", "start source should be RTK");
  ExpectTrue(reference.rows[4].source == "RTK", "end source should be RTK");
  ExpectNear(reference.rows[2].reference_up_m, 102.0, 0.03, "middle should use de-biased Stage2 bridge");
  ExpectTrue(reference.rows[2].source == "NAV_BRIDGE", "middle source should be nav bridge");
}

void TestSharedReferenceRejectsSingleMember() {
  offline_lc_minimal::SharedVerticalReferenceBuildRequest request;
  request.members.push_back(MakeMember("A", 0.0, 0.0));

  bool threw = false;
  try {
    (void)offline_lc_minimal::BuildSharedVerticalReference(std::move(request));
  } catch (const std::exception &exception) {
    threw = std::string(exception.what()).find("at least two") != std::string::npos;
  }
  ExpectTrue(threw, "shared reference should require at least two members");
}

}  // namespace

int main() {
  try {
    RunTest("TestProjectPointToSharedReferenceLine", TestProjectPointToSharedReferenceLine);
    RunTest(
      "TestSharedReferenceKeepsRtkAndUsesDebiasedBridge",
      TestSharedReferenceKeepsRtkAndUsesDebiasedBridge);
    RunTest("TestSharedReferenceRejectsSingleMember", TestSharedReferenceRejectsSingleMember);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
