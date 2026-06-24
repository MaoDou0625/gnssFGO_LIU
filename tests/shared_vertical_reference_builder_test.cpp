#include <cmath>
#include <iostream>
#include <numbers>
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
  const double height_m,
  const double lateral_m = 0.0,
  const double vz_mps = 0.0) {
  offline_lc_minimal::TrajectoryCsvRow row;
  row.trajectory.time_s = s_m;
  row.trajectory.enu_position_m = Eigen::Vector3d(s_m, lateral_m, height_m - 100.0);
  row.trajectory.enu_velocity_mps = Eigen::Vector3d(1.0, 0.0, vz_mps);
  row.lat_rad = kLat0Rad + lateral_m / kEarthRadiusM;
  row.lon_rad = LonAtDistance(s_m);
  row.h_m = height_m;
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

offline_lc_minimal::SharedVerticalReferenceMember MakeMemberFromHeights(
  const std::string &id,
  const std::vector<double> &heights_m,
  const double lateral_m,
  const std::vector<bool> *keep_sample = nullptr,
  const std::vector<double> *vz_mps_by_sample = nullptr) {
  offline_lc_minimal::SharedVerticalReferenceMember member;
  member.member_id = id;
  member.config = offline_lc_minimal::DefaultConfig();
  member.config.required_best_sol_status_code = 1;
  member.config.drop_nonfinite_sigma = true;
  for (std::size_t index = 0; index < heights_m.size(); ++index) {
    if (keep_sample != nullptr && index < keep_sample->size() && !(*keep_sample)[index]) {
      continue;
    }
    member.trajectory.push_back(
      MakeTrajectoryCsvRow(
        static_cast<double>(index),
        heights_m[index],
        lateral_m,
        vz_mps_by_sample != nullptr && index < vz_mps_by_sample->size()
          ? (*vz_mps_by_sample)[index]
          : 0.0));
  }
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

void TestSharedReferenceUsesStage2NavOnlyBridge() {
  offline_lc_minimal::SharedVerticalReferenceBuildRequest request;
  request.grid_spacing_m = 1.0;
  request.sigma_m = 0.02;
  request.members.push_back(MakeMember("A", 5.0, 0.0));
  request.members.push_back(MakeMember("B", -3.0, 0.2));

  const auto reference =
    offline_lc_minimal::BuildSharedVerticalReference(std::move(request));

  ExpectTrue(reference.rows.size() >= 5U, "shared reference should cover the longest member");
  ExpectNear(reference.rows[0].reference_up_m, 101.0, 0.02, "start should use Stage2 nav median");
  ExpectNear(reference.rows[4].reference_up_m, 105.0, 0.02, "end should use Stage2 nav median");
  ExpectTrue(
    reference.rows[0].source == "TRUSTED_TEXTURE_FUSION",
    "start source should use trusted Stage2 texture fusion");
  ExpectTrue(
    reference.rows[4].source == "TRUSTED_TEXTURE_FUSION",
    "end source should use trusted Stage2 texture fusion");
  ExpectNear(reference.rows[2].reference_up_m, 103.0, 0.03, "middle should use Stage2 nav median");
  ExpectTrue(
    reference.rows[2].rtk_weight == 0.0,
    "shared reference should not assign RTK weight");
}

void TestTrustedTextureFusionPrefersLowerVelocityDeltaSource() {
  std::vector<double> smooth_heights;
  std::vector<double> disturbed_heights;
  std::vector<double> smooth_vz_mps;
  std::vector<double> disturbed_vz_mps;
  for (int s = 0; s <= 120; ++s) {
    const double s_m = static_cast<double>(s);
    const double base_height_m =
      100.0 + 0.02 * s_m + 0.01 * std::sin(s_m / 7.0);
    double disturbed_vz_mps_value = 0.0;
    double disturbed_height_m = base_height_m;
    if (s >= 40 && s <= 55) {
      const double phase =
        (s_m - 40.0) / 15.0 * std::numbers::pi;
      disturbed_height_m += 0.12 * std::sin(phase);
      disturbed_vz_mps_value = 0.06 * std::sin(phase);
    }
    smooth_heights.push_back(base_height_m);
    disturbed_heights.push_back(disturbed_height_m);
    smooth_vz_mps.push_back(0.0);
    disturbed_vz_mps.push_back(disturbed_vz_mps_value);
  }

  offline_lc_minimal::SharedVerticalReferenceBuildRequest request;
  request.grid_spacing_m = 1.0;
  request.sigma_m = 0.02;
  request.trusted_texture_lowpass_radius_m = 20.0;
  request.trusted_texture_source_margin_min = 0.03;
  request.members.push_back(
    MakeMemberFromHeights("disturbed", disturbed_heights, 0.0, nullptr, &disturbed_vz_mps));
  request.members.push_back(
    MakeMemberFromHeights("smooth", smooth_heights, 0.2, nullptr, &smooth_vz_mps));

  const auto reference =
    offline_lc_minimal::BuildSharedVerticalReference(std::move(request));

  const std::size_t check_index = 48U;
  const double fused_height = reference.rows[check_index].reference_up_m;
  const double disturbance_m =
    std::abs(disturbed_heights[check_index] - smooth_heights[check_index]);
  ExpectTrue(
    std::abs(fused_height - smooth_heights[check_index]) <
      0.20 * disturbance_m,
    "trusted texture fusion should strongly move the local valley toward the lower-dvz member");
  ExpectTrue(
    std::abs(fused_height - smooth_heights[check_index]) <
      std::abs(fused_height - disturbed_heights[check_index]),
    "trusted texture fusion should be closer to the lower-dvz member");
}

void TestTrustedTextureFusionDoesNotTrustLongInterpolatedGap() {
  std::vector<double> measured_heights;
  std::vector<double> gap_heights;
  std::vector<bool> gap_keep_sample;
  for (int s = 0; s <= 120; ++s) {
    const double s_m = static_cast<double>(s);
    const double base_height_m =
      100.0 + 0.015 * s_m + 0.008 * std::sin(s_m / 8.0);
    double measured_height_m = base_height_m;
    if (s >= 40 && s <= 55) {
      const double phase =
        (s_m - 40.0) / 15.0 * std::numbers::pi;
      measured_height_m += 0.10 * std::sin(phase);
    }
    measured_heights.push_back(measured_height_m);
    gap_heights.push_back(base_height_m);
    gap_keep_sample.push_back(!(s >= 36 && s <= 60));
  }

  offline_lc_minimal::SharedVerticalReferenceBuildRequest request;
  request.grid_spacing_m = 1.0;
  request.sigma_m = 0.02;
  request.trusted_texture_lowpass_radius_m = 20.0;
  request.trusted_texture_source_margin_min = 0.03;
  request.members.push_back(MakeMemberFromHeights("measured", measured_heights, 0.0));
  request.members.push_back(
    MakeMemberFromHeights("gap", gap_heights, 0.2, &gap_keep_sample));

  const auto reference =
    offline_lc_minimal::BuildSharedVerticalReference(std::move(request));

  const std::size_t check_index = 48U;
  const double fused_height = reference.rows[check_index].reference_up_m;
  ExpectTrue(
    std::abs(fused_height - measured_heights[check_index]) <
      std::abs(fused_height - gap_heights[check_index]),
    "trusted texture fusion should not prefer a long interpolated member gap");
}

void TestSharedReferenceIgnoresRtkSamples() {
  offline_lc_minimal::SharedVerticalReferenceBuildRequest request;
  request.grid_spacing_m = 1.0;
  request.sigma_m = 0.02;
  request.members.push_back(MakeMember("A", 0.0, 0.0));
  request.members.push_back(MakeMember("B", 0.0, 0.2));
  const auto without_extra_rtk =
    offline_lc_minimal::BuildSharedVerticalReference(request);

  request.members.front().gnss_samples.push_back(MakeRtkSample(1.0, 101.0, 1.0));
  request.members.front().gnss_samples.push_back(MakeRtkSample(2.0, 112.20, 2.0));
  request.members.front().gnss_samples.push_back(MakeRtkSample(3.0, 103.0, 3.0));

  const auto with_extra_rtk =
    offline_lc_minimal::BuildSharedVerticalReference(std::move(request));

  ExpectTrue(
    without_extra_rtk.rows.size() == with_extra_rtk.rows.size(),
    "RTK samples should not change reference row count");
  ExpectNear(
    with_extra_rtk.rows[2].reference_up_m,
    without_extra_rtk.rows[2].reference_up_m,
    1.0e-12,
    "RTK samples should not change shared reference height");
  ExpectTrue(
    with_extra_rtk.rows[2].rtk_weight == 0.0,
    "RTK samples should not receive shared reference weight");
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
      "TestSharedReferenceUsesStage2NavOnlyBridge",
      TestSharedReferenceUsesStage2NavOnlyBridge);
    RunTest(
      "TestSharedReferenceIgnoresRtkSamples",
      TestSharedReferenceIgnoresRtkSamples);
    RunTest(
      "TestTrustedTextureFusionPrefersLowerVelocityDeltaSource",
      TestTrustedTextureFusionPrefersLowerVelocityDeltaSource);
    RunTest(
      "TestTrustedTextureFusionDoesNotTrustLongInterpolatedGap",
      TestTrustedTextureFusionDoesNotTrustLongInterpolatedGap);
    RunTest("TestSharedReferenceRejectsSingleMember", TestSharedReferenceRejectsSingleMember);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
