#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/factor/ReweightedCombinedImuFactor.h"

namespace {

using offline_lc_minimal::DefaultConfig;
using offline_lc_minimal::LoadConfigFile;

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

void ExpectNear(const double actual, const double expected, const double tolerance, const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
  }
}

gtsam::PreintegratedCombinedMeasurements MakePreintegratedMeasurements() {
  auto params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  params->accelerometerCovariance = 1e-4 * gtsam::I_3x3;
  params->gyroscopeCovariance = 1e-6 * gtsam::I_3x3;
  params->integrationCovariance = 1e-8 * gtsam::I_3x3;
  params->biasAccCovariance = 1e-10 * gtsam::I_3x3;
  params->biasOmegaCovariance = 1e-12 * gtsam::I_3x3;
  params->biasAccOmegaInt = gtsam::Matrix66::Zero();

  gtsam::imuBias::ConstantBias bias;
  gtsam::PreintegratedCombinedMeasurements measurements(params, bias);
  for (int sample_index = 0; sample_index < 5; ++sample_index) {
    measurements.integrateMeasurement(
      gtsam::Vector3(0.05, -0.02, 9.81),
      gtsam::Vector3(0.001, -0.002, 0.0005),
      0.01);
  }
  return measurements;
}

gtsam::Matrix CovarianceFromFactor(
  const offline_lc_minimal::factor::ReweightedCombinedImuFactor &factor) {
  const auto gaussian =
    boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(factor.noiseModel());
  ExpectTrue(static_cast<bool>(gaussian), "expected Gaussian noise model");
  return gaussian->covariance();
}

void TestZeroSpecificForcePreservesTranslationalCovariance() {
  const auto measurements = MakePreintegratedMeasurements();
  const gtsam::Matrix base_covariance = measurements.preintMeasCov();
  const offline_lc_minimal::factor::ReweightedCombinedImuFactor factor(
    1,
    2,
    3,
    4,
    5,
    6,
    measurements,
    1e6,
    gtsam::Vector3::Zero());
  const gtsam::Matrix reweighted_covariance = CovarianceFromFactor(factor);

  for (int row = 3; row <= 8; ++row) {
    ExpectNear(
      reweighted_covariance(row, row),
      base_covariance(row, row),
      1e-10,
      "zero specific-force sigma should preserve translational covariance");
  }
}

void TestOnlyZAxisTightensZBlocks() {
  const auto measurements = MakePreintegratedMeasurements();
  const offline_lc_minimal::factor::ReweightedCombinedImuFactor factor(
    1,
    2,
    3,
    4,
    5,
    6,
    measurements,
    1e6,
    gtsam::Vector3(0.0, 0.0, 1e-9));
  const gtsam::Matrix reweighted_covariance = CovarianceFromFactor(factor);
  const gtsam::Matrix base_covariance = measurements.preintMeasCov();

  ExpectNear(reweighted_covariance(3, 3), base_covariance(3, 3), 1e-10, "position x should stay unchanged");
  ExpectNear(reweighted_covariance(4, 4), base_covariance(4, 4), 1e-10, "position y should stay unchanged");
  ExpectNear(reweighted_covariance(6, 6), base_covariance(6, 6), 1e-10, "velocity x should stay unchanged");
  ExpectNear(reweighted_covariance(7, 7), base_covariance(7, 7), 1e-10, "velocity y should stay unchanged");
  ExpectTrue(
    reweighted_covariance(5, 5) < base_covariance(5, 5),
    "specific-force z sigma should tighten vertical position covariance");
  ExpectTrue(
    reweighted_covariance(8, 8) < base_covariance(8, 8),
    "specific-force z sigma should tighten vertical velocity covariance");
}

void TestLegacySpecificForceAliasMapsToAllAxes() {
  const std::filesystem::path temp_path =
    std::filesystem::temp_directory_path() / "reweighted_specific_force_legacy_alias.cfg";
  {
    std::ofstream stream(temp_path);
    stream << "reweighted_combined_imu_specific_force_sigma_mps2=0.01\n";
  }

  const auto config = LoadConfigFile(temp_path.string(), DefaultConfig());
  std::filesystem::remove(temp_path);

  ExpectNear(
    config.reweighted_combined_imu_specific_force_sigma_x_mps2,
    0.01,
    1e-12,
    "legacy scalar should map to x axis");
  ExpectNear(
    config.reweighted_combined_imu_specific_force_sigma_y_mps2,
    0.01,
    1e-12,
    "legacy scalar should map to y axis");
  ExpectNear(
    config.reweighted_combined_imu_specific_force_sigma_z_mps2,
    0.01,
    1e-12,
    "legacy scalar should map to z axis");
}

void TestAxisSpecificKeysOverrideLegacyScalar() {
  const std::filesystem::path temp_path =
    std::filesystem::temp_directory_path() / "reweighted_specific_force_axis_override.cfg";
  {
    std::ofstream stream(temp_path);
    stream << "reweighted_combined_imu_specific_force_sigma_mps2=0.02\n";
    stream << "reweighted_combined_imu_specific_force_sigma_z_mps2=0.01\n";
  }

  const auto config = LoadConfigFile(temp_path.string(), DefaultConfig());
  std::filesystem::remove(temp_path);

  ExpectNear(
    config.reweighted_combined_imu_specific_force_sigma_x_mps2,
    0.0,
    1e-12,
    "axis-specific override should ignore legacy scalar for x");
  ExpectNear(
    config.reweighted_combined_imu_specific_force_sigma_y_mps2,
    0.0,
    1e-12,
    "axis-specific override should ignore legacy scalar for y");
  ExpectNear(
    config.reweighted_combined_imu_specific_force_sigma_z_mps2,
    0.01,
    1e-12,
    "axis-specific override should keep explicit z value");
}

void TestNonZeroSpecificForceXYLoads() {
  const std::filesystem::path temp_path =
    std::filesystem::temp_directory_path() / "reweighted_specific_force_xy_loads.cfg";
  {
    std::ofstream stream(temp_path);
    stream << "reweighted_combined_imu_specific_force_sigma_x_mps2=0.01\n";
  }

  const auto config = LoadConfigFile(temp_path.string(), DefaultConfig());
  std::filesystem::remove(temp_path);

  ExpectNear(
    config.reweighted_combined_imu_specific_force_sigma_x_mps2,
    0.01,
    1e-12,
    "specific-force x sigma should load now that axis weighting is implemented");
}

}  // namespace

int main() {
  try {
    RunTest("TestZeroSpecificForcePreservesTranslationalCovariance", TestZeroSpecificForcePreservesTranslationalCovariance);
    RunTest("TestOnlyZAxisTightensZBlocks", TestOnlyZAxisTightensZBlocks);
    RunTest("TestLegacySpecificForceAliasMapsToAllAxes", TestLegacySpecificForceAliasMapsToAllAxes);
    RunTest("TestAxisSpecificKeysOverrideLegacyScalar", TestAxisSpecificKeysOverrideLegacyScalar);
    RunTest("TestNonZeroSpecificForceXYLoads", TestNonZeroSpecificForceXYLoads);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
