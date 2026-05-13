#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Rot3.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/SensorTypes.h"
#include "offline_lc_minimal/core/TrajectoryInitializer.h"

namespace {

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

void ExpectTrue(const bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

offline_lc_minimal::GnssSolutionSample MakeGnssSample(
  const double time_s,
  const Eigen::Vector3d &enu_position_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.enu_position_m = enu_position_m;
  sample.has_enu_position = true;
  sample.gnssfgo_type_code = 1;
  return sample;
}

void TestInitialYawOverrideReestimatesBias() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_yaw_override = true;
  config.initial_yaw_override_rad = M_PI / 2.0;
  config.prefer_imu_initial_yaw = false;
  config.static_alignment_duration_s = 1.0;
  config.stationary_window_s = 1.0;

  const Eigen::Vector3d earth_rate_enu(0.0, 3.0e-5, 2.0e-5);
  const gtsam::Rot3 overridden_rotation =
    gtsam::Rot3::Ypr(config.initial_yaw_override_rad, 0.0, 0.0);
  const Eigen::Vector3d expected_gyro_bias(1.0e-6, -2.0e-6, 3.0e-6);
  const Eigen::Vector3d mean_accel_body(0.0, 0.0, config.gravity_mps2);
  const Eigen::Vector3d mean_gyro_body =
    overridden_rotation.matrix().transpose() * earth_rate_enu + expected_gyro_bias;

  std::vector<offline_lc_minimal::ImuSample> imu_samples;
  for (const double time_s : {0.0, 0.5, 1.0}) {
    offline_lc_minimal::ImuSample sample;
    sample.time_s = time_s;
    sample.accel_mps2 = mean_accel_body;
    sample.gyro_radps = mean_gyro_body;
    imu_samples.push_back(sample);
  }

  const std::vector<offline_lc_minimal::GnssSolutionSample> gnss_samples{
    MakeGnssSample(0.0, Eigen::Vector3d(0.0, 0.0, 0.0)),
    MakeGnssSample(1.0, Eigen::Vector3d(1.0, 0.0, 0.0)),
  };

  const offline_lc_minimal::InitialPoseEstimate estimate =
    offline_lc_minimal::TrajectoryInitializer::Estimate(
      imu_samples,
      gnss_samples,
      0U,
      0.0,
      1.0,
      1.0,
      earth_rate_enu,
      {1U},
      config);

  ExpectTrue(estimate.yaw_source == "override", "yaw source should record override path");
  ExpectNear(estimate.yaw_rad, config.initial_yaw_override_rad, 1e-12, "yaw override should be applied");
  ExpectNear(estimate.roll_rad, 0.0, 1e-12, "roll should come from gravity alignment");
  ExpectNear(estimate.pitch_rad, 0.0, 1e-12, "pitch should come from gravity alignment");

  const Eigen::Vector3d estimated_acc_bias = estimate.imu_bias.accelerometer();
  const Eigen::Vector3d estimated_gyro_bias = estimate.imu_bias.gyroscope();
  ExpectNear(estimated_acc_bias.norm(), 0.0, 1e-12, "accelerometer bias should be recomputed");
  ExpectNear(
    (estimated_gyro_bias - expected_gyro_bias).norm(),
    0.0,
    1e-12,
    "gyro bias should be recomputed for the overridden yaw");
}

}  // namespace

int main() {
  try {
    TestInitialYawOverrideReestimatesBias();
  } catch (const std::exception &exception) {
    std::cerr << "trajectory_initializer_test failed: " << exception.what() << '\n';
    return 1;
  }
  std::cout << "trajectory_initializer_test passed\n";
  return 0;
}
