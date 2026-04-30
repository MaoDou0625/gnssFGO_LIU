#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"

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

void ExpectNear(const double actual, const double expected, const double tolerance, const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
  }
}

offline_lc_minimal::ReferenceNodeState MakeReferenceNodeState(
  const double time_s,
  const gtsam::Pose3 &pose,
  const gtsam::Vector3 &velocity) {
  offline_lc_minimal::ReferenceNodeState state;
  state.time_s = time_s;
  state.pose = pose;
  state.velocity = velocity;
  state.bias = gtsam::imuBias::ConstantBias();
  state.omega = gtsam::Vector3::Zero();
  return state;
}

void ConfigureSensitiveDetector(offline_lc_minimal::OfflineRunnerConfig *config) {
  config->body_z_jump_min_score_mps = 0.003;
  config->body_z_jump_threshold_ratio = 0.35;
  config->body_z_jump_support_ratio = 0.25;
  config->body_z_jump_max_window_duration_s = 0.75;
  config->body_z_jump_min_separation_s = 0.1;
}

std::vector<offline_lc_minimal::ReferenceNodeState> MakeFlatSeedStates(
  std::vector<double> *state_timestamps,
  const int state_count) {
  std::vector<offline_lc_minimal::ReferenceNodeState> seed_states;
  for (int state_index = 0; state_index <= state_count; ++state_index) {
    const double time_s = 0.05 * static_cast<double>(state_index);
    state_timestamps->push_back(time_s);
    seed_states.push_back(MakeReferenceNodeState(time_s, gtsam::Pose3(), gtsam::Vector3::Zero()));
  }
  return seed_states;
}

void TestBodyZSeedJumpDetectorFindsDownwardWindow() {
  auto config = offline_lc_minimal::DefaultConfig();
  ConfigureSensitiveDetector(&config);

  std::vector<offline_lc_minimal::ImuSample> imu_samples;
  for (int sample_index = 0; sample_index <= 600; ++sample_index) {
    const double time_s = 0.005 * static_cast<double>(sample_index);
    offline_lc_minimal::ImuSample sample;
    sample.time_s = time_s;
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
    if (time_s >= 1.0 && time_s <= 1.10) {
      sample.accel_mps2.z() -= 1.0;
    }
    imu_samples.push_back(sample);
  }

  std::vector<double> state_timestamps;
  const auto seed_states = MakeFlatSeedStates(&state_timestamps, 60);
  offline_lc_minimal::BodyZBidirectionalJumpDetector detector(config);
  const auto detection = detector.Detect(imu_samples, seed_states, state_timestamps, 0.0, 3.0);
  const auto down_window_it = std::find_if(
    detection.windows.begin(),
    detection.windows.end(),
    [](const auto &window) { return window.direction == "DOWN"; });
  ExpectTrue(down_window_it != detection.windows.end(), "body-z detector should find the downward velocity step");
  ExpectTrue(
    down_window_it->center_time_s > 0.8 && down_window_it->center_time_s < 1.3,
    "body-z detector center should be near the injected pulse");
  ExpectTrue(
    down_window_it->signed_delta_velocity_mps < 0.0,
    "DOWN body-z window should have a negative signed velocity delta");
  ExpectTrue(
    down_window_it->delta_vz_init_mps > 0.0,
    "DOWN body-z window should initialize an upward nav-z velocity correction");
}

void TestBodyZSeedJumpDetectorMergesNearbyDownwardWindows() {
  auto config = offline_lc_minimal::DefaultConfig();
  ConfigureSensitiveDetector(&config);
  config.body_z_jump_merge_gap_s = 1.5;
  config.body_z_jump_merge_max_duration_s = 3.0;

  std::vector<offline_lc_minimal::ImuSample> imu_samples;
  for (int sample_index = 0; sample_index <= 800; ++sample_index) {
    const double time_s = 0.005 * static_cast<double>(sample_index);
    offline_lc_minimal::ImuSample sample;
    sample.time_s = time_s;
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
    if ((time_s >= 1.0 && time_s <= 1.10) ||
        (time_s >= 2.0 && time_s <= 2.10)) {
      sample.accel_mps2.z() -= 1.0;
    }
    imu_samples.push_back(sample);
  }

  std::vector<double> state_timestamps;
  const auto seed_states = MakeFlatSeedStates(&state_timestamps, 80);
  offline_lc_minimal::BodyZBidirectionalJumpDetector detector(config);
  const auto detection = detector.Detect(imu_samples, seed_states, state_timestamps, 0.0, 4.0);
  const auto down_window_count = std::count_if(
    detection.windows.begin(),
    detection.windows.end(),
    [](const auto &window) { return window.direction == "DOWN"; });
  ExpectNear(
    static_cast<double>(down_window_count),
    1.0,
    0.0,
    "nearby DOWN body-z windows should merge into one correction window");
}

void TestBodyZSeedJumpDetectorDoesNotMergeAcrossOppositeDirectionWindow() {
  auto config = offline_lc_minimal::DefaultConfig();
  ConfigureSensitiveDetector(&config);
  config.body_z_jump_merge_gap_s = 2.0;
  config.body_z_jump_merge_max_duration_s = 4.0;

  std::vector<offline_lc_minimal::ImuSample> imu_samples;
  for (int sample_index = 0; sample_index <= 800; ++sample_index) {
    const double time_s = 0.005 * static_cast<double>(sample_index);
    offline_lc_minimal::ImuSample sample;
    sample.time_s = time_s;
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
    if ((time_s >= 1.0 && time_s <= 1.10) ||
        (time_s >= 2.0 && time_s <= 2.10)) {
      sample.accel_mps2.z() -= 1.0;
    }
    if (time_s >= 1.5 && time_s <= 1.60) {
      sample.accel_mps2.z() += 1.0;
    }
    imu_samples.push_back(sample);
  }

  std::vector<double> state_timestamps;
  const auto seed_states = MakeFlatSeedStates(&state_timestamps, 80);
  offline_lc_minimal::BodyZBidirectionalJumpDetector detector(config);
  const auto detection = detector.Detect(imu_samples, seed_states, state_timestamps, 0.0, 4.0);
  const auto down_window_count = std::count_if(
    detection.windows.begin(),
    detection.windows.end(),
    [](const auto &window) { return window.direction == "DOWN"; });
  const auto up_window_count = std::count_if(
    detection.windows.begin(),
    detection.windows.end(),
    [](const auto &window) { return window.direction == "UP"; });
  ExpectTrue(up_window_count >= 1U, "opposite-direction body-z window should be detected");
  ExpectTrue(
    down_window_count >= 2U,
    "same-direction body-z windows should not merge across an opposite-direction window");
}

void TestBodyZSeedJumpDetectorUsesSeedAttitudeGravityProjection() {
  auto config = offline_lc_minimal::DefaultConfig();
  std::vector<offline_lc_minimal::ImuSample> imu_samples;
  for (int sample_index = 0; sample_index <= 10; ++sample_index) {
    offline_lc_minimal::ImuSample sample;
    sample.time_s = 0.01 * static_cast<double>(sample_index);
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81);
    imu_samples.push_back(sample);
  }

  const gtsam::Pose3 tilted_pose(gtsam::Rot3::RzRyRx(0.0, 0.2, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
  const std::vector<double> state_timestamps{0.0, 0.1};
  const std::vector<offline_lc_minimal::ReferenceNodeState> seed_states{
    MakeReferenceNodeState(0.0, tilted_pose, gtsam::Vector3::Zero()),
    MakeReferenceNodeState(0.1, tilted_pose, gtsam::Vector3::Zero()),
  };

  offline_lc_minimal::BodyZBidirectionalJumpDetector detector(config);
  const auto detection = detector.Detect(imu_samples, seed_states, state_timestamps, 0.0, 0.1);
  ExpectTrue(!detection.signal.empty(), "body-z detector should emit a signal");
  ExpectNear(
    detection.signal.front().gravity_projection_z_mps2,
    9.81 * std::cos(0.2),
    1e-9,
    "body-z detector should use seed pitch when projecting gravity");
}

}  // namespace

int main() {
  try {
    RunTest("TestBodyZSeedJumpDetectorFindsDownwardWindow", TestBodyZSeedJumpDetectorFindsDownwardWindow);
    RunTest("TestBodyZSeedJumpDetectorMergesNearbyDownwardWindows", TestBodyZSeedJumpDetectorMergesNearbyDownwardWindows);
    RunTest(
      "TestBodyZSeedJumpDetectorDoesNotMergeAcrossOppositeDirectionWindow",
      TestBodyZSeedJumpDetectorDoesNotMergeAcrossOppositeDirectionWindow);
    RunTest(
      "TestBodyZSeedJumpDetectorUsesSeedAttitudeGravityProjection",
      TestBodyZSeedJumpDetectorUsesSeedAttitudeGravityProjection);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
