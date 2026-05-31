#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "offline_lc_minimal/core/StageAttitudeReference.h"

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

offline_lc_minimal::ReferenceNodeState MakeState(
  const double time_s,
  const double yaw_rad) {
  offline_lc_minimal::ReferenceNodeState state;
  state.time_s = time_s;
  state.pose = gtsam::Pose3(
    gtsam::Rot3::Ypr(yaw_rad, 0.0, 0.0),
    gtsam::Point3(time_s, 0.0, 0.0));
  return state;
}

double RelativeAngle(
  const offline_lc_minimal::ReferenceNodeState &lhs,
  const offline_lc_minimal::ReferenceNodeState &rhs) {
  return gtsam::Rot3::Logmap(lhs.pose.rotation().between(rhs.pose.rotation())).norm();
}

void TestInterpolatesRotationAcrossYawWrapWithShortestRotation() {
  const double deg = M_PI / 180.0;
  const std::vector<offline_lc_minimal::ReferenceNodeState> states{
    MakeState(0.0, 179.0 * deg),
    MakeState(1.0, -179.0 * deg)};

  const auto middle =
    offline_lc_minimal::InterpolateStageReferenceState(states, 0.5);

  ExpectNear(
    RelativeAngle(states.front(), states.back()),
    2.0 * deg,
    1.0e-12,
    "reference endpoints should be treated as a short rotation");
  ExpectNear(
    RelativeAngle(states.front(), middle),
    1.0 * deg,
    1.0e-12,
    "midpoint should move halfway along the short rotation");
}

void TestOutageReferenceUsesImuDeltaFromPreAnchor() {
  std::vector<offline_lc_minimal::ReferenceNodeState> base{
    MakeState(0.0, 1.0),
    MakeState(1.0, 0.0),
    MakeState(2.0, 0.0)};
  const std::vector<offline_lc_minimal::ReferenceNodeState> imu{
    MakeState(0.0, 1.0),
    MakeState(1.0, 1.01),
    MakeState(2.0, 1.02)};

  offline_lc_minimal::RtkOutageWindowRow window;
  window.window_index = 0U;
  window.pre_anchor_state_index = 0U;
  window.post_anchor_state_index = 2U;
  window.start_time_s = 0.0;
  window.end_time_s = 2.0;
  window.skip_reason = "PLANNED";

  const auto corrected =
    offline_lc_minimal::BuildImuDeltaOutageAttitudeReference(
      base,
      imu,
      std::vector<double>{0.0, 1.0, 2.0},
      std::vector<offline_lc_minimal::RtkOutageWindowRow>{window},
      0.0);

  ExpectNear(
    RelativeAngle(corrected[0], corrected[1]),
    0.01,
    1.0e-12,
    "outage reference should follow IMU delta instead of the base jump");
  ExpectNear(
    RelativeAngle(corrected[1], corrected[2]),
    0.01,
    1.0e-12,
    "outage reference should keep IMU attitude continuity");
  ExpectTrue(
    RelativeAngle(base[0], base[1]) > 0.9,
    "test fixture should contain a large base-reference jump");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestInterpolatesRotationAcrossYawWrapWithShortestRotation",
      TestInterpolatesRotationAcrossYawWrapWithShortestRotation);
    RunTest(
      "TestOutageReferenceUsesImuDeltaFromPreAnchor",
      TestOutageReferenceUsesImuDeltaFromPreAnchor);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
