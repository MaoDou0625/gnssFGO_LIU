#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace {

template <typename Function>
void RunTest(const std::string &name, Function &&function) {
  try {
    function();
  } catch (const std::exception &exception) {
    throw std::runtime_error(name + ": " + exception.what());
  }
}

void ExpectNear(const double actual, const double expected, const double tolerance, const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
  }
}

void TestVerticalPositionFactorUsesOnlyUp() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalPositionFactor factor(1, 3.0, noise);

  const gtsam::Pose3 pose_a(gtsam::Rot3::RzRyRx(0.1, -0.2, 0.3), gtsam::Point3(10.0, -20.0, 3.25));
  const gtsam::Pose3 pose_b(gtsam::Rot3::RzRyRx(-0.2, 0.4, -0.1), gtsam::Point3(-7.0, 6.0, 3.25));

  ExpectNear(factor.evaluateError(pose_a)(0), 0.25, 1e-12, "unexpected vertical residual");
  ExpectNear(factor.evaluateError(pose_b)(0), 0.25, 1e-12, "horizontal translation changed vertical residual");
}

void TestVerticalEnvelopeFactorUsesSoftGate() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalEnvelopeFactor factor(1, 3.0, 0.10, noise);

  const gtsam::Pose3 inside_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.05));
  const gtsam::Pose3 high_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.25));
  const gtsam::Pose3 low_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 2.75));

  ExpectNear(factor.evaluateError(inside_pose)(0), 0.0, 1e-12, "inside envelope residual should be zero");
  ExpectNear(factor.evaluateError(high_pose)(0), 0.15, 1e-12, "positive envelope violation is wrong");
  ExpectNear(factor.evaluateError(low_pose)(0), -0.15, 1e-12, "negative envelope violation is wrong");
}

void TestVerticalEnvelopeCenterPullFactorUsesClampedCenterResidual() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalEnvelopeCenterPullFactor factor(1, 3.0, 0.10, noise);

  const gtsam::Pose3 inside_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.05));
  const gtsam::Pose3 high_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.25));
  const gtsam::Pose3 low_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 2.75));

  ExpectNear(factor.evaluateError(inside_pose)(0), 0.05, 1e-12, "inside gate center residual should equal raw residual");
  ExpectNear(factor.evaluateError(high_pose)(0), 0.10, 1e-12, "positive center residual should clamp at half-width");
  ExpectNear(factor.evaluateError(low_pose)(0), -0.10, 1e-12, "negative center residual should clamp at half-width");
}

void TestGpInterpolatedVerticalFactorMatchesInterpolator() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const auto qc_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));
  const offline_lc_minimal::gp::GPWNOJInterpolator interpolator(qc_model, 1.0, 0.4);

  const gtsam::Pose3 pose_i(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.03), gtsam::Point3(2.0, 3.0, 1.0));
  const gtsam::Pose3 pose_j(gtsam::Rot3::RzRyRx(0.04, -0.01, -0.02), gtsam::Point3(5.0, -1.0, 4.0));
  const gtsam::Vector3 vel_i(0.2, -0.1, 0.3);
  const gtsam::Vector3 vel_j(-0.3, 0.4, -0.2);
  const gtsam::Vector3 omega_i(0.01, -0.02, 0.03);
  const gtsam::Vector3 omega_j(-0.02, 0.01, -0.01);

  const gtsam::Pose3 interpolated_pose =
    interpolator.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  const offline_lc_minimal::factor::GPInterpolatedVerticalPositionFactor factor(
    1, 2, 3, 4, 5, 6, interpolated_pose.translation().z(), interpolator, noise);

  ExpectNear(
    factor.evaluateError(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j)(0),
    0.0,
    1e-10,
    "interpolated vertical residual should match GP pose interpolation");
}

void TestGpInterpolatedVerticalEnvelopeFactorMatchesInterpolator() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const auto qc_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));
  const offline_lc_minimal::gp::GPWNOJInterpolator interpolator(qc_model, 1.0, 0.4);

  const gtsam::Pose3 pose_i(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.03), gtsam::Point3(2.0, 3.0, 1.0));
  const gtsam::Pose3 pose_j(gtsam::Rot3::RzRyRx(0.04, -0.01, -0.02), gtsam::Point3(5.0, -1.0, 4.0));
  const gtsam::Vector3 vel_i(0.2, -0.1, 0.3);
  const gtsam::Vector3 vel_j(-0.3, 0.4, -0.2);
  const gtsam::Vector3 omega_i(0.01, -0.02, 0.03);
  const gtsam::Vector3 omega_j(-0.02, 0.01, -0.01);

  const gtsam::Pose3 interpolated_pose =
    interpolator.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  const offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeFactor inside_factor(
    1, 2, 3, 4, 5, 6, interpolated_pose.translation().z() - 0.05, 0.10, interpolator, noise);
  const offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeFactor outside_factor(
    1, 2, 3, 4, 5, 6, interpolated_pose.translation().z() - 0.30, 0.10, interpolator, noise);

  ExpectNear(
    inside_factor.evaluateError(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j)(0),
    0.0,
    1e-10,
    "interpolated pose inside envelope should have zero residual");
  ExpectNear(
    outside_factor.evaluateError(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j)(0),
    0.20,
    1e-10,
    "interpolated pose outside envelope should report signed violation");
}

void TestGpInterpolatedVerticalEnvelopeCenterPullFactorMatchesInterpolator() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const auto qc_model = gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(10000.0));
  const offline_lc_minimal::gp::GPWNOJInterpolator interpolator(qc_model, 1.0, 0.4);

  const gtsam::Pose3 pose_i(gtsam::Rot3::RzRyRx(0.01, -0.02, 0.03), gtsam::Point3(2.0, 3.0, 1.0));
  const gtsam::Pose3 pose_j(gtsam::Rot3::RzRyRx(0.04, -0.01, -0.02), gtsam::Point3(5.0, -1.0, 4.0));
  const gtsam::Vector3 vel_i(0.2, -0.1, 0.3);
  const gtsam::Vector3 vel_j(-0.3, 0.4, -0.2);
  const gtsam::Vector3 omega_i(0.01, -0.02, 0.03);
  const gtsam::Vector3 omega_j(-0.02, 0.01, -0.01);

  const gtsam::Pose3 interpolated_pose =
    interpolator.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  const offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeCenterPullFactor inside_factor(
    1, 2, 3, 4, 5, 6, interpolated_pose.translation().z() - 0.05, 0.10, interpolator, noise);
  const offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeCenterPullFactor outside_factor(
    1, 2, 3, 4, 5, 6, interpolated_pose.translation().z() - 0.30, 0.10, interpolator, noise);

  ExpectNear(
    inside_factor.evaluateError(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j)(0),
    0.05,
    1e-10,
    "interpolated center residual inside gate should equal raw residual");
  ExpectNear(
    outside_factor.evaluateError(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j)(0),
    0.10,
    1e-10,
    "interpolated center residual outside gate should clamp at half-width");
}

}  // namespace

int main() {
  try {
    RunTest("TestVerticalPositionFactorUsesOnlyUp", TestVerticalPositionFactorUsesOnlyUp);
    RunTest("TestVerticalEnvelopeFactorUsesSoftGate", TestVerticalEnvelopeFactorUsesSoftGate);
    RunTest(
      "TestVerticalEnvelopeCenterPullFactorUsesClampedCenterResidual",
      TestVerticalEnvelopeCenterPullFactorUsesClampedCenterResidual);
    RunTest("TestGpInterpolatedVerticalFactorMatchesInterpolator", TestGpInterpolatedVerticalFactorMatchesInterpolator);
    RunTest(
      "TestGpInterpolatedVerticalEnvelopeFactorMatchesInterpolator",
      TestGpInterpolatedVerticalEnvelopeFactorMatchesInterpolator);
    RunTest(
      "TestGpInterpolatedVerticalEnvelopeCenterPullFactorMatchesInterpolator",
      TestGpInterpolatedVerticalEnvelopeCenterPullFactorMatchesInterpolator);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
