#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/Values.h>

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
  const offline_lc_minimal::factor::VerticalEnvelopeCenterPullFactor factor(1, 3.0, 0.10, 0.01, noise);

  const gtsam::Pose3 deadband_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.005));
  const gtsam::Pose3 inside_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.05));
  const gtsam::Pose3 high_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.25));
  const gtsam::Pose3 low_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 2.75));

  ExpectNear(factor.evaluateError(deadband_pose)(0), 0.0, 1e-12, "center residual should ignore deadband");
  ExpectNear(factor.evaluateError(inside_pose)(0), 0.04, 1e-12, "inside gate center residual should subtract deadband");
  ExpectNear(factor.evaluateError(high_pose)(0), 0.09, 1e-12, "positive center residual should clamp at half-width minus deadband");
  ExpectNear(factor.evaluateError(low_pose)(0), -0.09, 1e-12, "negative center residual should clamp at half-width minus deadband");
}

void TestVerticalEnvelopeCenterPullFactorSupportsZeroDeadband() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalEnvelopeCenterPullFactor factor(1, 3.0, 0.10, 0.0, noise);

  const gtsam::Pose3 inside_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.05));
  const gtsam::Pose3 high_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.25));
  const gtsam::Pose3 low_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 2.75));

  ExpectNear(factor.evaluateError(inside_pose)(0), 0.05, 1e-12, "zero-deadband center residual should keep raw residual inside gate");
  ExpectNear(factor.evaluateError(high_pose)(0), 0.10, 1e-12, "zero-deadband positive center residual should clamp at half-width");
  ExpectNear(factor.evaluateError(low_pose)(0), -0.10, 1e-12, "zero-deadband negative center residual should clamp at half-width");
}

void TestRtkVerticalReferenceMeasurementAndSmoothnessFactors() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::RtkVerticalReferenceMeasurementFactor measurement_factor(
    1,
    3.0,
    noise);
  const offline_lc_minimal::factor::RtkVerticalReferenceSmoothnessFactor smoothness_factor(
    1,
    2,
    noise);

  gtsam::Matrix H;
  ExpectNear(measurement_factor.evaluateError(3.25, H)(0), 0.25, 1e-12, "latent measurement residual is wrong");
  ExpectNear(H(0, 0), 1.0, 1e-12, "latent measurement Jacobian is wrong");

  gtsam::Matrix H1;
  gtsam::Matrix H2;
  ExpectNear(smoothness_factor.evaluateError(3.0, 3.4, H1, H2)(0), 0.4, 1e-12, "latent smoothness residual is wrong");
  ExpectNear(H1(0, 0), -1.0, 1e-12, "latent smoothness first Jacobian is wrong");
  ExpectNear(H2(0, 0), 1.0, 1e-12, "latent smoothness second Jacobian is wrong");
}

void TestVerticalEnvelopeLatentFactorsUseReferenceVariable() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalEnvelopeLatentReferenceFactor gate_factor(
    1,
    2,
    0.10,
    noise);
  const offline_lc_minimal::factor::VerticalEnvelopeLatentCenterPullFactor center_factor(
    1,
    2,
    0.10,
    0.0,
    noise);
  const gtsam::Pose3 pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(10.0, -20.0, 3.25));

  ExpectNear(gate_factor.evaluateError(pose, 3.20)(0), 0.0, 1e-12, "latent gate inside envelope should be zero");
  ExpectNear(gate_factor.evaluateError(pose, 3.00)(0), 0.15, 1e-12, "latent gate outside envelope is wrong");
  ExpectNear(center_factor.evaluateError(pose, 3.20)(0), 0.05, 1e-12, "latent center pull should use reference variable");
  ExpectNear(center_factor.evaluateError(pose, 3.00)(0), 0.10, 1e-12, "latent center pull should clamp at gate");
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
    1, 2, 3, 4, 5, 6, interpolated_pose.translation().z() - 0.05, 0.10, 0.01, interpolator, noise);
  const offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeCenterPullFactor outside_factor(
    1, 2, 3, 4, 5, 6, interpolated_pose.translation().z() - 0.30, 0.10, 0.01, interpolator, noise);

  ExpectNear(
    inside_factor.evaluateError(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j)(0),
    0.04,
    1e-10,
    "interpolated center residual inside gate should subtract deadband");
  ExpectNear(
    outside_factor.evaluateError(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j)(0),
    0.09,
    1e-10,
    "interpolated center residual outside gate should clamp at half-width minus deadband");
}

void TestGpInterpolatedLatentFactorsUseBinReferenceWithoutInterpolatingReference() {
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
  const offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeLatentReferenceFactor gate_factor(
    1, 2, 3, 4, 5, 6, 7, 0.10, interpolator, noise);
  const offline_lc_minimal::factor::GPInterpolatedVerticalEnvelopeLatentCenterPullFactor center_factor(
    1, 2, 3, 4, 5, 6, 7, 0.10, 0.0, interpolator, noise);

  gtsam::Values values;
  values.insert(1, pose_i);
  values.insert(2, vel_i);
  values.insert(3, omega_i);
  values.insert(4, pose_j);
  values.insert(5, vel_j);
  values.insert(6, omega_j);
  values.insert(7, interpolated_pose.translation().z() - 0.30);

  ExpectNear(gate_factor.unwhitenedError(values)(0), 0.20, 1e-10, "GP latent gate should use scalar reference");
  ExpectNear(center_factor.unwhitenedError(values)(0), 0.10, 1e-10, "GP latent center should clamp against scalar reference");
  values.update(7, interpolated_pose.translation().z() - 0.05);
  ExpectNear(gate_factor.unwhitenedError(values)(0), 0.0, 1e-10, "GP latent gate should update with reference variable");
  ExpectNear(center_factor.unwhitenedError(values)(0), 0.05, 1e-10, "GP latent center should update with reference variable");
}

}  // namespace

int main() {
  try {
    RunTest("TestVerticalPositionFactorUsesOnlyUp", TestVerticalPositionFactorUsesOnlyUp);
    RunTest("TestVerticalEnvelopeFactorUsesSoftGate", TestVerticalEnvelopeFactorUsesSoftGate);
    RunTest(
      "TestVerticalEnvelopeCenterPullFactorUsesClampedCenterResidual",
      TestVerticalEnvelopeCenterPullFactorUsesClampedCenterResidual);
    RunTest(
      "TestVerticalEnvelopeCenterPullFactorSupportsZeroDeadband",
      TestVerticalEnvelopeCenterPullFactorSupportsZeroDeadband);
    RunTest(
      "TestRtkVerticalReferenceMeasurementAndSmoothnessFactors",
      TestRtkVerticalReferenceMeasurementAndSmoothnessFactors);
    RunTest(
      "TestVerticalEnvelopeLatentFactorsUseReferenceVariable",
      TestVerticalEnvelopeLatentFactorsUseReferenceVariable);
    RunTest("TestGpInterpolatedVerticalFactorMatchesInterpolator", TestGpInterpolatedVerticalFactorMatchesInterpolator);
    RunTest(
      "TestGpInterpolatedVerticalEnvelopeFactorMatchesInterpolator",
      TestGpInterpolatedVerticalEnvelopeFactorMatchesInterpolator);
    RunTest(
      "TestGpInterpolatedVerticalEnvelopeCenterPullFactorMatchesInterpolator",
      TestGpInterpolatedVerticalEnvelopeCenterPullFactorMatchesInterpolator);
    RunTest(
      "TestGpInterpolatedLatentFactorsUseBinReferenceWithoutInterpolatingReference",
      TestGpInterpolatedLatentFactorsUseBinReferenceWithoutInterpolatingReference);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
