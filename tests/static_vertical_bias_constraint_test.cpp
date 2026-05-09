#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/core/InitialStaticBiasConstraintBuilder.h"
#include "offline_lc_minimal/core/InitialStaticPositionConstraintBuilder.h"
#include "offline_lc_minimal/core/InitialStaticRtkHeightConstraintBuilder.h"
#include "offline_lc_minimal/factor/StaticPositionHoldFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalAccelBiasFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalPositionHoldFactor.h"

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

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

void TestStaticVerticalAccelBiasFactorUsesOnlyBaz() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::StaticVerticalAccelBiasFactor factor(
    B(0),
    gtsam::Symbol('a', 0),
    noise);
  const gtsam::imuBias::ConstantBias bias(
    gtsam::Vector3(10.0, -20.0, 0.003),
    gtsam::Vector3(1.0, 2.0, 3.0));
  const gtsam::Vector3 global_acc_bias(100.0, -200.0, 0.001);

  gtsam::Matrix h_bias;
  gtsam::Matrix h_global;
  const gtsam::Vector residual = factor.evaluateError(bias, global_acc_bias, h_bias, h_global);

  ExpectNear(residual(0), 0.002, 1e-12, "static vertical bias residual is wrong");
  ExpectNear(static_cast<double>(h_bias.rows()), 1.0, 0.0, "bias Jacobian row count is wrong");
  ExpectNear(static_cast<double>(h_bias.cols()), 6.0, 0.0, "bias Jacobian column count is wrong");
  ExpectNear(h_bias(0, 0), 0.0, 1e-12, "bias Jacobian should not touch bax");
  ExpectNear(h_bias(0, 1), 0.0, 1e-12, "bias Jacobian should not touch bay");
  ExpectNear(h_bias(0, 2), 1.0, 1e-12, "bias Jacobian should touch baz");
  ExpectNear(h_bias(0, 3), 0.0, 1e-12, "bias Jacobian should not touch bgx");
  ExpectNear(h_global(0, 0), 0.0, 1e-12, "global Jacobian should not touch x");
  ExpectNear(h_global(0, 1), 0.0, 1e-12, "global Jacobian should not touch y");
  ExpectNear(h_global(0, 2), -1.0, 1e-12, "global Jacobian should touch z");
}

void TestInitialStaticBiasConstraintBuilderHonorsEnableFlag() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_static_vertical_bias_soft_prior = false;
  gtsam::NonlinearFactorGraph graph;

  const bool disabled_added =
    offline_lc_minimal::InitialStaticBiasConstraintBuilder::AddVerticalAccelBiasSoftPrior(
    config,
    graph,
    B(0),
    gtsam::Symbol('a', 0));
  ExpectTrue(!disabled_added, "disabled static bias builder should report no factor");
  ExpectTrue(graph.empty(), "disabled static bias builder should add no factors");

  config.enable_initial_static_vertical_bias_soft_prior = true;
  config.initial_static_vertical_bias_global_tie_sigma_mps2 = 5e-5;
  const bool enabled_added =
    offline_lc_minimal::InitialStaticBiasConstraintBuilder::AddVerticalAccelBiasSoftPrior(
    config,
    graph,
    B(0),
    gtsam::Symbol('a', 0));
  ExpectTrue(enabled_added, "enabled static bias builder should report one factor");
  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "enabled static bias builder should add one factor");
}

void TestInitialStaticBiasGmTighteningSelectsStaticSigmaOnlyInsideStaticWindow() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_acc_bias_sigma_mps2 = 0.01;
  config.enable_initial_static_vertical_bias_gm_tightening = true;
  config.initial_static_vertical_bias_gm_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.02);

  ExpectNear(
    offline_lc_minimal::InitialStaticBiasConstraintBuilder::ResolveVerticalGmSigmaMps2(config, true),
    offline_lc_minimal::MicroGToMps2(0.02),
    1e-18,
    "static GM sigma should use the tightened static sigma");
  ExpectNear(
    offline_lc_minimal::InitialStaticBiasConstraintBuilder::ResolveVerticalGmSigmaMps2(config, false),
    0.01,
    1e-18,
    "dynamic GM sigma should keep the normal vertical sigma");
}

void TestStaticVerticalPositionHoldFactorUsesOnlyZ() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::StaticVerticalPositionHoldFactor factor(
    X(0),
    X(1),
    noise);
  const gtsam::Pose3 reference_pose(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.3), gtsam::Point3(10.0, -20.0, 5.0));
  const gtsam::Pose3 same_z_pose(gtsam::Rot3::RzRyRx(-0.4, 0.5, -0.6), gtsam::Point3(-30.0, 40.0, 5.0));
  const gtsam::Pose3 shifted_z_pose(gtsam::Rot3(), gtsam::Point3(-30.0, 40.0, 5.25));

  gtsam::Matrix h_reference;
  gtsam::Matrix h_pose;
  const gtsam::Vector zero_residual =
    factor.evaluateError(reference_pose, same_z_pose, h_reference, h_pose);
  const gtsam::Vector shifted_residual = factor.evaluateError(reference_pose, shifted_z_pose);

  ExpectNear(zero_residual(0), 0.0, 1e-12, "static vertical position hold should ignore x/y and attitude");
  ExpectNear(shifted_residual(0), 0.25, 1e-12, "static vertical position hold residual is wrong");
  ExpectNear(h_reference(0, 5), -1.0, 1e-12, "reference pose z Jacobian is wrong");
  ExpectNear(h_pose(0, 5), 1.0, 1e-12, "pose z Jacobian is wrong");
  for (int column = 0; column < 5; ++column) {
    ExpectNear(h_reference(0, column), 0.0, 1e-12, "reference pose Jacobian should only touch z");
    ExpectNear(h_pose(0, column), 0.0, 1e-12, "pose Jacobian should only touch z");
  }
}

void TestStaticPositionHoldFactorUsesTranslationOnly() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  const offline_lc_minimal::factor::StaticPositionHoldFactor factor(
    X(0),
    X(1),
    noise);
  const gtsam::Pose3 reference_pose(gtsam::Rot3::RzRyRx(0.2, -0.1, 0.3), gtsam::Point3(10.0, -20.0, 5.0));
  const gtsam::Pose3 same_position_pose(gtsam::Rot3::RzRyRx(-0.4, 0.5, -0.6), gtsam::Point3(10.0, -20.0, 5.0));
  const gtsam::Pose3 shifted_pose(gtsam::Rot3(), gtsam::Point3(11.0, -22.0, 5.25));

  gtsam::Matrix h_reference;
  gtsam::Matrix h_pose;
  const gtsam::Vector zero_residual =
    factor.evaluateError(reference_pose, same_position_pose, h_reference, h_pose);
  const gtsam::Vector shifted_residual = factor.evaluateError(reference_pose, shifted_pose);

  ExpectNear(zero_residual.norm(), 0.0, 1e-12, "static position hold should ignore attitude");
  ExpectNear(shifted_residual(0), 1.0, 1e-12, "static position hold x residual is wrong");
  ExpectNear(shifted_residual(1), -2.0, 1e-12, "static position hold y residual is wrong");
  ExpectNear(shifted_residual(2), 0.25, 1e-12, "static position hold z residual is wrong");
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      ExpectNear(h_reference(row, column), 0.0, 1e-12, "reference pose rotation Jacobian should be zero");
      ExpectNear(h_pose(row, column), 0.0, 1e-12, "pose rotation Jacobian should be zero");
      ExpectNear(
        h_reference(row, column + 3),
        -reference_pose.rotation().matrix()(row, column),
        1e-12,
        "reference pose translation Jacobian is wrong");
      ExpectNear(
        h_pose(row, column + 3),
        same_position_pose.rotation().matrix()(row, column),
        1e-12,
        "pose translation Jacobian is wrong");
    }
  }
}

void TestInitialStaticPositionConstraintBuilderHonorsEnableFlag() {
  auto config = offline_lc_minimal::DefaultConfig();
  gtsam::NonlinearFactorGraph graph;

  const bool disabled_added =
    offline_lc_minimal::InitialStaticPositionConstraintBuilder::AddVerticalPositionHold(
      config,
      graph,
      X(0),
      X(1));
  ExpectTrue(!disabled_added, "disabled static position builder should report no factor");
  ExpectTrue(graph.empty(), "disabled static position builder should add no factors");

  config.enable_initial_static_vertical_position_hold = true;
  const bool enabled_added =
    offline_lc_minimal::InitialStaticPositionConstraintBuilder::AddVerticalPositionHold(
      config,
      graph,
      X(0),
      X(1));
  ExpectTrue(enabled_added, "enabled static position builder should report one factor");
  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "enabled static position builder should add one factor");

  const bool full_disabled_added =
    offline_lc_minimal::InitialStaticPositionConstraintBuilder::AddPositionHold(
      config,
      graph,
      X(0),
      X(2));
  ExpectTrue(!full_disabled_added, "disabled full static position builder should report no factor");

  config.enable_initial_static_position_hold = true;
  const bool full_enabled_added =
    offline_lc_minimal::InitialStaticPositionConstraintBuilder::AddPositionHold(
      config,
      graph,
      X(0),
      X(2));
  ExpectTrue(full_enabled_added, "enabled full static position builder should report one factor");
  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "full static position builder should add one factor");
}

offline_lc_minimal::GnssSolutionSample MakeStaticRtkSample(const double time_s, const double up_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 1.0;
  sample.lon_rad = 2.0;
  sample.h_m = 3.0;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.02;
  sample.best_sol_status_code = 1;
  sample.gnssfgo_type_code = 1;
  sample.enu_position_m = Eigen::Vector3d(0.0, 0.0, up_m);
  sample.has_enu_position = true;
  return sample;
}

void TestInitialStaticRtkHeightReferenceUsesStaticRtkMedian() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_static_rtk_height_reference = true;
  config.initial_static_rtk_height_reference_min_sample_count = 3;
  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeStaticRtkSample(10.0, 1.0),
    MakeStaticRtkSample(11.0, 1.2),
    MakeStaticRtkSample(12.0, 1.1),
    MakeStaticRtkSample(30.0, 9.0)};

  const auto reference =
    offline_lc_minimal::InitialStaticRtkHeightConstraintBuilder::BuildReference(
      samples,
      10.0,
      12.0,
      config);

  ExpectTrue(reference.valid, "static RTK height reference should be valid");
  ExpectNear(static_cast<double>(reference.sample_count), 3.0, 0.0, "static RTK sample count is wrong");
  ExpectNear(reference.reference_up_m, 1.1, 1e-12, "static RTK height reference should use median up");
}

void TestInitialStaticRtkHeightReferenceAddsVerticalPositionFactorOnlyWhenValid() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_initial_static_rtk_height_reference = true;
  config.initial_static_rtk_height_reference_sigma_m = 0.02;
  gtsam::NonlinearFactorGraph graph;

  offline_lc_minimal::InitialStaticRtkHeightReference invalid_reference;
  const bool invalid_added =
    offline_lc_minimal::InitialStaticRtkHeightConstraintBuilder::AddVerticalReference(
      invalid_reference,
      config,
      graph,
      X(0));
  ExpectTrue(!invalid_added, "invalid static RTK reference should add no factor");
  ExpectTrue(graph.empty(), "invalid static RTK reference should leave graph empty");

  offline_lc_minimal::InitialStaticRtkHeightReference reference;
  reference.valid = true;
  reference.sample_count = 10;
  reference.reference_up_m = 1.25;
  const bool valid_added =
    offline_lc_minimal::InitialStaticRtkHeightConstraintBuilder::AddVerticalReference(
      reference,
      config,
      graph,
      X(0));
  ExpectTrue(valid_added, "valid static RTK reference should add one factor");
  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "static RTK reference should add one factor");
}

}  // namespace

int main() {
  try {
    RunTest("TestStaticVerticalAccelBiasFactorUsesOnlyBaz", TestStaticVerticalAccelBiasFactorUsesOnlyBaz);
    RunTest(
      "TestInitialStaticBiasConstraintBuilderHonorsEnableFlag",
      TestInitialStaticBiasConstraintBuilderHonorsEnableFlag);
    RunTest(
      "TestInitialStaticBiasGmTighteningSelectsStaticSigmaOnlyInsideStaticWindow",
      TestInitialStaticBiasGmTighteningSelectsStaticSigmaOnlyInsideStaticWindow);
    RunTest(
      "TestStaticVerticalPositionHoldFactorUsesOnlyZ",
      TestStaticVerticalPositionHoldFactorUsesOnlyZ);
    RunTest(
      "TestStaticPositionHoldFactorUsesTranslationOnly",
      TestStaticPositionHoldFactorUsesTranslationOnly);
    RunTest(
      "TestInitialStaticPositionConstraintBuilderHonorsEnableFlag",
      TestInitialStaticPositionConstraintBuilderHonorsEnableFlag);
    RunTest(
      "TestInitialStaticRtkHeightReferenceUsesStaticRtkMedian",
      TestInitialStaticRtkHeightReferenceUsesStaticRtkMedian);
    RunTest(
      "TestInitialStaticRtkHeightReferenceAddsVerticalPositionFactorOnlyWhenValid",
      TestInitialStaticRtkHeightReferenceAddsVerticalPositionFactorOnlyWhenValid);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
