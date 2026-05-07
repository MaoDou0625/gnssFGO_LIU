#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gtsam/inference/Symbol.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/core/InitialStaticBiasConstraintBuilder.h"
#include "offline_lc_minimal/core/StaticVerticalBiasCalibrator.h"
#include "offline_lc_minimal/factor/StaticVerticalAccelBiasFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalBiasReferenceFactor.h"

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
  config.initial_static_vertical_bias_sigma_mps2 = 5e-5;
  const bool enabled_added =
    offline_lc_minimal::InitialStaticBiasConstraintBuilder::AddVerticalAccelBiasSoftPrior(
    config,
    graph,
    B(0),
    gtsam::Symbol('a', 0));
  ExpectTrue(enabled_added, "enabled static bias builder should report one factor");
  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "enabled static bias builder should add one factor");
}

void TestStaticVerticalBiasReferenceFactorUsesOnlyGlobalBaz() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::StaticVerticalBiasReferenceFactor factor(
    gtsam::Symbol('a', 0),
    0.005,
    noise);

  gtsam::Matrix h_global;
  const gtsam::Vector residual =
    factor.evaluateError(gtsam::Vector3(100.0, -200.0, 0.006), h_global);
  const gtsam::Vector xy_changed_residual =
    factor.evaluateError(gtsam::Vector3(-10.0, 20.0, 0.006));
  const gtsam::Vector z_changed_residual =
    factor.evaluateError(gtsam::Vector3(100.0, -200.0, 0.004));

  ExpectNear(residual(0), 0.001, 1e-12, "reference factor residual is wrong");
  ExpectNear(xy_changed_residual(0), 0.001, 1e-12, "reference factor should ignore global x/y");
  ExpectNear(z_changed_residual(0), -0.001, 1e-12, "reference factor should use global z");
  ExpectNear(static_cast<double>(h_global.rows()), 1.0, 0.0, "global Jacobian row count is wrong");
  ExpectNear(static_cast<double>(h_global.cols()), 3.0, 0.0, "global Jacobian column count is wrong");
  ExpectNear(h_global(0, 0), 0.0, 1e-12, "global reference Jacobian should not touch x");
  ExpectNear(h_global(0, 1), 0.0, 1e-12, "global reference Jacobian should not touch y");
  ExpectNear(h_global(0, 2), 1.0, 1e-12, "global reference Jacobian should touch z");
}

void TestStaticVerticalBiasCarryoverDiagnosticDoesNotAddDynamicFactors() {
  offline_lc_minimal::StaticVerticalBiasCalibrationResult calibration;
  calibration.static_window_start_s = 10.0;
  calibration.static_window_end_s = 110.0;
  calibration.static_sample_count = 1000;
  calibration.static_state_count = 201;
  calibration.initial_baz_mps2 = 0.004;
  calibration.static_baz_ref_mps2 = 0.005;
  calibration.initial_error = 10.0;
  calibration.final_error = 1.0;

  const gtsam::Key global_acc_bias_key = gtsam::Symbol('a', 0);
  gtsam::Values values;
  values.insert(global_acc_bias_key, gtsam::Vector3(0.0, 0.0, 0.0051));
  const std::vector<double> timestamps{100.0, 120.0, 121.0};
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 1.0)));
  values.insert(V(0), gtsam::Vector3(0.0, 0.0, 0.1));
  values.insert(B(0), gtsam::imuBias::ConstantBias(gtsam::Vector3(0.0, 0.0, 0.0052), gtsam::Vector3::Zero()));
  values.insert(X(1), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 1.2)));
  values.insert(V(1), gtsam::Vector3(0.0, 0.0, 0.3));
  values.insert(B(1), gtsam::imuBias::ConstantBias(gtsam::Vector3(0.0, 0.0, 0.0048), gtsam::Vector3::Zero()));
  values.insert(X(2), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 1.4)));
  values.insert(V(2), gtsam::Vector3(0.0, 0.0, -0.2));
  values.insert(B(2), gtsam::imuBias::ConstantBias(gtsam::Vector3(0.0, 0.0, 0.0060), gtsam::Vector3::Zero()));

  const offline_lc_minimal::StaticVerticalBiasCarryoverDiagnosticRow row =
    offline_lc_minimal::BuildStaticVerticalBiasCarryoverDiagnostic(
      calibration,
      values,
      timestamps,
      0U,
      global_acc_bias_key);

  ExpectNear(static_cast<double>(row.dynamic_first20_added_factor_count), 0.0, 0.0, "diagnostic should add no factors");
  ExpectNear(static_cast<double>(row.dynamic_first20_state_count), 2.0, 0.0, "diagnostic should inspect first20 states");
  ExpectNear(row.optimized_global_baz_delta_mps2, 0.0001, 1e-12, "global delta diagnostic is wrong");
  ExpectNear(row.dynamic_first20_max_abs_baz_delta_mps2, 0.0002, 1e-12, "max dynamic bias delta is wrong");
  ExpectNear(row.dynamic_first20_up_delta_m, 0.2, 1e-12, "dynamic up delta is wrong");
  ExpectNear(row.dynamic_first20_vz_range_mps, 0.2, 1e-12, "dynamic vz range is wrong");
}

}  // namespace

int main() {
  try {
    RunTest("TestStaticVerticalAccelBiasFactorUsesOnlyBaz", TestStaticVerticalAccelBiasFactorUsesOnlyBaz);
    RunTest(
      "TestInitialStaticBiasConstraintBuilderHonorsEnableFlag",
      TestInitialStaticBiasConstraintBuilderHonorsEnableFlag);
    RunTest(
      "TestStaticVerticalBiasReferenceFactorUsesOnlyGlobalBaz",
      TestStaticVerticalBiasReferenceFactorUsesOnlyGlobalBaz);
    RunTest(
      "TestStaticVerticalBiasCarryoverDiagnosticDoesNotAddDynamicFactors",
      TestStaticVerticalBiasCarryoverDiagnosticDoesNotAddDynamicFactors);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
