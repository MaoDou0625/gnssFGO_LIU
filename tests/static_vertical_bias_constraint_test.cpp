#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/core/InitialStaticBiasConstraintBuilder.h"
#include "offline_lc_minimal/factor/StaticVerticalAccelBiasFactor.h"

namespace {

using gtsam::symbol_shorthand::B;

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

}  // namespace

int main() {
  try {
    RunTest("TestStaticVerticalAccelBiasFactorUsesOnlyBaz", TestStaticVerticalAccelBiasFactorUsesOnlyBaz);
    RunTest(
      "TestInitialStaticBiasConstraintBuilderHonorsEnableFlag",
      TestInitialStaticBiasConstraintBuilderHonorsEnableFlag);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
