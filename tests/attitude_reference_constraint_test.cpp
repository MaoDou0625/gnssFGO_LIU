#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/core/AttitudeReferenceConstraintBuilder.h"
#include "offline_lc_minimal/factor/AttitudeReferenceFactor.h"

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

void TestAttitudeReferenceFactorIgnoresTranslation() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  const gtsam::Rot3 reference_rotation = gtsam::Rot3::Ypr(0.2, -0.1, 0.05);
  const offline_lc_minimal::factor::AttitudeReferenceFactor factor(1, reference_rotation, noise);

  const gtsam::Vector same_rotation_residual = factor.evaluateError(
    gtsam::Pose3(reference_rotation, gtsam::Point3(100.0, -50.0, 3.0)));
  ExpectNear(same_rotation_residual.norm(), 0.0, 1e-12, "translation should not affect attitude reference");

  const gtsam::Vector changed_rotation_residual = factor.evaluateError(
    gtsam::Pose3(reference_rotation.compose(gtsam::Rot3::Expmap(gtsam::Vector3(0.01, 0.0, 0.0))),
                 gtsam::Point3::Zero()));
  ExpectTrue(changed_rotation_residual.norm() > 0.005, "rotation change should create attitude residual");
}

std::vector<offline_lc_minimal::ReferenceNodeState> MakeReferenceStates(const std::size_t count = 4U) {
  std::vector<offline_lc_minimal::ReferenceNodeState> states(count);
  for (std::size_t index = 0; index < states.size(); ++index) {
    states[index].time_s = static_cast<double>(index);
    states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.1 * static_cast<double>(index), 0.01, -0.02),
      gtsam::Point3::Zero());
  }
  return states;
}

void TestBuilderAddsOnlyDynamicAttitudeReferences() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_attitude_reference_constraint = true;
  config.attitude_reference_sigma_rad = 0.01;
  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::AttitudeReferenceDiagnosticRow> diagnostics;

  offline_lc_minimal::AttitudeReferenceConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.dynamic_start_index = 2;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::AttitudeReferenceConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "only dynamic states should receive attitude factors");
  const auto &keys = graph.at(0)->keys();
  ExpectNear(static_cast<double>(keys.size()), 1.0, 0.0, "attitude factor should only connect pose");
  ExpectTrue(keys.front() == gtsam::symbol_shorthand::X(2), "attitude factor should connect the first dynamic pose");
  ExpectNear(
    static_cast<double>(summary.attitude_reference_factor_count),
    2.0,
    0.0,
    "attitude factor count is wrong");
  ExpectNear(static_cast<double>(diagnostics.front().state_index), 2.0, 0.0, "first diagnostic state is wrong");
}

void TestBuilderRejectsMissingSeedReferences() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_attitude_reference_constraint = true;
  const std::vector<double> timestamps{0.0, 1.0};
  const std::vector<offline_lc_minimal::ReferenceNodeState> reference_states;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::AttitudeReferenceDiagnosticRow> diagnostics;

  offline_lc_minimal::AttitudeReferenceConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;

  bool threw = false;
  try {
    offline_lc_minimal::AttitudeReferenceConstraintBuilder(std::move(request)).Build();
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("successful seed reference optimization") !=
            std::string::npos;
  }
  ExpectTrue(threw, "enabled attitude reference should require seed reference states");
}

void TestPopulateAttitudeReferenceDiagnostics() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_attitude_reference_constraint = true;
  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::AttitudeReferenceDiagnosticRow> diagnostics;

  offline_lc_minimal::AttitudeReferenceConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::AttitudeReferenceConstraintBuilder(std::move(request)).Build();

  gtsam::Values values;
  values.insert(gtsam::symbol_shorthand::X(1), reference_states[1].pose);
  values.insert(
    gtsam::symbol_shorthand::X(2),
    gtsam::Pose3(
      reference_states[2].pose.rotation().compose(gtsam::Rot3::Expmap(gtsam::Vector3(0.02, 0.0, 0.0))),
      gtsam::Point3::Zero()));
  offline_lc_minimal::PopulateAttitudeReferenceDiagnostics(values, diagnostics);

  ExpectNear(diagnostics.front().residual_norm_rad, 0.0, 1e-12, "matching attitude residual should be zero");
  ExpectTrue(diagnostics.back().residual_norm_rad > 0.01, "changed attitude residual should be reported");
}

}  // namespace

int main() {
  try {
    RunTest("TestAttitudeReferenceFactorIgnoresTranslation", TestAttitudeReferenceFactorIgnoresTranslation);
    RunTest("TestBuilderAddsOnlyDynamicAttitudeReferences", TestBuilderAddsOnlyDynamicAttitudeReferences);
    RunTest("TestBuilderRejectsMissingSeedReferences", TestBuilderRejectsMissingSeedReferences);
    RunTest("TestPopulateAttitudeReferenceDiagnostics", TestPopulateAttitudeReferenceDiagnostics);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
