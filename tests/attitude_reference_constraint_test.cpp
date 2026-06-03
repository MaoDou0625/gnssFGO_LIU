#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
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

void TestRollPitchReferenceFactorIgnoresTranslationAndYaw() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  const gtsam::Rot3 reference_rotation = gtsam::Rot3::Ypr(0.2, -0.1, 0.05);
  const offline_lc_minimal::factor::RollPitchReferenceFactor factor(1, reference_rotation, noise);

  const gtsam::Vector same_roll_pitch_residual = factor.evaluateError(
    gtsam::Pose3(gtsam::Rot3::Ypr(1.2, -0.1, 0.05), gtsam::Point3(100.0, -50.0, 3.0)));
  ExpectNear(
    same_roll_pitch_residual.norm(),
    0.0,
    1e-12,
    "translation and yaw should not affect roll/pitch reference");

  const gtsam::Vector changed_pitch_residual = factor.evaluateError(
    gtsam::Pose3(gtsam::Rot3::Ypr(1.2, -0.08, 0.05), gtsam::Point3::Zero()));
  ExpectTrue(changed_pitch_residual.norm() > 0.005, "pitch change should create roll/pitch residual");

  const gtsam::Vector changed_roll_residual = factor.evaluateError(
    gtsam::Pose3(gtsam::Rot3::Ypr(1.2, -0.1, 0.07), gtsam::Point3::Zero()));
  ExpectTrue(changed_roll_residual.norm() > 0.005, "roll change should create roll/pitch residual");
}

void TestRelativeYawReferenceFactorIgnoresCommonYawOffset() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const gtsam::Rot3 reference_i = gtsam::Rot3::Ypr(0.2, 0.0, 0.0);
  const gtsam::Rot3 reference_j = gtsam::Rot3::Ypr(0.5, 0.0, 0.0);
  const offline_lc_minimal::factor::RelativeYawReferenceFactor factor(
    1,
    2,
    reference_i,
    reference_j,
    noise);

  const gtsam::Rot3 common_yaw = gtsam::Rot3::Ypr(1.0, 0.0, 0.0);
  const gtsam::Vector common_offset_residual = factor.evaluateError(
    gtsam::Pose3(common_yaw.compose(reference_i), gtsam::Point3(1.0, 2.0, 3.0)),
    gtsam::Pose3(common_yaw.compose(reference_j), gtsam::Point3(-1.0, -2.0, -3.0)));
  ExpectNear(common_offset_residual.norm(), 0.0, 1e-12, "common yaw offset should not affect relative yaw");

  const gtsam::Vector changed_delta_residual = factor.evaluateError(
    gtsam::Pose3(common_yaw.compose(reference_i), gtsam::Point3::Zero()),
    gtsam::Pose3(gtsam::Rot3::Ypr(1.55, 0.0, 0.0), gtsam::Point3::Zero()));
  ExpectTrue(changed_delta_residual.norm() > 0.04, "relative yaw delta error should create residual");
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

std::vector<offline_lc_minimal::ReferenceNodeState> MakeRelativeYawReferenceStates(
  const std::size_t count = 4U) {
  std::vector<offline_lc_minimal::ReferenceNodeState> states(count);
  for (std::size_t index = 0; index < states.size(); ++index) {
    states[index].time_s = static_cast<double>(index);
    states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.25 * static_cast<double>(index), 0.01, -0.02),
      gtsam::Point3::Zero());
  }
  return states;
}

void TestBuilderAddsDynamicRollPitchAndFullYawChain() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_attitude_reference_constraint = true;
  config.attitude_reference_sigma_rad = 0.01;
  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  const auto relative_yaw_reference_states = MakeRelativeYawReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::AttitudeReferenceDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::RelativeYawReferenceDiagnosticRow> relative_yaw_diagnostics;

  offline_lc_minimal::AttitudeReferenceConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.relative_yaw_reference_states = &relative_yaw_reference_states;
  request.dynamic_start_index = 2;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.relative_yaw_diagnostics = &relative_yaw_diagnostics;
  offline_lc_minimal::AttitudeReferenceConstraintBuilder(std::move(request)).Build();

  ExpectNear(
    static_cast<double>(graph.size()),
    5.0,
    0.0,
    "dynamic roll/pitch factors and full-chain relative yaw edges should be added");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::RollPitchReferenceFactor>(graph.at(0)).get() !=
      nullptr,
    "first factor should be roll/pitch reference");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::RollPitchReferenceFactor>(graph.at(1)).get() !=
      nullptr,
    "second factor should be roll/pitch reference");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::RelativeYawReferenceFactor>(graph.at(2)).get() !=
      nullptr,
    "third factor should be first relative yaw reference");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::RelativeYawReferenceFactor>(graph.at(3)).get() !=
      nullptr,
    "fourth factor should be relative yaw reference");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::RelativeYawReferenceFactor>(graph.at(4)).get() !=
      nullptr,
    "fifth factor should be relative yaw reference");
  const auto &roll_pitch_keys = graph.at(0)->keys();
  ExpectNear(static_cast<double>(roll_pitch_keys.size()), 1.0, 0.0, "roll/pitch factor should only connect pose");
  ExpectTrue(
    roll_pitch_keys.front() == gtsam::symbol_shorthand::X(2),
    "roll/pitch factor should connect the first dynamic pose");
  const auto &relative_yaw_keys = graph.at(2)->keys();
  ExpectNear(static_cast<double>(relative_yaw_keys.size()), 2.0, 0.0, "relative yaw factor should connect two poses");
  ExpectTrue(
    relative_yaw_keys[0] == gtsam::symbol_shorthand::X(0) &&
      relative_yaw_keys[1] == gtsam::symbol_shorthand::X(1),
    "relative yaw chain should start at the first graph state");
  const auto &boundary_yaw_keys = graph.at(3)->keys();
  ExpectTrue(
    boundary_yaw_keys[0] == gtsam::symbol_shorthand::X(1) &&
      boundary_yaw_keys[1] == gtsam::symbol_shorthand::X(2),
    "relative yaw chain should include the static-to-dynamic boundary");
  const auto &dynamic_yaw_keys = graph.at(4)->keys();
  ExpectTrue(
    dynamic_yaw_keys[0] == gtsam::symbol_shorthand::X(2) &&
      dynamic_yaw_keys[1] == gtsam::symbol_shorthand::X(3),
    "relative yaw chain should include all later adjacent dynamic poses");
  ExpectNear(
    static_cast<double>(summary.attitude_reference_factor_count),
    5.0,
    0.0,
    "attitude factor count is wrong");
  ExpectNear(static_cast<double>(diagnostics.size()), 2.0, 0.0, "roll/pitch diagnostic count is wrong");
  ExpectNear(
    static_cast<double>(relative_yaw_diagnostics.size()),
    3.0,
    0.0,
    "relative yaw diagnostic count is wrong");
  ExpectNear(static_cast<double>(diagnostics.front().state_index), 2.0, 0.0, "first diagnostic state is wrong");
  ExpectNear(
    relative_yaw_diagnostics.front().reference_delta_yaw_rad,
    offline_lc_minimal::factor::RelativeYawRad(
      relative_yaw_reference_states[0].pose.rotation(),
      relative_yaw_reference_states[1].pose.rotation()),
    1e-12,
    "relative yaw reference should come from the base yaw reference states");
}

void TestBuilderDisabledAddsNoAttitudeReferences() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_attitude_reference_constraint = false;
  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::AttitudeReferenceDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::RelativeYawReferenceDiagnosticRow> relative_yaw_diagnostics;

  offline_lc_minimal::AttitudeReferenceConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.relative_yaw_reference_states = &reference_states;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.relative_yaw_diagnostics = &relative_yaw_diagnostics;
  offline_lc_minimal::AttitudeReferenceConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 0.0, 0.0, "disabled builder should add no factors");
  ExpectNear(
    static_cast<double>(summary.attitude_reference_factor_count),
    0.0,
    0.0,
    "disabled builder should not count attitude factors");
  ExpectTrue(diagnostics.empty(), "disabled builder should add no roll/pitch diagnostics");
  ExpectTrue(relative_yaw_diagnostics.empty(), "disabled builder should add no relative yaw diagnostics");
}

void TestBuilderRejectsMissingSeedReferences() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_attitude_reference_constraint = true;
  const std::vector<double> timestamps{0.0, 1.0};
  const std::vector<offline_lc_minimal::ReferenceNodeState> reference_states;
  const auto relative_yaw_reference_states = MakeRelativeYawReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::AttitudeReferenceDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::RelativeYawReferenceDiagnosticRow> relative_yaw_diagnostics;

  offline_lc_minimal::AttitudeReferenceConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.relative_yaw_reference_states = &relative_yaw_reference_states;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.relative_yaw_diagnostics = &relative_yaw_diagnostics;

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
  const auto relative_yaw_reference_states = MakeRelativeYawReferenceStates(timestamps.size());
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::AttitudeReferenceDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::RelativeYawReferenceDiagnosticRow> relative_yaw_diagnostics;

  offline_lc_minimal::AttitudeReferenceConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.reference_states = &reference_states;
  request.relative_yaw_reference_states = &relative_yaw_reference_states;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.relative_yaw_diagnostics = &relative_yaw_diagnostics;
  offline_lc_minimal::AttitudeReferenceConstraintBuilder(std::move(request)).Build();

  const double common_yaw_offset_rad = 0.5;
  gtsam::Values values;
  values.insert(
    gtsam::symbol_shorthand::X(0),
    gtsam::Pose3(
      gtsam::Rot3::Ypr(
        relative_yaw_reference_states[0].pose.rotation().ypr().x() + common_yaw_offset_rad,
        reference_states[0].pose.rotation().ypr().y(),
        reference_states[0].pose.rotation().ypr().z()),
      gtsam::Point3::Zero()));
  values.insert(
    gtsam::symbol_shorthand::X(1),
    gtsam::Pose3(
      gtsam::Rot3::Ypr(
        relative_yaw_reference_states[1].pose.rotation().ypr().x() + common_yaw_offset_rad,
        reference_states[1].pose.rotation().ypr().y(),
        reference_states[1].pose.rotation().ypr().z()),
      gtsam::Point3::Zero()));
  values.insert(
    gtsam::symbol_shorthand::X(2),
    gtsam::Pose3(
      gtsam::Rot3::Ypr(
        relative_yaw_reference_states[2].pose.rotation().ypr().x() + common_yaw_offset_rad + 0.02,
        reference_states[2].pose.rotation().ypr().y(),
        reference_states[2].pose.rotation().ypr().z() + 0.01),
      gtsam::Point3::Zero()));
  offline_lc_minimal::PopulateAttitudeReferenceDiagnostics(values, diagnostics);
  offline_lc_minimal::PopulateRelativeYawReferenceDiagnostics(values, relative_yaw_diagnostics);

  ExpectNear(
    diagnostics.front().residual_norm_rad,
    0.0,
    1e-12,
    "matching roll/pitch residual should be zero despite yaw offset");
  ExpectTrue(std::isnan(diagnostics.front().residual_x_rad), "absolute yaw residual should not be reported");
  ExpectTrue(diagnostics.back().residual_norm_rad > 0.005, "changed roll/pitch residual should be reported");
  ExpectTrue(
    std::abs(relative_yaw_diagnostics.front().residual_yaw_rad) < 1e-12,
    "common yaw offset should not affect the first relative yaw edge");
  ExpectTrue(
    relative_yaw_diagnostics.back().residual_yaw_rad > 0.005,
    "changed relative yaw residual should be reported");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestRollPitchReferenceFactorIgnoresTranslationAndYaw",
      TestRollPitchReferenceFactorIgnoresTranslationAndYaw);
    RunTest(
      "TestRelativeYawReferenceFactorIgnoresCommonYawOffset",
      TestRelativeYawReferenceFactorIgnoresCommonYawOffset);
    RunTest(
      "TestBuilderAddsDynamicRollPitchAndFullYawChain",
      TestBuilderAddsDynamicRollPitchAndFullYawChain);
    RunTest("TestBuilderDisabledAddsNoAttitudeReferences", TestBuilderDisabledAddsNoAttitudeReferences);
    RunTest("TestBuilderRejectsMissingSeedReferences", TestBuilderRejectsMissingSeedReferences);
    RunTest("TestPopulateAttitudeReferenceDiagnostics", TestPopulateAttitudeReferenceDiagnostics);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
