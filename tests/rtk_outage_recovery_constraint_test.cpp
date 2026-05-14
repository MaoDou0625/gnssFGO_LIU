#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/RtkOutageRecoveryConstraintBuilder.h"
#include "offline_lc_minimal/factor/AttitudeHoldFactor.h"
#include "offline_lc_minimal/factor/VelocityDeltaFactor.h"

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

std::vector<offline_lc_minimal::ReferenceNodeState> MakeReferenceStates(
  const std::size_t count) {
  std::vector<offline_lc_minimal::ReferenceNodeState> states(count);
  for (std::size_t index = 0; index < count; ++index) {
    states[index].time_s = static_cast<double>(index);
    states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(
        0.05 * static_cast<double>(index),
        -0.01 * static_cast<double>(index),
        0.02 * static_cast<double>(index)),
      gtsam::Point3::Zero());
  }
  return states;
}

offline_lc_minimal::RtkOutageWindowRow MakeWindow() {
  offline_lc_minimal::RtkOutageWindowRow window;
  window.window_index = 7U;
  window.pre_anchor_state_index = 1U;
  window.post_anchor_state_index = 3U;
  window.start_time_s = 1.0;
  window.end_time_s = 3.0;
  window.skip_reason = "PLANNED";
  return window;
}

std::vector<offline_lc_minimal::VelocityDeltaPropagationRecord> MakeVelocityRecords() {
  return {
    {0U, 1U, 0.0, 1.0, gtsam::Vector3(10.0, 0.0, 0.0)},
    {1U, 2U, 1.0, 2.0, gtsam::Vector3(1.0, -2.0, 0.5)},
    {2U, 3U, 2.0, 3.0, gtsam::Vector3(-0.25, 0.75, -0.5)},
    {3U, 4U, 3.0, 4.0, gtsam::Vector3(0.0, 9.0, 0.0)},
  };
}

void TestAttitudeHoldFactorIgnoresTranslationAndPenalizesAttitude() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  const gtsam::Rot3 reference = gtsam::Rot3::Ypr(0.2, -0.1, 0.3);
  const offline_lc_minimal::factor::AttitudeHoldFactor factor(1, reference, noise);

  const gtsam::Pose3 translated(reference, gtsam::Point3(10.0, -3.0, 7.0));
  ExpectNear(
    factor.evaluateError(translated).norm(),
    0.0,
    1e-12,
    "attitude hold should ignore translation");

  const gtsam::Pose3 rotated(gtsam::Rot3::Ypr(0.25, -0.1, 0.3), gtsam::Point3::Zero());
  ExpectTrue(
    factor.evaluateError(rotated).norm() > 1e-3,
    "attitude hold should penalize yaw/pitch/roll changes");
}

void TestVelocityDeltaFactorUsesOnlyVelocityKeys() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
  const gtsam::Vector3 target_delta(1.0, -2.0, 0.5);
  const offline_lc_minimal::factor::VelocityDeltaFactor factor(
    gtsam::symbol_shorthand::V(1),
    gtsam::symbol_shorthand::V(2),
    target_delta,
    noise);

  gtsam::Matrix h_i;
  gtsam::Matrix h_j;
  const gtsam::Vector residual = factor.evaluateError(
    gtsam::Vector3(0.0, 0.0, 1.0),
    gtsam::Vector3(1.0, -2.0, 1.5),
    h_i,
    h_j);
  ExpectNear(residual.norm(), 0.0, 1e-12, "matching delta velocity should have zero residual");
  ExpectNear(h_i(0, 0), -1.0, 1e-12, "velocity_i jacobian is wrong");
  ExpectNear(h_i(1, 1), -1.0, 1e-12, "velocity_i jacobian is wrong");
  ExpectNear(h_i(2, 2), -1.0, 1e-12, "velocity_i jacobian is wrong");
  ExpectNear(h_j(0, 0), 1.0, 1e-12, "velocity_j jacobian is wrong");
  ExpectTrue(factor.keys().size() == 2U, "3D velocity delta should connect only two velocity keys");
  ExpectTrue(factor.keys()[0] == gtsam::symbol_shorthand::V(1), "first key should be V_i");
  ExpectTrue(factor.keys()[1] == gtsam::symbol_shorthand::V(2), "second key should be V_j");
}

void TestBuilderAddsOutageAttitudeAndVelocityConstraints() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_attitude_hold = true;
  config.rtk_outage_absolute_attitude_sigma_rad = 1.0e-4;
  config.rtk_outage_relative_attitude_sigma_rad = 1.0e-4;
  config.enable_rtk_outage_velocity_delta_3d = true;
  config.rtk_outage_velocity_delta_3d_sigma_mps = 0.20;

  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> windows{MakeWindow()};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  const auto velocity_records = MakeVelocityRecords();
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageAttitudeHoldDiagnosticRow> attitude_diagnostics;
  std::vector<offline_lc_minimal::RtkOutageVelocityDelta3dDiagnosticRow> velocity_diagnostics;

  offline_lc_minimal::RtkOutageRecoveryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.outage_windows = &windows;
  request.reference_states = &reference_states;
  request.velocity_delta_records = &velocity_records;
  request.graph = &graph;
  request.run_summary = &summary;
  request.attitude_diagnostics = &attitude_diagnostics;
  request.velocity_diagnostics = &velocity_diagnostics;
  offline_lc_minimal::RtkOutageRecoveryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(summary.rtk_outage_attitude_hold_factor_count == 5U,
             "absolute attitude hold should include the configured boundary guard");
  ExpectTrue(summary.rtk_outage_relative_attitude_factor_count == 4U,
             "relative yaw should cover guarded adjacent outage edges");
  ExpectTrue(summary.rtk_outage_velocity_delta_3d_factor_count == 2U,
             "3D velocity delta should cover intervals inside the outage");
  ExpectTrue(graph.size() == 11U, "unexpected outage recovery factor count");
  ExpectTrue(attitude_diagnostics.size() == 9U, "attitude diagnostics should include absolute and relative rows");
  ExpectTrue(velocity_diagnostics.size() == 2U, "velocity diagnostics should include each added interval");

  const auto first_attitude_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::AttitudeHoldFactor>(graph.at(0));
  ExpectTrue(first_attitude_factor.get() != nullptr, "first outage factor should hold attitude");
  ExpectTrue(
    first_attitude_factor->keys()[0] == gtsam::symbol_shorthand::X(0),
    "guarded attitude hold should start before the outage pre-anchor");
  const auto first_velocity_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VelocityDeltaFactor>(graph.at(9));
  ExpectTrue(first_velocity_factor.get() != nullptr, "velocity factor should be present after attitude factors");
  ExpectTrue(first_velocity_factor->keys()[0] == gtsam::symbol_shorthand::V(1),
             "velocity factor should start at the first outage interval");

  gtsam::Values values;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    values.insert(gtsam::symbol_shorthand::X(index), reference_states[index].pose);
  }
  const gtsam::Vector3 zero_velocity(0.0, 0.0, 0.0);
  const gtsam::Vector3 velocity_3 =
    (velocity_records[1].target_delta_v_mps + velocity_records[2].target_delta_v_mps).eval();
  values.insert(gtsam::symbol_shorthand::V(0), zero_velocity);
  values.insert(gtsam::symbol_shorthand::V(1), zero_velocity);
  values.insert(gtsam::symbol_shorthand::V(2), velocity_records[1].target_delta_v_mps);
  values.insert(gtsam::symbol_shorthand::V(3), velocity_3);
  values.insert(gtsam::symbol_shorthand::V(4), zero_velocity);
  offline_lc_minimal::PopulateRtkOutageRecoveryDiagnostics(
    values,
    attitude_diagnostics,
    velocity_diagnostics,
    summary);
  ExpectNear(
    summary.rtk_outage_attitude_hold_max_abs_residual_rad,
    0.0,
    1e-12,
    "absolute attitude residual should be zero at the reference");
  ExpectNear(
    summary.rtk_outage_relative_attitude_max_abs_residual_rad,
    0.0,
    1e-12,
    "relative yaw residual should be zero at the reference");
  ExpectNear(
    summary.rtk_outage_velocity_delta_3d_rms_mps,
    0.0,
    1e-12,
    "3D velocity delta residual should be zero for matching velocities");
}

void TestBuilderDisabledAddsNoFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_attitude_hold = false;
  config.enable_rtk_outage_velocity_delta_3d = false;
  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> windows{MakeWindow()};
  const auto reference_states = MakeReferenceStates(timestamps.size());
  const auto velocity_records = MakeVelocityRecords();
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageAttitudeHoldDiagnosticRow> attitude_diagnostics;
  std::vector<offline_lc_minimal::RtkOutageVelocityDelta3dDiagnosticRow> velocity_diagnostics;

  offline_lc_minimal::RtkOutageRecoveryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.outage_windows = &windows;
  request.reference_states = &reference_states;
  request.velocity_delta_records = &velocity_records;
  request.graph = &graph;
  request.run_summary = &summary;
  request.attitude_diagnostics = &attitude_diagnostics;
  request.velocity_diagnostics = &velocity_diagnostics;
  offline_lc_minimal::RtkOutageRecoveryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.empty(), "disabled outage recovery constraints should not add factors");
  ExpectTrue(attitude_diagnostics.empty(), "disabled attitude hold should not emit diagnostics");
  ExpectTrue(velocity_diagnostics.empty(), "disabled velocity delta should not emit diagnostics");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestAttitudeHoldFactorIgnoresTranslationAndPenalizesAttitude",
      TestAttitudeHoldFactorIgnoresTranslationAndPenalizesAttitude);
    RunTest("TestVelocityDeltaFactorUsesOnlyVelocityKeys", TestVelocityDeltaFactorUsesOnlyVelocityKeys);
    RunTest(
      "TestBuilderAddsOutageAttitudeAndVelocityConstraints",
      TestBuilderAddsOutageAttitudeAndVelocityConstraints);
    RunTest("TestBuilderDisabledAddsNoFactors", TestBuilderDisabledAddsNoFactors);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
