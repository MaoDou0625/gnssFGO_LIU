#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryAttitudeHandoff.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryBiasHandoff.h"
#include "offline_lc_minimal/core/RtkOutageCausalReferenceBuilder.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryAttitudeReference.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryConstraintBuilder.h"
#include "offline_lc_minimal/core/RtkOutagePreOutageVerticalFenceBuilder.h"
#include "offline_lc_minimal/core/RtkOutageRecoveryConstraintBuilder.h"
#include "offline_lc_minimal/core/RtkOutageRecoveryReferenceBuilder.h"
#include "offline_lc_minimal/factor/AttitudeReferenceFactor.h"
#include "offline_lc_minimal/factor/AttitudeHoldFactor.h"
#include "offline_lc_minimal/factor/HorizontalPositionVelocityHandoffFactor.h"
#include "offline_lc_minimal/factor/VerticalPositionVelocityHandoffFactor.h"
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

void TestHorizontalPositionVelocityHandoffFactorUsesTrapezoidVelocityDelta() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);
  const offline_lc_minimal::factor::HorizontalPositionVelocityHandoffFactor factor(
    gtsam::symbol_shorthand::X(1),
    gtsam::symbol_shorthand::V(1),
    gtsam::Vector2(2.0, -1.0),
    gtsam::Vector2(3.0, -2.0),
    0.5,
    noise);

  const gtsam::Pose3 matching_pose(
    gtsam::Rot3::Identity(),
    gtsam::Point3(1.0, -0.25, 4.0));
  const gtsam::Vector3 matching_velocity(1.0, -1.0, 9.0);
  ExpectNear(
    factor.evaluateError(matching_pose, matching_velocity).norm(),
    0.0,
    1.0e-12,
    "horizontal handoff should be zero for trapezoid-integrated position");

  const gtsam::Pose3 shifted_pose(
    gtsam::Rot3::Identity(),
    gtsam::Point3(1.2, -0.25, 4.0));
  ExpectTrue(
    factor.evaluateError(shifted_pose, matching_velocity).norm() > 0.1,
    "horizontal handoff should penalize position mismatch");

  const gtsam::Vector3 shifted_velocity(1.4, -1.0, 9.0);
  ExpectTrue(
    factor.evaluateError(matching_pose, shifted_velocity).norm() > 0.05,
    "horizontal handoff should penalize velocity-integrated mismatch");
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
             "relative rotation should cover guarded adjacent outage edges");
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
  const auto first_relative_rotation_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::RelativeRotationReferenceFactor>(graph.at(5));
  ExpectTrue(first_relative_rotation_factor.get() != nullptr,
             "guarded outage edges should use full relative rotation factors");
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
    "relative rotation residual should be zero at the reference");
  ExpectNear(
    summary.rtk_outage_velocity_delta_3d_rms_mps,
    0.0,
    1e-12,
    "3D velocity delta residual should be zero for matching velocities");
}

void TestOutageVelocityDeltaUsesVerticalClampedVzTarget() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_attitude_hold = false;
  config.enable_rtk_outage_velocity_delta_3d = true;
  config.rtk_outage_velocity_delta_3d_sigma_mps = 0.20;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 0.25;

  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> windows{MakeWindow()};
  const auto velocity_records = MakeVelocityRecords();
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord>
    vertical_velocity_records{
      {1U, 2U, 1.0, 2.0, 2.0, 0.0},
      {2U, 3U, 2.0, 3.0, -2.0, 0.0},
    };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageVelocityDelta3dDiagnosticRow> velocity_diagnostics;

  offline_lc_minimal::RtkOutageRecoveryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.outage_windows = &windows;
  request.velocity_delta_records = &velocity_records;
  request.vertical_velocity_delta_records = &vertical_velocity_records;
  request.graph = &graph;
  request.run_summary = &summary;
  request.velocity_diagnostics = &velocity_diagnostics;
  offline_lc_minimal::RtkOutageRecoveryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 2U, "only the two outage velocity factors should be added");
  ExpectTrue(summary.rtk_outage_velocity_delta_3d_factor_count == 2U,
             "3D velocity delta should still cover outage intervals");
  ExpectTrue(velocity_diagnostics.size() == 2U,
             "velocity diagnostics should reflect the adjusted target");

  const auto first_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VelocityDeltaFactor>(graph.at(0));
  const auto second_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VelocityDeltaFactor>(graph.at(1));
  ExpectTrue(first_factor.get() != nullptr, "first outage velocity factor should be present");
  ExpectTrue(second_factor.get() != nullptr, "second outage velocity factor should be present");
  ExpectNear(first_factor->targetDeltaVMps().x(), 1.0, 1.0e-12,
             "x target should come from the 3D IMU propagation");
  ExpectNear(first_factor->targetDeltaVMps().y(), -2.0, 1.0e-12,
             "y target should come from the 3D IMU propagation");
  ExpectNear(first_factor->targetDeltaVMps().z(), 0.25, 1.0e-12,
             "z target should use the clamped vertical dvz target");
  ExpectNear(second_factor->targetDeltaVMps().x(), -0.25, 1.0e-12,
             "x target should come from the 3D IMU propagation");
  ExpectNear(second_factor->targetDeltaVMps().y(), 0.75, 1.0e-12,
             "y target should come from the 3D IMU propagation");
  ExpectNear(second_factor->targetDeltaVMps().z(), -0.25, 1.0e-12,
             "z target should use the clamped vertical dvz target");
  ExpectNear(velocity_diagnostics[0].target_delta_v_mps.z(), 0.25, 1.0e-12,
             "diagnostic z target should match the factor target");
  ExpectNear(velocity_diagnostics[1].target_delta_v_mps.z(), -0.25, 1.0e-12,
             "diagnostic z target should match the factor target");
}

void TestBuilderSplitsOutageTiltAndYawWhenBaseTiltReferenceIsEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_attitude_hold = true;
  config.enable_base_graph_tilt_reference_constraint = true;
  config.base_graph_tilt_reference_sigma_rad = 0.01;
  config.rtk_outage_absolute_attitude_sigma_rad = 1.0e-4;
  config.rtk_outage_relative_attitude_sigma_rad = 1.0e-4;
  config.enable_rtk_outage_velocity_delta_3d = true;
  config.rtk_outage_velocity_delta_3d_sigma_mps = 0.20;

  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0, 4.0};
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> windows{MakeWindow()};
  const auto yaw_reference_states = MakeReferenceStates(timestamps.size());
  auto tilt_reference_states = MakeReferenceStates(timestamps.size());
  for (std::size_t index = 0; index < tilt_reference_states.size(); ++index) {
    tilt_reference_states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(
        1.0,
        -0.04 + 0.002 * static_cast<double>(index),
        0.03 - 0.001 * static_cast<double>(index)),
      gtsam::Point3::Zero());
  }
  const auto velocity_records = MakeVelocityRecords();
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageAttitudeHoldDiagnosticRow> attitude_diagnostics;
  std::vector<offline_lc_minimal::RtkOutageVelocityDelta3dDiagnosticRow> velocity_diagnostics;

  offline_lc_minimal::RtkOutageRecoveryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.outage_windows = &windows;
  request.reference_states = &yaw_reference_states;
  request.attitude_reference_source = "stage1_yaw_refined_reference_states";
  request.tilt_reference_states = &tilt_reference_states;
  request.tilt_reference_source = "base_graph_optimized";
  request.velocity_delta_records = &velocity_records;
  request.graph = &graph;
  request.run_summary = &summary;
  request.attitude_diagnostics = &attitude_diagnostics;
  request.velocity_diagnostics = &velocity_diagnostics;
  offline_lc_minimal::RtkOutageRecoveryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(summary.rtk_outage_attitude_hold_factor_count == 10U,
             "split outage attitude hold should add tilt and yaw factors per guarded state");
  ExpectTrue(summary.rtk_outage_relative_attitude_factor_count == 4U,
             "relative rotation should still cover guarded adjacent outage edges");
  ExpectTrue(graph.size() == 16U, "unexpected split outage recovery factor count");
  ExpectTrue(attitude_diagnostics.size() == 14U,
             "attitude diagnostics should include tilt, yaw, and relative rows");

  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::TiltReferenceFactor>(graph.at(0)).get() !=
      nullptr,
    "first split outage factor should constrain base tilt");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::YawReferenceFactor>(graph.at(1)).get() !=
      nullptr,
    "second split outage factor should constrain yaw only");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::AttitudeHoldFactor>(graph.at(0)).get() ==
      nullptr,
    "split outage path must not add full attitude hold");
  ExpectTrue(attitude_diagnostics[0].constraint_type == "tilt",
             "first split diagnostic should be tilt");
  ExpectTrue(attitude_diagnostics[1].constraint_type == "yaw",
             "second split diagnostic should be yaw");
  ExpectTrue(attitude_diagnostics[0].reference_source == "base_graph_optimized",
             "tilt diagnostic should identify base graph reference");
  ExpectTrue(attitude_diagnostics[1].reference_source == "stage1_yaw_refined_reference_states",
             "yaw diagnostic should identify yaw reference");
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

void TestPreOutageVerticalFenceConstrainsOnlyUpAndVz() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_preoutage_vertical_fence = true;
  config.rtk_outage_preoutage_fence_stride_s = 1.0;
  config.rtk_outage_preoutage_fence_up_sigma_m = 0.01;
  config.rtk_outage_preoutage_fence_vz_sigma_mps = 0.01;

  std::vector<offline_lc_minimal::RtkOutageCausalStateReferenceRow> state_refs(2U);
  for (std::size_t index = 0; index < state_refs.size(); ++index) {
    state_refs[index].state_index = index;
    state_refs[index].time_s = static_cast<double>(index);
    state_refs[index].reference_up_m = 10.0 + static_cast<double>(index);
    state_refs[index].reference_vz_mps = -0.01 * static_cast<double>(index);
    state_refs[index].valid = true;
    state_refs[index].skip_reason = "OK";
  }

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  offline_lc_minimal::RtkOutagePreOutageVerticalFenceBuildRequest request;
  request.config = &config;
  request.state_references = &state_refs;
  request.graph = &graph;
  request.run_summary = &summary;
  offline_lc_minimal::RtkOutagePreOutageVerticalFenceBuilder(std::move(request)).Build();

  ExpectTrue(summary.rtk_outage_preoutage_vertical_fence_factor_count == 4U,
             "two states should receive up and vz fence factors");
  ExpectTrue(graph.size() == 4U, "fence should add two scalar factors per selected state");

  gtsam::Values matching_values;
  gtsam::Values shifted_xy_values;
  gtsam::Values shifted_z_values;
  for (std::size_t index = 0; index < state_refs.size(); ++index) {
    const auto &row = state_refs[index];
    matching_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(gtsam::Rot3::Ypr(0.1, -0.2, 0.3), gtsam::Point3(0.0, 0.0, row.reference_up_m)));
    matching_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(100.0, -50.0, row.reference_vz_mps));
    shifted_xy_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(gtsam::Rot3::Ypr(-0.3, 0.2, -0.1), gtsam::Point3(100.0, -50.0, row.reference_up_m)));
    shifted_xy_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(-10.0, 20.0, row.reference_vz_mps));
    shifted_z_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, row.reference_up_m + 0.1)));
    shifted_z_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, row.reference_vz_mps - 0.02));
  }

  ExpectNear(graph.error(matching_values), 0.0, 1.0e-12,
             "matching vertical states should have zero fence error");
  ExpectNear(graph.error(shifted_xy_values), 0.0, 1.0e-12,
             "fence should ignore x/y, horizontal velocity, and attitude");
  ExpectTrue(graph.error(shifted_z_values) > 1.0,
             "fence should penalize up and vz changes");

  offline_lc_minimal::PopulateRtkOutagePreOutageVerticalFenceSummary(
    shifted_z_values,
    state_refs,
    summary);
  ExpectNear(
    summary.rtk_outage_preoutage_vertical_fence_max_delta_m,
    0.1,
    1.0e-12,
    "summary should report max vertical position delta");
  ExpectNear(
    summary.rtk_outage_preoutage_vertical_fence_max_vz_delta_mps,
    0.02,
    1.0e-12,
    "summary should report max vertical velocity delta");
}

void TestPreOutageVerticalFenceDisabledAddsNoFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_preoutage_vertical_fence = false;
  std::vector<offline_lc_minimal::RtkOutageCausalStateReferenceRow> state_refs(1U);
  state_refs.front().state_index = 0U;
  state_refs.front().time_s = 0.0;
  state_refs.front().reference_up_m = 1.0;
  state_refs.front().reference_vz_mps = 0.0;
  state_refs.front().valid = true;

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  offline_lc_minimal::RtkOutagePreOutageVerticalFenceBuildRequest request;
  request.config = &config;
  request.state_references = &state_refs;
  request.graph = &graph;
  request.run_summary = &summary;
  offline_lc_minimal::RtkOutagePreOutageVerticalFenceBuilder(std::move(request)).Build();

  ExpectTrue(graph.empty(), "disabled pre-outage fence should add no factors");
  ExpectTrue(summary.rtk_outage_preoutage_vertical_fence_factor_count == 0U,
             "disabled pre-outage fence should report zero factors");
}

void TestBoundaryAttitudeReferenceAnchorsStartAndEndWithImuDelta() {
  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0};
  std::vector<offline_lc_minimal::ReferenceNodeState> imu_states(timestamps.size());
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    imu_states[index].time_s = timestamps[index];
    imu_states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.1 * static_cast<double>(index), 0.0, 0.0),
      gtsam::Point3::Zero());
  }

  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> refs(2U);
  refs[0].window_index = 9U;
  refs[0].boundary_role = "OUTAGE_START";
  refs[0].target_time_s = 1.0;
  refs[0].valid = true;
  refs[0].has_attitude = true;
  refs[0].reference_rotation = gtsam::Rot3::Ypr(1.0, 0.0, 0.0);
  refs[1].window_index = 9U;
  refs[1].boundary_role = "OUTAGE_END";
  refs[1].target_time_s = 3.0;
  refs[1].valid = true;
  refs[1].has_attitude = true;
  refs[1].reference_rotation = gtsam::Rot3::Ypr(1.4, 0.0, 0.0);

  const auto reference =
    offline_lc_minimal::BuildRtkOutageBoundaryAttitudeReference(
      timestamps,
      imu_states,
      refs);
  ExpectTrue(reference.valid(), "boundary attitude reference should be valid");
  ExpectTrue(reference.outage_windows.size() == 1U, "one synthetic outage window expected");
  ExpectTrue(reference.outage_windows.front().pre_anchor_state_index == 1U,
             "synthetic outage start index is wrong");
  ExpectTrue(reference.outage_windows.front().post_anchor_state_index == 3U,
             "synthetic outage end index is wrong");
  ExpectNear(reference.reference_states[1].pose.rotation().ypr().x(), 1.0, 1e-12,
             "start rotation must match the pre boundary");
  ExpectNear(reference.reference_states[2].pose.rotation().ypr().x(), 1.2, 1e-12,
             "middle rotation should preserve IMU delta with smooth end correction");
  ExpectNear(reference.reference_states[3].pose.rotation().ypr().x(), 1.4, 1e-12,
             "end rotation must match the post boundary");

  gtsam::Values values;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Identity(),
        gtsam::Point3(static_cast<double>(index), 0.0, 0.0)));
  }
  offline_lc_minimal::ApplyRtkOutageBoundaryAttitudeInitialValues(
    reference,
    timestamps,
    0.0,
    values);
  ExpectNear(
    values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(0)).rotation().ypr().x(),
    0.0,
    1e-12,
    "states outside the synthetic outage should not be changed");
  ExpectNear(
    values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(2)).rotation().ypr().x(),
    1.2,
    1e-12,
    "initial values should be moved onto the boundary-anchored branch");
}

void TestBoundaryAttitudeReferencePropagatesThroughGuardedSpan() {
  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0};
  std::vector<offline_lc_minimal::ReferenceNodeState> imu_states(timestamps.size());
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    imu_states[index].time_s = timestamps[index];
    imu_states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.1 * static_cast<double>(index), 0.0, 0.0),
      gtsam::Point3::Zero());
  }

  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> refs(2U);
  refs[0].window_index = 11U;
  refs[0].boundary_role = "OUTAGE_START";
  refs[0].target_time_s = 1.0;
  refs[0].valid = true;
  refs[0].has_attitude = true;
  refs[0].reference_rotation = gtsam::Rot3::Ypr(1.0, 0.0, 0.0);
  refs[1].window_index = 11U;
  refs[1].boundary_role = "OUTAGE_END";
  refs[1].target_time_s = 2.0;
  refs[1].valid = true;
  refs[1].has_up = true;
  refs[1].reference_up_m = 5.0;

  const auto reference =
    offline_lc_minimal::BuildRtkOutageBoundaryAttitudeReference(
      timestamps,
      imu_states,
      refs,
      1.0);

  ExpectTrue(reference.valid(), "guarded boundary attitude reference should be valid");
  ExpectTrue(reference.outage_windows.front().pre_anchor_state_index == 1U,
             "synthetic window should keep the outage start anchor");
  ExpectTrue(reference.outage_windows.front().post_anchor_state_index == 2U,
             "non-attitude outage end should still define the window extent");
  ExpectNear(reference.reference_states[0].pose.rotation().ypr().x(), 0.9, 1e-12,
             "pre-start guard state should be reverse-propagated from the start anchor");
  ExpectNear(reference.reference_states[1].pose.rotation().ypr().x(), 1.0, 1e-12,
             "start state should match the boundary attitude");
  ExpectNear(reference.reference_states[2].pose.rotation().ypr().x(), 1.1, 1e-12,
             "outage state should preserve IMU yaw delta from the start anchor");
  ExpectNear(reference.reference_states[3].pose.rotation().ypr().x(), 1.2, 1e-12,
             "post-end guard state should remain on the same propagated branch");
}

void TestBoundaryAttitudeReferenceUsesPostStartAsSingleSidedAnchor() {
  const std::vector<double> timestamps{1.0, 2.0, 3.0};
  std::vector<offline_lc_minimal::ReferenceNodeState> imu_states(timestamps.size());
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    imu_states[index].time_s = timestamps[index];
    imu_states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.1 * static_cast<double>(index), 0.0, 0.0),
      gtsam::Point3::Zero());
  }

  offline_lc_minimal::RtkOutageBoundaryReferenceRow post_start;
  post_start.window_index = 17U;
  post_start.boundary_role = "POST_START";
  post_start.source_type = offline_lc_minimal::kPostStartImuRelativeHandoffSource;
  post_start.target_time_s = 1.0;
  post_start.valid = true;
  post_start.has_attitude = true;
  post_start.reference_rotation = gtsam::Rot3::Ypr(1.0, 0.0, 0.0);

  const auto reference =
    offline_lc_minimal::BuildRtkOutageBoundaryAttitudeReference(
      timestamps,
      imu_states,
      {post_start},
      2.0);

  ExpectTrue(reference.valid(), "POST_START handoff should build a boundary attitude reference");
  ExpectTrue(reference.outage_windows.front().pre_anchor_state_index == 0U,
             "POST_START synthetic window should start at the post first state");
  ExpectTrue(reference.outage_windows.front().post_anchor_state_index == 0U,
             "single-sided POST_START anchor should not invent a second anchor");
  ExpectNear(reference.reference_states[0].pose.rotation().ypr().x(), 1.0, 1e-12,
             "post first reference should match the IMU-relative handoff");
  ExpectNear(reference.reference_states[1].pose.rotation().ypr().x(), 1.1, 1e-12,
             "post guard reference should preserve IMU yaw delta from post start");
  ExpectNear(reference.reference_states[2].pose.rotation().ypr().x(), 1.2, 1e-12,
             "later post guard reference should stay on the handoff branch");
}

void TestBoundaryAttitudeReferenceKeepsOffGridAnchorsExact() {
  const std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0};
  std::vector<offline_lc_minimal::ReferenceNodeState> imu_states(timestamps.size());
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    imu_states[index].time_s = timestamps[index];
    imu_states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.1 * static_cast<double>(index), 0.0, 0.0),
      gtsam::Point3::Zero());
  }

  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> refs(2U);
  refs[0].window_index = 13U;
  refs[0].boundary_role = "OUTAGE_START";
  refs[0].target_time_s = 0.6;
  refs[0].valid = true;
  refs[0].has_attitude = true;
  refs[0].reference_rotation = gtsam::Rot3::Ypr(1.0, 0.0, 0.0);
  refs[1].window_index = 13U;
  refs[1].boundary_role = "OUTAGE_END";
  refs[1].target_time_s = 2.4;
  refs[1].valid = true;
  refs[1].has_attitude = true;
  refs[1].reference_rotation = gtsam::Rot3::Ypr(1.4, 0.0, 0.0);

  const auto reference =
    offline_lc_minimal::BuildRtkOutageBoundaryAttitudeReference(
      timestamps,
      imu_states,
      refs);

  ExpectTrue(reference.valid(), "off-grid boundary attitude reference should be valid");
  ExpectTrue(reference.outage_windows.front().pre_anchor_state_index == 1U,
             "off-grid start should bind first state at or after the boundary");
  ExpectTrue(reference.outage_windows.front().post_anchor_state_index == 2U,
             "off-grid end should bind last state at or before the boundary");
  ExpectNear(reference.reference_states[1].pose.rotation().ypr().x(), 1.0, 1e-12,
             "off-grid start anchor should exactly match the start attitude");
  ExpectNear(reference.reference_states[2].pose.rotation().ypr().x(), 1.4, 1e-12,
             "off-grid end anchor should exactly match the end attitude");
}

void TestOutageEndBoundaryReferenceFromRecoveryUsesRecoveryPositionVelocity() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_boundary_up_sigma_m = 0.01;
  config.rtk_outage_boundary_vz_sigma_mps = 0.02;
  config.stage2_horizontal_position_hold_sigma_m = 0.03;
  config.stage2_horizontal_velocity_hold_sigma_mps = 0.04;

  offline_lc_minimal::RtkOutageRecoveryReferenceRow recovery;
  recovery.window_index = 3U;
  recovery.outage_end_time_s = 9.0;
  recovery.reference_up_m = 4.0;
  recovery.reference_vz_mps = -0.5;
  recovery.reference_horizontal_position_m = Eigen::Vector2d(2.0, -3.0);
  recovery.reference_horizontal_velocity_mps = Eigen::Vector2d(0.1, -1.4);
  recovery.valid = true;
  recovery.skip_reason = "OK";

  const offline_lc_minimal::RtkOutageBoundaryReferenceRow boundary =
    offline_lc_minimal::MakeOutageEndBoundaryReferenceFromRecovery(config, recovery);
  ExpectTrue(boundary.boundary_role == "OUTAGE_END",
             "recovery boundary should target outage end");
  ExpectTrue(boundary.has_up && boundary.add_up_constraint,
             "recovery boundary should constrain up");
  ExpectTrue(boundary.has_vz && boundary.add_vz_constraint,
             "recovery boundary should constrain vz");
  ExpectTrue(boundary.has_horizontal_position && boundary.add_horizontal_position_constraint,
             "recovery boundary should constrain horizontal position");
  ExpectTrue(boundary.has_horizontal_velocity && boundary.add_horizontal_velocity_constraint,
             "recovery boundary should constrain horizontal velocity");
  ExpectNear(boundary.reference_horizontal_position_m.x(), 2.0, 1e-12,
             "recovery boundary should carry east");
  ExpectNear(boundary.reference_horizontal_position_m.y(), -3.0, 1e-12,
             "recovery boundary should carry north");
  ExpectNear(boundary.reference_horizontal_velocity_mps.x(), 0.1, 1e-12,
             "recovery boundary should carry east velocity");
  ExpectNear(boundary.reference_horizontal_velocity_mps.y(), -1.4, 1e-12,
             "recovery boundary should carry north velocity");
  ExpectTrue(!boundary.has_attitude && !boundary.add_attitude_constraint,
             "recovery boundary should not invent an attitude reference");
  ExpectNear(boundary.target_time_s, 9.0, 1e-12,
             "recovery boundary time should match outage end");
}

void TestBoundaryStateInitialValuesDoNotRequireAttitude() {
  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> refs(2U);
  refs[0].window_index = 5U;
  refs[0].boundary_role = "OUTAGE_START";
  refs[0].target_time_s = 1.0;
  refs[0].valid = true;
  refs[0].has_horizontal_position = true;
  refs[0].has_horizontal_velocity = true;
  refs[0].has_up = true;
  refs[0].has_vz = true;
  refs[0].reference_horizontal_position_m = Eigen::Vector2d(10.0, 20.0);
  refs[0].reference_horizontal_velocity_mps = Eigen::Vector2d(1.0, 2.0);
  refs[0].reference_up_m = 3.0;
  refs[0].reference_vz_mps = 4.0;
  refs[1].window_index = 5U;
  refs[1].boundary_role = "OUTAGE_END";
  refs[1].target_time_s = 2.0;
  refs[1].valid = true;
  refs[1].has_horizontal_position = true;
  refs[1].has_horizontal_velocity = true;
  refs[1].has_up = true;
  refs[1].has_vz = true;
  refs[1].reference_horizontal_position_m = Eigen::Vector2d(12.0, 24.0);
  refs[1].reference_horizontal_velocity_mps = Eigen::Vector2d(3.0, 6.0);
  refs[1].reference_up_m = 5.0;
  refs[1].reference_vz_mps = 8.0;

  offline_lc_minimal::RtkOutageBoundaryAttitudeReference reference;
  reference.reference_states.resize(timestamps.size());
  reference.outage_windows.resize(1U);
  reference.outage_windows.front().window_index = 5U;
  reference.outage_windows.front().pre_anchor_state_index = 1U;
  reference.outage_windows.front().post_anchor_state_index = 2U;
  reference.outage_windows.front().start_time_s = 1.0;
  reference.outage_windows.front().end_time_s = 2.0;

  gtsam::Values values;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Identity(),
        gtsam::Point3(0.0, 0.0, 0.0)));
    values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, 0.0));
  }
  offline_lc_minimal::ApplyRtkOutageBoundaryStateInitialValues(
    reference,
    refs,
    timestamps,
    0.0,
    values);

  const auto start_pose = values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(1));
  const auto end_pose = values.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(2));
  const auto start_velocity = values.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(1));
  const auto end_velocity = values.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(2));
  ExpectNear(start_pose.translation().x(), 10.0, 1e-12,
             "start horizontal position should not require attitude");
  ExpectNear(start_pose.translation().z(), 3.0, 1e-12,
             "start vertical position should not require attitude");
  ExpectNear(end_pose.translation().y(), 24.0, 1e-12,
             "end horizontal position should not require attitude");
  ExpectNear(end_pose.translation().z(), 5.0, 1e-12,
             "end vertical position should not require attitude");
  ExpectNear(start_velocity.y(), 2.0, 1e-12,
             "start horizontal velocity should not require attitude");
  ExpectNear(start_velocity.z(), 4.0, 1e-12,
             "start vertical velocity should not require attitude");
  ExpectNear(end_velocity.x(), 3.0, 1e-12,
             "end horizontal velocity should not require attitude");
  ExpectNear(end_velocity.z(), 8.0, 1e-12,
             "end vertical velocity should not require attitude");
}

void TestBoundaryConstraintBuilderUsesBoundarySideForOffGridTimes() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_boundary_constraints = true;
  config.rtk_outage_boundary_up_sigma_m = 0.01;

  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> references(2U);
  references[0].window_index = 1U;
  references[0].boundary_role = "OUTAGE_START";
  references[0].target_time_s = 0.6;
  references[0].valid = true;
  references[0].has_up = true;
  references[0].add_up_constraint = true;
  references[0].reference_up_m = 10.0;
  references[0].up_sigma_m = config.rtk_outage_boundary_up_sigma_m;
  references[1].window_index = 1U;
  references[1].boundary_role = "OUTAGE_END";
  references[1].target_time_s = 1.4;
  references[1].valid = true;
  references[1].has_up = true;
  references[1].add_up_constraint = true;
  references[1].reference_up_m = 20.0;
  references[1].up_sigma_m = config.rtk_outage_boundary_up_sigma_m;

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageBoundaryDiagnosticRow> diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.boundary_references = &references;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(diagnostics.size() == 2U, "two boundary diagnostics expected");
  ExpectTrue(diagnostics[0].target_state_index == 1U,
             "outage start should bind the first state at or after the boundary");
  ExpectTrue(diagnostics[1].target_state_index == 1U,
             "outage end should bind the last state at or before the boundary");
}

void TestBoundaryConstraintBuilderConstrainsUpVzBazAndAttitude() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_boundary_constraints = true;
  config.rtk_outage_boundary_up_sigma_m = 0.01;
  config.rtk_outage_boundary_vz_sigma_mps = 0.02;
  config.rtk_outage_boundary_baz_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(50.0);

  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> references(1U);
  references.front().window_index = 1U;
  references.front().boundary_role = "OUTAGE_END";
  references.front().source_type = "POST_RECOVERY_OPTIMIZED";
  references.front().target_time_s = 1.0;
  references.front().valid = true;
  references.front().has_up = true;
  references.front().has_vz = true;
  references.front().has_ba_z = true;
  references.front().has_horizontal_position = true;
  references.front().has_horizontal_velocity = true;
  references.front().has_attitude = true;
  references.front().add_up_constraint = true;
  references.front().add_vz_constraint = true;
  references.front().add_ba_z_constraint = true;
  references.front().add_horizontal_position_constraint = true;
  references.front().add_horizontal_velocity_constraint = true;
  references.front().add_attitude_constraint = true;
  references.front().reference_up_m = 10.0;
  references.front().reference_vz_mps = -0.1;
  references.front().reference_ba_z_mps2 =
    offline_lc_minimal::MicroGToMps2(120.0);
  references.front().reference_horizontal_position_m = Eigen::Vector2d(5.0, -6.0);
  references.front().reference_horizontal_velocity_mps = Eigen::Vector2d(2.0, 3.0);
  references.front().reference_rotation = gtsam::Rot3::Ypr(0.1, -0.2, 0.3);
  references.front().up_sigma_m = config.rtk_outage_boundary_up_sigma_m;
  references.front().vz_sigma_mps = config.rtk_outage_boundary_vz_sigma_mps;
  references.front().ba_z_sigma_mps2 = config.rtk_outage_boundary_baz_sigma_mps2;
  references.front().horizontal_position_sigma_m = config.stage2_horizontal_position_hold_sigma_m;
  references.front().horizontal_velocity_sigma_mps =
    config.stage2_horizontal_velocity_hold_sigma_mps;
  references.front().attitude_sigma_rad = config.rtk_outage_absolute_attitude_sigma_rad;
  references.front().skip_reason = "OK";

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageBoundaryDiagnosticRow> diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.boundary_references = &references;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(
    graph.size() == 6U,
    "boundary should add up, vz, ba_z, horizontal position, horizontal velocity, and attitude factors");
  ExpectTrue(summary.rtk_outage_boundary_up_factor_count == 1U,
             "up factor count is wrong");
  ExpectTrue(summary.rtk_outage_boundary_vz_factor_count == 1U,
             "vz factor count is wrong");
  ExpectTrue(summary.rtk_outage_boundary_baz_factor_count == 1U,
             "ba_z factor count is wrong");
  ExpectTrue(summary.rtk_outage_boundary_horizontal_position_factor_count == 1U,
             "horizontal position factor count is wrong");
  ExpectTrue(summary.rtk_outage_boundary_horizontal_velocity_factor_count == 1U,
             "horizontal velocity factor count is wrong");
  ExpectTrue(summary.rtk_outage_boundary_attitude_factor_count == 1U,
             "attitude factor count is wrong");

  gtsam::Values matching_values;
  gtsam::Values shifted_attitude_values;
  gtsam::Values shifted_horizontal_values;
  gtsam::Values shifted_vertical_values;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    matching_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        references.front().reference_rotation,
        gtsam::Point3(5.0, -6.0, index == 1U ? 10.0 : 0.0)));
    matching_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(2.0, 3.0, index == 1U ? -0.1 : 0.0));
    matching_values.insert(
      gtsam::symbol_shorthand::B(index),
      gtsam::imuBias::ConstantBias(
        gtsam::Vector3(0.0, 0.0, index == 1U ? offline_lc_minimal::MicroGToMps2(120.0) : 0.0),
        gtsam::Vector3::Zero()));

    shifted_attitude_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Ypr(-1.0, 0.5, -0.3),
        gtsam::Point3(5.0, -6.0, index == 1U ? 10.0 : 0.0)));
    shifted_attitude_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(2.0, 3.0, index == 1U ? -0.1 : 0.0));
    shifted_attitude_values.insert(
      gtsam::symbol_shorthand::B(index),
      gtsam::imuBias::ConstantBias(
        gtsam::Vector3(9.0, -8.0, index == 1U ? offline_lc_minimal::MicroGToMps2(120.0) : 0.0),
        gtsam::Vector3::Zero()));

    shifted_horizontal_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        references.front().reference_rotation,
        gtsam::Point3(index == 1U ? 5.3 : 5.0, index == 1U ? -6.4 : -6.0,
                      index == 1U ? 10.0 : 0.0)));
    shifted_horizontal_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(index == 1U ? 2.2 : 2.0, index == 1U ? 2.7 : 3.0,
                     index == 1U ? -0.1 : 0.0));
    shifted_horizontal_values.insert(
      gtsam::symbol_shorthand::B(index),
      gtsam::imuBias::ConstantBias(
        gtsam::Vector3(0.0, 0.0, index == 1U ? offline_lc_minimal::MicroGToMps2(120.0) : 0.0),
        gtsam::Vector3::Zero()));

    shifted_vertical_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        references.front().reference_rotation,
        gtsam::Point3(5.0, -6.0, index == 1U ? 10.2 : 0.0)));
    shifted_vertical_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(2.0, 3.0, index == 1U ? -0.2 : 0.0));
    shifted_vertical_values.insert(
      gtsam::symbol_shorthand::B(index),
      gtsam::imuBias::ConstantBias(
        gtsam::Vector3(0.0, 0.0, index == 1U ? offline_lc_minimal::MicroGToMps2(220.0) : 0.0),
        gtsam::Vector3::Zero()));
  }

  ExpectNear(graph.error(matching_values), 0.0, 1.0e-12,
             "matching boundary state should have zero error");
  ExpectTrue(graph.error(shifted_attitude_values) > 1.0,
             "boundary should penalize attitude changes");
  ExpectTrue(graph.error(shifted_horizontal_values) > 1.0,
             "boundary should penalize horizontal position and velocity changes");
  ExpectTrue(graph.error(shifted_vertical_values) > 1.0,
             "boundary should penalize up, vz, and ba_z changes");

  offline_lc_minimal::PopulateRtkOutageBoundaryDiagnostics(
    shifted_attitude_values,
    diagnostics);
  ExpectTrue(diagnostics.front().attitude_residual_norm_rad > 0.1,
             "boundary diagnostics should report attitude residual");
  offline_lc_minimal::PopulateRtkOutageBoundaryDiagnostics(
    shifted_horizontal_values,
    diagnostics);
  ExpectTrue(diagnostics.front().horizontal_position_residual_norm_m > 0.4,
             "boundary diagnostics should report horizontal position residual");
  ExpectTrue(diagnostics.front().horizontal_velocity_residual_norm_mps > 0.3,
             "boundary diagnostics should report horizontal velocity residual");
  offline_lc_minimal::PopulateRtkOutageBoundaryDiagnostics(
    shifted_vertical_values,
    diagnostics);
  ExpectNear(diagnostics.front().up_residual_m, 0.2, 1.0e-12,
             "boundary diagnostics should report up residual");
  ExpectNear(diagnostics.front().vz_residual_mps, -0.1, 1.0e-12,
             "boundary diagnostics should report vz residual");
  ExpectNear(diagnostics.front().ba_z_residual_ug, 100.0, 1.0e-9,
             "boundary diagnostics should report ba_z residual in ug");
}

void TestBoundaryConstraintBuilderAddsHorizontalPositionVelocityHandoff() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_boundary_constraints = true;
  config.stage2_horizontal_position_hold_sigma_m = 0.05;

  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> references(1U);
  references.front().window_index = 4U;
  references.front().boundary_role = "OUTAGE_END_HORIZONTAL_HANDOFF";
  references.front().source_type =
    "POST_RECOVERY_OPTIMIZED_HORIZONTAL_POSITION_VELOCITY_HANDOFF";
  references.front().target_time_s = 1.0;
  references.front().valid = true;
  references.front().has_horizontal_position = true;
  references.front().has_horizontal_velocity = true;
  references.front().has_horizontal_position_velocity_handoff = true;
  references.front().add_horizontal_position_constraint = false;
  references.front().add_horizontal_velocity_constraint = false;
  references.front().add_horizontal_position_velocity_handoff_constraint = true;
  references.front().reference_horizontal_position_m = Eigen::Vector2d(2.0, -1.0);
  references.front().reference_horizontal_velocity_mps = Eigen::Vector2d(3.0, -2.0);
  references.front().horizontal_position_velocity_handoff_reference_time_s = 1.5;
  references.front().horizontal_position_velocity_handoff_sigma_m =
    config.stage2_horizontal_position_hold_sigma_m;
  references.front().skip_reason = "OK";

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageBoundaryDiagnosticRow> diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.boundary_references = &references;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 1U, "horizontal handoff should add one boundary factor");
  ExpectTrue(
    boost::dynamic_pointer_cast<
      offline_lc_minimal::factor::HorizontalPositionVelocityHandoffFactor>(graph.at(0)).get() !=
      nullptr,
    "boundary factor should be a horizontal position-velocity handoff");
  ExpectTrue(
    summary.rtk_outage_boundary_horizontal_position_velocity_handoff_factor_count == 1U,
    "horizontal handoff factor count is wrong");
  ExpectTrue(diagnostics.size() == 1U, "horizontal handoff should emit one diagnostic");
  ExpectTrue(diagnostics.front().target_state_index == 1U,
             "horizontal handoff should target the outage last state");
  ExpectNear(
    diagnostics.front().horizontal_position_velocity_handoff_dt_s,
    0.5,
    1.0e-12,
    "horizontal handoff diagnostic should report the post reference dt");
  ExpectTrue(!diagnostics.front().horizontal_position_factor_added,
             "horizontal handoff must not add a direct position hold");

  gtsam::Values matching_values;
  gtsam::Values shifted_values;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    matching_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Identity(),
        gtsam::Point3(index == 1U ? 1.0 : 0.0,
                      index == 1U ? -0.25 : 0.0,
                      0.0)));
    matching_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(index == 1U ? 1.0 : 0.0,
                     index == 1U ? -1.0 : 0.0,
                     0.0));
    shifted_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Identity(),
        gtsam::Point3(index == 1U ? 1.2 : 0.0,
                      index == 1U ? -0.25 : 0.0,
                      0.0)));
    shifted_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(index == 1U ? 1.0 : 0.0,
                     index == 1U ? -1.0 : 0.0,
                     0.0));
  }

  ExpectNear(graph.error(matching_values), 0.0, 1.0e-12,
             "matching handoff state should have zero error");
  ExpectTrue(graph.error(shifted_values) > 1.0,
             "horizontal handoff should penalize position-velocity mismatch");
  offline_lc_minimal::PopulateRtkOutageBoundaryDiagnostics(
    shifted_values,
    diagnostics);
  ExpectTrue(
    diagnostics.front().horizontal_position_velocity_handoff_residual_norm_m > 0.1,
    "horizontal handoff diagnostics should report residual norm");
}

void TestBoundaryConstraintBuilderAddsVerticalPositionVelocityHandoff() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_boundary_constraints = true;
  config.vertical_position_velocity_consistency_sigma_m = 0.001;

  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> references(1U);
  references.front().window_index = 4U;
  references.front().boundary_role = "POST_START";
  references.front().source_type = "POST_START_VERTICAL_POSITION_VELOCITY_HANDOFF";
  references.front().target_time_s = 1.0;
  references.front().valid = true;
  references.front().has_up = true;
  references.front().has_vz = true;
  references.front().has_vertical_position_velocity_handoff = true;
  references.front().add_up_constraint = false;
  references.front().add_vz_constraint = false;
  references.front().add_vertical_position_velocity_handoff_constraint = true;
  references.front().reference_up_m = 2.0;
  references.front().reference_vz_mps = 3.0;
  references.front().vertical_position_velocity_handoff_reference_time_s = 0.5;
  references.front().vertical_position_velocity_handoff_sigma_m =
    config.vertical_position_velocity_consistency_sigma_m;
  references.front().skip_reason = "OK";

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageBoundaryDiagnosticRow> diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.boundary_references = &references;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 1U, "vertical handoff should add one boundary factor");
  ExpectTrue(
    boost::dynamic_pointer_cast<
      offline_lc_minimal::factor::VerticalPositionVelocityHandoffFactor>(graph.at(0)).get() !=
      nullptr,
    "boundary factor should be a vertical position-velocity handoff");
  ExpectTrue(
    summary.rtk_outage_boundary_vertical_position_velocity_handoff_factor_count == 1U,
    "vertical handoff factor count is wrong");
  ExpectTrue(diagnostics.size() == 1U, "vertical handoff should emit one diagnostic");
  ExpectNear(
    diagnostics.front().vertical_position_velocity_handoff_dt_s,
    -0.5,
    1.0e-12,
    "vertical handoff diagnostic should report signed reference dt");
  ExpectTrue(!diagnostics.front().up_factor_added,
             "vertical handoff must not add a direct height hold");

  gtsam::Values matching_values;
  gtsam::Values shifted_values;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    matching_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Identity(),
        gtsam::Point3(0.0, 0.0, index == 1U ? 3.0 : 0.0)));
    matching_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, index == 1U ? 1.0 : 0.0));
    shifted_values.insert(
      gtsam::symbol_shorthand::X(index),
      gtsam::Pose3(
        gtsam::Rot3::Identity(),
        gtsam::Point3(0.0, 0.0, index == 1U ? 3.2 : 0.0)));
    shifted_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, index == 1U ? 1.0 : 0.0));
  }

  ExpectNear(graph.error(matching_values), 0.0, 1.0e-12,
             "matching vertical handoff state should have zero error");
  ExpectTrue(graph.error(shifted_values) > 1.0,
             "vertical handoff should penalize position-velocity mismatch");
  offline_lc_minimal::PopulateRtkOutageBoundaryDiagnostics(
    shifted_values,
    diagnostics);
  ExpectTrue(
    std::abs(diagnostics.front().vertical_position_velocity_handoff_residual_m) > 0.1,
    "vertical handoff diagnostics should report residual");
}

void TestBoundaryConstraintBuilderSplitsTiltAndYawWhenBaseTiltReferenceIsEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_boundary_constraints = true;
  config.enable_base_graph_tilt_reference_constraint = true;
  config.base_graph_tilt_reference_sigma_rad = 0.01;
  config.rtk_outage_absolute_attitude_sigma_rad = 0.0001;

  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> references(1U);
  references.front().window_index = 4U;
  references.front().boundary_role = "POST_START";
  references.front().source_type = "OUTAGE_ATTITUDE_ONLY";
  references.front().target_time_s = 1.4;
  references.front().valid = true;
  references.front().has_attitude = true;
  references.front().add_attitude_constraint = true;
  references.front().reference_rotation = gtsam::Rot3::Ypr(0.3, -0.1, 0.2);
  references.front().attitude_sigma_rad = config.rtk_outage_absolute_attitude_sigma_rad;
  references.front().skip_reason = "OK";

  auto tilt_reference_states = MakeReferenceStates(timestamps.size());
  for (std::size_t index = 0; index < tilt_reference_states.size(); ++index) {
    tilt_reference_states[index].pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(1.0, -0.03, 0.04),
      gtsam::Point3::Zero());
  }

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageBoundaryDiagnosticRow> diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.boundary_references = &references;
  request.tilt_reference_states = &tilt_reference_states;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 2U, "split boundary attitude should add tilt and yaw factors");
  ExpectTrue(summary.rtk_outage_boundary_attitude_factor_count == 2U,
             "split boundary attitude should count both factors");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::TiltReferenceFactor>(graph.at(0)).get() !=
      nullptr,
    "first split boundary factor should constrain base tilt");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::YawReferenceFactor>(graph.at(1)).get() !=
      nullptr,
    "second split boundary factor should constrain yaw only");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::AttitudeHoldFactor>(graph.at(0)).get() ==
      nullptr,
    "split boundary path must not add full attitude hold");
  ExpectTrue(diagnostics.front().attitude_constraint_type == "tilt_yaw",
             "boundary diagnostic should report split attitude type");
  ExpectNear(
    diagnostics.front().reference_ypr_rad.x(),
    references.front().reference_rotation.ypr().x(),
    1e-12,
    "boundary yaw reference should come from the original boundary reference");
  ExpectNear(
    diagnostics.front().reference_ypr_rad.y(),
    tilt_reference_states[1].pose.rotation().ypr().y(),
    1e-12,
    "boundary pitch reference should come from base graph tilt");
}

void TestBoundaryConstraintBuilderAddsPostStartAttitude() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_boundary_constraints = true;
  config.rtk_outage_absolute_attitude_sigma_rad = 0.0001;

  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> references(1U);
  references.front().window_index = 4U;
  references.front().boundary_role = "POST_START";
  references.front().source_type = "OUTAGE_ATTITUDE_ONLY";
  references.front().target_time_s = 1.4;
  references.front().valid = true;
  references.front().has_attitude = true;
  references.front().add_attitude_constraint = true;
  references.front().reference_rotation = gtsam::Rot3::Ypr(0.3, -0.1, 0.2);
  references.front().attitude_sigma_rad = config.rtk_outage_absolute_attitude_sigma_rad;
  references.front().skip_reason = "OK";

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageBoundaryDiagnosticRow> diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.boundary_references = &references;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 1U, "post-start attitude boundary should add one factor");
  ExpectTrue(summary.rtk_outage_boundary_attitude_factor_count == 1U,
             "post-start attitude should be counted");
  ExpectTrue(diagnostics.size() == 1U, "post-start attitude should emit one diagnostic");
  ExpectTrue(diagnostics.front().target_state_index == 1U,
             "post-start boundary should bind the last state at or before the boundary");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::AttitudeHoldFactor>(graph.at(0)) !=
      nullptr,
    "post-start boundary should add an attitude hold factor");
}

void TestBoundaryConstraintBuilderKeepsImuHandoffAsFullAttitudeWhenBaseTiltIsEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_outage_boundary_constraints = true;
  config.enable_base_graph_tilt_reference_constraint = true;
  config.base_graph_tilt_reference_sigma_rad = 0.01;
  config.rtk_outage_absolute_attitude_sigma_rad = 0.0001;

  const std::vector<double> timestamps{0.0, 1.0, 2.0};
  std::vector<offline_lc_minimal::RtkOutageBoundaryReferenceRow> references(1U);
  references.front().window_index = 4U;
  references.front().boundary_role = "POST_START";
  references.front().source_type = offline_lc_minimal::kPostStartImuRelativeHandoffSource;
  references.front().target_time_s = 1.0;
  references.front().valid = true;
  references.front().has_attitude = true;
  references.front().add_attitude_constraint = true;
  references.front().reference_rotation = gtsam::Rot3::Ypr(0.3, -0.1, 0.2);
  references.front().attitude_sigma_rad = config.rtk_outage_absolute_attitude_sigma_rad;
  references.front().skip_reason = "OK";

  auto tilt_reference_states = MakeReferenceStates(timestamps.size());
  for (auto &state : tilt_reference_states) {
    state.pose = gtsam::Pose3(
      gtsam::Rot3::Ypr(0.0, 0.8, -0.7),
      gtsam::Point3::Zero());
  }

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkOutageBoundaryDiagnosticRow> diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.boundary_references = &references;
  request.tilt_reference_states = &tilt_reference_states;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::RtkOutageBoundaryConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 1U,
             "IMU handoff boundary should use one full attitude factor even with base tilt enabled");
  ExpectTrue(summary.rtk_outage_boundary_attitude_factor_count == 1U,
             "IMU handoff should count one full attitude factor");
  ExpectTrue(
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::AttitudeHoldFactor>(graph.at(0)) !=
      nullptr,
    "IMU handoff should not split pitch/roll away from the propagated reference");
  ExpectTrue(diagnostics.front().attitude_constraint_type == "relative_handoff",
             "boundary diagnostic should identify relative handoff attitude");
}

void TestBoundaryAttitudeHandoffUsesOutageLastPlusImuDelta() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_absolute_attitude_sigma_rad = 0.001;

  offline_lc_minimal::DataSet dataset;
  dataset.imu_samples = {
    {0.0, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d::Zero()},
    {1.0, Eigen::Vector3d(0.0, 0.0, 0.1), Eigen::Vector3d::Zero()},
  };

  offline_lc_minimal::OfflineRunResult outage_result;
  outage_result.optimized_reference_states.resize(2U);
  outage_result.optimized_reference_states[0].time_s = 0.0;
  outage_result.optimized_reference_states[0].pose =
    gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3::Zero());
  outage_result.optimized_reference_states[0].velocity = gtsam::Vector3::Zero();
  outage_result.optimized_reference_states[1].time_s = 0.5;
  outage_result.optimized_reference_states[1].pose =
    gtsam::Pose3(gtsam::Rot3::Identity(), gtsam::Point3::Zero());
  outage_result.optimized_reference_states[1].velocity = gtsam::Vector3::Zero();

  offline_lc_minimal::RtkOutageWindowRow outage;
  outage.window_index = 3U;
  outage.start_time_s = 0.0;
  outage.end_time_s = 1.0;

  const auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(9.81);
  imu_params->accelerometerCovariance = 1.0e-6 * gtsam::I_3x3;
  imu_params->gyroscopeCovariance = 1.0e-6 * gtsam::I_3x3;
  imu_params->integrationCovariance = 1.0e-6 * gtsam::I_3x3;
  imu_params->biasAccCovariance = 1.0e-6 * gtsam::I_3x3;
  imu_params->biasOmegaCovariance = 1.0e-6 * gtsam::I_3x3;

  const auto handoff =
    offline_lc_minimal::BuildRtkOutageBoundaryAttitudeHandoff(
      offline_lc_minimal::RtkOutageBoundaryAttitudeHandoffRequest{
        &config,
        &dataset,
        &outage_result,
        &outage,
        imu_params,
        1.0});

  ExpectTrue(handoff.valid, "IMU handoff should be valid");
  ExpectNear(
    handoff.boundary_reference.reference_rotation.ypr().x(),
    0.05,
    1.0e-9,
    "handoff yaw should equal outage last yaw plus integrated gyro delta");
  ExpectNear(
    handoff.diagnostic.reference_relative_rotvec_rad.z(),
    0.05,
    1.0e-9,
    "handoff diagnostic should expose the IMU relative rotation");

  offline_lc_minimal::RtkOutageBoundaryReferenceRow post_reference;
  post_reference.window_index = 3U;
  post_reference.boundary_role = "POST_START";
  post_reference.source_type = "POST_RECOVERY_RTK";
  post_reference.target_time_s = 1.0;
  post_reference.valid = true;
  post_reference.has_up = true;
  post_reference.add_up_constraint = true;
  post_reference.reference_up_m = 12.0;
  offline_lc_minimal::AttachRtkOutageBoundaryAttitudeHandoff(
    handoff,
    post_reference);
  ExpectTrue(post_reference.source_type == offline_lc_minimal::kPostStartImuRelativeHandoffSource,
             "post reference should identify IMU handoff as the attitude source");
  ExpectTrue(post_reference.has_up && post_reference.add_up_constraint,
             "attaching attitude handoff should preserve recovery position/velocity fields");

  offline_lc_minimal::OfflineRunResult post_result;
  post_result.optimized_reference_states.resize(1U);
  post_result.optimized_reference_states.front().time_s = 1.0;
  post_result.optimized_reference_states.front().pose =
    gtsam::Pose3(gtsam::Rot3::Ypr(0.05, 0.0, 0.0), gtsam::Point3::Zero());
  auto diagnostic = handoff.diagnostic;
  offline_lc_minimal::PopulateRtkOutageBoundaryAttitudeHandoffDiagnostic(
    post_result,
    diagnostic);
  ExpectNear(
    diagnostic.residual_norm_rad,
    0.0,
    1.0e-12,
    "matching post start attitude should have zero handoff residual");
}

void TestBoundaryBiasHandoffUsesOutageLastBias() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_outage_boundary_baz_sigma_mps2 = 2.0e-4;

  offline_lc_minimal::OfflineRunResult outage_result;
  outage_result.optimized_reference_states.resize(3U);
  outage_result.optimized_reference_states[0].time_s = 0.0;
  outage_result.optimized_reference_states[0].bias =
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.0, 0.0, 0.01),
      gtsam::Vector3::Zero());
  outage_result.optimized_reference_states[1].time_s = 0.5;
  outage_result.optimized_reference_states[1].bias =
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.0, 0.0, -0.123),
      gtsam::Vector3::Zero());
  outage_result.optimized_reference_states[2].time_s = 1.0;
  outage_result.optimized_reference_states[2].bias =
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.0, 0.0, 0.5),
      gtsam::Vector3::Zero());

  offline_lc_minimal::RtkOutageWindowRow outage;
  outage.window_index = 4U;
  outage.start_time_s = 0.0;
  outage.end_time_s = 1.0;

  const auto handoff =
    offline_lc_minimal::BuildRtkOutageBoundaryBiasHandoff(
      offline_lc_minimal::RtkOutageBoundaryBiasHandoffRequest{
        &config,
        &outage_result,
        &outage,
        1.0});

  ExpectTrue(handoff.valid, "ba_z handoff should be valid");
  ExpectTrue(handoff.boundary_reference.boundary_role == "POST_START",
             "ba_z handoff should target post start");
  ExpectNear(
    handoff.boundary_reference.reference_ba_z_mps2,
    -0.123,
    1.0e-12,
    "ba_z handoff must use the last kept outage state before post start");
  ExpectNear(
    handoff.boundary_reference.ba_z_sigma_mps2,
    config.rtk_outage_boundary_baz_sigma_mps2,
    1.0e-12,
    "ba_z handoff should use the configured boundary sigma");

  offline_lc_minimal::RtkOutageBoundaryReferenceRow post_reference;
  post_reference.source_type =
    offline_lc_minimal::kPostStartImuRelativeHandoffSource;
  post_reference.boundary_role = "POST_START";
  post_reference.target_time_s = 1.0;
  post_reference.valid = true;
  post_reference.has_up = true;
  post_reference.add_up_constraint = true;
  post_reference.reference_up_m = 12.0;
  offline_lc_minimal::AttachRtkOutageBoundaryBiasHandoff(
    handoff,
    post_reference);
  ExpectTrue(post_reference.source_type ==
               offline_lc_minimal::kPostStartImuRelativeHandoffSource,
             "attitude handoff source should be preserved when adding ba_z");
  ExpectTrue(post_reference.has_up && post_reference.add_up_constraint,
             "attaching ba_z handoff should preserve recovery up constraint");
  ExpectTrue(post_reference.has_ba_z && post_reference.add_ba_z_constraint,
             "post reference should get a ba_z constraint");
  ExpectNear(
    post_reference.reference_ba_z_mps2,
    -0.123,
    1.0e-12,
    "attached post reference should carry outage-last ba_z");
}

void TestBoundaryBiasHandoffUsesGmScaleSigmaWhenEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_acc_bias_gm_process = true;
  config.vertical_acc_bias_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.1);
  config.rtk_outage_boundary_baz_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(50.0);

  offline_lc_minimal::OfflineRunResult outage_result;
  outage_result.optimized_reference_states.resize(2U);
  outage_result.optimized_reference_states[0].time_s = 0.0;
  outage_result.optimized_reference_states[0].bias =
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.0, 0.0, 0.01),
      gtsam::Vector3::Zero());
  outage_result.optimized_reference_states[1].time_s = 0.5;
  outage_result.optimized_reference_states[1].bias =
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.0, 0.0, -0.02),
      gtsam::Vector3::Zero());

  offline_lc_minimal::RtkOutageWindowRow outage;
  outage.window_index = 4U;
  outage.start_time_s = 0.0;
  outage.end_time_s = 1.0;

  const auto handoff =
    offline_lc_minimal::BuildRtkOutageBoundaryBiasHandoff(
      offline_lc_minimal::RtkOutageBoundaryBiasHandoffRequest{
        &config,
        &outage_result,
        &outage,
        1.0});

  ExpectTrue(handoff.valid, "ba_z handoff should be valid");
  ExpectNear(
    handoff.boundary_reference.ba_z_sigma_mps2,
    config.vertical_acc_bias_sigma_mps2,
    1.0e-12,
    "ba_z handoff should use GM-scale sigma when GM process is enabled");
}

offline_lc_minimal::GnssSolutionSample MakeCausalReferenceSample(
  const double time_s,
  const double up_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.enu_position_m = Eigen::Vector3d(0.0, 0.0, up_m);
  sample.has_enu_position = true;
  return sample;
}

offline_lc_minimal::TrajectoryRow MakeCausalReferenceTrajectoryRow(
  const double time_s,
  const double up_m,
  const double vz_mps) {
  offline_lc_minimal::TrajectoryRow row;
  row.time_s = time_s;
  row.enu_position_m = Eigen::Vector3d(0.0, 0.0, up_m);
  row.enu_velocity_mps = Eigen::Vector3d(0.0, 0.0, vz_mps);
  row.ypr_rad = Eigen::Vector3d::Zero();
  row.omega_radps = Eigen::Vector3d::Zero();
  return row;
}

void TestCausalReferenceBuilderUsesPrefixBaseConfig() {
  auto current_config = offline_lc_minimal::DefaultConfig();
  current_config.enable_rtk_outage_causal_drift_reference = true;
  current_config.enable_rtk_outage_preoutage_vertical_fence = true;
  current_config.rtk_outage_causal_reference_max_prefix_runs = 1;
  current_config.stage1_heading_window_s = 1.0;

  auto prefix_base_config = current_config;
  prefix_base_config.stage1_heading_window_s = 42.0;
  prefix_base_config.enable_initial_yaw_override = false;

  offline_lc_minimal::DataSet dataset;
  dataset.gnss_samples = {
    MakeCausalReferenceSample(0.0, 1.0),
    MakeCausalReferenceSample(0.5, 2.0),
    MakeCausalReferenceSample(1.5, 3.0),
  };

  std::vector<offline_lc_minimal::RtkOutageWindowRow> outage_windows(1U);
  outage_windows.front().start_time_s = 1.0;
  outage_windows.front().end_time_s = 2.0;

  bool prefix_called = false;
  offline_lc_minimal::RtkOutageCausalReferenceBuildRequest request;
  request.config = &current_config;
  request.prefix_base_config = &prefix_base_config;
  request.dataset = &dataset;
  request.outage_windows = &outage_windows;
  request.dynamic_start_time_s = -1.0;
  request.should_use_sample = [](const offline_lc_minimal::GnssSolutionSample &) {
    return true;
  };
  request.corrected_time_s = [](const offline_lc_minimal::GnssSolutionSample &sample) {
    return sample.time_s;
  };
  request.is_within_imu_coverage = [](double) { return true; };
  request.run_prefix = [&](offline_lc_minimal::OfflineRunnerConfig config,
                           offline_lc_minimal::DataSet run_dataset) {
    prefix_called = true;
    ExpectTrue(run_dataset.gnss_samples.size() == dataset.gnss_samples.size(),
               "prefix builder should pass the source dataset through");
    ExpectNear(config.processing_end_time_s, 1.0, 1.0e-12,
               "prefix processing end should be the first outage start");
    ExpectNear(config.stage1_heading_window_s, 42.0, 1.0e-12,
               "prefix builder should use the causal base config");
    ExpectTrue(!config.enable_rtk_outage_causal_drift_reference,
               "prefix run should disable causal reference recursion");
    ExpectTrue(!config.enable_rtk_outage_preoutage_vertical_fence,
               "prefix run should disable its own pre-outage fence");

    offline_lc_minimal::OfflineRunResult result;
    result.trajectory = {
      MakeCausalReferenceTrajectoryRow(0.0, 10.0, 0.1),
      MakeCausalReferenceTrajectoryRow(0.5, 11.0, 0.2),
      MakeCausalReferenceTrajectoryRow(1.0, 12.0, 0.3),
    };
    return result;
  };

  const offline_lc_minimal::RtkOutageCausalReferenceResult result =
    offline_lc_minimal::RtkOutageCausalReferenceBuilder(std::move(request)).Build();

  ExpectTrue(prefix_called, "causal reference builder should run the prefix solve");
  ExpectTrue(result.valid, "causal reference builder should return a valid result");
  ExpectTrue(result.prefix_run_count == 1U, "causal reference builder should run once");
  ExpectTrue(result.nav_reference_rows.size() == dataset.gnss_samples.size(),
             "causal nav reference rows should align with GNSS samples");
  ExpectTrue(result.nav_reference_rows[0].valid,
             "pre-outage GNSS sample should get a causal reference");
  ExpectNear(result.nav_reference_rows[0].causal_nav_reference_up_m, 10.0, 1.0e-12,
             "causal nav reference should come from the prefix trajectory");
  ExpectTrue(!result.nav_reference_rows[2].valid,
             "post-outage GNSS sample should not get a pre-outage causal reference");
  ExpectTrue(result.state_reference_rows.size() == 3U,
             "state references should cover prefix states through the outage boundary");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestAttitudeHoldFactorIgnoresTranslationAndPenalizesAttitude",
      TestAttitudeHoldFactorIgnoresTranslationAndPenalizesAttitude);
    RunTest("TestVelocityDeltaFactorUsesOnlyVelocityKeys", TestVelocityDeltaFactorUsesOnlyVelocityKeys);
    RunTest(
      "TestHorizontalPositionVelocityHandoffFactorUsesTrapezoidVelocityDelta",
      TestHorizontalPositionVelocityHandoffFactorUsesTrapezoidVelocityDelta);
    RunTest(
      "TestBuilderAddsOutageAttitudeAndVelocityConstraints",
      TestBuilderAddsOutageAttitudeAndVelocityConstraints);
    RunTest(
      "TestOutageVelocityDeltaUsesVerticalClampedVzTarget",
      TestOutageVelocityDeltaUsesVerticalClampedVzTarget);
    RunTest(
      "TestBuilderSplitsOutageTiltAndYawWhenBaseTiltReferenceIsEnabled",
      TestBuilderSplitsOutageTiltAndYawWhenBaseTiltReferenceIsEnabled);
    RunTest("TestBuilderDisabledAddsNoFactors", TestBuilderDisabledAddsNoFactors);
    RunTest(
      "TestPreOutageVerticalFenceConstrainsOnlyUpAndVz",
      TestPreOutageVerticalFenceConstrainsOnlyUpAndVz);
    RunTest(
      "TestPreOutageVerticalFenceDisabledAddsNoFactors",
      TestPreOutageVerticalFenceDisabledAddsNoFactors);
    RunTest(
      "TestBoundaryAttitudeReferenceAnchorsStartAndEndWithImuDelta",
      TestBoundaryAttitudeReferenceAnchorsStartAndEndWithImuDelta);
    RunTest(
      "TestBoundaryAttitudeReferencePropagatesThroughGuardedSpan",
      TestBoundaryAttitudeReferencePropagatesThroughGuardedSpan);
    RunTest(
      "TestBoundaryAttitudeReferenceUsesPostStartAsSingleSidedAnchor",
      TestBoundaryAttitudeReferenceUsesPostStartAsSingleSidedAnchor);
    RunTest(
      "TestBoundaryAttitudeReferenceKeepsOffGridAnchorsExact",
      TestBoundaryAttitudeReferenceKeepsOffGridAnchorsExact);
    RunTest(
      "TestOutageEndBoundaryReferenceFromRecoveryUsesRecoveryPositionVelocity",
      TestOutageEndBoundaryReferenceFromRecoveryUsesRecoveryPositionVelocity);
    RunTest(
      "TestBoundaryStateInitialValuesDoNotRequireAttitude",
      TestBoundaryStateInitialValuesDoNotRequireAttitude);
    RunTest(
      "TestBoundaryConstraintBuilderUsesBoundarySideForOffGridTimes",
      TestBoundaryConstraintBuilderUsesBoundarySideForOffGridTimes);
    RunTest(
      "TestBoundaryConstraintBuilderConstrainsUpVzBazAndAttitude",
      TestBoundaryConstraintBuilderConstrainsUpVzBazAndAttitude);
    RunTest(
      "TestBoundaryConstraintBuilderAddsHorizontalPositionVelocityHandoff",
      TestBoundaryConstraintBuilderAddsHorizontalPositionVelocityHandoff);
    RunTest(
      "TestBoundaryConstraintBuilderAddsVerticalPositionVelocityHandoff",
      TestBoundaryConstraintBuilderAddsVerticalPositionVelocityHandoff);
    RunTest(
      "TestBoundaryConstraintBuilderSplitsTiltAndYawWhenBaseTiltReferenceIsEnabled",
      TestBoundaryConstraintBuilderSplitsTiltAndYawWhenBaseTiltReferenceIsEnabled);
    RunTest(
      "TestBoundaryConstraintBuilderAddsPostStartAttitude",
      TestBoundaryConstraintBuilderAddsPostStartAttitude);
    RunTest(
      "TestBoundaryConstraintBuilderKeepsImuHandoffAsFullAttitudeWhenBaseTiltIsEnabled",
      TestBoundaryConstraintBuilderKeepsImuHandoffAsFullAttitudeWhenBaseTiltIsEnabled);
    RunTest(
      "TestBoundaryAttitudeHandoffUsesOutageLastPlusImuDelta",
      TestBoundaryAttitudeHandoffUsesOutageLastPlusImuDelta);
    RunTest(
      "TestBoundaryBiasHandoffUsesOutageLastBias",
      TestBoundaryBiasHandoffUsesOutageLastBias);
    RunTest(
      "TestBoundaryBiasHandoffUsesGmScaleSigmaWhenEnabled",
      TestBoundaryBiasHandoffUsesGmScaleSigmaWhenEnabled);
    RunTest(
      "TestCausalReferenceBuilderUsesPrefixBaseConfig",
      TestCausalReferenceBuilderUsesPrefixBaseConfig);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
