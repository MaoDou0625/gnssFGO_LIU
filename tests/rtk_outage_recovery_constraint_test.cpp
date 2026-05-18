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

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/RtkOutageCausalReferenceBuilder.h"
#include "offline_lc_minimal/core/RtkOutagePreOutageVerticalFenceBuilder.h"
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
      "TestBuilderAddsOutageAttitudeAndVelocityConstraints",
      TestBuilderAddsOutageAttitudeAndVelocityConstraints);
    RunTest("TestBuilderDisabledAddsNoFactors", TestBuilderDisabledAddsNoFactors);
    RunTest(
      "TestPreOutageVerticalFenceConstrainsOnlyUpAndVz",
      TestPreOutageVerticalFenceConstrainsOnlyUpAndVz);
    RunTest(
      "TestPreOutageVerticalFenceDisabledAddsNoFactors",
      TestPreOutageVerticalFenceDisabledAddsNoFactors);
    RunTest(
      "TestCausalReferenceBuilderUsesPrefixBaseConfig",
      TestCausalReferenceBuilderUsesPrefixBaseConfig);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
