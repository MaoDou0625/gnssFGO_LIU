#include <algorithm>
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

#include "offline_lc_minimal/core/BodyZNHCConstraintBuilder.h"
#include "offline_lc_minimal/core/BodyZHorizontalLeakageEstimator.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/factor/BodyZLeakageCorrectedVelocityFactor.h"
#include "offline_lc_minimal/factor/BodyZLeakageCorrectedWindowDisplacementFactor.h"
#include "offline_lc_minimal/factor/BodyZVelocityZeroFactor.h"
#include "offline_lc_minimal/factor/BodyZWindowDisplacementZeroFactor.h"

namespace {

namespace symbol = gtsam::symbol_shorthand;

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

gtsam::Values MakeValues(
  const std::vector<double> &timestamps,
  const gtsam::Rot3 &rotation,
  const gtsam::Vector3 &velocity) {
  gtsam::Values values;
  for (std::size_t index = 0U; index < timestamps.size(); ++index) {
    values.insert(symbol::X(index), gtsam::Pose3(rotation, gtsam::Point3(0.0, 0.0, 0.0)));
    values.insert(symbol::V(index), velocity);
  }
  return values;
}

offline_lc_minimal::BodyZSeedJumpWindowRow MakeJumpWindow(
  const double start_time_s,
  const double end_time_s) {
  offline_lc_minimal::BodyZSeedJumpWindowRow window;
  window.start_time_s = start_time_s;
  window.end_time_s = end_time_s;
  return window;
}

void TestBodyZVelocityZeroFactorUsesBodyFrameZ() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::BodyZVelocityZeroFactor identity_factor(
    symbol::V(0),
    gtsam::Vector3(0.0, 0.0, 1.0),
    noise);
  ExpectNear(
    identity_factor.evaluateError(gtsam::Vector3(0.0, 0.0, 0.25))(0),
    0.25,
    1e-12,
    "identity fixed axis should use nav z as body z");

  const gtsam::Pose3 tilted_pose(
    gtsam::Rot3::RzRyRx(0.0, 1.5707963267948966, 0.0),
    gtsam::Point3(0.0, 0.0, 0.0));
  const offline_lc_minimal::factor::BodyZVelocityZeroFactor tilted_factor(
    symbol::V(1),
    offline_lc_minimal::factor::BodyZAxisNavFromPose(tilted_pose),
    noise);
  const gtsam::Vector3 horizontal_velocity(1.0, 0.0, 0.0);
  const double residual = tilted_factor.evaluateError(horizontal_velocity)(0);
  ExpectTrue(std::abs(horizontal_velocity.z()) < 1e-12, "test velocity should have zero nav z");
  ExpectTrue(std::abs(residual) > 0.5, "tilted fixed axis should project horizontal nav velocity into body z");
  ExpectNear(
    static_cast<double>(tilted_factor.keys().size()),
    1.0,
    0.0,
    "fixed-axis velocity factor should only connect the velocity key");

  gtsam::Matrix h_velocity;
  const gtsam::Vector unused_residual = tilted_factor.evaluateError(horizontal_velocity, h_velocity);
  (void)unused_residual;
  ExpectNear(h_velocity.rows(), 1.0, 0.0, "velocity Jacobian should have one residual row");
  ExpectNear(h_velocity.cols(), 3.0, 0.0, "velocity Jacobian should have three velocity columns");
  ExpectNear(
    h_velocity(0, 0),
    tilted_factor.bodyZAxisNav().x(),
    1e-12,
    "velocity Jacobian should use the fixed nav-frame body-z axis");
}

void TestBodyZWindowDisplacementZeroFactorIntegratesBodyZVelocity() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const std::vector<gtsam::Key> velocity_keys{symbol::V(0), symbol::V(1), symbol::V(2)};
  const std::vector<gtsam::Vector3> body_z_axes_nav{
    gtsam::Vector3(0.0, 0.0, 1.0),
    gtsam::Vector3(0.0, 0.0, 1.0),
    gtsam::Vector3(0.0, 0.0, 1.0),
  };
  const std::vector<double> times{0.0, 0.5, 1.0};
  const offline_lc_minimal::factor::BodyZWindowDisplacementZeroFactor factor(
    velocity_keys,
    body_z_axes_nav,
    times,
    noise);

  gtsam::Values values = MakeValues(times, gtsam::Rot3(), gtsam::Vector3(0.0, 0.0, 0.0));
  ExpectNear(factor.unwhitenedError(values)(0), 0.0, 1e-12, "zero body-z velocity should integrate to zero");

  values = MakeValues(times, gtsam::Rot3(), gtsam::Vector3(0.0, 0.0, 0.10));
  ExpectNear(factor.unwhitenedError(values)(0), 0.10, 1e-12, "constant body-z velocity integral is wrong");
  ExpectNear(
    static_cast<double>(factor.keys().size()),
    3.0,
    0.0,
    "fixed-axis displacement factor should only connect velocity keys");
}

void TestLeakageCorrectedVelocityFactorUsesVelocityOnly() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const auto axes = offline_lc_minimal::factor::BodyFrameAxesNavFromPose(gtsam::Pose3());
  offline_lc_minimal::factor::BodyZHorizontalLeakageModel leakage;
  leakage.leak_x_rad = 0.10;
  leakage.leak_y_rad = -0.20;
  const offline_lc_minimal::factor::BodyZLeakageCorrectedVelocityZeroFactor factor(
    symbol::V(0),
    axes,
    leakage,
    noise);

  const gtsam::Vector3 velocity(2.0, 3.0, 1.0);
  ExpectNear(
    factor.evaluateError(velocity)(0),
    1.0 - 0.10 * 2.0 - (-0.20) * 3.0,
    1e-12,
    "leakage-corrected velocity residual is wrong");
  ExpectNear(
    static_cast<double>(factor.keys().size()),
    1.0,
    0.0,
    "leakage-corrected velocity factor should only connect velocity");

  gtsam::Matrix h_velocity;
  const gtsam::Vector unused = factor.evaluateError(velocity, h_velocity);
  (void)unused;
  ExpectNear(h_velocity(0, 0), -0.10, 1e-12, "x leakage Jacobian is wrong");
  ExpectNear(h_velocity(0, 1), 0.20, 1e-12, "y leakage Jacobian is wrong");
  ExpectNear(h_velocity(0, 2), 1.0, 1e-12, "z leakage Jacobian is wrong");
}

void TestLeakageCorrectedWindowDisplacementFactorIntegratesCorrectedVelocity() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const std::vector<gtsam::Key> velocity_keys{symbol::V(0), symbol::V(1), symbol::V(2)};
  const auto axes = offline_lc_minimal::factor::BodyFrameAxesNavFromPose(gtsam::Pose3());
  const std::vector<offline_lc_minimal::factor::BodyFrameAxesNav> body_axes{axes, axes, axes};
  offline_lc_minimal::factor::BodyZHorizontalLeakageModel leakage;
  leakage.leak_x_rad = 0.10;
  const std::vector<double> times{0.0, 0.5, 1.0};
  const offline_lc_minimal::factor::BodyZLeakageCorrectedWindowDisplacementZeroFactor factor(
    velocity_keys,
    body_axes,
    leakage,
    times,
    noise);

  const gtsam::Values values =
    MakeValues(times, gtsam::Rot3(), gtsam::Vector3(1.0, 0.0, 0.10));
  ExpectNear(
    factor.unwhitenedError(values)(0),
    0.0,
    1e-12,
    "constant leakage-corrected zero body-z velocity should integrate to zero");
}

void TestBodyZHorizontalLeakageEstimatorUsesOutsideWindowSamples() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_horizontal_leakage_correction = true;
  config.body_z_nhc_horizontal_leakage_min_speed_mps = 0.0;
  config.body_z_nhc_horizontal_leakage_min_sample_count = 3;
  config.body_z_nhc_horizontal_leakage_huber_sigma_mps = 0.01;
  config.body_z_nhc_horizontal_leakage_max_abs_coeff_rad = 0.50;
  config.body_z_nhc_horizontal_leakage_guard_s = 0.0;
  const std::vector<double> timestamps{0.0, 0.5, 1.0, 1.5, 2.0};
  const std::vector<gtsam::Vector3> velocities{
    gtsam::Vector3(1.0, 0.0, 0.10),
    gtsam::Vector3(0.0, 1.0, -0.02),
    gtsam::Vector3(5.0, 5.0, 0.0),
    gtsam::Vector3(5.0, 5.0, 0.0),
    gtsam::Vector3(1.0, 2.0, 0.06),
  };
  gtsam::Values values;
  std::vector<offline_lc_minimal::ReferenceNodeState> reference_states;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
    values.insert(symbol::X(index), pose);
    values.insert(symbol::V(index), velocities[index]);
    offline_lc_minimal::ReferenceNodeState state;
    state.time_s = timestamps[index];
    state.pose = pose;
    state.velocity = velocities[index];
    reference_states.push_back(state);
  }
  const std::vector<offline_lc_minimal::BodyZJumpConstraintWindow> excluded_windows{
    offline_lc_minimal::BodyZJumpConstraintWindow{0U, 1U, 1.0, 1.5},
  };

  offline_lc_minimal::BodyZHorizontalLeakageEstimateRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.excluded_windows = &excluded_windows;
  request.initial_values = &values;
  request.reference_states = &reference_states;
  request.dynamic_start_index = 0U;
  const auto estimate =
    offline_lc_minimal::BodyZHorizontalLeakageEstimator(std::move(request)).Estimate();

  ExpectTrue(estimate.valid, "leakage estimator should accept outside-window samples");
  ExpectNear(estimate.model.leak_x_rad, 0.10, 1e-12, "estimated x leakage is wrong");
  ExpectNear(estimate.model.leak_y_rad, -0.02, 1e-12, "estimated y leakage is wrong");
  ExpectNear(
    static_cast<double>(estimate.diagnostic.skipped_window_count),
    2.0,
    0.0,
    "estimator should exclude jump-window samples");
  ExpectTrue(
    estimate.diagnostic.velocity_source == "REFERENCE_STATES",
    "estimator should prefer reference states when available");
}

void TestFixedAxisGraphErrorIgnoresPoseChanges() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = true;
  config.body_z_nhc_jump_padding_s = 0.0;
  config.body_z_nhc_min_window_s = 0.5;
  const std::vector<double> timestamps{0.0, 0.5, 1.0};
  const gtsam::Rot3 tilted_rotation = gtsam::Rot3::RzRyRx(0.0, 1.5707963267948966, 0.0);
  const gtsam::Values initial_values =
    MakeValues(timestamps, tilted_rotation, gtsam::Vector3(1.0, 0.0, 0.0));
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(0.0, 1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  gtsam::Values changed_pose_values =
    MakeValues(timestamps, gtsam::Rot3(), gtsam::Vector3(1.0, 0.0, 0.0));
  ExpectNear(
    graph.error(initial_values),
    graph.error(changed_pose_values),
    1e-12,
    "fixed-axis NHC graph error should not change when only optimized poses change");
}

void TestPopulateBodyZNHCDiagnosticsUsesFixedInitialAxis() {
  const std::vector<double> timestamps{0.0, 0.5, 1.0};
  const gtsam::Rot3 tilted_rotation = gtsam::Rot3::RzRyRx(0.0, 1.5707963267948966, 0.0);
  const gtsam::Values initial_values =
    MakeValues(timestamps, tilted_rotation, gtsam::Vector3::Zero());
  const gtsam::Values optimized_values =
    MakeValues(timestamps, gtsam::Rot3(), gtsam::Vector3(1.0, 0.0, 0.0));

  offline_lc_minimal::BodyZNHCDiagnosticRow row;
  row.factor_added = true;
  row.start_state_index = 0U;
  row.end_state_index = 2U;
  row.state_count = 3U;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics{row};
  std::vector<offline_lc_minimal::BodyZNHCStateDiagnosticRow> state_diagnostics;

  offline_lc_minimal::PopulateBodyZNHCDiagnostics(
    initial_values,
    optimized_values,
    timestamps,
    diagnostics,
    &state_diagnostics);

  ExpectNear(
    diagnostics.front().optimized_mean_abs_body_z_velocity_mps,
    1.0,
    1e-12,
    "postfit diagnostics should use the initial fixed body-z axis");
  ExpectNear(
    diagnostics.front().optimized_pose_mean_abs_body_z_velocity_mps,
    0.0,
    1e-12,
    "pose-based diagnostic should still show the optimized-pose projection");
  ExpectNear(
    diagnostics.front().optimized_body_z_displacement_m,
    1.0,
    1e-12,
    "fixed-axis postfit displacement should use the initial fixed body-z axis");
  ExpectTrue(state_diagnostics.size() == 3U, "state diagnostics should include each NHC state");
  ExpectNear(
    state_diagnostics.front().fixed_horizontal_projection_mps,
    1.0,
    1e-12,
    "state diagnostics should expose the fixed-axis horizontal projection");
  ExpectNear(
    state_diagnostics.front().fixed_vertical_projection_mps,
    0.0,
    1e-12,
    "state diagnostics should expose the fixed-axis vertical projection");
  ExpectNear(
    state_diagnostics.front().fixed_body_z_velocity_mps,
    1.0,
    1e-12,
    "state diagnostics should decompose fixed-axis body-z velocity");
}

void TestBuilderAddsJumpWindowNHC() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_body_z_nhc_constraint = true;
  config.body_z_nhc_jump_padding_s = 0.0;
  config.body_z_nhc_min_window_s = 0.5;
  const std::vector<double> timestamps{0.0, 0.5, 1.0, 1.5, 2.0};
  const gtsam::Values values = MakeValues(
    timestamps,
    gtsam::Rot3::RzRyRx(0.0, 1.5707963267948966, 0.0),
    gtsam::Vector3(1.0, 0.0, 0.0));
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(0.5, 1.5),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(summary.body_z_nhc_window_count), 1.0, 0.0, "one NHC window should be added");
  ExpectNear(static_cast<double>(summary.body_z_nhc_velocity_factor_count), 3.0, 0.0, "jump window velocity factor count is wrong");
  ExpectNear(static_cast<double>(summary.body_z_nhc_displacement_factor_count), 1.0, 0.0, "jump window displacement factor count is wrong");
  ExpectNear(static_cast<double>(graph.size()), 4.0, 0.0, "jump NHC should add one velocity factor per state plus one displacement factor");
  ExpectNear(static_cast<double>(graph.at(0)->keys().size()), 1.0, 0.0, "velocity NHC should not connect pose keys");
  ExpectNear(static_cast<double>(graph.at(3)->keys().size()), 3.0, 0.0, "window NHC should connect one velocity key per state");
  ExpectTrue(diagnostics.size() == 1U && diagnostics.front().from_jump_window, "jump NHC diagnostic should be recorded");
  ExpectNear(diagnostics.front().velocity_sigma_mps, config.body_z_nhc_jump_velocity_sigma_mps, 1e-12, "jump NHC should use jump velocity sigma");
  ExpectTrue(
    diagnostics.front().initial_mean_abs_body_z_velocity_mps > 0.5,
    "builder should derive fixed body-z axes from tilted initial poses");
}

void TestBuilderMergesOverlappingJumpWindows() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_body_z_nhc_constraint = true;
  config.body_z_nhc_jump_padding_s = 0.0;
  config.body_z_nhc_merge_gap_s = 0.25;
  config.body_z_nhc_min_window_s = 0.5;
  const std::vector<double> timestamps{0.0, 0.5, 1.0, 1.5, 2.0};
  const gtsam::Values values = MakeValues(timestamps, gtsam::Rot3(), gtsam::Vector3::Zero());
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(0.5, 1.0),
    MakeJumpWindow(1.15, 1.5),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(summary.body_z_nhc_window_count), 1.0, 0.0, "overlapping jump windows should merge");
  ExpectTrue(diagnostics.size() == 1U, "merged jump NHC should produce one diagnostic row");
  ExpectNear(static_cast<double>(diagnostics.front().source_window_count), 2.0, 0.0, "merged source count is wrong");
}

void TestJumpConstraintWindowPlannerAllowsLongMergedWindows() {
  offline_lc_minimal::BodyZJumpConstraintWindowOptions options;
  options.padding_s = 0.0;
  options.merge_gap_s = 1.0;
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(0.0, 1.0),
    MakeJumpWindow(1.5, 2.5),
    MakeJumpWindow(3.0, 4.0),
  };

  const std::vector<offline_lc_minimal::BodyZJumpConstraintWindow> windows =
    offline_lc_minimal::BuildBodyZJumpConstraintWindows(jump_windows, options);
  ExpectNear(static_cast<double>(windows.size()), 1.0, 0.0, "planner should merge adjacent windows by gap only");
  ExpectNear(windows.front().start_time_s, 0.0, 1e-12, "merged window start is wrong");
  ExpectNear(windows.front().end_time_s, 4.0, 1e-12, "merged window end is wrong");
  ExpectNear(
    static_cast<double>(windows.front().source_window_count),
    3.0,
    0.0,
    "merged window should retain merged source count");
}

void TestBuilderSkipsShortWindow() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_body_z_nhc_constraint = true;
  config.body_z_nhc_jump_padding_s = 0.0;
  config.body_z_nhc_min_window_s = 0.5;
  const std::vector<double> timestamps{0.0, 0.1, 1.0};
  const gtsam::Values values = MakeValues(timestamps, gtsam::Rot3(), gtsam::Vector3::Zero());
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(0.0, 0.6),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.empty(), "short NHC window should not add factors");
  ExpectNear(static_cast<double>(summary.body_z_nhc_skipped_short_window_count), 1.0, 0.0, "short skip count is wrong");
  ExpectTrue(!diagnostics.empty() && diagnostics.front().skip_reason == "SHORT_WINDOW", "short skip diagnostic is wrong");
  ExpectNear(diagnostics.front().actual_state_span_s, 0.1, 1e-12, "short window should be gated on actual state span");
}

void TestBuilderDoesNotAddGlobalWeakWindowsWhenDisabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_global_weak_constraint = false;
  const std::vector<double> timestamps{0.0, 0.5, 1.0, 1.5, 2.0};
  const gtsam::Values values = MakeValues(timestamps, gtsam::Rot3(), gtsam::Vector3::Zero());
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.empty(), "global weak NHC should remain disabled");
  ExpectTrue(diagnostics.empty(), "disabled global weak NHC should not emit window diagnostics");
}

void TestBuilderAddsGlobalWeakWindowsWhenEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_global_weak_constraint = true;
  config.body_z_nhc_min_window_s = 0.5;
  config.body_z_nhc_global_window_s = 1.0;
  config.body_z_nhc_global_stride_s = 1.0;
  const std::vector<double> timestamps{0.0, 0.5, 1.0, 1.5, 2.0};
  const gtsam::Values values = MakeValues(timestamps, gtsam::Rot3(), gtsam::Vector3::Zero());
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(summary.body_z_nhc_window_count), 2.0, 0.0, "global weak NHC should add two windows");
  ExpectNear(static_cast<double>(summary.body_z_nhc_displacement_factor_count), 2.0, 0.0, "global weak displacement factor count is wrong");
  ExpectTrue(diagnostics.size() == 2U, "global weak diagnostics should include two rows");
  ExpectTrue(!diagnostics.front().from_jump_window, "global weak windows should not be marked as jump windows");
  ExpectNear(
    diagnostics.front().velocity_sigma_mps,
    config.body_z_nhc_global_velocity_sigma_mps,
    1e-12,
    "global weak NHC should use global velocity sigma");
}

void TestBuilderStrictWeightingUsesUniqueVelocityFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_global_weak_constraint = true;
  config.enable_body_z_nhc_strict_effective_weighting = true;
  config.body_z_nhc_jump_padding_s = 0.0;
  config.body_z_nhc_min_window_s = 0.5;
  config.body_z_nhc_global_window_s = 2.0;
  config.body_z_nhc_global_stride_s = 2.0;
  config.body_z_nhc_jump_velocity_sigma_mps = 0.005;
  config.body_z_nhc_jump_displacement_sigma_m = 0.005;
  config.body_z_nhc_global_velocity_sigma_mps = 0.005;
  config.body_z_nhc_global_displacement_sigma_m = 0.005;
  const std::vector<double> timestamps{
    0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0,
    3.5, 4.0, 4.5, 5.0, 5.5, 6.0};
  const gtsam::Values values = MakeValues(timestamps, gtsam::Rot3(), gtsam::Vector3::Zero());
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(2.0, 2.5),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  ExpectTrue(summary.body_z_nhc_strict_effective_weighting_enabled, "strict NHC weighting should be reported");
  ExpectNear(static_cast<double>(summary.body_z_nhc_window_count), 4.0, 0.0, "strict NHC should add jump plus non-overlapping global windows");
  ExpectNear(static_cast<double>(summary.body_z_nhc_velocity_factor_count), 13.0, 0.0, "strict NHC should add one velocity factor per dynamic state");
  ExpectNear(static_cast<double>(summary.body_z_nhc_unique_velocity_factor_count), 13.0, 0.0, "strict NHC unique velocity count is wrong");
  ExpectNear(static_cast<double>(summary.body_z_nhc_velocity_duplicate_state_count), 0.0, 0.0, "strict NHC should avoid duplicate velocity states");
  ExpectNear(static_cast<double>(summary.body_z_nhc_interval_overlap_count), 0.0, 0.0, "strict NHC global windows should not overlap jump windows");
  ExpectNear(static_cast<double>(summary.body_z_nhc_jump_velocity_factor_count), 2.0, 0.0, "jump velocity factor count is wrong");
  ExpectNear(static_cast<double>(summary.body_z_nhc_global_velocity_factor_count), 11.0, 0.0, "global velocity factor count is wrong");
  ExpectTrue(
    std::all_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const offline_lc_minimal::BodyZNHCDiagnosticRow &row) {
        return row.strict_weighting_enabled &&
               std::abs(row.applied_velocity_sigma_mps - 0.005) < 1e-12 &&
               std::abs(row.applied_displacement_sigma_m - 0.005) < 1e-12 &&
               row.velocity_state_duplicate_count == 0U &&
               row.interval_overlap_count == 0U;
      }),
    "strict NHC diagnostics should expose applied sigma without duplicates");
}

void TestBuilderAddsLeakageCorrectedFactorsAcrossJumpAndGlobalWindows() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_global_weak_constraint = true;
  config.enable_body_z_nhc_horizontal_leakage_correction = true;
  config.body_z_nhc_jump_padding_s = 0.0;
  config.body_z_nhc_min_window_s = 0.5;
  config.body_z_nhc_global_window_s = 1.0;
  config.body_z_nhc_global_stride_s = 1.0;
  config.body_z_nhc_global_velocity_sigma_mps = 0.02;
  config.body_z_nhc_global_displacement_sigma_m = 0.02;
  config.body_z_nhc_horizontal_leakage_min_speed_mps = 0.0;
  config.body_z_nhc_horizontal_leakage_min_sample_count = 3;
  config.body_z_nhc_horizontal_leakage_max_abs_coeff_rad = 0.50;
  config.body_z_nhc_horizontal_leakage_guard_s = 0.0;
  const std::vector<double> timestamps{0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0};
  const std::vector<gtsam::Vector3> velocities{
    gtsam::Vector3(1.0, 0.0, 0.10),
    gtsam::Vector3(0.0, 1.0, 0.0),
    gtsam::Vector3(1.0, 1.0, 0.10),
    gtsam::Vector3(2.0, 1.0, 0.20),
    gtsam::Vector3(9.0, 9.0, 0.0),
    gtsam::Vector3(9.0, 9.0, 0.0),
    gtsam::Vector3(1.0, 2.0, 0.10),
    gtsam::Vector3(2.0, 0.0, 0.20),
    gtsam::Vector3(0.0, 2.0, 0.0),
  };
  gtsam::Values values;
  std::vector<offline_lc_minimal::ReferenceNodeState> reference_states;
  for (std::size_t index = 0; index < timestamps.size(); ++index) {
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0));
    values.insert(symbol::X(index), pose);
    values.insert(symbol::V(index), velocities[index]);
    offline_lc_minimal::ReferenceNodeState state;
    state.time_s = timestamps[index];
    state.pose = pose;
    state.velocity = velocities[index];
    reference_states.push_back(state);
  }
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(2.0, 2.5),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::BodyZNHCDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::BodyZHorizontalLeakageDiagnosticRow> leakage_diagnostics;

  offline_lc_minimal::BodyZNHCConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &values;
  request.reference_states = &reference_states;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.horizontal_leakage_diagnostics = &leakage_diagnostics;
  offline_lc_minimal::BodyZNHCConstraintBuilder(std::move(request)).Build();

  ExpectTrue(summary.body_z_nhc_horizontal_leakage_estimate_valid, "leakage estimate should be valid");
  ExpectTrue(!leakage_diagnostics.empty(), "leakage diagnostic should be recorded");
  ExpectNear(summary.body_z_nhc_horizontal_leakage_x_rad, 0.10, 1e-12, "builder leakage x is wrong");
  ExpectNear(
    static_cast<double>(summary.body_z_nhc_leakage_corrected_velocity_factor_count),
    static_cast<double>(summary.body_z_nhc_velocity_factor_count),
    0.0,
    "leakage-corrected factors should replace raw velocity factors");
  ExpectNear(
    static_cast<double>(summary.body_z_nhc_leakage_corrected_displacement_factor_count),
    static_cast<double>(summary.body_z_nhc_displacement_factor_count),
    0.0,
    "leakage-corrected factors should replace raw displacement factors");
  ExpectTrue(
    std::any_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const offline_lc_minimal::BodyZNHCDiagnosticRow &row) {
        return row.from_jump_window && row.horizontal_leakage_correction_enabled;
      }),
    "jump windows should use leakage-corrected NHC");
  ExpectTrue(
    std::any_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const offline_lc_minimal::BodyZNHCDiagnosticRow &row) {
        return !row.from_jump_window && row.horizontal_leakage_correction_enabled;
      }),
    "global windows should use leakage-corrected NHC");
}

}  // namespace

int main() {
  try {
    RunTest("TestBodyZVelocityZeroFactorUsesBodyFrameZ", TestBodyZVelocityZeroFactorUsesBodyFrameZ);
    RunTest("TestBodyZWindowDisplacementZeroFactorIntegratesBodyZVelocity", TestBodyZWindowDisplacementZeroFactorIntegratesBodyZVelocity);
    RunTest("TestLeakageCorrectedVelocityFactorUsesVelocityOnly", TestLeakageCorrectedVelocityFactorUsesVelocityOnly);
    RunTest("TestLeakageCorrectedWindowDisplacementFactorIntegratesCorrectedVelocity", TestLeakageCorrectedWindowDisplacementFactorIntegratesCorrectedVelocity);
    RunTest("TestBodyZHorizontalLeakageEstimatorUsesOutsideWindowSamples", TestBodyZHorizontalLeakageEstimatorUsesOutsideWindowSamples);
    RunTest("TestFixedAxisGraphErrorIgnoresPoseChanges", TestFixedAxisGraphErrorIgnoresPoseChanges);
    RunTest("TestPopulateBodyZNHCDiagnosticsUsesFixedInitialAxis", TestPopulateBodyZNHCDiagnosticsUsesFixedInitialAxis);
    RunTest("TestBuilderAddsJumpWindowNHC", TestBuilderAddsJumpWindowNHC);
    RunTest("TestBuilderMergesOverlappingJumpWindows", TestBuilderMergesOverlappingJumpWindows);
    RunTest(
      "TestJumpConstraintWindowPlannerAllowsLongMergedWindows",
      TestJumpConstraintWindowPlannerAllowsLongMergedWindows);
    RunTest("TestBuilderSkipsShortWindow", TestBuilderSkipsShortWindow);
    RunTest("TestBuilderDoesNotAddGlobalWeakWindowsWhenDisabled", TestBuilderDoesNotAddGlobalWeakWindowsWhenDisabled);
    RunTest("TestBuilderAddsGlobalWeakWindowsWhenEnabled", TestBuilderAddsGlobalWeakWindowsWhenEnabled);
    RunTest("TestBuilderStrictWeightingUsesUniqueVelocityFactors", TestBuilderStrictWeightingUsesUniqueVelocityFactors);
    RunTest("TestBuilderAddsLeakageCorrectedFactorsAcrossJumpAndGlobalWindows", TestBuilderAddsLeakageCorrectedFactorsAcrossJumpAndGlobalWindows);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
