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

  offline_lc_minimal::PopulateBodyZNHCDiagnostics(
    initial_values,
    optimized_values,
    timestamps,
    diagnostics);

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

}  // namespace

int main() {
  try {
    RunTest("TestBodyZVelocityZeroFactorUsesBodyFrameZ", TestBodyZVelocityZeroFactorUsesBodyFrameZ);
    RunTest("TestBodyZWindowDisplacementZeroFactorIntegratesBodyZVelocity", TestBodyZWindowDisplacementZeroFactorIntegratesBodyZVelocity);
    RunTest("TestFixedAxisGraphErrorIgnoresPoseChanges", TestFixedAxisGraphErrorIgnoresPoseChanges);
    RunTest("TestPopulateBodyZNHCDiagnosticsUsesFixedInitialAxis", TestPopulateBodyZNHCDiagnosticsUsesFixedInitialAxis);
    RunTest("TestBuilderAddsJumpWindowNHC", TestBuilderAddsJumpWindowNHC);
    RunTest("TestBuilderMergesOverlappingJumpWindows", TestBuilderMergesOverlappingJumpWindows);
    RunTest("TestBuilderSkipsShortWindow", TestBuilderSkipsShortWindow);
    RunTest("TestBuilderDoesNotAddGlobalWeakWindowsWhenDisabled", TestBuilderDoesNotAddGlobalWeakWindowsWhenDisabled);
    RunTest("TestBuilderAddsGlobalWeakWindowsWhenEnabled", TestBuilderAddsGlobalWeakWindowsWhenEnabled);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
