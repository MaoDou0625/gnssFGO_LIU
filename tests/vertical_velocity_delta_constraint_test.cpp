#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaFactor.h"

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

void TestVerticalVelocityDeltaFactorUsesOnlyZ() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const offline_lc_minimal::factor::VerticalVelocityDeltaFactor factor(1, 2, 0.25, noise);

  const gtsam::Vector target_residual =
    factor.evaluateError(gtsam::Vector3(10.0, 20.0, 1.0), gtsam::Vector3(-7.0, 6.0, 1.25));
  ExpectNear(
    target_residual(0),
    0.0,
    1e-12,
    "target delta should have zero residual");
  ExpectNear(
    factor.evaluateError(gtsam::Vector3(-100.0, 200.0, 1.0), gtsam::Vector3(70.0, -60.0, 1.40))(0),
    0.15,
    1e-12,
    "positive vertical delta residual is wrong");
  ExpectNear(
    factor.evaluateError(gtsam::Vector3(0.0, 0.0, 1.0), gtsam::Vector3(0.0, 0.0, 1.10))(0),
    -0.15,
    1e-12,
    "negative vertical delta residual is wrong");
}

std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> MakePropagationRecords() {
  return {
    {0, 1, 0.0, 0.5, 0.01},
    {1, 2, 1.0, 1.5, 0.02},
    {2, 3, 1.9, 2.1, 0.03},
    {3, 4, 2.25, 2.35, 0.04},
    {4, 5, 3.0, 3.5, 0.05},
  };
}

std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> MakeJumpWindows() {
  offline_lc_minimal::BodyZSeedJumpWindowRow window;
  window.start_time_s = 2.2;
  window.end_time_s = 2.4;
  return {window};
}

void TestBuilderAddsDynamicBoundaryAndDynamicNonJumpIntervals() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_jump_padding_s = 0.25;
  const auto records = MakePropagationRecords();
  const auto jump_windows = MakeJumpWindows();
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "three intervals should receive factors");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_factor_count), 3.0, 0.0, "factor count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_static_count), 0.0, 0.0, "static skip count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_jump_count), 2.0, 0.0, "jump skip count is wrong");
  ExpectTrue(diagnostics[0].factor_added, "static-to-dynamic boundary should receive a factor");
  ExpectNear(diagnostics[1].sigma_mps, 0.25, 1e-12, "sigma should use acc_sigma * dt");
  ExpectTrue(diagnostics[2].in_jump_padding, "padding-overlapped interval should be marked");
  ExpectTrue(diagnostics[3].in_jump_padding, "jump interval should be marked");
}

void TestSigmaModelLegacyMatchesExistingFormula() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_velocity_delta_bias_consistent_sigma = false;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;

  const offline_lc_minimal::VerticalVelocityDeltaSigmaModel model(config);
  const auto long_dt = model.Compute(0.5);
  ExpectTrue(long_dt.model == "legacy", "legacy sigma model should be reported");
  ExpectNear(long_dt.sigma_mps, 0.25, 1e-12, "legacy sigma should use acc_sigma * dt");
  ExpectNear(long_dt.legacy_sigma_mps, 0.25, 1e-12, "legacy diagnostic sigma is wrong");

  const auto short_dt = model.Compute(0.01);
  ExpectNear(short_dt.sigma_mps, 0.02, 1e-12, "legacy sigma should respect min sigma");
}

void TestSigmaModelBiasConsistentComponents() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.gravity_mps2 = 9.80665;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(10.0);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;

  const offline_lc_minimal::VerticalVelocityDeltaSigmaModel model(config);
  const auto sigma = model.Compute(0.05);
  const double expected_bias_mps = offline_lc_minimal::MicroGToMps2(10.0) * 0.05;
  const double expected_attitude_mps = 9.80665 * 1.0e-4 * 0.05;
  const double expected_sigma_mps = std::sqrt(
    expected_bias_mps * expected_bias_mps +
    expected_attitude_mps * expected_attitude_mps +
    1.0e-5 * 1.0e-5);

  ExpectTrue(sigma.model == "bias_consistent", "bias-consistent sigma model should be reported");
  ExpectNear(sigma.legacy_sigma_mps, 0.005, 1e-12, "legacy comparison sigma is wrong");
  ExpectNear(sigma.bias_sigma_mps, expected_bias_mps, 1e-15, "bias sigma component is wrong");
  ExpectNear(sigma.attitude_sigma_mps, expected_attitude_mps, 1e-15, "attitude sigma component is wrong");
  ExpectNear(sigma.sigma_mps, expected_sigma_mps, 1e-15, "bias-consistent sigma is wrong");
}

void TestSigmaModelClampFlags() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(10.0);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;

  offline_lc_minimal::VerticalVelocityDeltaSigmaModel model(config);
  const auto floor_dominated = model.Compute(0.0);
  ExpectTrue(floor_dominated.clamped_floor, "zero dt should be floor dominated");
  ExpectTrue(!floor_dominated.clamped_ceiling, "zero dt should not be ceiling clamped");

  config.vertical_velocity_delta_attitude_sigma_rad = 1.0;
  const offline_lc_minimal::VerticalVelocityDeltaSigmaModel ceiling_model(config);
  const auto ceiling_clamped = ceiling_model.Compute(1.0);
  ExpectNear(ceiling_clamped.sigma_mps, 5.0e-4, 1e-15, "ceiling clamp sigma is wrong");
  ExpectTrue(ceiling_clamped.clamped_ceiling, "large attitude sigma should clamp to ceiling");
}

void TestBuilderSkipsStaticInteriorButAddsStaticDynamicBoundary() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {0, 1, 0.0, 0.5, 0.01},
    {1, 2, 0.5, 1.0, 0.02},
    {2, 3, 1.0, 1.5, 0.03},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 2;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "boundary and dynamic intervals should receive factors");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_factor_count), 2.0, 0.0, "factor count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_static_count), 1.0, 0.0, "static skip count is wrong");
  ExpectTrue(diagnostics[0].skip_reason == "STATIC_INTERIOR", "static interior interval should be skipped");
  ExpectTrue(diagnostics[1].factor_added, "last static to first dynamic interval should receive a factor");
  ExpectTrue(diagnostics[1].skip_reason == "ADDED", "boundary interval should be marked as added");
}

void TestBuilderSkipsStaticDynamicBoundaryInsideJumpPadding() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_jump_padding_s = 0.10;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 0.5, 1.0, 0.02},
  };
  offline_lc_minimal::BodyZSeedJumpWindowRow window;
  window.start_time_s = 0.75;
  window.end_time_s = 0.80;
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{window};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 2;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.empty(), "boundary interval inside jump padding should not receive a factor");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_factor_count), 0.0, 0.0, "factor count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_static_count), 0.0, 0.0, "static skip count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_jump_count), 1.0, 0.0, "jump skip count is wrong");
  ExpectTrue(diagnostics.front().in_jump_padding, "boundary interval should be marked as jump padding");
  ExpectTrue(diagnostics.front().skip_reason == "JUMP_PADDING", "boundary interval should be skipped by jump padding");
}

void TestBuilderDisabledDoesNotMutateGraph() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = false;
  const auto records = MakePropagationRecords();
  const auto jump_windows = MakeJumpWindows();
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.empty(), "disabled builder should not add factors");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_skipped_disabled_count),
    static_cast<double>(records.size()),
    0.0,
    "disabled skip count is wrong");
  ExpectTrue(!diagnostics.empty() && diagnostics.front().skip_reason == "DISABLED", "disabled diagnostics are wrong");
}

void TestBuilderSkipsIntervalsAfterGnssSupport() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_jump_padding_s = 0.25;
  const auto records = MakePropagationRecords();
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.gnss_support_end_time_s = 2.0;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "supported boundary and dynamic intervals should receive factors");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_skipped_gnss_support_count),
    3.0,
    0.0,
    "GNSS support skip count is wrong");
  ExpectTrue(
    diagnostics.back().skip_reason == "OUTSIDE_GNSS_SUPPORT",
    "unsupported tail interval should be marked");
}

void TestBuilderClampsVelocityDeltaTarget() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_jump_padding_s = 0.25;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 0.1;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.0, 1.5, 1.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "clamped interval should still add a factor");
  ExpectNear(diagnostics.front().raw_target_delta_vz_mps, 1.0, 1e-12, "raw target delta should be preserved");
  ExpectNear(diagnostics.front().target_delta_vz_mps, 0.05, 1e-12, "target delta should be acceleration-limited");
  ExpectTrue(diagnostics.front().target_clamped, "clamped target should be flagged");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_target_clamped_count),
    1.0,
    0.0,
    "clamped target count is wrong");
}

void TestBuilderUsesBiasConsistentSigmaAndSummary() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.gravity_mps2 = 9.80665;
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(10.0);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.0, 1.05, 0.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  const double expected_bias_mps = offline_lc_minimal::MicroGToMps2(10.0) * 0.05;
  const double expected_attitude_mps = 9.80665 * 1.0e-4 * 0.05;
  const double expected_sigma_mps = std::sqrt(
    expected_bias_mps * expected_bias_mps +
    expected_attitude_mps * expected_attitude_mps +
    1.0e-5 * 1.0e-5);
  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "bias-consistent interval should add a factor");
  ExpectTrue(summary.vertical_velocity_delta_bias_consistent_sigma_enabled, "summary should report sigma mode");
  ExpectNear(diagnostics.front().sigma_mps, expected_sigma_mps, 1e-15, "builder should use bias sigma");
  ExpectNear(diagnostics.front().legacy_sigma_mps, 0.005, 1e-12, "builder should preserve legacy diagnostic");
  ExpectTrue(diagnostics.front().sigma_model == "bias_consistent", "diagnostic sigma model is wrong");
  ExpectNear(summary.vertical_velocity_delta_sigma_mean_mps, expected_sigma_mps, 1e-15, "summary mean sigma is wrong");
  ExpectNear(summary.vertical_velocity_delta_sigma_max_mps, expected_sigma_mps, 1e-15, "summary max sigma is wrong");
}

void TestBuilderUsesLegacySigmaForClampedTargetInBiasConsistentMode() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 0.1;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(10.0);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.0, 1.5, 1.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "clamped interval should still add a factor");
  ExpectTrue(diagnostics.front().target_clamped, "target should be clamped");
  ExpectTrue(
    diagnostics.front().sigma_model == "legacy_clamped_target",
    "clamped targets should fall back to legacy sigma");
  ExpectNear(diagnostics.front().target_delta_vz_mps, 0.05, 1e-12, "clamped target value is wrong");
  ExpectNear(diagnostics.front().legacy_sigma_mps, 0.25, 1e-12, "legacy comparison sigma is wrong");
  ExpectNear(diagnostics.front().sigma_mps, 0.25, 1e-12, "clamped target should use legacy sigma");
  ExpectNear(summary.vertical_velocity_delta_sigma_mean_mps, 0.25, 1e-12, "summary mean sigma is wrong");
}

}  // namespace

int main() {
  try {
    RunTest("TestVerticalVelocityDeltaFactorUsesOnlyZ", TestVerticalVelocityDeltaFactorUsesOnlyZ);
    RunTest("TestSigmaModelLegacyMatchesExistingFormula", TestSigmaModelLegacyMatchesExistingFormula);
    RunTest("TestSigmaModelBiasConsistentComponents", TestSigmaModelBiasConsistentComponents);
    RunTest("TestSigmaModelClampFlags", TestSigmaModelClampFlags);
    RunTest(
      "TestBuilderAddsDynamicBoundaryAndDynamicNonJumpIntervals",
      TestBuilderAddsDynamicBoundaryAndDynamicNonJumpIntervals);
    RunTest(
      "TestBuilderSkipsStaticInteriorButAddsStaticDynamicBoundary",
      TestBuilderSkipsStaticInteriorButAddsStaticDynamicBoundary);
    RunTest(
      "TestBuilderSkipsStaticDynamicBoundaryInsideJumpPadding",
      TestBuilderSkipsStaticDynamicBoundaryInsideJumpPadding);
    RunTest("TestBuilderDisabledDoesNotMutateGraph", TestBuilderDisabledDoesNotMutateGraph);
    RunTest("TestBuilderSkipsIntervalsAfterGnssSupport", TestBuilderSkipsIntervalsAfterGnssSupport);
    RunTest("TestBuilderClampsVelocityDeltaTarget", TestBuilderClampsVelocityDeltaTarget);
    RunTest("TestBuilderUsesBiasConsistentSigmaAndSummary", TestBuilderUsesBiasConsistentSigmaAndSummary);
    RunTest(
      "TestBuilderUsesLegacySigmaForClampedTargetInBiasConsistentMode",
      TestBuilderUsesLegacySigmaForClampedTargetInBiasConsistentMode);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
