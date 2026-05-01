#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"
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

void TestBuilderAddsOnlyDynamicNonJumpIntervals() {
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

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "only two intervals should receive factors");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_factor_count), 2.0, 0.0, "factor count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_static_count), 1.0, 0.0, "static skip count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_jump_count), 2.0, 0.0, "jump skip count is wrong");
  ExpectNear(diagnostics[1].sigma_mps, 0.25, 1e-12, "sigma should use acc_sigma * dt");
  ExpectTrue(diagnostics[2].in_jump_padding, "padding-overlapped interval should be marked");
  ExpectTrue(diagnostics[3].in_jump_padding, "jump interval should be marked");
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

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "only supported interval should receive a factor");
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

}  // namespace

int main() {
  try {
    RunTest("TestVerticalVelocityDeltaFactorUsesOnlyZ", TestVerticalVelocityDeltaFactorUsesOnlyZ);
    RunTest("TestBuilderAddsOnlyDynamicNonJumpIntervals", TestBuilderAddsOnlyDynamicNonJumpIntervals);
    RunTest("TestBuilderDisabledDoesNotMutateGraph", TestBuilderDisabledDoesNotMutateGraph);
    RunTest("TestBuilderSkipsIntervalsAfterGnssSupport", TestBuilderSkipsIntervalsAfterGnssSupport);
    RunTest("TestBuilderClampsVelocityDeltaTarget", TestBuilderClampsVelocityDeltaTarget);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
