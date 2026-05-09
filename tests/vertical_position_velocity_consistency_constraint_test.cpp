#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/core/VerticalPositionVelocityConsistencyConstraintBuilder.h"
#include "offline_lc_minimal/factor/VerticalPositionVelocityConsistencyFactor.h"

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

void InsertState(gtsam::Values &values, const std::size_t state_index, const double z_m, const double vz_mps) {
  values.insert(
    symbol::X(state_index),
    gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, z_m)));
  values.insert(symbol::V(state_index), gtsam::Vector3(0.0, 0.0, vz_mps));
}

offline_lc_minimal::BodyZSeedJumpWindowRow MakeJumpWindow(
  const double start_time_s,
  const double end_time_s) {
  offline_lc_minimal::BodyZSeedJumpWindowRow window;
  window.start_time_s = start_time_s;
  window.end_time_s = end_time_s;
  return window;
}

void TestBuilderDisabledAddsNothing() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_position_velocity_consistency_all_states = false;
  const std::vector<double> state_timestamps{0.0, 1.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::Values initial_values;
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalPositionVelocityConsistencyDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalPositionVelocityConsistencyBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalPositionVelocityConsistencyConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 0.0, 0.0, "disabled builder should add no factors");
  ExpectNear(
    static_cast<double>(summary.vertical_position_velocity_consistency_factor_count),
    0.0,
    0.0,
    "disabled builder summary should be zero");
  ExpectTrue(diagnostics.empty(), "disabled builder should emit no diagnostics");
}

void TestBuilderAddsAllAdjacentFactorsAndClassifiesIntervals() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_position_velocity_consistency_all_states = true;
  config.vertical_position_velocity_consistency_sigma_m = 0.001;
  const std::vector<double> state_timestamps{0.0, 0.5, 1.0, 1.5};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    MakeJumpWindow(1.10, 1.20)};
  gtsam::Values initial_values;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    InsertState(initial_values, index, static_cast<double>(index), 0.0);
  }
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalPositionVelocityConsistencyDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalPositionVelocityConsistencyBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.dynamic_start_index = 2;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalPositionVelocityConsistencyConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "N states should add N-1 factors");
  ExpectNear(
    static_cast<double>(summary.vertical_position_velocity_consistency_factor_count),
    3.0,
    0.0,
    "summary factor count is wrong");
  ExpectNear(static_cast<double>(diagnostics.size()), 3.0, 0.0, "diagnostic count is wrong");
  ExpectTrue(diagnostics[0].interval_type == "static", "first interval should be static");
  ExpectTrue(
    diagnostics[1].interval_type == "static_dynamic_boundary",
    "second interval should be static-dynamic boundary");
  ExpectTrue(diagnostics[2].interval_type == "jump_padding", "third interval should be jump padding");
  for (std::size_t index = 0; index < graph.size(); ++index) {
    ExpectTrue(
      boost::dynamic_pointer_cast<
        offline_lc_minimal::factor::VerticalPositionVelocityConsistencyFactor>(graph.at(index)) != nullptr,
      "builder should add only position-velocity consistency factors");
  }
}

void TestBuilderSkipsInvalidIntervals() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_position_velocity_consistency_all_states = true;
  const std::vector<double> state_timestamps{0.0, 0.0, 0.5};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::Values initial_values;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    InsertState(initial_values, index, 0.0, 0.0);
  }
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalPositionVelocityConsistencyDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalPositionVelocityConsistencyBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.dynamic_start_index = 2;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalPositionVelocityConsistencyConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "only valid intervals should add factors");
  ExpectNear(
    static_cast<double>(summary.vertical_position_velocity_consistency_skipped_invalid_count),
    1.0,
    0.0,
    "invalid skip count is wrong");
  ExpectTrue(diagnostics.front().skip_reason == "INVALID_DT", "invalid interval skip reason is wrong");
}

void TestPopulateDiagnosticsComputesResidualsAndSummary() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_position_velocity_consistency_all_states = true;
  config.vertical_position_velocity_consistency_sigma_m = 0.001;
  const std::vector<double> state_timestamps{0.0, 1.0};
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  gtsam::Values initial_values;
  InsertState(initial_values, 0, 0.0, 0.0);
  InsertState(initial_values, 1, 1.0, 0.0);
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalPositionVelocityConsistencyDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalPositionVelocityConsistencyBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalPositionVelocityConsistencyConstraintBuilder(std::move(request)).Build();

  gtsam::Values optimized_values;
  InsertState(optimized_values, 0, 0.0, 0.5);
  InsertState(optimized_values, 1, 0.75, 0.5);
  offline_lc_minimal::PopulateVerticalPositionVelocityConsistencyDiagnostics(
    optimized_values,
    diagnostics,
    summary);

  ExpectNear(diagnostics.front().initial_mismatch_m, 1.0, 1e-12, "initial mismatch is wrong");
  ExpectNear(diagnostics.front().optimized_delta_z_m, 0.75, 1e-12, "optimized delta-z is wrong");
  ExpectNear(
    diagnostics.front().optimized_trapezoid_vz_integral_m,
    0.5,
    1e-12,
    "optimized velocity integral is wrong");
  ExpectNear(diagnostics.front().optimized_mismatch_m, 0.25, 1e-12, "optimized mismatch is wrong");
  ExpectNear(diagnostics.front().normalized_residual, 250.0, 1e-9, "normalized residual is wrong");
  ExpectNear(
    summary.vertical_position_velocity_consistency_max_abs_mismatch_m,
    0.25,
    1e-12,
    "summary max mismatch is wrong");
  ExpectNear(
    summary.vertical_position_velocity_consistency_static_dynamic_boundary_mismatch_m,
    0.25,
    1e-12,
    "summary boundary mismatch is wrong");
}

}  // namespace

int main() {
  try {
    RunTest("TestBuilderDisabledAddsNothing", TestBuilderDisabledAddsNothing);
    RunTest(
      "TestBuilderAddsAllAdjacentFactorsAndClassifiesIntervals",
      TestBuilderAddsAllAdjacentFactorsAndClassifiesIntervals);
    RunTest("TestBuilderSkipsInvalidIntervals", TestBuilderSkipsInvalidIntervals);
    RunTest(
      "TestPopulateDiagnosticsComputesResidualsAndSummary",
      TestPopulateDiagnosticsComputesResidualsAndSummary);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
