#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/Stage2LowfreqVerticalReferenceOptimizationRunner.h"
#include "offline_lc_minimal/core/Stage3JumpRegularizerConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceOptimizationRunner.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceTimelineAligner.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaDeadbandFactor.h"

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

std::vector<offline_lc_minimal::TrajectoryRow> MakeTrajectory(
  const std::size_t count,
  const double high_frequency_amplitude_m = 0.0) {
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory;
  trajectory.reserve(count);
  for (std::size_t index = 0; index < count; ++index) {
    offline_lc_minimal::TrajectoryRow row;
    row.time_s = static_cast<double>(index);
    const double high_frequency_m =
      index % 2U == 0U ? high_frequency_amplitude_m : -high_frequency_amplitude_m;
    row.enu_position_m =
      Eigen::Vector3d(0.0, 0.0, 0.01 * static_cast<double>(index) + high_frequency_m);
    row.enu_velocity_mps = Eigen::Vector3d(1.0, 0.0, 0.0);
    trajectory.push_back(row);
  }
  return trajectory;
}

gtsam::Pose3 MakePose(const double up_m) {
  return gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 2.0, up_m));
}

double RelativeRotationAngleRad(
  const gtsam::Rot3 &lhs,
  const gtsam::Rot3 &rhs) {
  return gtsam::Rot3::Logmap(lhs.between(rhs)).norm();
}

void TestProfilePlannerBuildsFiniteZeroPhaseLowpass() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.05;
  config.stage3_vertical_anchor_sigma_m = 0.015;
  const auto trajectory = MakeTrajectory(21U, 0.05);

  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest request;
  request.config = &config;
  request.stage2_trajectory = &trajectory;
  const auto reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(request)).Plan();

  ExpectTrue(
    reference.rows.size() == trajectory.size(),
    "profile should keep one row per Stage2 trajectory state");
  double max_abs_delta_m = 0.0;
  for (std::size_t index = 0; index < reference.rows.size(); ++index) {
    const auto &row = reference.rows[index];
    ExpectTrue(row.state_index == index, "profile row should stay state-aligned");
    ExpectNear(row.time_s, trajectory[index].time_s, 1.0e-12, "profile time should stay aligned");
    ExpectTrue(std::isfinite(row.stage2_lowpass_up_m), "lowpass reference should be finite");
    ExpectTrue(row.skip_reason == "PLANNED", "finite rows should be planned");
    max_abs_delta_m = std::max(max_abs_delta_m, std::abs(row.lowpass_delta_m));
  }
  ExpectTrue(max_abs_delta_m > 0.005, "lowpass should remove visible high-frequency content");
}

void TestProfilePlannerBuildsSplineBaselineReference() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_smoothing_method =
    offline_lc_minimal::Stage3VerticalReferenceSmoothingMethod::kSplineBaseline;
  config.stage3_vertical_reference_spline_knot_spacing_m = 1.0;
  config.stage3_vertical_reference_spline_smooth_lambda = 10000.0;
  config.stage3_vertical_reference_spline_anchor_weight = 100000.0;
  config.stage3_vertical_reference_spline_slope_weight = 1000.0;

  auto trajectory = MakeTrajectory(41U, 0.05);
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    trajectory[index].enu_position_m.x() = static_cast<double>(index);
  }

  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest request;
  request.config = &config;
  request.stage2_trajectory = &trajectory;
  const auto reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(request)).Plan();

  ExpectTrue(
    reference.rows.size() == trajectory.size(),
    "spline baseline should keep one row per Stage2 trajectory state");
  double max_abs_delta_m = 0.0;
  for (std::size_t index = 0; index < reference.rows.size(); ++index) {
    const auto &row = reference.rows[index];
    ExpectTrue(row.state_index == index, "spline row should stay state-aligned");
    ExpectTrue(std::isfinite(row.stage2_lowpass_up_m), "spline reference should be finite");
    ExpectTrue(row.skip_reason == "PLANNED", "finite spline rows should be planned");
    max_abs_delta_m = std::max(max_abs_delta_m, std::abs(row.lowpass_delta_m));
  }

  double raw_second_difference_sum = 0.0;
  double spline_second_difference_sum = 0.0;
  for (std::size_t index = 1; index + 1U < trajectory.size(); ++index) {
    raw_second_difference_sum += std::abs(
      trajectory[index + 1U].enu_position_m.z() -
      2.0 * trajectory[index].enu_position_m.z() +
      trajectory[index - 1U].enu_position_m.z());
    spline_second_difference_sum += std::abs(
      reference.rows[index + 1U].stage2_lowpass_up_m -
      2.0 * reference.rows[index].stage2_lowpass_up_m +
      reference.rows[index - 1U].stage2_lowpass_up_m);
  }
  ExpectTrue(
    max_abs_delta_m > 0.005,
    "spline baseline should remove visible high-frequency content");
  ExpectTrue(
    spline_second_difference_sum < raw_second_difference_sum * 0.5,
    "spline baseline should reduce vertical roughness before Stage3 uses it");
}

void TestProfilePlannerCanHoldInitialDynamicStaticReference() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.05;
  config.enable_stage3_initial_dynamic_static_reference_hold = true;
  config.stage3_initial_dynamic_static_reference_hold_duration_s = 2.0;
  config.stage3_initial_dynamic_static_reference_hold_blend_s = 0.0;

  auto trajectory = MakeTrajectory(7U, 0.0);
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    trajectory[index].enu_position_m.z() = index <= 2U ? 10.0 : 0.0;
  }
  trajectory[3].time_s = 2.997;

  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest request;
  request.config = &config;
  request.stage2_trajectory = &trajectory;
  request.dynamic_start_index = 3U;
  request.dynamic_start_time_s = 3.0;
  const auto reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(request)).Plan();

  auto baseline_config = config;
  baseline_config.enable_stage3_initial_dynamic_static_reference_hold = false;
  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest baseline_request;
  baseline_request.config = &baseline_config;
  baseline_request.stage2_trajectory = &trajectory;
  baseline_request.dynamic_start_index = 3U;
  baseline_request.dynamic_start_time_s = 3.0;
  const auto baseline_reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(
      std::move(baseline_request)).Plan();

  ExpectNear(
    reference.rows[3].stage2_lowpass_up_m,
    10.0,
    1.0e-12,
    "initial dynamic static interval should keep the static height");
  ExpectNear(
    reference.rows[4].stage2_lowpass_up_m,
    10.0,
    1.0e-12,
    "hold should include the configured duration end");
  ExpectTrue(
    std::abs(reference.rows[6].stage2_lowpass_up_m - 10.0) > 1.0e-3,
    "rows after the hold window should return to the lowpass profile");
  ExpectTrue(
    std::abs(reference.rows[6].stage2_lowpass_up_m - 10.0) <
      std::abs(baseline_reference.rows[6].stage2_lowpass_up_m - 10.0),
    "static hold should feed the lowpass input and keep the post-hold transition closer to static height");
  ExpectNear(
    reference.rows[3].lowpass_delta_m,
    10.0 - trajectory[3].enu_position_m.z(),
    1.0e-12,
    "lowpass delta should be recomputed from the held reference");
}

void TestProfilePlannerUsesDetectedInitialDynamicStaticWindows() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.05;
  config.enable_stage3_initial_dynamic_static_reference_hold = false;
  config.enable_initial_dynamic_static_detection = true;
  config.enable_initial_dynamic_static_lowpass_protection = true;
  config.initial_dynamic_static_lowpass_blend_s = 0.0;

  auto trajectory = MakeTrajectory(8U, 0.0);
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    trajectory[index].enu_position_m.z() = 0.2 * static_cast<double>(index);
  }
  trajectory[3].enu_position_m.z() = 2.0;
  trajectory[4].enu_position_m.z() = 4.0;
  trajectory[5].enu_position_m.z() = 6.0;

  std::vector<offline_lc_minimal::LateStaticWindowRow> windows(1U);
  windows.front().start_time_s = 3.0;
  windows.front().end_time_s = 5.0;
  windows.front().duration_s = 2.0;
  windows.front().valid = true;
  windows.front().skip_reason = "OK";

  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest request;
  request.config = &config;
  request.stage2_trajectory = &trajectory;
  request.initial_dynamic_static_windows = &windows;
  const auto reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(request)).Plan();

  auto baseline_config = config;
  baseline_config.enable_initial_dynamic_static_lowpass_protection = false;
  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest baseline_request;
  baseline_request.config = &baseline_config;
  baseline_request.stage2_trajectory = &trajectory;
  const auto baseline_reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(baseline_request)).Plan();

  ExpectNear(
    reference.rows[3].stage2_lowpass_up_m,
    4.0,
    1.0e-12,
    "detected initial dynamic static window should keep its first reference row static");
  ExpectNear(
    reference.rows[5].stage2_lowpass_up_m,
    4.0,
    1.0e-12,
    "detected initial dynamic static window should keep its last reference row static");
  ExpectTrue(
    std::abs(baseline_reference.rows[3].stage2_lowpass_up_m - 4.0) > 1.0e-3,
    "unprotected lowpass should still be dragged by the surrounding motion");
}

void TestProfilePlannerStartsLowpassAfterInitialStatic() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.05;

  auto trajectory = MakeTrajectory(10U, 0.0);
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    trajectory[index].enu_position_m.z() = index < 5U ? 10.0 : 0.0;
  }

  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest request;
  request.config = &config;
  request.stage2_trajectory = &trajectory;
  request.dynamic_start_index = 5U;
  request.dynamic_start_time_s = 5.0;
  const auto reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(request)).Plan();

  ExpectNear(
    reference.rows[4].stage2_lowpass_up_m,
    10.0,
    1.0e-12,
    "initial static rows should keep their unfiltered diagnostic height");
  ExpectNear(
    reference.rows[5].stage2_lowpass_up_m,
    0.0,
    1.0e-12,
    "dynamic lowpass should not be initialized from the static height");
  ExpectNear(
    reference.rows[9].stage2_lowpass_up_m,
    0.0,
    1.0e-12,
    "dynamic lowpass should only use dynamic samples in this fixture");
}

void TestProfilePlannerExcludesTerminalStaticFromLowpass() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.05;
  config.stage3_vertical_reference_terminal_static_min_duration_s = 2.0;

  auto trajectory = MakeTrajectory(10U, 0.0);
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    trajectory[index].enu_position_m.z() = index < 6U ? 0.0 : 10.0;
    if (index >= 6U) {
      trajectory[index].enu_velocity_mps = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
  }

  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest request;
  request.config = &config;
  request.stage2_trajectory = &trajectory;
  request.dynamic_start_index = 0U;
  request.dynamic_start_time_s = 0.0;
  const auto reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(request)).Plan();

  ExpectNear(
    reference.rows[5].stage2_lowpass_up_m,
    0.0,
    1.0e-12,
    "dynamic lowpass should not be pulled by the terminal static suffix");
  ExpectNear(
    reference.rows[6].stage2_lowpass_up_m,
    10.0,
    1.0e-12,
    "terminal static rows should retain unfiltered diagnostic height");
  ExpectTrue(
    reference.rows[6].skip_reason == "TERMINAL_STATIC",
    "terminal static rows should stay marked while remaining outside the smoother");
}

void TestConstraintBuilderAddsOnlyDynamicAnchorsAndDiagnostics() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.05;
  config.stage3_vertical_anchor_sigma_m = 0.015;
  config.stage3_vertical_reference_constraint_mode =
    offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kGaussian;

  offline_lc_minimal::Stage3VerticalReference reference;
  for (std::size_t index = 0; index < 4U; ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = static_cast<double>(index);
    row.stage2_up_m = 10.0 * static_cast<double>(index);
    row.stage2_lowpass_up_m = row.stage2_up_m;
    row.lowpass_delta_m = 0.0;
    row.skip_reason = "PLANNED";
    reference.rows.push_back(row);
  }
  const std::vector<double> state_timestamps{0.0, 1.0, 2.0, 3.0};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow> diagnostics;

  offline_lc_minimal::Stage3VerticalReferenceConstraintBuildRequest request;
  request.config = &config;
  request.reference = &reference;
  request.state_timestamps = &state_timestamps;
  request.dynamic_start_index = 2U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::Stage3VerticalReferenceConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 2U, "only dynamic states should receive Stage3 vertical anchors");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalPositionFactor>(graph[0])),
    "gaussian mode should add vertical position factors");
  ExpectTrue(diagnostics.size() == 4U, "diagnostics should include skipped static states");
  ExpectTrue(!diagnostics[0].factor_added, "initial static state 0 should be skipped");
  ExpectTrue(!diagnostics[1].factor_added, "initial static state 1 should be skipped");
  ExpectTrue(diagnostics[2].factor_added, "dynamic state 2 should get an anchor");
  ExpectTrue(diagnostics[3].factor_added, "dynamic state 3 should get an anchor");
  ExpectTrue(summary.stage3_vertical_reference_factor_count == 2U, "factor count should be tracked");
  ExpectTrue(
    summary.stage3_vertical_reference_constraint_mode == "gaussian",
    "summary should record gaussian mode");
  ExpectTrue(summary.stage3_vertical_reference_skipped_count == 2U, "skip count should be tracked");

  gtsam::Values optimized_values;
  optimized_values.insert(gtsam::symbol_shorthand::X(0), MakePose(0.0));
  optimized_values.insert(gtsam::symbol_shorthand::X(1), MakePose(10.0));
  optimized_values.insert(gtsam::symbol_shorthand::X(2), MakePose(20.01));
  optimized_values.insert(gtsam::symbol_shorthand::X(3), MakePose(29.98));
  offline_lc_minimal::PopulateStage3VerticalReferenceDiagnostics(
    optimized_values,
    diagnostics,
    summary);

  ExpectNear(diagnostics[2].residual_m, 0.01, 1.0e-12, "state 2 residual should be optimized minus reference");
  ExpectNear(diagnostics[3].residual_m, -0.02, 1.0e-12, "state 3 residual should be optimized minus reference");
  ExpectNear(
    summary.stage3_vertical_reference_mean_abs_residual_m,
    0.015,
    1.0e-12,
    "mean absolute Stage3 residual should be summarized");
  ExpectNear(
    summary.stage3_vertical_reference_max_abs_residual_m,
    0.02,
    1.0e-12,
      "max absolute Stage3 residual should be summarized");
}

void TestConstraintBuilderAnchorsTerminalStaticReferences() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_lowpass_cutoff_hz = 0.05;
  config.stage3_vertical_reference_terminal_static_min_duration_s = 2.0;

  auto trajectory = MakeTrajectory(10U, 0.0);
  std::vector<double> timestamps;
  timestamps.reserve(trajectory.size());
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    trajectory[index].enu_position_m.z() = index < 6U ? 0.0 : 10.0;
    if (index >= 6U) {
      trajectory[index].enu_velocity_mps = Eigen::Vector3d(0.0, 0.0, 0.0);
    }
    timestamps.push_back(trajectory[index].time_s);
  }

  offline_lc_minimal::Stage3VerticalReferenceProfilePlanRequest plan_request;
  plan_request.config = &config;
  plan_request.stage2_trajectory = &trajectory;
  plan_request.dynamic_start_index = 0U;
  auto reference =
    offline_lc_minimal::Stage3VerticalReferenceProfilePlanner(std::move(plan_request)).Plan();

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow> diagnostics;
  offline_lc_minimal::Stage3VerticalReferenceConstraintBuildRequest build_request;
  build_request.config = &config;
  build_request.state_timestamps = &timestamps;
  build_request.reference = &reference;
  build_request.dynamic_start_index = 0U;
  build_request.graph = &graph;
  build_request.run_summary = &summary;
  build_request.diagnostics = &diagnostics;
  offline_lc_minimal::Stage3VerticalReferenceConstraintBuilder(std::move(build_request)).Build();

  ExpectTrue(summary.stage3_vertical_reference_factor_count == 10U,
             "terminal static rows should still receive Stage3 vertical anchors");
  ExpectTrue(summary.stage3_vertical_reference_skipped_count == 0U,
             "terminal static rows should not be counted as skipped references");
  ExpectTrue(diagnostics[6].factor_added, "first terminal static row should be anchored");
  ExpectTrue(diagnostics[6].skip_reason == "ADDED",
             "terminal static row should become an added vertical reference factor");
  ExpectNear(
    diagnostics[6].reference_up_m,
    10.0,
    1.0e-12,
    "terminal static anchor should use the unfiltered terminal static reference height");
}

void TestConstraintBuilderEnvelopeModeUsesGateAndCenterPull() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.stage3_vertical_reference_constraint_mode =
    offline_lc_minimal::Stage3VerticalReferenceConstraintMode::kEnvelope;
  config.stage3_vertical_envelope_half_width_m = 0.008;
  config.stage3_vertical_envelope_sigma_m = 0.003;
  config.enable_stage3_vertical_envelope_center_pull = true;
  config.stage3_vertical_envelope_center_sigma_m = 0.006;
  config.stage3_vertical_envelope_center_deadband_m = 0.002;

  offline_lc_minimal::Stage3VerticalReference reference;
  for (std::size_t index = 0; index < 3U; ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = static_cast<double>(index);
    row.stage2_up_m = 1.0;
    row.stage2_lowpass_up_m = 1.0;
    row.lowpass_delta_m = 0.0;
    row.skip_reason = "PLANNED";
    reference.rows.push_back(row);
  }
  const std::vector<double> state_timestamps{0.0, 1.0, 2.0};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow> diagnostics;

  offline_lc_minimal::Stage3VerticalReferenceConstraintBuildRequest request;
  request.config = &config;
  request.reference = &reference;
  request.state_timestamps = &state_timestamps;
  request.dynamic_start_index = 1U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::Stage3VerticalReferenceConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 4U, "envelope mode should add gate and center-pull factors per dynamic state");
  const auto envelope_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalEnvelopeFactor>(graph[0]);
  const auto center_pull_factor =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalEnvelopeCenterPullFactor>(graph[1]);
  ExpectTrue(static_cast<bool>(envelope_factor), "primary Stage3 factor should be an envelope");
  ExpectTrue(static_cast<bool>(center_pull_factor), "center-pull factor should be added");
  ExpectNear(
    envelope_factor->evaluateError(MakePose(1.004))[0],
    0.0,
    1.0e-12,
    "inside-gate envelope residual should be zero");
  ExpectNear(
    envelope_factor->evaluateError(MakePose(1.012))[0],
    0.004,
    1.0e-12,
    "outside-gate envelope residual should be overflow amount");
  ExpectNear(
    center_pull_factor->evaluateError(MakePose(1.001))[0],
    0.0,
    1.0e-12,
    "inside center deadband residual should be zero");
  ExpectNear(
    center_pull_factor->evaluateError(MakePose(1.012))[0],
    0.006,
    1.0e-12,
    "center-pull residual should be bounded by gate half-width");
  ExpectTrue(summary.stage3_vertical_reference_factor_count == 2U, "primary factor count should be tracked");
  ExpectTrue(
    summary.stage3_vertical_reference_center_pull_factor_count == 2U,
    "center-pull factor count should be tracked");
  ExpectTrue(
    summary.stage3_vertical_reference_total_factor_count == 4U,
    "total Stage3 factor count should include gate and center-pull factors");
  ExpectTrue(diagnostics[1].constraint_mode == "envelope", "diagnostic should record envelope mode");
  ExpectNear(diagnostics[1].envelope_half_width_m, 0.008, 1.0e-12, "diagnostic should record half-width");

  gtsam::Values optimized_values;
  optimized_values.insert(gtsam::symbol_shorthand::X(0), MakePose(1.0));
  optimized_values.insert(gtsam::symbol_shorthand::X(1), MakePose(1.004));
  optimized_values.insert(gtsam::symbol_shorthand::X(2), MakePose(1.012));
  offline_lc_minimal::PopulateStage3VerticalReferenceDiagnostics(
    optimized_values,
    diagnostics,
    summary);

  ExpectTrue(!diagnostics[1].outside_gate, "inside-gate optimized state should be marked inside");
  ExpectTrue(diagnostics[2].outside_gate, "outside-gate optimized state should be marked outside");
  ExpectNear(
    diagnostics[2].envelope_overflow_residual_m,
    0.004,
    1.0e-12,
    "diagnostic overflow residual should match factor residual");
  ExpectNear(
    diagnostics[2].center_pull_residual_m,
    0.006,
    1.0e-12,
    "diagnostic center-pull residual should match factor residual");
  ExpectTrue(
    summary.stage3_vertical_envelope_outside_gate_count == 1U,
    "summary should count optimized states outside the gate");
  ExpectNear(
    summary.stage3_vertical_envelope_max_abs_overflow_residual_m,
    0.004,
    1.0e-12,
    "summary should track max overflow residual");
}

void TestVerticalVelocityDeltaDeadbandFactor() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  offline_lc_minimal::factor::VerticalVelocityDeltaDeadbandFactor factor(
    gtsam::symbol_shorthand::V(0),
    gtsam::symbol_shorthand::V(1),
    0.02,
    noise);

  gtsam::Matrix h_i;
  gtsam::Matrix h_j;
  const gtsam::Vector3 velocity_i(0.0, 0.0, 0.0);
  const gtsam::Vector3 inside_velocity_j(0.0, 0.0, 0.01);
  const auto inside_residual =
    factor.evaluateError(velocity_i, inside_velocity_j, h_i, h_j);
  ExpectNear(inside_residual[0], 0.0, 1.0e-12, "inside deadband residual should be zero");
  ExpectNear(h_i(0, 2), 0.0, 1.0e-12, "inside deadband H_i should be zero");
  ExpectNear(h_j(0, 2), 0.0, 1.0e-12, "inside deadband H_j should be zero");

  const gtsam::Vector3 outside_velocity_j(0.0, 0.0, 0.05);
  const auto outside_residual =
    factor.evaluateError(velocity_i, outside_velocity_j, h_i, h_j);
  ExpectNear(outside_residual[0], 0.03, 1.0e-12, "outside deadband residual should be overflow");
  ExpectNear(h_i(0, 2), -1.0, 1.0e-12, "outside deadband H_i should match delta-v Jacobian");
  ExpectNear(h_j(0, 2), 1.0, 1.0e-12, "outside deadband H_j should match delta-v Jacobian");

  const gtsam::Vector3 negative_velocity_j(0.0, 0.0, -0.05);
  const auto negative_residual =
    factor.evaluateError(velocity_i, negative_velocity_j, h_i, h_j);
  ExpectNear(negative_residual[0], -0.03, 1.0e-12, "negative overflow should keep sign");
  ExpectNear(h_i(0, 2), -1.0, 1.0e-12, "negative overflow H_i should match delta-v Jacobian");
  ExpectNear(h_j(0, 2), 1.0, 1.0e-12, "negative overflow H_j should match delta-v Jacobian");

  offline_lc_minimal::factor::VerticalVelocityDeadbandFactor velocity_factor(
    gtsam::symbol_shorthand::V(2),
    0.10,
    0.02,
    noise);
  gtsam::Matrix h;
  const auto velocity_inside =
    velocity_factor.evaluateError(gtsam::Vector3(0.0, 0.0, 0.115), h);
  ExpectNear(velocity_inside[0], 0.0, 1.0e-12, "inside velocity envelope residual should be zero");
  ExpectNear(h(0, 2), 0.0, 1.0e-12, "inside velocity envelope Jacobian should be zero");

  const auto velocity_outside =
    velocity_factor.evaluateError(gtsam::Vector3(0.0, 0.0, 0.15), h);
  ExpectNear(
    velocity_outside[0],
    0.03,
    1.0e-12,
    "outside velocity envelope residual should be overflow from target");
  ExpectNear(h(0, 2), 1.0, 1.0e-12, "outside velocity envelope Jacobian should be active");
}

void TestStage3JumpRegularizerAddsOnlyJumpWindowFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = false;
  config.vertical_velocity_delta_jump_padding_s = 0.0;
  config.enable_stage3_jump_velocity_smoothness_regularizer = true;
  config.stage3_jump_velocity_smoothness_deadband_mps = 0.02;
  config.stage3_jump_velocity_smoothness_sigma_mps = 0.03;
  config.enable_stage3_jump_height_highfreq_deadband = true;
  config.stage3_jump_height_highfreq_deadband_m = 0.002;
  config.stage3_jump_height_highfreq_sigma_m = 0.004;

  const std::vector<double> state_timestamps{0.0, 2.01, 2.06, 2.20};
  offline_lc_minimal::Stage3VerticalReference reference;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = state_timestamps[index];
    row.stage2_up_m = 10.0;
    row.stage2_lowpass_up_m = 10.0;
    row.lowpass_delta_m = 0.0;
    row.skip_reason = "PLANNED";
    reference.rows.push_back(row);
  }
  std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows(1U);
  jump_windows.front().start_time_s = 2.0;
  jump_windows.front().end_time_s = 2.1;

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage3JumpRegularizerDiagnosticRow> diagnostics;
  offline_lc_minimal::Stage3JumpRegularizerConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.reference = &reference;
  request.jump_windows = &jump_windows;
  request.dynamic_start_index = 1U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::Stage3JumpRegularizerConstraintBuilder(std::move(request)).Build();

  ExpectTrue(graph.size() == 4U, "regularizer should add two velocity and two height factors");
  ExpectTrue(diagnostics.size() == 4U, "regularizer diagnostics should track all added factors");
  ExpectTrue(
    summary.stage3_jump_velocity_smoothness_factor_count == 2U,
    "velocity smoothness factor count should be tracked");
  ExpectTrue(
    summary.stage3_jump_height_highfreq_deadband_factor_count == 2U,
    "height highfreq factor count should be tracked");

  gtsam::Values optimized_values;
  optimized_values.insert(gtsam::symbol_shorthand::V(1), gtsam::Vector3(0.0, 0.0, 0.0));
  optimized_values.insert(gtsam::symbol_shorthand::V(2), gtsam::Vector3(0.0, 0.0, 0.05));
  optimized_values.insert(gtsam::symbol_shorthand::V(3), gtsam::Vector3(0.0, 0.0, 0.055));
  optimized_values.insert(gtsam::symbol_shorthand::X(1), MakePose(10.004));
  optimized_values.insert(gtsam::symbol_shorthand::X(2), MakePose(10.001));
  optimized_values.insert(gtsam::symbol_shorthand::X(3), MakePose(10.000));

  offline_lc_minimal::PopulateStage3JumpRegularizerDiagnostics(
    optimized_values,
    diagnostics,
    summary);

  ExpectNear(
    summary.stage3_jump_velocity_smoothness_max_abs_residual_mps,
    0.03,
    1.0e-12,
    "velocity smoothness summary should report max overflow residual");
  ExpectNear(
    summary.stage3_jump_height_highfreq_deadband_max_abs_raw_residual_m,
    0.004,
    1.0e-12,
    "height regularizer should report max raw residual");
  ExpectNear(
    summary.stage3_jump_height_highfreq_deadband_max_abs_overflow_residual_m,
    0.002,
    1.0e-12,
    "height regularizer should report deadband overflow residual");
}

void TestStage3JumpAdaptiveContextEnvelopeUsesNearbyFluctuation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = false;
  config.vertical_velocity_delta_jump_padding_s = 0.0;
  config.enable_stage3_jump_velocity_smoothness_regularizer = true;
  config.stage3_jump_velocity_smoothness_deadband_mps = 0.02;
  config.stage3_jump_velocity_smoothness_sigma_mps = 0.03;
  config.enable_stage3_jump_height_highfreq_deadband = true;
  config.stage3_jump_height_highfreq_deadband_m = 0.010;
  config.stage3_jump_height_highfreq_sigma_m = 0.004;
  config.enable_stage3_jump_adaptive_context_envelope = true;
  config.stage3_jump_context_window_s = 2.0;
  config.stage3_jump_context_min_sample_count = 2;
  config.stage3_jump_context_quantile = 0.95;
  config.stage3_jump_context_velocity_multiplier = 1.0;
  config.stage3_jump_context_height_multiplier = 1.0;
  config.stage3_jump_context_preserve_local_center = true;
  config.stage3_jump_context_velocity_floor_mps = 0.0;
  config.stage3_jump_context_height_floor_m = 0.0;
  config.stage3_jump_context_velocity_cap_mps = 0.20;
  config.stage3_jump_context_height_cap_m = 0.006;

  const std::vector<double> state_timestamps{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  offline_lc_minimal::Stage3VerticalReference reference;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = state_timestamps[index];
    row.stage2_lowpass_up_m = 10.0;
    row.stage2_up_m = 10.0;
    row.skip_reason = "PLANNED";
    reference.rows.push_back(row);
  }
  reference.rows[1].stage2_up_m = 10.001;
  reference.rows[2].stage2_up_m = 9.999;
  reference.rows[5].stage2_up_m = 10.002;
  reference.rows[6].stage2_up_m = 9.998;

  std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows(1U);
  jump_windows.front().start_time_s = 3.0;
  jump_windows.front().end_time_s = 4.0;

  gtsam::Values initial_values;
  const std::vector<double> initial_vz{0.0, 0.10, 0.11, 0.50, -0.40, 0.09, 0.105};
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    initial_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, initial_vz[index]));
  }

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage3JumpRegularizerDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::Stage3JumpContextEnvelopeProfileRow> profiles;
  offline_lc_minimal::Stage3JumpRegularizerConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.reference = &reference;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.context_profiles = &profiles;
  offline_lc_minimal::Stage3JumpRegularizerConstraintBuilder(std::move(request)).Build();

  ExpectTrue(profiles.size() == 1U, "adaptive context envelope should output one profile");
  ExpectTrue(
    !profiles.front().velocity_fallback &&
      !profiles.front().velocity_delta_fallback &&
      !profiles.front().height_fallback,
    "profile should use nearby samples without fallback");
  ExpectTrue(
    profiles.front().height_deadband_m < config.stage3_jump_height_highfreq_deadband_m,
    "adaptive height deadband should tighten to local fluctuation");
  ExpectNear(
    profiles.front().context_vz_median_mps,
    0.1025,
    1.0e-12,
    "adaptive velocity envelope should center on nearby lowpass velocity plus residual median");
  ExpectNear(
    profiles.front().context_vz_residual_median_mps,
    0.1025,
    1.0e-12,
    "profile should expose the local velocity residual median");
  ExpectTrue(
    summary.stage3_jump_velocity_context_envelope_factor_count == 2U,
    "adaptive context should add per-state velocity envelope factors in jump window");
  ExpectTrue(
    std::any_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const offline_lc_minimal::Stage3JumpRegularizerDiagnosticRow &row) {
        return row.constraint_type == "velocity_context_envelope" &&
               row.factor_added &&
               std::abs(row.reference_vz_mps - 0.1025) < 1.0e-12;
      }),
    "diagnostics should expose the adaptive velocity context envelope target");

  gtsam::Values optimized_values;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    optimized_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, initial_vz[index]));
    optimized_values.insert(gtsam::symbol_shorthand::X(index), MakePose(10.0));
  }
  optimized_values.update(gtsam::symbol_shorthand::V(3), gtsam::Vector3(0.0, 0.0, 0.16));
  offline_lc_minimal::PopulateStage3JumpRegularizerDiagnostics(
    optimized_values,
    diagnostics,
    summary);
  ExpectTrue(
    summary.stage3_jump_velocity_context_envelope_max_abs_overflow_residual_mps > 0.0,
    "optimized velocity above the local context envelope should report overflow");
}

void TestStage3JumpAdaptiveContextEnvelopeSkipsVelocityWhenContextInsufficient() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_nhc_constraint = false;
  config.vertical_velocity_delta_jump_padding_s = 0.0;
  config.enable_stage3_jump_velocity_smoothness_regularizer = true;
  config.stage3_jump_velocity_smoothness_deadband_mps = 0.02;
  config.stage3_jump_velocity_smoothness_sigma_mps = 0.03;
  config.enable_stage3_jump_adaptive_context_envelope = true;
  config.stage3_jump_context_window_s = 0.25;
  config.stage3_jump_context_min_sample_count = 10;

  const std::vector<double> state_timestamps{0.0, 1.0, 2.0, 3.0, 4.0};
  offline_lc_minimal::Stage3VerticalReference reference;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = state_timestamps[index];
    row.stage2_up_m = 10.0;
    row.stage2_lowpass_up_m = 10.0;
    row.skip_reason = "PLANNED";
    reference.rows.push_back(row);
  }

  std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows(1U);
  jump_windows.front().start_time_s = 2.0;
  jump_windows.front().end_time_s = 3.0;

  gtsam::Values initial_values;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    initial_values.insert(
      gtsam::symbol_shorthand::V(index),
      gtsam::Vector3(0.0, 0.0, 0.0));
  }

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::Stage3JumpRegularizerDiagnosticRow> diagnostics;
  std::vector<offline_lc_minimal::Stage3JumpContextEnvelopeProfileRow> profiles;
  offline_lc_minimal::Stage3JumpRegularizerConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.reference = &reference;
  request.jump_windows = &jump_windows;
  request.initial_values = &initial_values;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.context_profiles = &profiles;
  offline_lc_minimal::Stage3JumpRegularizerConstraintBuilder(std::move(request)).Build();

  ExpectTrue(profiles.size() == 1U, "fallback test should still output one profile");
  ExpectTrue(
    profiles.front().velocity_fallback,
    "insufficient nearby velocity samples should mark velocity fallback");
  ExpectTrue(
    summary.stage3_jump_velocity_context_envelope_factor_count == 0U,
    "insufficient context should not add the new absolute velocity envelope");
  ExpectTrue(
    summary.stage3_jump_velocity_context_envelope_skipped_count > 0U,
    "insufficient context should be visible in diagnostics");
  ExpectTrue(
    std::any_of(
      diagnostics.begin(),
      diagnostics.end(),
      [](const offline_lc_minimal::Stage3JumpRegularizerDiagnosticRow &row) {
        return row.constraint_type == "velocity_context_envelope" &&
               !row.factor_added &&
               row.skip_reason == "INSUFFICIENT_CONTEXT";
      }),
    "diagnostics should label skipped velocity context envelope rows");
}

void TestTimelineAlignerResamplesSegmentedStage2Reference() {
  offline_lc_minimal::Stage2VelocityReference stage2_reference;
  stage2_reference.trajectory = MakeTrajectory(5U, 0.0);
  for (std::size_t index = 0; index < stage2_reference.trajectory.size(); ++index) {
    stage2_reference.trajectory[index].time_s = static_cast<double>(index);
    stage2_reference.trajectory[index].enu_position_m.z() = 10.0 * static_cast<double>(index);
    stage2_reference.trajectory[index].enu_velocity_mps.z() = static_cast<double>(index);
    stage2_reference.trajectory[index].bias_acc.z() = 0.1 * static_cast<double>(index);
  }

  offline_lc_minimal::Stage3VerticalReference stage3_reference;
  for (std::size_t index = 0; index < stage2_reference.trajectory.size(); ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = stage2_reference.trajectory[index].time_s;
    row.stage2_up_m = stage2_reference.trajectory[index].enu_position_m.z();
    row.stage2_lowpass_up_m = row.stage2_up_m + 1.0;
    row.lowpass_delta_m = 1.0;
    row.sigma_m = 0.015;
    row.skip_reason = "PLANNED";
    stage3_reference.rows.push_back(row);
  }

  const std::vector<double> target_timestamps{0.0, 0.5, 1.0, 1.5, 2.0};
  const auto aligned =
    offline_lc_minimal::AlignStage3VerticalReferencesToTimeline(
      stage2_reference,
      stage3_reference,
      target_timestamps);

  ExpectTrue(
    aligned.stage2_reference.trajectory.size() == target_timestamps.size(),
    "aligned Stage2 reference should match target timeline size");
  ExpectTrue(
    aligned.stage3_reference.rows.size() == target_timestamps.size(),
    "aligned Stage3 reference should match target timeline size");
  ExpectNear(
    aligned.stage2_reference.trajectory[1].enu_position_m.z(),
    5.0,
    1.0e-12,
    "Stage2 position should be linearly interpolated");
  ExpectNear(
    aligned.stage2_reference.trajectory[3].enu_velocity_mps.z(),
    1.5,
    1.0e-12,
    "Stage2 velocity should be linearly interpolated");
  ExpectNear(
    aligned.stage2_reference.trajectory[3].bias_acc.z(),
    0.15,
    1.0e-12,
    "Stage2 accel bias should be linearly interpolated");
  ExpectNear(
    aligned.stage3_reference.rows[1].stage2_lowpass_up_m,
    6.0,
    1.0e-12,
    "Stage3 lowpass up should be interpolated on the same timeline");
  ExpectNear(
    aligned.stage3_reference.rows[1].lowpass_delta_m,
    1.0,
    1.0e-12,
    "Stage3 lowpass delta should remain lowpass minus Stage2 up");
}

void TestTimelineAlignerUsesRot3InterpolationAcrossYawWrap() {
  const double deg = M_PI / 180.0;
  offline_lc_minimal::Stage2VelocityReference stage2_reference;
  stage2_reference.trajectory = MakeTrajectory(2U, 0.0);
  stage2_reference.trajectory[0].time_s = 0.0;
  stage2_reference.trajectory[0].ypr_rad = Eigen::Vector3d(179.0 * deg, 0.0, 0.0);
  stage2_reference.trajectory[1].time_s = 1.0;
  stage2_reference.trajectory[1].ypr_rad = Eigen::Vector3d(-179.0 * deg, 0.0, 0.0);

  offline_lc_minimal::Stage3VerticalReference stage3_reference;
  for (std::size_t index = 0; index < stage2_reference.trajectory.size(); ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = stage2_reference.trajectory[index].time_s;
    row.stage2_up_m = stage2_reference.trajectory[index].enu_position_m.z();
    row.stage2_lowpass_up_m = row.stage2_up_m;
    row.skip_reason = "PLANNED";
    stage3_reference.rows.push_back(row);
  }

  const auto aligned =
    offline_lc_minimal::AlignStage3VerticalReferencesToTimeline(
      stage2_reference,
      stage3_reference,
      std::vector<double>{0.0, 0.5, 1.0});

  ExpectTrue(
    aligned.stage2_reference.reference_states.size() == 3U,
    "aligned Stage2 reference should retain Rot3 states");
  ExpectNear(
    RelativeRotationAngleRad(
      aligned.stage2_reference.reference_states[0].pose.rotation(),
      aligned.stage2_reference.reference_states[1].pose.rotation()),
    1.0 * deg,
    1.0e-12,
    "Stage3 timeline alignment should interpolate along the short Rot3 path");
}

void TestTimelineAlignerRejectsMissingStage2Coverage() {
  offline_lc_minimal::Stage2VelocityReference stage2_reference;
  stage2_reference.trajectory = MakeTrajectory(4U, 0.0);
  stage2_reference.trajectory[0].time_s = 0.0;
  stage2_reference.trajectory[1].time_s = 1.0;
  stage2_reference.trajectory[2].time_s = 3.0;
  stage2_reference.trajectory[3].time_s = 4.0;

  offline_lc_minimal::Stage3VerticalReference stage3_reference;
  for (std::size_t index = 0; index < stage2_reference.trajectory.size(); ++index) {
    offline_lc_minimal::Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = stage2_reference.trajectory[index].time_s;
    row.stage2_up_m = stage2_reference.trajectory[index].enu_position_m.z();
    row.stage2_lowpass_up_m = row.stage2_up_m;
    row.skip_reason = "PLANNED";
    stage3_reference.rows.push_back(row);
  }

  bool threw = false;
  try {
    (void)offline_lc_minimal::AlignStage3VerticalReferencesToTimeline(
      stage2_reference,
      stage3_reference,
      std::vector<double>{0.0, 1.0, 2.0, 3.0});
  } catch (const std::runtime_error &exception) {
    threw =
      std::string(exception.what()).find("does not cover the Stage3 graph timeline") !=
      std::string::npos;
  }
  ExpectTrue(threw, "Stage3 aligner should reject missing Stage2 trajectory coverage");
}

void TestStage3RunnerRunsStage2OnceThenStage3WithoutRecursion() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage3_vertical_reference_optimization = true;
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage3_initial_dynamic_static_reference_hold = true;
  config.stage3_initial_dynamic_static_reference_hold_duration_s = 2.0;
  config.stage3_initial_dynamic_static_reference_hold_blend_s = 0.0;
  config.enable_stage1_yaw_refinement = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.stage3_disable_rtk_outage_segmented_batch = true;
  config.enable_rtk_vertical_drift_reference = true;
  config.enable_rtk_vertical_lowpass_reference = true;
  config.enable_rtk_outage_smoothing = true;
  config.enable_rtk_outage_velocity_delta_3d = true;
  config.enable_rtk_outage_causal_drift_reference = true;
  config.enable_rtk_outage_preoutage_vertical_fence = true;
  config.enable_late_static_detection = true;
  config.enable_initial_static_rtk_height_reference = true;
  config.enable_initial_dynamic_static_detection = true;
  config.enable_initial_dynamic_static_lowpass_protection = true;
  config.enable_initial_dynamic_static_vz_constraint = true;
  config.enable_vertical_jump_masked_imu = true;
  config.enable_vertical_jump_impulse = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_segmented_bias = true;
  config.enable_vertical_jump_spectral_bias_relaxation = true;
  config.enable_vertical_jump_velocity_ramp_smoothing = true;
  config.enable_vertical_jump_position_ramp_smoothing = true;
  config.enable_vertical_jump_velocity_continuity = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.enable_vertical_jump_context_mean_continuity = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.enable_vertical_jump_velocity_height_slope_constraint = true;
  config.enable_attitude_reference_constraint = true;
  config.enable_base_graph_tilt_reference_constraint = true;
  config.enable_stage3_jump_velocity_smoothness_regularizer = true;
  config.enable_stage3_jump_height_highfreq_deadband = true;
  config.enable_stage3_jump_adaptive_context_envelope = true;
  config.stage3_disable_stage2_vehicle_nhc_constraint = true;

  struct Call {
    bool enable_stage3 = false;
    bool enable_stage2 = false;
    bool enable_stage1 = false;
    bool enable_segmented_batch = false;
    bool enable_rtk_vertical_drift = false;
    bool enable_rtk_vertical_lowpass = false;
    bool enable_rtk_outage_smoothing = false;
    bool enable_rtk_outage_velocity_delta_3d = false;
    bool enable_causal_reference = false;
    bool enable_preoutage_fence = false;
    bool enable_late_static = false;
    bool enable_initial_static_rtk_height = false;
    bool enable_initial_dynamic_static = false;
    bool enable_initial_dynamic_static_lowpass = false;
    bool enable_initial_dynamic_static_vz = false;
    bool enable_vertical_jump_masked_imu = false;
    bool enable_vertical_jump_impulse = false;
    bool enable_vertical_jump_bias = false;
    bool enable_vertical_jump_segmented_bias = false;
    bool enable_vertical_jump_spectral = false;
    bool enable_vertical_jump_velocity_ramp = false;
    bool enable_vertical_jump_position_ramp = false;
    bool enable_vertical_jump_velocity_continuity = false;
    bool enable_vertical_jump_velocity_context_mean = false;
    bool enable_vertical_jump_context_mean_continuity = false;
    bool enable_vertical_jump_position_velocity = false;
    bool enable_vertical_jump_height_slope = false;
    bool enable_body_z_nhc = false;
    bool enable_stage2_vehicle_nhc = false;
    bool enable_attitude_reference = false;
    bool enable_base_graph_tilt_reference = false;
    bool enable_stage3_jump_velocity_regularizer = false;
    bool enable_stage3_jump_height_deadband = false;
    bool enable_stage3_jump_adaptive_context = false;
    bool stage3_disable_stage2_vehicle_nhc = false;
    bool has_stage2_reference = false;
    bool has_stage3_reference = false;
  };
  std::vector<Call> calls;

  offline_lc_minimal::Stage3VerticalReferenceOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.run_once = [&](const offline_lc_minimal::OfflineRunnerConfig &run_config,
                         std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> stage2_reference,
                         std::shared_ptr<const offline_lc_minimal::Stage3VerticalReference> stage3_reference,
                         offline_lc_minimal::DataSet) {
    calls.push_back(Call{
      run_config.enable_stage3_vertical_reference_optimization,
      run_config.enable_stage2_velocity_optimization,
      run_config.enable_stage1_yaw_refinement,
      run_config.enable_rtk_outage_segmented_batch,
      run_config.enable_rtk_vertical_drift_reference,
      run_config.enable_rtk_vertical_lowpass_reference,
      run_config.enable_rtk_outage_smoothing,
      run_config.enable_rtk_outage_velocity_delta_3d,
      run_config.enable_rtk_outage_causal_drift_reference,
      run_config.enable_rtk_outage_preoutage_vertical_fence,
      run_config.enable_late_static_detection,
      run_config.enable_initial_static_rtk_height_reference,
      run_config.enable_initial_dynamic_static_detection,
      run_config.enable_initial_dynamic_static_lowpass_protection,
      run_config.enable_initial_dynamic_static_vz_constraint,
      run_config.enable_vertical_jump_masked_imu,
      run_config.enable_vertical_jump_impulse,
      run_config.enable_vertical_jump_bias,
      run_config.enable_vertical_jump_segmented_bias,
      run_config.enable_vertical_jump_spectral_bias_relaxation,
      run_config.enable_vertical_jump_velocity_ramp_smoothing,
      run_config.enable_vertical_jump_position_ramp_smoothing,
      run_config.enable_vertical_jump_velocity_continuity,
      run_config.enable_vertical_jump_velocity_context_mean,
      run_config.enable_vertical_jump_context_mean_continuity,
      run_config.enable_vertical_jump_position_velocity_consistency,
      run_config.enable_vertical_jump_velocity_height_slope_constraint,
      run_config.enable_body_z_nhc_constraint,
      run_config.enable_stage2_vehicle_nhc_constraint,
      run_config.enable_attitude_reference_constraint,
      run_config.enable_base_graph_tilt_reference_constraint,
      run_config.enable_stage3_jump_velocity_smoothness_regularizer,
      run_config.enable_stage3_jump_height_highfreq_deadband,
      run_config.enable_stage3_jump_adaptive_context_envelope,
      run_config.stage3_disable_stage2_vehicle_nhc_constraint,
      static_cast<bool>(stage2_reference),
      static_cast<bool>(stage3_reference)});

    offline_lc_minimal::OfflineRunResult result;
    if (!stage2_reference && !stage3_reference) {
      result.trajectory = MakeTrajectory(11U, 0.02);
      result.trajectory[3].enu_position_m.z() =
        result.trajectory[2].enu_position_m.z() + 1.0;
      result.trajectory[3].time_s = 2.997;
      result.run_summary.initial_static_state_count = 3U;
      result.run_summary.dynamic_start_time_s = 3.0;
      return result;
    }
    ExpectTrue(static_cast<bool>(stage2_reference), "Stage3 pass should receive Stage2 reference");
    ExpectTrue(static_cast<bool>(stage3_reference), "Stage3 pass should receive lowpass reference");
    ExpectNear(
      stage3_reference->rows[3].stage2_lowpass_up_m,
      stage2_reference->trajectory[2].enu_position_m.z(),
      1.0e-12,
      "Stage3 runner should hold the first dynamic state to the last static height");
    result.trajectory = stage2_reference->trajectory;
    result.stage3_vertical_reference_diagnostics = stage3_reference->rows;
    return result;
  };

  const auto result =
    offline_lc_minimal::Stage3VerticalReferenceOptimizationRunner(std::move(request)).Run();
  ExpectTrue(calls.size() == 2U, "Stage3 runner should run exactly Stage2 then Stage3");
  ExpectTrue(!calls[0].enable_stage3, "Stage2 source pass should disable Stage3 recursion");
  ExpectTrue(!calls[0].has_stage2_reference, "Stage2 source pass should not receive a Stage2 reference");
  ExpectTrue(!calls[0].has_stage3_reference, "Stage2 source pass should not receive a Stage3 reference");
  ExpectTrue(!calls[1].enable_stage3, "Stage3 pass should also disable Stage3 recursion");
  ExpectTrue(calls[1].enable_stage2, "Stage3 pass should keep Stage2 constraint system enabled");
  ExpectTrue(!calls[1].enable_stage1, "Stage3 pass should not re-run Stage1 yaw refinement");
  ExpectTrue(!calls[1].enable_segmented_batch, "Stage3 pass should disable RTK outage segmented batch");
  ExpectTrue(!calls[1].enable_rtk_vertical_drift, "Stage3 pass should disable RTK vertical drift reference");
  ExpectTrue(!calls[1].enable_rtk_vertical_lowpass, "Stage3 pass should disable RTK vertical lowpass reference");
  ExpectTrue(!calls[1].enable_rtk_outage_smoothing, "Stage3 pass should disable RTK outage smoothing");
  ExpectTrue(!calls[1].enable_rtk_outage_velocity_delta_3d, "Stage3 pass should disable outage 3D velocity delta");
  ExpectTrue(!calls[1].enable_causal_reference, "Stage3 pass should disable causal RTK drift reference");
  ExpectTrue(!calls[1].enable_preoutage_fence, "Stage3 pass should disable pre-outage vertical fence");
  ExpectTrue(!calls[1].enable_late_static, "Stage3 pass should disable late-static raw RTK height anchors");
  ExpectTrue(!calls[1].enable_initial_static_rtk_height, "Stage3 pass should disable initial static RTK height anchors");
  ExpectTrue(!calls[1].enable_initial_dynamic_static, "Stage3 pass should disable initial dynamic static detection");
  ExpectTrue(
    !calls[1].enable_initial_dynamic_static_lowpass,
    "Stage3 pass should not protect its final reference with raw initial dynamic static windows");
  ExpectTrue(
    !calls[1].enable_initial_dynamic_static_vz,
    "Stage3 pass should not add initial dynamic static vertical factors");
  ExpectTrue(calls[0].enable_vertical_jump_bias, "Stage2 source pass should keep requested jump bias");
  ExpectTrue(calls[1].enable_vertical_jump_masked_imu, "Stage3 pass should keep vertical jump IMU masking");
  ExpectTrue(calls[1].enable_vertical_jump_impulse, "Stage3 pass should keep vertical jump impulse factors");
  ExpectTrue(calls[1].enable_vertical_jump_bias, "Stage3 pass should keep vertical jump bias factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_segmented_bias,
    "Stage3 pass should keep segmented vertical jump bias factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_spectral,
    "Stage3 pass should keep spectral vertical jump relaxation");
  ExpectTrue(
    calls[1].enable_vertical_jump_velocity_ramp,
    "Stage3 pass should keep vertical jump velocity ramp factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_position_ramp,
    "Stage3 pass should keep vertical jump position ramp factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_velocity_continuity,
    "Stage3 pass should keep vertical jump velocity continuity factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_velocity_context_mean,
    "Stage3 pass should keep vertical jump context mean factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_context_mean_continuity,
    "Stage3 pass should keep vertical jump context mean continuity factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_position_velocity,
    "Stage3 pass should keep vertical jump position-velocity factors");
  ExpectTrue(
    calls[1].enable_vertical_jump_height_slope,
    "Stage3 pass should keep vertical jump height-slope factors");
  ExpectTrue(!calls[1].enable_body_z_nhc, "Stage3 pass should keep Stage2 policy's Body-Z NHC disabled");
  ExpectTrue(
    !calls[1].enable_stage2_vehicle_nhc,
    "Stage3 pass should inherit Stage2 horizontal velocity through holds instead of rerunning vehicle NHC");
  ExpectTrue(!calls[1].enable_attitude_reference, "Stage3 pass should not restore generic attitude reference");
  ExpectTrue(
    !calls[1].enable_base_graph_tilt_reference,
    "Stage3 pass should not use base-graph tilt as a competing attitude reference");
  ExpectTrue(
    !calls[1].enable_stage3_jump_velocity_regularizer,
    "Stage3 pass should not use the legacy Stage3 jump velocity regularizer");
  ExpectTrue(
    !calls[1].enable_stage3_jump_height_deadband,
    "Stage3 pass should not use the legacy Stage3 jump height deadband");
  ExpectTrue(
    !calls[1].enable_stage3_jump_adaptive_context,
    "Stage3 pass should not use the legacy Stage3 jump context envelope");
  ExpectTrue(
    !calls[1].stage3_disable_stage2_vehicle_nhc,
    "Stage3 child config should clear wrapper-only vehicle NHC switches");
  ExpectTrue(calls[1].has_stage2_reference, "Stage3 pass should receive Stage2 reference");
  ExpectTrue(calls[1].has_stage3_reference, "Stage3 pass should receive Stage3 reference");
  ExpectTrue(
    result.run_summary.stage3_vertical_reference_optimization_enabled,
    "runner summary should mark Stage3 enabled");
}

void TestStage3RunnerAlwaysDisablesSegmentedBatch() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage3_vertical_reference_optimization = true;
  config.enable_stage2_velocity_optimization = true;
  config.enable_rtk_outage_segmented_batch = true;
  config.stage3_disable_rtk_outage_segmented_batch = false;

  std::vector<bool> segmented_flags;
  offline_lc_minimal::Stage3VerticalReferenceOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.run_once = [&](const offline_lc_minimal::OfflineRunnerConfig &run_config,
                         std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> stage2_reference,
                         std::shared_ptr<const offline_lc_minimal::Stage3VerticalReference> stage3_reference,
                         offline_lc_minimal::DataSet) {
    segmented_flags.push_back(run_config.enable_rtk_outage_segmented_batch);
    offline_lc_minimal::OfflineRunResult result;
    if (!stage2_reference && !stage3_reference) {
      result.trajectory = MakeTrajectory(5U, 0.01);
      return result;
    }
    result.trajectory = stage2_reference->trajectory;
    return result;
  };

  (void)offline_lc_minimal::Stage3VerticalReferenceOptimizationRunner(std::move(request)).Run();
  ExpectTrue(segmented_flags.size() == 2U, "Stage3 runner should run two passes");
  ExpectTrue(segmented_flags[0], "Stage2 source pass should preserve the requested segmented setting");
  ExpectTrue(
    !segmented_flags[1],
    "Stage3 pass should always disable segmented batch even if the compatibility flag is false");
}

void TestStage3RunnerAlwaysDisablesFinalVehicleNHC() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage3_vertical_reference_optimization = true;
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage2_vehicle_nhc_constraint = true;
  config.stage3_disable_stage2_vehicle_nhc_constraint = true;

  std::vector<bool> vehicle_nhc_flags;
  std::vector<bool> child_disable_flags;
  offline_lc_minimal::Stage3VerticalReferenceOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.run_once = [&](const offline_lc_minimal::OfflineRunnerConfig &run_config,
                         std::shared_ptr<const offline_lc_minimal::Stage2VelocityReference> stage2_reference,
                         std::shared_ptr<const offline_lc_minimal::Stage3VerticalReference> stage3_reference,
                         offline_lc_minimal::DataSet) {
    offline_lc_minimal::ValidateConfig(run_config);
    vehicle_nhc_flags.push_back(run_config.enable_stage2_vehicle_nhc_constraint);
    child_disable_flags.push_back(run_config.stage3_disable_stage2_vehicle_nhc_constraint);
    offline_lc_minimal::OfflineRunResult result;
    if (!stage2_reference && !stage3_reference) {
      result.trajectory = MakeTrajectory(5U, 0.01);
      return result;
    }
    result.trajectory = stage2_reference->trajectory;
    return result;
  };

  (void)offline_lc_minimal::Stage3VerticalReferenceOptimizationRunner(std::move(request)).Run();
  ExpectTrue(vehicle_nhc_flags.size() == 2U, "Stage3 runner should run two passes");
  ExpectTrue(
    vehicle_nhc_flags[0],
    "Stage2 source pass should keep vehicle NHC enabled for the lowpass source");
  ExpectTrue(
    !vehicle_nhc_flags[1],
    "Stage3 final pass should always disable vehicle NHC after inheriting the Stage2 velocity reference");
  ExpectTrue(
    !child_disable_flags[0] && !child_disable_flags[1],
    "Stage3 child configs should clear the wrapper-only vehicle NHC disable flag");
}

void TestStage2LowfreqRunnerRunsRawSourceThenLowpassStage2() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_lowfreq_vertical_reference_optimization = true;
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage3_vertical_reference_optimization = false;
  config.stage2_lowfreq_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
  config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk;
  config.stage2_lowfreq_vertical_reference_cutoff_hz = 0.05;
  config.enable_stage2_lowfreq_final_dvz_relaxation = true;
  config.stage2_lowfreq_final_dvz_sigma_scale = 10.0;
  config.enable_stage2_lowfreq_final_hold_relaxation = true;
  config.stage2_lowfreq_final_attitude_hold_sigma_scale = 20.0;
  config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale = 50.0;
  config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale = 80.0;
  config.stage2_attitude_hold_sigma_rad = 1.0e-5;
  config.stage2_horizontal_position_hold_sigma_m = 1.0e-4;
  config.stage2_horizontal_velocity_hold_sigma_mps = 1.0e-4;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_bias_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;

  struct Call {
    bool enable_stage2_lowfreq = false;
    bool enable_stage3 = false;
    offline_lc_minimal::GnssVerticalReferenceSource gnss_source =
      offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk;
    bool has_lowpass_reference = false;
    double dvz_acc_sigma_mps2 = 0.0;
    double dvz_min_sigma_mps = 0.0;
    double dvz_floor_mps = 0.0;
    double dvz_ceiling_mps = 0.0;
    double dvz_attitude_sigma_rad = 0.0;
    double dvz_sigma_scale = 0.0;
    double computed_dvz_sigma_mps = 0.0;
    bool final_hold_relaxation_enabled = false;
    double attitude_hold_sigma_rad = 0.0;
    double horizontal_position_hold_sigma_m = 0.0;
    double horizontal_velocity_hold_sigma_mps = 0.0;
  };
  std::vector<Call> calls;

  offline_lc_minimal::Stage2LowfreqVerticalReferenceOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.run_once = [&](const offline_lc_minimal::OfflineRunnerConfig &run_config,
                         std::shared_ptr<const offline_lc_minimal::Stage3VerticalReference> lowpass_reference,
                         offline_lc_minimal::DataSet) {
    offline_lc_minimal::ValidateConfig(run_config);
    calls.push_back(Call{
      run_config.enable_stage2_lowfreq_vertical_reference_optimization,
      run_config.enable_stage3_vertical_reference_optimization,
      run_config.gnss_vertical_reference_source,
      static_cast<bool>(lowpass_reference),
      run_config.vertical_velocity_delta_acc_sigma_mps2,
      run_config.vertical_velocity_delta_min_sigma_mps,
      run_config.vertical_velocity_delta_sigma_floor_mps,
      run_config.vertical_velocity_delta_sigma_ceiling_mps,
      run_config.vertical_velocity_delta_attitude_sigma_rad,
      run_config.vertical_velocity_delta_sigma_scale,
      offline_lc_minimal::VerticalVelocityDeltaSigmaModel(run_config)
        .Compute(0.05)
        .sigma_mps,
      run_config.enable_stage2_lowfreq_final_hold_relaxation,
      run_config.stage2_attitude_hold_sigma_rad,
      run_config.stage2_horizontal_position_hold_sigma_m,
      run_config.stage2_horizontal_velocity_hold_sigma_mps});

    offline_lc_minimal::OfflineRunResult result;
    if (!lowpass_reference) {
      result.trajectory = MakeTrajectory(11U, 0.02);
      return result;
    }
    result.trajectory = MakeTrajectory(11U, 0.0);
    return result;
  };

  const auto result =
    offline_lc_minimal::Stage2LowfreqVerticalReferenceOptimizationRunner(
      std::move(request)).Run();

  ExpectTrue(calls.size() == 2U, "Stage2 lowfreq runner should run source then final pass");
  ExpectTrue(!calls[0].enable_stage2_lowfreq, "source pass should disable Stage2 lowfreq recursion");
  ExpectTrue(!calls[0].enable_stage3, "source pass should disable Stage3");
  ExpectTrue(
    calls[0].gnss_source == offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk,
    "source pass should use raw RTK vertical GNSS");
  ExpectTrue(!calls[0].has_lowpass_reference, "source pass should not receive a lowpass reference");
  ExpectNear(calls[0].dvz_acc_sigma_mps2, 0.10, 1e-15, "source pass should not relax DVZ acc sigma");
  ExpectNear(calls[0].dvz_min_sigma_mps, 0.003, 1e-15, "source pass should not relax DVZ min sigma");
  ExpectNear(calls[0].dvz_floor_mps, 1.0e-5, 1e-15, "source pass should not relax DVZ floor");
  ExpectNear(calls[0].dvz_ceiling_mps, 5.0e-4, 1e-15, "source pass should not relax DVZ ceiling");
  ExpectNear(
    calls[0].dvz_attitude_sigma_rad,
    1.0e-4,
    1e-15,
    "source pass should not relax DVZ attitude sigma");
  ExpectNear(calls[0].dvz_sigma_scale, 1.0, 1e-15, "source pass should not relax DVZ output sigma");
  ExpectTrue(!calls[0].final_hold_relaxation_enabled, "source pass should clear final hold relaxation");
  ExpectNear(calls[0].attitude_hold_sigma_rad, 1.0e-5, 1e-15, "source pass should not relax attitude hold");
  ExpectNear(
    calls[0].horizontal_position_hold_sigma_m,
    1.0e-4,
    1e-15,
    "source pass should not relax horizontal position hold");
  ExpectNear(
    calls[0].horizontal_velocity_hold_sigma_mps,
    1.0e-4,
    1e-15,
    "source pass should not relax horizontal velocity hold");
  ExpectTrue(calls[1].enable_stage2_lowfreq, "final pass should keep the wrapper flag valid");
  ExpectTrue(!calls[1].enable_stage3, "final pass should keep Stage3 disabled");
  ExpectTrue(
    calls[1].gnss_source == offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass,
    "final pass should switch GNSS vertical reference to Stage2 lowpass");
  ExpectTrue(calls[1].has_lowpass_reference, "final pass should receive the planned lowpass reference");
  ExpectNear(calls[1].dvz_acc_sigma_mps2, 0.10, 1e-15, "final pass should preserve DVZ acc sigma");
  ExpectNear(calls[1].dvz_min_sigma_mps, 0.003, 1e-15, "final pass should preserve DVZ min sigma");
  ExpectNear(calls[1].dvz_floor_mps, 1.0e-5, 1e-15, "final pass should preserve DVZ floor");
  ExpectNear(calls[1].dvz_ceiling_mps, 5.0e-4, 1e-15, "final pass should preserve DVZ ceiling");
  ExpectNear(
    calls[1].dvz_attitude_sigma_rad,
    1.0e-4,
    1e-15,
    "final pass should preserve DVZ attitude sigma");
  ExpectNear(calls[1].dvz_sigma_scale, 10.0, 1e-15, "final pass should relax DVZ output sigma");
  ExpectTrue(calls[1].final_hold_relaxation_enabled, "final pass should keep final hold relaxation enabled");
  ExpectNear(calls[1].attitude_hold_sigma_rad, 2.0e-4, 1e-15, "final pass should relax attitude hold");
  ExpectNear(
    calls[1].horizontal_position_hold_sigma_m,
    5.0e-3,
    1e-15,
    "final pass should relax horizontal position hold");
  ExpectNear(
    calls[1].horizontal_velocity_hold_sigma_mps,
    8.0e-3,
    1e-15,
    "final pass should relax horizontal velocity hold");
  ExpectNear(
    calls[1].computed_dvz_sigma_mps,
    calls[0].computed_dvz_sigma_mps * 10.0,
    1e-15,
    "final pass should relax the computed DVZ sigma by 10x");
  ExpectTrue(
    result.run_summary.stage2_lowfreq_vertical_reference_optimization_enabled,
    "runner summary should mark Stage2 lowfreq enabled");
  ExpectTrue(
    result.run_summary.stage2_lowfreq_final_dvz_relaxation_enabled,
    "runner summary should mark final DVZ relaxation enabled");
  ExpectNear(
    result.run_summary.stage2_lowfreq_final_dvz_sigma_scale,
    10.0,
    1e-15,
    "runner summary should report final DVZ relaxation scale");
  ExpectTrue(
    result.run_summary.stage2_lowfreq_final_hold_relaxation_enabled,
    "runner summary should mark final hold relaxation enabled");
  ExpectNear(
    result.run_summary.stage2_lowfreq_final_attitude_hold_sigma_scale,
    20.0,
    1e-15,
    "runner summary should report final attitude hold relaxation scale");
  ExpectNear(
    result.run_summary.stage2_lowfreq_final_horizontal_position_hold_sigma_scale,
    50.0,
    1e-15,
    "runner summary should report final horizontal position hold relaxation scale");
  ExpectNear(
    result.run_summary.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale,
    80.0,
    1e-15,
    "runner summary should report final horizontal velocity hold relaxation scale");
  ExpectTrue(
    !result.stage2_lowfreq_vertical_reference_diagnostics.empty(),
    "runner should store lowfreq diagnostics in the Stage2-lowfreq container");
  ExpectTrue(
    result.stage3_vertical_reference_diagnostics.empty(),
    "Stage2 lowfreq runner should not populate Stage3 diagnostics");
}

void TestStage2LowfreqRunnerRelaxesContextDvzScalesOnce() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage2_lowfreq_vertical_reference_optimization = true;
  config.enable_stage2_velocity_optimization = true;
  config.enable_stage3_vertical_reference_optimization = false;
  config.stage2_lowfreq_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kStage2Lowpass;
  config.gnss_vertical_reference_source =
    offline_lc_minimal::GnssVerticalReferenceSource::kRawRtk;
  config.enable_stage2_lowfreq_final_dvz_relaxation = true;
  config.stage2_lowfreq_final_dvz_sigma_scale = 10.0;
  config.vertical_velocity_delta_sigma_scale = 2.0;
  config.enable_vertical_velocity_delta_context_sigma_scale = true;
  config.vertical_velocity_delta_context_normal_sigma_scale = 100.0;
  config.vertical_velocity_delta_context_rough_sigma_scale = 1000.0;
  config.vertical_velocity_delta_context_outage_sigma_scale = 800.0;
  config.vertical_velocity_delta_context_jump_sigma_scale = 500.0;
  config.vertical_velocity_delta_context_jump_extra_padding_s = 0.75;

  struct Call {
    double global_scale = 0.0;
    bool context_enabled = false;
    double normal_scale = 0.0;
    double rough_scale = 0.0;
    double outage_scale = 0.0;
    double jump_scale = 0.0;
    double jump_extra_padding_s = 0.0;
    bool has_lowpass_reference = false;
  };
  std::vector<Call> calls;

  offline_lc_minimal::Stage2LowfreqVerticalReferenceOptimizationRequest request;
  request.config = config;
  request.dataset = offline_lc_minimal::DataSet{};
  request.run_once = [&](const offline_lc_minimal::OfflineRunnerConfig &run_config,
                         std::shared_ptr<const offline_lc_minimal::Stage3VerticalReference> lowpass_reference,
                         offline_lc_minimal::DataSet) {
    offline_lc_minimal::ValidateConfig(run_config);
    calls.push_back(Call{
      run_config.vertical_velocity_delta_sigma_scale,
      run_config.enable_vertical_velocity_delta_context_sigma_scale,
      run_config.vertical_velocity_delta_context_normal_sigma_scale,
      run_config.vertical_velocity_delta_context_rough_sigma_scale,
      run_config.vertical_velocity_delta_context_outage_sigma_scale,
      run_config.vertical_velocity_delta_context_jump_sigma_scale,
      run_config.vertical_velocity_delta_context_jump_extra_padding_s,
      static_cast<bool>(lowpass_reference)});

    offline_lc_minimal::OfflineRunResult result;
    result.trajectory = MakeTrajectory(11U, lowpass_reference ? 0.0 : 0.02);
    return result;
  };

  (void)offline_lc_minimal::Stage2LowfreqVerticalReferenceOptimizationRunner(
    std::move(request)).Run();

  ExpectTrue(calls.size() == 2U, "Stage2 lowfreq context test should run source then final pass");
  ExpectTrue(!calls[0].has_lowpass_reference, "source pass should not receive lowpass reference");
  ExpectTrue(calls[1].has_lowpass_reference, "final pass should receive lowpass reference");
  ExpectNear(calls[0].global_scale, 2.0, 1e-15, "source pass should preserve global scale");
  ExpectNear(calls[1].global_scale, 2.0, 1e-15, "context final relaxation should not multiply global scale");
  ExpectTrue(calls[0].context_enabled && calls[1].context_enabled, "context scaling should remain enabled");
  ExpectNear(calls[0].normal_scale, 100.0, 1e-15, "source normal context scale is wrong");
  ExpectNear(calls[0].rough_scale, 1000.0, 1e-15, "source rough context scale is wrong");
  ExpectNear(calls[0].outage_scale, 800.0, 1e-15, "source outage context scale is wrong");
  ExpectNear(calls[0].jump_scale, 500.0, 1e-15, "source jump context scale is wrong");
  ExpectNear(calls[1].normal_scale, 1000.0, 1e-15, "final normal context scale should relax once");
  ExpectNear(calls[1].rough_scale, 10000.0, 1e-15, "final rough context scale should relax once");
  ExpectNear(calls[1].outage_scale, 8000.0, 1e-15, "final outage context scale should relax once");
  ExpectNear(calls[1].jump_scale, 5000.0, 1e-15, "final jump context scale should relax once");
  ExpectNear(
    calls[1].jump_extra_padding_s,
    0.75,
    1e-15,
    "final context relaxation should preserve jump extra padding");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestProfilePlannerBuildsFiniteZeroPhaseLowpass",
      TestProfilePlannerBuildsFiniteZeroPhaseLowpass);
    RunTest(
      "TestProfilePlannerBuildsSplineBaselineReference",
      TestProfilePlannerBuildsSplineBaselineReference);
    RunTest(
      "TestProfilePlannerCanHoldInitialDynamicStaticReference",
      TestProfilePlannerCanHoldInitialDynamicStaticReference);
    RunTest(
      "TestProfilePlannerUsesDetectedInitialDynamicStaticWindows",
      TestProfilePlannerUsesDetectedInitialDynamicStaticWindows);
    RunTest(
      "TestProfilePlannerStartsLowpassAfterInitialStatic",
      TestProfilePlannerStartsLowpassAfterInitialStatic);
    RunTest(
      "TestProfilePlannerExcludesTerminalStaticFromLowpass",
      TestProfilePlannerExcludesTerminalStaticFromLowpass);
    RunTest(
      "TestConstraintBuilderAddsOnlyDynamicAnchorsAndDiagnostics",
      TestConstraintBuilderAddsOnlyDynamicAnchorsAndDiagnostics);
    RunTest(
      "TestConstraintBuilderAnchorsTerminalStaticReferences",
      TestConstraintBuilderAnchorsTerminalStaticReferences);
    RunTest(
      "TestConstraintBuilderEnvelopeModeUsesGateAndCenterPull",
      TestConstraintBuilderEnvelopeModeUsesGateAndCenterPull);
    RunTest(
      "TestVerticalVelocityDeltaDeadbandFactor",
      TestVerticalVelocityDeltaDeadbandFactor);
    RunTest(
      "TestStage3JumpRegularizerAddsOnlyJumpWindowFactors",
      TestStage3JumpRegularizerAddsOnlyJumpWindowFactors);
    RunTest(
      "TestStage3JumpAdaptiveContextEnvelopeUsesNearbyFluctuation",
      TestStage3JumpAdaptiveContextEnvelopeUsesNearbyFluctuation);
    RunTest(
      "TestStage3JumpAdaptiveContextEnvelopeSkipsVelocityWhenContextInsufficient",
      TestStage3JumpAdaptiveContextEnvelopeSkipsVelocityWhenContextInsufficient);
    RunTest(
      "TestTimelineAlignerResamplesSegmentedStage2Reference",
      TestTimelineAlignerResamplesSegmentedStage2Reference);
    RunTest(
      "TestTimelineAlignerUsesRot3InterpolationAcrossYawWrap",
      TestTimelineAlignerUsesRot3InterpolationAcrossYawWrap);
    RunTest(
      "TestTimelineAlignerRejectsMissingStage2Coverage",
      TestTimelineAlignerRejectsMissingStage2Coverage);
    RunTest(
      "TestStage3RunnerRunsStage2OnceThenStage3WithoutRecursion",
      TestStage3RunnerRunsStage2OnceThenStage3WithoutRecursion);
    RunTest(
      "TestStage3RunnerAlwaysDisablesSegmentedBatch",
      TestStage3RunnerAlwaysDisablesSegmentedBatch);
    RunTest(
      "TestStage3RunnerAlwaysDisablesFinalVehicleNHC",
      TestStage3RunnerAlwaysDisablesFinalVehicleNHC);
    RunTest(
      "TestStage2LowfreqRunnerRunsRawSourceThenLowpassStage2",
      TestStage2LowfreqRunnerRunsRawSourceThenLowpassStage2);
    RunTest(
      "TestStage2LowfreqRunnerRelaxesContextDvzScalesOnce",
      TestStage2LowfreqRunnerRelaxesContextDvzScalesOnce);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
