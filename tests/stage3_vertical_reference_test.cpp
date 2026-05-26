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
#include "offline_lc_minimal/core/Stage3VerticalReferenceConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceOptimizationRunner.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceTimelineAligner.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"

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
  config.enable_rtk_outage_causal_drift_reference = true;
  config.enable_rtk_outage_preoutage_vertical_fence = true;
  config.enable_late_static_detection = true;

  struct Call {
    bool enable_stage3 = false;
    bool enable_stage2 = false;
    bool enable_stage1 = false;
    bool enable_segmented_batch = false;
    bool enable_rtk_vertical_drift = false;
    bool enable_rtk_vertical_lowpass = false;
    bool enable_causal_reference = false;
    bool enable_preoutage_fence = false;
    bool enable_late_static = false;
    bool enable_body_z_nhc = false;
    bool enable_stage2_vehicle_nhc = false;
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
      run_config.enable_rtk_outage_causal_drift_reference,
      run_config.enable_rtk_outage_preoutage_vertical_fence,
      run_config.enable_late_static_detection,
      run_config.enable_body_z_nhc_constraint,
      run_config.enable_stage2_vehicle_nhc_constraint,
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
  ExpectTrue(!calls[1].enable_causal_reference, "Stage3 pass should disable causal RTK drift reference");
  ExpectTrue(!calls[1].enable_preoutage_fence, "Stage3 pass should disable pre-outage vertical fence");
  ExpectTrue(!calls[1].enable_late_static, "Stage3 pass should disable late-static raw RTK height anchors");
  ExpectTrue(!calls[1].enable_body_z_nhc, "Stage3 pass should keep Stage2 policy's Body-Z NHC disabled");
  ExpectTrue(calls[1].enable_stage2_vehicle_nhc, "Stage3 pass should keep Stage2 vehicle NHC enabled");
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

void TestStage3RunnerCanDisableFinalVehicleNHCOnly() {
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
    "Stage3 final pass should disable vehicle NHC when requested");
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

}  // namespace

int main() {
  try {
    RunTest(
      "TestProfilePlannerBuildsFiniteZeroPhaseLowpass",
      TestProfilePlannerBuildsFiniteZeroPhaseLowpass);
    RunTest(
      "TestProfilePlannerCanHoldInitialDynamicStaticReference",
      TestProfilePlannerCanHoldInitialDynamicStaticReference);
    RunTest(
      "TestConstraintBuilderAddsOnlyDynamicAnchorsAndDiagnostics",
      TestConstraintBuilderAddsOnlyDynamicAnchorsAndDiagnostics);
    RunTest(
      "TestConstraintBuilderEnvelopeModeUsesGateAndCenterPull",
      TestConstraintBuilderEnvelopeModeUsesGateAndCenterPull);
    RunTest(
      "TestTimelineAlignerResamplesSegmentedStage2Reference",
      TestTimelineAlignerResamplesSegmentedStage2Reference);
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
      "TestStage3RunnerCanDisableFinalVehicleNHCOnly",
      TestStage3RunnerCanDisableFinalVehicleNHCOnly);
    RunTest(
      "TestStage2LowfreqRunnerRunsRawSourceThenLowpassStage2",
      TestStage2LowfreqRunnerRunsRawSourceThenLowpassStage2);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
