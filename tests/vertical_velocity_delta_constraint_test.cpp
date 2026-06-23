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

#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/core/VerticalAccelBiasGmConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalMotionStabilityEstimator.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaBiasTargetAdjuster.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaContextScalePlanner.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaTargetPlanner.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaBiasFactor.h"
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

void TestVerticalVelocityDeltaBiasFactorUsesBaZ() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const double reference_ba_z_mps2 = 0.01;
  const double dt_s = 0.5;
  const offline_lc_minimal::factor::VerticalVelocityDeltaBiasFactor factor(
    1,
    2,
    3,
    0.10,
    reference_ba_z_mps2,
    dt_s,
    noise);

  gtsam::Matrix h_vi;
  gtsam::Matrix h_vj;
  gtsam::Matrix h_bi;
  const gtsam::imuBias::ConstantBias bias(
    gtsam::Vector3(0.0, 0.0, reference_ba_z_mps2 + 0.02),
    gtsam::Vector3::Zero());
  const gtsam::Vector residual = factor.evaluateError(
    gtsam::Vector3(0.0, 0.0, 1.0),
    gtsam::Vector3(0.0, 0.0, 1.09),
    bias,
    h_vi,
    h_vj,
    h_bi);

  ExpectNear(residual(0), 0.0, 1e-12, "bias-aware residual should account for ba_z correction");
  ExpectNear(h_vi(0, 2), -1.0, 1e-12, "velocity_i z jacobian is wrong");
  ExpectNear(h_vj(0, 2), 1.0, 1e-12, "velocity_j z jacobian is wrong");
  ExpectNear(h_bi(0, 2), dt_s, 1e-12, "ba_z jacobian is wrong");
  ExpectNear(h_bi(0, 0), 0.0, 1e-12, "ba_x jacobian should be zero");
  ExpectNear(h_bi(0, 5), 0.0, 1e-12, "gyro bias jacobian should be zero");
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

offline_lc_minimal::RtkOutageWindowRow MakeRtkOutageWindow(
  const double start_time_s,
  const double end_time_s) {
  offline_lc_minimal::RtkOutageWindowRow window;
  window.start_time_s = start_time_s;
  window.end_time_s = end_time_s;
  return window;
}

offline_lc_minimal::BodyZBiasReestimateSegmentRow MakeBiasSegment(
  const std::string &source_type,
  const double start_time_s,
  const double end_time_s) {
  offline_lc_minimal::BodyZBiasReestimateSegmentRow segment;
  segment.source_type = source_type;
  segment.start_time_s = start_time_s;
  segment.end_time_s = end_time_s;
  return segment;
}

offline_lc_minimal::BodyZBiasReestimateSegmentRow MakeBiasSegment(
  const std::string &source_type,
  const double start_time_s,
  const double end_time_s,
  const double prior_target_ba_z_mps2) {
  auto segment = MakeBiasSegment(source_type, start_time_s, end_time_s);
  segment.prior_target_ba_z_mps2 = prior_target_ba_z_mps2;
  return segment;
}

void TestStaticMotionWindowPlannerMergesStaticSources() {
  offline_lc_minimal::LateStaticWindowRow initial_dynamic_window;
  initial_dynamic_window.valid = true;
  initial_dynamic_window.start_time_s = 13.0;
  initial_dynamic_window.end_time_s = 18.0;

  offline_lc_minimal::LateStaticWindowRow late_window;
  late_window.valid = true;
  late_window.start_time_s = 19.0;
  late_window.end_time_s = 25.0;

  const std::vector<offline_lc_minimal::LateStaticWindowRow> initial_dynamic_windows{
    initial_dynamic_window};
  const std::vector<offline_lc_minimal::LateStaticWindowRow> late_windows{late_window};
  const auto windows =
    offline_lc_minimal::BuildStaticMotionWindows(
      offline_lc_minimal::StaticMotionWindowPlanRequest{
        0.0,
        10.0,
        &initial_dynamic_windows,
        &late_windows,
        2.0});

  ExpectTrue(windows.size() == 2U, "alignment and detected static spans should form two merged windows");
  ExpectNear(windows[0].start_time_s, 0.0, 1e-12, "alignment static start is wrong");
  ExpectNear(windows[0].end_time_s, 10.0, 1e-12, "alignment static end is wrong");
  ExpectNear(windows[1].start_time_s, 13.0, 1e-12, "detected static start is wrong");
  ExpectNear(windows[1].end_time_s, 25.0, 1e-12, "detected static end is wrong");
  ExpectTrue(windows[1].source_window_count == 2U, "detected static source count should merge");
  ExpectTrue(
    offline_lc_minimal::IntervalOverlapsStaticMotionWindow(24.9, 25.1, windows),
    "boundary intervals should overlap the merged static window");
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
  ExpectNear(
    short_dt.sigma_mps,
    0.005,
    1e-12,
    "legacy sigma should directly follow acc_sigma * dt");

  config.vertical_velocity_delta_acc_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(10.0);
  const auto ten_ug = offline_lc_minimal::VerticalVelocityDeltaSigmaModel(config).Compute(0.05);
  config.vertical_velocity_delta_acc_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(1000.0);
  const auto thousand_ug =
    offline_lc_minimal::VerticalVelocityDeltaSigmaModel(config).Compute(0.05);
  ExpectNear(
    thousand_ug.sigma_mps / ten_ug.sigma_mps,
    100.0,
    1e-12,
    "legacy sigma should scale linearly with vertical_velocity_delta_acc_sigma");
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
  config.vertical_velocity_delta_sigma_ceiling_mps = 1.0e-2;

  const offline_lc_minimal::VerticalVelocityDeltaSigmaModel model(config);
  const auto sigma = model.Compute(0.05);
  const double expected_bias_mps = offline_lc_minimal::MicroGToMps2(10.0) * 0.05;
  const double expected_attitude_mps = 9.80665 * 1.0e-4 * 0.05;
  const double expected_sigma_mps = 0.10 * 0.05;

  ExpectTrue(sigma.model == "bias_consistent", "bias-consistent sigma model should be reported");
  ExpectNear(sigma.legacy_sigma_mps, 0.005, 1e-12, "legacy comparison sigma is wrong");
  ExpectNear(sigma.bias_sigma_mps, expected_bias_mps, 1e-15, "bias sigma component is wrong");
  ExpectNear(sigma.attitude_sigma_mps, expected_attitude_mps, 1e-15, "attitude sigma component is wrong");
  ExpectNear(sigma.sigma_mps, expected_sigma_mps, 1e-15, "bias-consistent sigma should follow acc sigma");

  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-12;
  config.vertical_velocity_delta_sigma_ceiling_mps = 1.0;
  config.vertical_velocity_delta_acc_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(10.0);
  const auto ten_ug = offline_lc_minimal::VerticalVelocityDeltaSigmaModel(config).Compute(0.05);
  config.vertical_velocity_delta_acc_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(1000.0);
  const auto thousand_ug =
    offline_lc_minimal::VerticalVelocityDeltaSigmaModel(config).Compute(0.05);
  ExpectNear(
    thousand_ug.sigma_mps / ten_ug.sigma_mps,
    100.0,
    1e-12,
    "bias-consistent sigma should scale linearly with vertical_velocity_delta_acc_sigma");
}

void TestSigmaModelOutputScalePreservesPhysicalComponents() {
  auto base_config = offline_lc_minimal::DefaultConfig();
  base_config.gravity_mps2 = 9.80665;
  base_config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  base_config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  base_config.vertical_velocity_delta_min_sigma_mps = 0.003;
  base_config.vertical_velocity_delta_bias_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.1);
  base_config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  base_config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  base_config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;

  auto scaled_config = base_config;
  scaled_config.vertical_velocity_delta_sigma_scale = 10.0;
  const auto base = offline_lc_minimal::VerticalVelocityDeltaSigmaModel(base_config).Compute(0.05);
  const auto scaled = offline_lc_minimal::VerticalVelocityDeltaSigmaModel(scaled_config).Compute(0.05);

  ExpectNear(scaled.bias_sigma_mps, base.bias_sigma_mps, 1e-15, "output scale should not change bias component");
  ExpectNear(
    scaled.attitude_sigma_mps,
    base.attitude_sigma_mps,
    1e-15,
    "output scale should not change attitude component");
  ExpectNear(scaled.sigma_floor_mps, base.sigma_floor_mps * 10.0, 1e-15, "output scale should scale floor");
  ExpectNear(
    scaled.sigma_ceiling_mps,
    base.sigma_ceiling_mps * 10.0,
    1e-15,
    "output scale should scale ceiling");
  ExpectNear(scaled.sigma_mps, base.sigma_mps * 10.0, 1e-15, "output scale should scale sigma");
}

void TestContextScalePlannerUsesGlobalWhenDisabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_velocity_delta_sigma_scale = 7.0;
  config.enable_vertical_velocity_delta_context_sigma_scale = false;
  const std::vector<offline_lc_minimal::BodyZJumpConstraintWindow> jump_windows;
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> outage_windows{
    MakeRtkOutageWindow(10.0, 20.0),
  };
  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> bias_segments{
    MakeBiasSegment("BODY_Z_BIAS", 10.0, 20.0),
  };

  const offline_lc_minimal::VerticalVelocityDeltaContextScalePlanner planner({
    &config,
    &jump_windows,
    &outage_windows,
    &bias_segments,
  });
  const auto decision = planner.Evaluate(12.0, 12.5);

  ExpectTrue(decision.context == "GLOBAL", "disabled context scaling should report GLOBAL");
  ExpectNear(decision.output_sigma_scale, 7.0, 1e-12, "disabled context scaling should use global scale");
}

void TestContextScalePlannerClassifiesRoughOutageAndJump() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_velocity_delta_context_sigma_scale = true;
  config.vertical_velocity_delta_context_normal_sigma_scale = 100.0;
  config.vertical_velocity_delta_context_rough_sigma_scale = 1000.0;
  config.vertical_velocity_delta_context_outage_sigma_scale = 800.0;
  config.vertical_velocity_delta_context_jump_sigma_scale = 500.0;
  config.vertical_velocity_delta_context_jump_extra_padding_s = 0.5;

  const std::vector<offline_lc_minimal::BodyZJumpConstraintWindow> jump_windows{
    {0U, 1U, 2.0, 2.2},
  };
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> outage_windows{
    MakeRtkOutageWindow(3.0, 3.5),
  };
  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> bias_segments{
    MakeBiasSegment("BODY_Z_BIAS", 4.0, 5.0),
  };
  const offline_lc_minimal::VerticalVelocityDeltaContextScalePlanner planner({
    &config,
    &jump_windows,
    &outage_windows,
    &bias_segments,
  });

  const auto normal = planner.Evaluate(0.0, 0.1);
  ExpectTrue(normal.context == "NORMAL", "non-overlap interval should be normal");
  ExpectNear(normal.output_sigma_scale, 100.0, 1e-12, "normal context scale is wrong");

  const auto jump = planner.Evaluate(1.6, 1.7);
  ExpectTrue(jump.context == "JUMP", "jump interval should be classified");
  ExpectNear(jump.output_sigma_scale, 500.0, 1e-12, "jump context scale is wrong");

  const auto outage = planner.Evaluate(3.2, 3.25);
  ExpectTrue(outage.context == "RTK_OUTAGE", "outage interval should be classified");
  ExpectNear(outage.output_sigma_scale, 800.0, 1e-12, "outage context scale is wrong");

  const auto rough = planner.Evaluate(4.2, 4.25);
  ExpectTrue(rough.context == "ROUGH_BIAS", "rough bias interval should be classified");
  ExpectNear(rough.output_sigma_scale, 1000.0, 1e-12, "rough context scale is wrong");

  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> road_high_noise_segments{
    MakeBiasSegment("ROAD_HIGH_NOISE", 6.0, 7.0),
  };
  const offline_lc_minimal::VerticalVelocityDeltaContextScalePlanner road_high_noise_planner({
    &config,
    &jump_windows,
    &outage_windows,
    &road_high_noise_segments,
  });
  const auto road_high_noise = road_high_noise_planner.Evaluate(6.2, 6.25);
  ExpectTrue(
    road_high_noise.context == "ROUGH_BIAS",
    "ROAD_HIGH_NOISE interval should use rough bias context");
  ExpectNear(
    road_high_noise.output_sigma_scale,
    1000.0,
    1e-12,
    "ROAD_HIGH_NOISE context scale is wrong");
}

void TestBiasTargetAdjusterAppliesReestimateBazBeforeClamp() {
  const double reference_ba_z_mps2 = 0.01;
  const double reestimated_ba_z_mps2 = 0.03;
  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> segments{
    MakeBiasSegment("ROAD_HIGH_NOISE", 0.5, 2.0, reestimated_ba_z_mps2),
  };

  const auto adjusted =
    offline_lc_minimal::AdjustVerticalVelocityDeltaTargetForBiasReestimate({
      1.0,
      1.5,
      0.10,
      reference_ba_z_mps2,
      &segments});

  ExpectTrue(adjusted.applied, "bias reestimate should be applied inside the segment");
  ExpectNear(
    adjusted.target_delta_vz_mps,
    0.09,
    1e-12,
    "target should be corrected by reestimated ba_z before clamping");
  ExpectNear(
    adjusted.reference_ba_z_mps2,
    reestimated_ba_z_mps2,
    1e-12,
    "bias-aware reference should move to the reestimated ba_z");

  const auto outside =
    offline_lc_minimal::AdjustVerticalVelocityDeltaTargetForBiasReestimate({
      2.5,
      3.0,
      0.10,
      reference_ba_z_mps2,
      &segments});
  ExpectTrue(!outside.applied, "bias reestimate should not affect intervals outside the segment");
  ExpectNear(outside.target_delta_vz_mps, 0.10, 1e-12, "outside target should remain unchanged");
  ExpectNear(
    outside.reference_ba_z_mps2,
    reference_ba_z_mps2,
    1e-12,
    "outside reference should remain unchanged");
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

  config.vertical_velocity_delta_acc_sigma_mps2 = 1.0;
  const offline_lc_minimal::VerticalVelocityDeltaSigmaModel ceiling_model(config);
  const auto ceiling_clamped = ceiling_model.Compute(1.0);
  ExpectNear(ceiling_clamped.sigma_mps, 5.0e-4, 1e-15, "ceiling clamp sigma is wrong");
  ExpectTrue(ceiling_clamped.clamped_ceiling, "large acc sigma should clamp to ceiling");
}

void TestAdaptiveSigmaUsesStaticAndDynamicLimits() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.gravity_mps2 = 9.80665;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_velocity_delta_jump_padding_s = 0.0;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  config.vertical_acc_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_motion_adaptive_static_baz_gm_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.02);
  config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.02);
  config.vertical_motion_adaptive_static_attitude_sigma_rad = 1.0e-5;
  config.vertical_motion_adaptive_static_sigma_floor_mps = 2.0e-6;
  config.vertical_motion_adaptive_static_sigma_ceiling_mps = 5.0e-5;

  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow low_motion;
  low_motion.motion_score = 0.0;
  const offline_lc_minimal::VerticalVelocityDeltaSigmaModel model(config);
  const auto adaptive_static = model.Compute(0.05, &low_motion);
  const double expected_static_sigma = config.vertical_motion_adaptive_static_sigma_ceiling_mps;
  ExpectTrue(adaptive_static.model == "adaptive_static", "low motion should use static adaptive model");
  ExpectNear(
    adaptive_static.sigma_mps,
    expected_static_sigma,
    1e-15,
    "static adaptive sigma is wrong");

  low_motion.motion_score = 1.0;
  const auto adaptive_dynamic = model.Compute(0.05, &low_motion);
  const auto base = model.Compute(0.05);
  ExpectNear(
    adaptive_dynamic.sigma_mps,
    base.sigma_mps,
    1e-15,
    "dynamic adaptive sigma should match base sigma");
}

void TestAdaptiveSigmaIntermediateScoreRespectsFloor() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.gravity_mps2 = 9.80665;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-9;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-2;
  config.vertical_velocity_delta_sigma_ceiling_mps = 2.0e-2;
  config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.02);
  config.vertical_motion_adaptive_static_attitude_sigma_rad = 1.0e-9;
  config.vertical_motion_adaptive_static_sigma_floor_mps = 1.0e-4;
  config.vertical_motion_adaptive_static_sigma_ceiling_mps = 1.0e-3;

  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow mixed_motion;
  mixed_motion.motion_score = 0.5;
  const offline_lc_minimal::VerticalVelocityDeltaSigmaModel model(config);
  const auto sigma = model.Compute(0.05, &mixed_motion);
  const double expected_floor = 0.5 * config.vertical_motion_adaptive_static_sigma_floor_mps +
                                0.5 * config.vertical_velocity_delta_sigma_floor_mps;

  ExpectTrue(sigma.model == "adaptive_motion", "mixed score should use adaptive motion model");
  ExpectNear(sigma.sigma_floor_mps, expected_floor, 1e-15, "blended floor is wrong");
  ExpectNear(sigma.sigma_mps, expected_floor, 1e-15, "adaptive sigma should respect blended floor");
  ExpectTrue(sigma.clamped_floor, "mixed adaptive sigma should report floor clamp");
}

void TestAdaptiveGmTransitionKeepsLegacyFloorWithoutProfile() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_acc_bias_tau_s = 100.0;
  config.vertical_acc_bias_process_noise_scale = 1.0;
  config.vertical_acc_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_motion_adaptive_static_baz_gm_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.02);

  const double no_profile_sigma = offline_lc_minimal::VerticalAccelBiasGmConstraintBuilder::TransitionSigmaMps2(
    config,
    0.05,
    false,
    nullptr);
  ExpectNear(no_profile_sigma, 1.0e-6, 1e-15, "pass0/no-profile GM sigma should keep legacy floor");

  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow low_motion;
  low_motion.motion_score = 0.0;
  const double low_motion_sigma = offline_lc_minimal::VerticalAccelBiasGmConstraintBuilder::TransitionSigmaMps2(
    config,
    0.05,
    false,
    &low_motion);
  ExpectTrue(
    low_motion_sigma < no_profile_sigma,
    "low-motion profile should tighten ba_z GM transition sigma");

  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow dynamic_motion;
  dynamic_motion.motion_score = 1.0;
  const double dynamic_sigma = offline_lc_minimal::VerticalAccelBiasGmConstraintBuilder::TransitionSigmaMps2(
    config,
    0.05,
    false,
    &dynamic_motion);
  ExpectNear(dynamic_sigma, no_profile_sigma, 1e-15, "dynamic profile should preserve legacy GM floor");

  low_motion.in_jump_padding = true;
  const double jump_sigma = offline_lc_minimal::VerticalAccelBiasGmConstraintBuilder::TransitionSigmaMps2(
    config,
    0.05,
    false,
    &low_motion);
  ExpectNear(jump_sigma, no_profile_sigma, 1e-15, "jump padding should preserve legacy GM floor");
}

void TestStabilityEstimatorClassifiesLowMotionAndJumpPadding() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_velocity_delta_jump_padding_s = 0.0;
  config.vertical_motion_adaptive_static_horizontal_speed_rms_mps = 0.15;
  config.vertical_motion_adaptive_static_vz_rms_mps = 0.003;
  config.vertical_motion_adaptive_static_target_acc_rms_mps2 = 0.03;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  config.vertical_acc_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_motion_adaptive_static_baz_gm_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.02);

  const std::vector<double> state_timestamps{0.0, 0.05, 0.10, 0.15};
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {0, 1, 0.0, 0.05, 0.0001, 0.0},
    {1, 2, 0.05, 0.10, 0.0001, 0.0},
    {2, 3, 0.10, 0.15, 0.0001, 0.0},
  };
  offline_lc_minimal::BodyZSeedJumpWindowRow jump_window;
  jump_window.start_time_s = 0.10;
  jump_window.end_time_s = 0.15;
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{jump_window};
  gtsam::Values values;
  for (std::size_t i = 0; i < state_timestamps.size(); ++i) {
    values.insert(gtsam::symbol_shorthand::V(i), gtsam::Vector3(0.001, 0.0, 0.0002));
  }

  offline_lc_minimal::VerticalMotionStabilityEstimateRequest request;
  request.config = &config;
  request.state_timestamps = &state_timestamps;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.optimized_values = &values;
  request.outer_pass = 1;
  const auto profile = offline_lc_minimal::VerticalMotionStabilityEstimator(std::move(request)).Estimate();
  ExpectTrue(profile.size() == records.size(), "profile size should match records");
  ExpectTrue(profile[0].motion_score < 0.25, "first interval should be low motion");
  ExpectTrue(profile[0].stability_class == "LOW_MOTION", "first interval class is wrong");
  ExpectNear(profile[0].baz_gm_sigma_before_ug, 0.1, 1e-12, "base ba_z GM sigma diagnostic is wrong");
  ExpectTrue(
    profile[0].baz_gm_sigma_after_ug < 0.03,
    "low-motion ba_z GM sigma should be close to the static adaptive limit");
  ExpectTrue(
    profile[0].baz_gm_sigma_after_ug < profile[0].baz_gm_sigma_before_ug,
    "adaptive ba_z GM sigma should tighten low-motion intervals");
  ExpectTrue(profile[2].in_jump_padding, "jump interval should be marked");
  ExpectNear(profile[2].motion_score, 1.0, 1e-12, "jump interval should keep dynamic score");
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

void TestBuilderUsesUnifiedStaticWindowsForDetectedStaticDvz() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_initial_static_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {10, 11, 100.0, 100.5, 0.01},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  const std::vector<offline_lc_minimal::StaticMotionWindow> static_motion_windows{
    {99.0, 101.0, 1U},
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.static_motion_windows = &static_motion_windows;
  request.dynamic_start_index = 0;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "detected static interval should receive a DVZ factor");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_static_factor_count),
    1.0,
    0.0,
    "detected static interval should be counted as static DVZ");
  ExpectTrue(diagnostics.front().factor_added, "detected static interval should be added");

  config.enable_vertical_velocity_delta_initial_static_constraint = false;
  gtsam::NonlinearFactorGraph skipped_graph;
  offline_lc_minimal::RunSummary skipped_summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> skipped_diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuildRequest skipped_request;
  skipped_request.config = &config;
  skipped_request.propagation_records = &records;
  skipped_request.jump_windows = &jump_windows;
  skipped_request.static_motion_windows = &static_motion_windows;
  skipped_request.dynamic_start_index = 0;
  skipped_request.graph = &skipped_graph;
  skipped_request.run_summary = &skipped_summary;
  skipped_request.diagnostics = &skipped_diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(skipped_request)).Build();

  ExpectNear(static_cast<double>(skipped_graph.size()), 0.0, 0.0, "disabled static DVZ should skip detected static intervals");
  ExpectNear(
    static_cast<double>(skipped_summary.vertical_velocity_delta_skipped_static_count),
    1.0,
    0.0,
    "detected static interval should use the static skip counter");
  ExpectTrue(
    skipped_diagnostics.front().skip_reason == "STATIC_INTERIOR",
    "detected static interval should use static skip reason");
}

void TestBuilderUsesAdaptiveMotionProfileSigma() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.02);
  config.vertical_motion_adaptive_static_attitude_sigma_rad = 1.0e-5;
  config.vertical_motion_adaptive_static_sigma_floor_mps = 2.0e-6;
  config.vertical_motion_adaptive_static_sigma_ceiling_mps = 5.0e-5;

  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.0, 1.05, 0.0001, 0.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow profile_row;
  profile_row.state_index_i = 1;
  profile_row.state_index_j = 2;
  profile_row.motion_score = 0.0;
  profile_row.dvz_sigma_before_mps =
    offline_lc_minimal::VerticalVelocityDeltaSigmaModel(config).Compute(0.05).sigma_mps;
  const offline_lc_minimal::VerticalMotionStabilityProfile profile{profile_row};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.stability_profile = &profile;
  request.dynamic_start_index = 1;
  request.outer_pass = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "adaptive interval should add a factor");
  ExpectTrue(diagnostics.front().sigma_model == "adaptive_static", "builder should use adaptive static sigma");
  ExpectNear(diagnostics.front().outer_pass, 1.0, 0.0, "diagnostic should record outer pass");
  ExpectNear(diagnostics.front().adaptive_motion_score, 0.0, 1e-12, "motion score should be copied");
  ExpectTrue(
    diagnostics.front().sigma_mps < diagnostics.front().legacy_sigma_mps,
    "adaptive sigma should be tighter than legacy sigma");
  ExpectNear(
    diagnostics.front().adaptive_sigma_mps,
    diagnostics.front().sigma_mps,
    1e-15,
    "adaptive sigma diagnostic should match factor sigma");
}

void TestLowSpeedVerticalHandoffClampsTargetDelta() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 0.85;
  config.vertical_motion_adaptive_static_horizontal_speed_rms_mps = 0.15;
  config.vertical_motion_adaptive_static_target_acc_rms_mps2 = 0.03;

  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow low_speed_handoff;
  low_speed_handoff.horizontal_speed_rms_mps = 0.10;
  low_speed_handoff.target_vertical_acc_rms_mps2 = 0.02;
  low_speed_handoff.motion_score = 1.0;

  const double target = offline_lc_minimal::PlanVerticalVelocityDeltaTarget(
    config,
    0.008491646647841404,
    0.05,
    &low_speed_handoff);

  ExpectNear(target, 0.0015, 1e-12, "low-speed handoff should use the static target acceleration limit");

  low_speed_handoff.horizontal_speed_rms_mps = 0.20;
  const double dynamic_target = offline_lc_minimal::PlanVerticalVelocityDeltaTarget(
    config,
    0.008491646647841404,
    0.05,
    &low_speed_handoff);
  ExpectNear(
    dynamic_target,
    0.008491646647841404,
    1e-15,
    "dynamic intervals should preserve targets within the global acceleration limit");

  low_speed_handoff.horizontal_speed_rms_mps = 0.10;
  config.enable_vertical_motion_adaptive_reweighting = false;
  const double disabled_target = offline_lc_minimal::PlanVerticalVelocityDeltaTarget(
    config,
    0.008491646647841404,
    0.05,
    &low_speed_handoff);
  ExpectNear(
    disabled_target,
    0.008491646647841404,
    1e-15,
    "disabled adaptive reweighting should preserve targets within the global acceleration limit");

  config.enable_vertical_motion_adaptive_reweighting = true;
  const double no_profile_target = offline_lc_minimal::PlanVerticalVelocityDeltaTarget(
    config,
    0.008491646647841404,
    0.05,
    nullptr);
  ExpectNear(
    no_profile_target,
    0.008491646647841404,
    1e-15,
    "missing stability profile should preserve targets within the global acceleration limit");

  low_speed_handoff.in_jump_padding = true;
  const double jump_padding_target = offline_lc_minimal::PlanVerticalVelocityDeltaTarget(
    config,
    0.008491646647841404,
    0.05,
    &low_speed_handoff);
  ExpectNear(
    jump_padding_target,
    0.008491646647841404,
    1e-15,
    "jump padding should preserve targets within the global acceleration limit");
}

void TestBuilderClampsLowSpeedVerticalHandoffTarget() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 0.85;
  config.vertical_motion_adaptive_static_horizontal_speed_rms_mps = 0.15;
  config.vertical_motion_adaptive_static_target_acc_rms_mps2 = 0.03;

  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 331.85, 331.90, 0.008491646647841404, 0.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow profile_row;
  profile_row.state_index_i = 1;
  profile_row.state_index_j = 2;
  profile_row.motion_score = 1.0;
  profile_row.horizontal_speed_rms_mps = 0.10;
  profile_row.target_vertical_acc_rms_mps2 = 0.02;
  profile_row.dvz_sigma_before_mps =
    offline_lc_minimal::VerticalVelocityDeltaSigmaModel(config).Compute(0.05).sigma_mps;
  const offline_lc_minimal::VerticalMotionStabilityProfile profile{profile_row};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.stability_profile = &profile;
  request.dynamic_start_index = 1;
  request.outer_pass = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "low-speed handoff interval should add a factor");
  ExpectTrue(diagnostics.front().factor_added, "low-speed handoff diagnostic should mark factor added");
  ExpectNear(
    diagnostics.front().raw_target_delta_vz_mps,
    0.008491646647841404,
    1e-15,
    "raw target should be preserved");
  ExpectNear(
    diagnostics.front().target_delta_vz_mps,
    0.0015,
    1e-12,
    "low-speed handoff target should be clamped");
  ExpectTrue(diagnostics.front().target_clamped, "low-speed handoff clamp should be reported");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_target_clamped_count),
    1.0,
    0.0,
    "low-speed handoff clamp count is wrong");
}

void TestBuilderUsesContextScaleForRoughBiasIntervals() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.enable_vertical_velocity_delta_context_sigma_scale = true;
  config.vertical_velocity_delta_context_normal_sigma_scale = 2.0;
  config.vertical_velocity_delta_context_rough_sigma_scale = 10.0;
  config.vertical_velocity_delta_context_outage_sigma_scale = 3.0;
  config.vertical_velocity_delta_context_jump_sigma_scale = 4.0;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 10.0, 10.5, 0.01, 0.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> bias_segments{
    MakeBiasSegment("BODY_Z_BIAS", 9.0, 11.0),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.bias_reestimate_segments = &bias_segments;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "rough interval should add a factor");
  ExpectTrue(diagnostics.front().sigma_context == "ROUGH_BIAS", "rough interval context is wrong");
  ExpectNear(diagnostics.front().sigma_output_scale, 10.0, 1e-12, "rough interval output scale is wrong");
  ExpectNear(diagnostics.front().sigma_mps, 2.5, 1e-12, "rough interval sigma should use context scale");
}

void TestBuilderUsesContextScaleForJumpAdjacentRing() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_jump_padding_s = 0.1;
  config.enable_vertical_velocity_delta_context_sigma_scale = true;
  config.vertical_velocity_delta_context_normal_sigma_scale = 2.0;
  config.vertical_velocity_delta_context_rough_sigma_scale = 10.0;
  config.vertical_velocity_delta_context_outage_sigma_scale = 3.0;
  config.vertical_velocity_delta_context_jump_sigma_scale = 4.0;
  config.vertical_velocity_delta_context_jump_extra_padding_s = 0.5;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.6, 1.7, 0.01, 0.0},
  };
  offline_lc_minimal::BodyZSeedJumpWindowRow window;
  window.start_time_s = 2.0;
  window.end_time_s = 2.2;
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{window};
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

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "jump-adjacent interval should add a factor");
  ExpectTrue(diagnostics.front().sigma_context == "JUMP", "jump-adjacent interval context is wrong");
  ExpectTrue(!diagnostics.front().in_jump_padding, "jump-adjacent interval should not be hard-skipped padding");
  ExpectNear(diagnostics.front().sigma_output_scale, 4.0, 1e-12, "jump context output scale is wrong");
  ExpectNear(diagnostics.front().sigma_mps, 0.2, 1e-12, "jump context sigma should use extra-ring scale");
}

void TestBuilderKeepsLegacySigmaForClampedAdaptiveTarget() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 0.1;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(0.1);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  config.vertical_motion_adaptive_static_dvz_bias_sigma_mps2 =
    offline_lc_minimal::MicroGToMps2(0.02);
  config.vertical_motion_adaptive_static_attitude_sigma_rad = 1.0e-5;
  config.vertical_motion_adaptive_static_sigma_floor_mps = 2.0e-6;
  config.vertical_motion_adaptive_static_sigma_ceiling_mps = 5.0e-5;

  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.0, 1.5, 1.0, 0.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  offline_lc_minimal::VerticalMotionAdaptiveReweightingDiagnosticRow profile_row;
  profile_row.state_index_i = 1;
  profile_row.state_index_j = 2;
  profile_row.motion_score = 0.0;
  profile_row.dvz_sigma_before_mps =
    offline_lc_minimal::VerticalVelocityDeltaSigmaModel(config).Compute(0.5).sigma_mps;
  const offline_lc_minimal::VerticalMotionStabilityProfile profile{profile_row};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.stability_profile = &profile;
  request.dynamic_start_index = 1;
  request.outer_pass = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "clamped adaptive interval should add a factor");
  ExpectTrue(diagnostics.front().target_clamped, "adaptive target should be clamped");
  ExpectTrue(
    diagnostics.front().sigma_model == "legacy_clamped_target",
    "clamped adaptive target should fall back to legacy sigma");
  ExpectNear(diagnostics.front().target_delta_vz_mps, 0.05, 1e-12, "clamped target value is wrong");
  ExpectNear(diagnostics.front().sigma_mps, 0.25, 1e-12, "clamped adaptive target should use legacy sigma");
}

void TestBuilderUsesContextScaleForClampedTargetFallback() {
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
  config.enable_vertical_velocity_delta_context_sigma_scale = true;
  config.vertical_velocity_delta_context_normal_sigma_scale = 2.0;
  config.vertical_velocity_delta_context_rough_sigma_scale = 10.0;
  config.vertical_velocity_delta_context_outage_sigma_scale = 3.0;
  config.vertical_velocity_delta_context_jump_sigma_scale = 4.0;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 10.0, 10.5, 1.0, 0.0},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> bias_segments{
    MakeBiasSegment("BODY_Z_BIAS", 9.0, 11.0),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.bias_reestimate_segments = &bias_segments;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "clamped rough interval should add a factor");
  ExpectTrue(diagnostics.front().target_clamped, "target should be clamped");
  ExpectTrue(
    diagnostics.front().sigma_model == "legacy_clamped_target",
    "clamped target should use legacy fallback model");
  ExpectTrue(diagnostics.front().sigma_context == "ROUGH_BIAS", "rough context should be preserved");
  ExpectNear(diagnostics.front().sigma_output_scale, 10.0, 1e-12, "rough output scale is wrong");
  ExpectNear(diagnostics.front().legacy_sigma_mps, 0.25, 1e-12, "legacy sigma is wrong");
  ExpectNear(diagnostics.front().sigma_mps, 2.5, 1e-12, "fallback sigma should use context scale");
}

void TestBuilderAddsStaticInteriorWhenConfigured() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_initial_static_constraint = true;
  config.enable_vertical_velocity_delta_bias_consistent_sigma = true;
  config.enable_vertical_velocity_delta_bias_aware_target = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.10;
  config.vertical_velocity_delta_min_sigma_mps = 0.003;
  config.vertical_velocity_delta_bias_sigma_mps2 = offline_lc_minimal::MicroGToMps2(10.0);
  config.vertical_velocity_delta_attitude_sigma_rad = 1.0e-4;
  config.vertical_velocity_delta_sigma_floor_mps = 1.0e-5;
  config.vertical_velocity_delta_sigma_ceiling_mps = 5.0e-4;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {0, 1, 0.0, 0.5, 0.01, 0.001},
    {1, 2, 0.5, 1.0, 0.02, 0.002},
    {2, 3, 1.0, 1.5, 0.03, 0.003},
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

  ExpectNear(static_cast<double>(graph.size()), 3.0, 0.0, "all intervals should receive factors");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_factor_count), 3.0, 0.0, "factor count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_static_factor_count),
    1.0,
    0.0,
    "static factor count is wrong");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_skipped_static_count),
    0.0,
    0.0,
    "static skip count is wrong");
  ExpectTrue(diagnostics[0].factor_added, "static interior interval should receive a factor");
  ExpectTrue(diagnostics[0].skip_reason == "ADDED", "static interior interval should be marked as added");
  ExpectTrue(diagnostics[0].sigma_model == "bias_consistent", "static interval should use bias-consistent sigma");
  ExpectTrue(diagnostics[0].bias_aware_factor, "static interval should use bias-aware dvz target");
  const double expected_sigma_mps = config.vertical_velocity_delta_sigma_ceiling_mps;
  ExpectNear(diagnostics[0].sigma_mps, expected_sigma_mps, 1e-15, "static interval sigma is wrong");
  ExpectNear(
    static_cast<double>(graph.at(0)->keys().size()),
    3.0,
    0.0,
    "bias-aware static factor should connect V_i, V_j, and B_i");
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

void TestBuilderCanKeepDvzInsideJumpPaddingWithWideSigma() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_jump_padding_s = 0.10;
  config.vertical_velocity_delta_skip_jump_padding = false;
  config.enable_vertical_velocity_delta_context_sigma_scale = true;
  config.vertical_velocity_delta_context_normal_sigma_scale = 2.0;
  config.vertical_velocity_delta_context_jump_sigma_scale = 4.0;
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

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "jump interval should receive a DVZ factor");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_factor_count), 1.0, 0.0, "factor count is wrong");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_jump_count), 0.0, 0.0, "jump skip count is wrong");
  ExpectTrue(diagnostics.front().factor_added, "jump interval diagnostic should mark factor added");
  ExpectTrue(diagnostics.front().in_jump_padding, "jump interval should still be marked as jump padding");
  ExpectTrue(diagnostics.front().skip_reason == "ADDED", "jump interval should be added when skip is disabled");
  ExpectTrue(diagnostics.front().sigma_context == "JUMP", "jump interval should use jump context");
  ExpectNear(diagnostics.front().sigma_output_scale, 4.0, 1e-15, "jump context scale is wrong");
  ExpectNear(diagnostics.front().sigma_mps, 1.0, 1e-12, "jump sigma should be widened by context scale");
}

void TestBuilderUsesNHCJumpWindowsWhenNHCEnabled() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_body_z_nhc_constraint = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_jump_padding_s = 0.0;
  config.body_z_nhc_jump_padding_s = 0.10;
  config.body_z_nhc_merge_gap_s = 0.50;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.35, 1.45, 0.02},
  };
  offline_lc_minimal::BodyZSeedJumpWindowRow first_window;
  first_window.start_time_s = 1.0;
  first_window.end_time_s = 1.1;
  offline_lc_minimal::BodyZSeedJumpWindowRow second_window;
  second_window.start_time_s = 1.7;
  second_window.end_time_s = 1.8;
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows{
    first_window,
    second_window,
  };
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

  ExpectTrue(graph.empty(), "dvz should skip intervals inside merged NHC jump windows");
  ExpectNear(static_cast<double>(summary.vertical_velocity_delta_skipped_jump_count), 1.0, 0.0, "jump skip count is wrong");
  ExpectTrue(diagnostics.front().in_jump_padding, "diagnostic should mark merged NHC window overlap");
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

  const double expected_sigma_mps = config.vertical_velocity_delta_sigma_ceiling_mps;
  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "bias-consistent interval should add a factor");
  ExpectTrue(summary.vertical_velocity_delta_bias_consistent_sigma_enabled, "summary should report sigma mode");
  ExpectNear(diagnostics.front().sigma_mps, expected_sigma_mps, 1e-15, "builder should use direct acc sigma");
  ExpectNear(diagnostics.front().legacy_sigma_mps, 0.005, 1e-12, "builder should preserve legacy diagnostic");
  ExpectTrue(diagnostics.front().sigma_model == "bias_consistent", "diagnostic sigma model is wrong");
  ExpectNear(summary.vertical_velocity_delta_sigma_mean_mps, expected_sigma_mps, 1e-15, "summary mean sigma is wrong");
  ExpectNear(summary.vertical_velocity_delta_sigma_max_mps, expected_sigma_mps, 1e-15, "summary max sigma is wrong");
}

void TestBuilderUsesBiasAwareVelocityDeltaFactor() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_bias_aware_target = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  const double reference_ba_z_mps2 = offline_lc_minimal::MicroGToMps2(-500.0);
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.0, 1.5, 0.10, reference_ba_z_mps2},
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

  const auto &keys = graph.at(0)->keys();
  ExpectNear(static_cast<double>(keys.size()), 3.0, 0.0, "bias-aware factor should connect V_i, V_j, and B_i");
  ExpectTrue(keys[0] == gtsam::symbol_shorthand::V(1), "bias-aware factor first key should be V_i");
  ExpectTrue(keys[1] == gtsam::symbol_shorthand::V(2), "bias-aware factor second key should be V_j");
  ExpectTrue(keys[2] == gtsam::symbol_shorthand::B(1), "bias-aware factor third key should be B_i");

  gtsam::Values values;
  values.insert(gtsam::symbol_shorthand::V(1), gtsam::Vector3(0.0, 0.0, 1.0));
  values.insert(gtsam::symbol_shorthand::V(2), gtsam::Vector3(0.0, 0.0, 1.09));
  values.insert(
    gtsam::symbol_shorthand::B(1),
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.0, 0.0, reference_ba_z_mps2 + 0.02),
      gtsam::Vector3::Zero()));
  offline_lc_minimal::PopulateVerticalVelocityDeltaDiagnostics(values, diagnostics);

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "bias-aware interval should add a factor");
  ExpectTrue(summary.vertical_velocity_delta_bias_aware_target_enabled, "summary should report bias-aware mode");
  ExpectNear(
    static_cast<double>(summary.vertical_velocity_delta_bias_aware_factor_count),
    1.0,
    0.0,
    "bias-aware factor count is wrong");
  ExpectTrue(diagnostics.front().bias_aware_factor, "diagnostic should mark bias-aware factor");
  ExpectNear(diagnostics.front().bias_delta_ug, offline_lc_minimal::Mps2ToMicroG(0.02), 1e-9, "bias delta is wrong");
  ExpectNear(
    diagnostics.front().bias_delta_velocity_correction_mps,
    0.01,
    1e-12,
    "bias delta velocity correction is wrong");
  ExpectNear(diagnostics.front().residual_mps, 0.0, 1e-12, "bias-aware diagnostic residual is wrong");
}

void TestBuilderAppliesBiasReestimateBeforeDvzTargetClamp() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_body_z_jump_detection = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_velocity_delta_bias_aware_target = true;
  config.vertical_velocity_delta_acc_sigma_mps2 = 0.5;
  config.vertical_velocity_delta_min_sigma_mps = 0.02;
  config.vertical_velocity_delta_target_acc_limit_mps2 = 1.0;
  const double reference_ba_z_mps2 = 0.01;
  const double reestimated_ba_z_mps2 = 0.03;
  const std::vector<offline_lc_minimal::VerticalVelocityDeltaPropagationRecord> records{
    {1, 2, 1.0, 1.5, 0.10, reference_ba_z_mps2},
  };
  const std::vector<offline_lc_minimal::BodyZSeedJumpWindowRow> jump_windows;
  const std::vector<offline_lc_minimal::BodyZBiasReestimateSegmentRow> bias_segments{
    MakeBiasSegment("ROAD_HIGH_NOISE", 0.5, 2.0, reestimated_ba_z_mps2),
  };
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::VerticalVelocityDeltaDiagnosticRow> diagnostics;

  offline_lc_minimal::VerticalMotionConstraintBuildRequest request;
  request.config = &config;
  request.propagation_records = &records;
  request.jump_windows = &jump_windows;
  request.bias_reestimate_segments = &bias_segments;
  request.dynamic_start_index = 1;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  offline_lc_minimal::VerticalMotionConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "reestimated interval should receive a factor");
  ExpectNear(
    diagnostics.front().raw_target_delta_vz_mps,
    0.09,
    1e-12,
    "raw diagnostic target should already include reestimated ba_z correction");
  ExpectNear(
    diagnostics.front().target_delta_vz_mps,
    0.09,
    1e-12,
    "clamped target should be based on the reestimated ba_z correction");
  ExpectNear(
    diagnostics.front().reference_ba_z_ug,
    offline_lc_minimal::Mps2ToMicroG(reestimated_ba_z_mps2),
    1e-9,
    "diagnostic reference ba_z should use the reestimated target");

  gtsam::Values values;
  values.insert(gtsam::symbol_shorthand::V(1), gtsam::Vector3(0.0, 0.0, 1.0));
  values.insert(gtsam::symbol_shorthand::V(2), gtsam::Vector3(0.0, 0.0, 1.09));
  values.insert(
    gtsam::symbol_shorthand::B(1),
    gtsam::imuBias::ConstantBias(
      gtsam::Vector3(0.0, 0.0, reestimated_ba_z_mps2),
      gtsam::Vector3::Zero()));
  offline_lc_minimal::PopulateVerticalVelocityDeltaDiagnostics(values, diagnostics);

  ExpectNear(diagnostics.front().bias_delta_velocity_correction_mps, 0.0, 1e-12, "applied bias should not be counted twice");
  ExpectNear(diagnostics.front().residual_mps, 0.0, 1e-12, "reestimated target should close with matching ba_z");
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
    RunTest("TestVerticalVelocityDeltaBiasFactorUsesBaZ", TestVerticalVelocityDeltaBiasFactorUsesBaZ);
    RunTest("TestSigmaModelLegacyMatchesExistingFormula", TestSigmaModelLegacyMatchesExistingFormula);
    RunTest("TestSigmaModelBiasConsistentComponents", TestSigmaModelBiasConsistentComponents);
    RunTest(
      "TestSigmaModelOutputScalePreservesPhysicalComponents",
      TestSigmaModelOutputScalePreservesPhysicalComponents);
    RunTest(
      "TestContextScalePlannerUsesGlobalWhenDisabled",
      TestContextScalePlannerUsesGlobalWhenDisabled);
    RunTest(
      "TestContextScalePlannerClassifiesRoughOutageAndJump",
      TestContextScalePlannerClassifiesRoughOutageAndJump);
    RunTest(
      "TestBiasTargetAdjusterAppliesReestimateBazBeforeClamp",
      TestBiasTargetAdjusterAppliesReestimateBazBeforeClamp);
    RunTest("TestSigmaModelClampFlags", TestSigmaModelClampFlags);
    RunTest("TestAdaptiveSigmaUsesStaticAndDynamicLimits", TestAdaptiveSigmaUsesStaticAndDynamicLimits);
    RunTest("TestAdaptiveSigmaIntermediateScoreRespectsFloor", TestAdaptiveSigmaIntermediateScoreRespectsFloor);
    RunTest(
      "TestAdaptiveGmTransitionKeepsLegacyFloorWithoutProfile",
      TestAdaptiveGmTransitionKeepsLegacyFloorWithoutProfile);
    RunTest(
      "TestStabilityEstimatorClassifiesLowMotionAndJumpPadding",
      TestStabilityEstimatorClassifiesLowMotionAndJumpPadding);
    RunTest(
      "TestBuilderAddsDynamicBoundaryAndDynamicNonJumpIntervals",
      TestBuilderAddsDynamicBoundaryAndDynamicNonJumpIntervals);
    RunTest(
      "TestStaticMotionWindowPlannerMergesStaticSources",
      TestStaticMotionWindowPlannerMergesStaticSources);
    RunTest(
      "TestBuilderSkipsStaticInteriorButAddsStaticDynamicBoundary",
      TestBuilderSkipsStaticInteriorButAddsStaticDynamicBoundary);
    RunTest(
      "TestBuilderUsesUnifiedStaticWindowsForDetectedStaticDvz",
      TestBuilderUsesUnifiedStaticWindowsForDetectedStaticDvz);
    RunTest("TestBuilderUsesAdaptiveMotionProfileSigma", TestBuilderUsesAdaptiveMotionProfileSigma);
    RunTest(
      "TestLowSpeedVerticalHandoffClampsTargetDelta",
      TestLowSpeedVerticalHandoffClampsTargetDelta);
    RunTest(
      "TestBuilderClampsLowSpeedVerticalHandoffTarget",
      TestBuilderClampsLowSpeedVerticalHandoffTarget);
    RunTest(
      "TestBuilderUsesContextScaleForRoughBiasIntervals",
      TestBuilderUsesContextScaleForRoughBiasIntervals);
    RunTest(
      "TestBuilderUsesContextScaleForJumpAdjacentRing",
      TestBuilderUsesContextScaleForJumpAdjacentRing);
    RunTest(
      "TestBuilderKeepsLegacySigmaForClampedAdaptiveTarget",
      TestBuilderKeepsLegacySigmaForClampedAdaptiveTarget);
    RunTest(
      "TestBuilderUsesContextScaleForClampedTargetFallback",
      TestBuilderUsesContextScaleForClampedTargetFallback);
    RunTest("TestBuilderAddsStaticInteriorWhenConfigured", TestBuilderAddsStaticInteriorWhenConfigured);
    RunTest(
      "TestBuilderSkipsStaticDynamicBoundaryInsideJumpPadding",
      TestBuilderSkipsStaticDynamicBoundaryInsideJumpPadding);
    RunTest(
      "TestBuilderCanKeepDvzInsideJumpPaddingWithWideSigma",
      TestBuilderCanKeepDvzInsideJumpPaddingWithWideSigma);
    RunTest(
      "TestBuilderUsesNHCJumpWindowsWhenNHCEnabled",
      TestBuilderUsesNHCJumpWindowsWhenNHCEnabled);
    RunTest("TestBuilderDisabledDoesNotMutateGraph", TestBuilderDisabledDoesNotMutateGraph);
    RunTest("TestBuilderSkipsIntervalsAfterGnssSupport", TestBuilderSkipsIntervalsAfterGnssSupport);
    RunTest("TestBuilderClampsVelocityDeltaTarget", TestBuilderClampsVelocityDeltaTarget);
    RunTest("TestBuilderUsesBiasConsistentSigmaAndSummary", TestBuilderUsesBiasConsistentSigmaAndSummary);
    RunTest("TestBuilderUsesBiasAwareVelocityDeltaFactor", TestBuilderUsesBiasAwareVelocityDeltaFactor);
    RunTest(
      "TestBuilderAppliesBiasReestimateBeforeDvzTargetClamp",
      TestBuilderAppliesBiasReestimateBeforeDvzTargetClamp);
    RunTest(
      "TestBuilderUsesLegacySigmaForClampedTargetInBiasConsistentMode",
      TestBuilderUsesLegacySigmaForClampedTargetInBiasConsistentMode);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
