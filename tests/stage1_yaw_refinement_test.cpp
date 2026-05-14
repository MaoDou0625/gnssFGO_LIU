#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkHeadingAlignmentEstimator.h"
#include "offline_lc_minimal/core/Stage1YawRefinementRunner.h"

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
    throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

offline_lc_minimal::GnssSolutionSample MakeRtkFixSample(
  const double time_s,
  const double east_m,
  const double north_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 0.1;
  sample.lon_rad = 0.2;
  sample.h_m = 10.0;
  sample.gnssfgo_type_code = 1;
  sample.enu_position_m = Eigen::Vector3d(east_m, north_m, 0.0);
  sample.has_enu_position = true;
  return sample;
}

std::vector<offline_lc_minimal::GnssSolutionSample> MakeEastboundRtkSamples() {
  std::vector<offline_lc_minimal::GnssSolutionSample> samples;
  for (int index = 0; index <= 8; ++index) {
    const double time_s = 0.5 * static_cast<double>(index);
    samples.push_back(MakeRtkFixSample(time_s, time_s, 0.0));
  }
  return samples;
}

std::vector<offline_lc_minimal::TrajectoryRow> MakeTrajectory(const double yaw_rad) {
  std::vector<offline_lc_minimal::TrajectoryRow> rows;
  for (int index = 0; index <= 8; ++index) {
    offline_lc_minimal::TrajectoryRow row;
    row.time_s = 0.5 * static_cast<double>(index);
    row.ypr_rad = Eigen::Vector3d(yaw_rad, 0.0, 0.0);
    rows.push_back(row);
  }
  return rows;
}

void TestStage1PolicyDisablesSecondStageConstraints() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_yaw_refinement = true;
  config.enable_attitude_reference_constraint = true;
  config.enable_body_z_jump_detection = true;
  config.enable_rtk_velocity_constraint = true;
  config.enable_rtk_outage_smoothing = true;
  config.enable_vertical_velocity_delta_constraint = true;
  config.enable_vertical_motion_adaptive_reweighting = true;
  config.enable_vertical_position_velocity_consistency_all_states = true;
  config.enable_vertical_position_velocity_window_consistency = true;
  config.enable_body_z_nhc_constraint = true;
  config.enable_body_z_nhc_global_weak_constraint = true;
  config.enable_body_z_nhc_strict_effective_weighting = true;
  config.enable_body_z_nhc_horizontal_leakage_correction = true;
  config.enable_vertical_jump_masked_imu = true;
  config.enable_vertical_jump_impulse = true;
  config.enable_vertical_jump_bias = true;
  config.enable_vertical_jump_segmented_bias = true;
  config.enable_vertical_jump_velocity_ramp_smoothing = true;
  config.enable_vertical_jump_position_ramp_smoothing = true;
  config.enable_vertical_jump_velocity_continuity = true;
  config.enable_vertical_jump_velocity_context_mean = true;
  config.enable_vertical_jump_context_mean_continuity = true;
  config.enable_vertical_jump_position_velocity_consistency = true;
  config.enable_vertical_jump_velocity_height_slope_constraint = true;

  const auto stage_config = offline_lc_minimal::MakeStage1YawRefinementConfig(config);
  ExpectTrue(!stage_config.enable_stage1_yaw_refinement, "stage1 recursion flag should be disabled");
  ExpectTrue(stage_config.enable_attitude_reference_constraint, "attitude reference should be preserved");
  ExpectTrue(stage_config.enable_body_z_jump_detection, "body-z jump detection should be preserved");
  ExpectTrue(!stage_config.enable_rtk_velocity_constraint, "RTK velocity should be disabled");
  ExpectTrue(!stage_config.enable_rtk_outage_smoothing, "RTK outage smoothing should be disabled");
  ExpectTrue(!stage_config.enable_vertical_velocity_delta_constraint, "vertical velocity delta should be disabled");
  ExpectTrue(!stage_config.enable_vertical_motion_adaptive_reweighting, "adaptive reweighting should be disabled");
  ExpectTrue(
    !stage_config.enable_vertical_position_velocity_consistency_all_states,
    "all-state position/velocity consistency should be disabled");
  ExpectTrue(
    !stage_config.enable_vertical_position_velocity_window_consistency,
    "window position/velocity consistency should be disabled");
  ExpectTrue(!stage_config.enable_body_z_nhc_constraint, "body-z NHC should be disabled");
  ExpectTrue(!stage_config.enable_body_z_nhc_global_weak_constraint, "global body-z NHC should be disabled");
  ExpectTrue(
    !stage_config.enable_body_z_nhc_horizontal_leakage_correction,
    "body-z horizontal leakage should be disabled");
  ExpectTrue(!stage_config.enable_vertical_jump_masked_imu, "masked jump IMU should be disabled");
  ExpectTrue(!stage_config.enable_vertical_jump_impulse, "jump impulse should be disabled");
  ExpectTrue(!stage_config.enable_vertical_jump_bias, "jump bias should be disabled");
  ExpectTrue(!stage_config.enable_vertical_jump_segmented_bias, "segmented jump bias should be disabled");
  ExpectTrue(
    !stage_config.enable_vertical_jump_position_velocity_consistency,
    "jump position/velocity consistency should be disabled");
}

void TestRtkHeadingAlignmentEstimatorComputesOffset() {
  const auto gnss_samples = MakeEastboundRtkSamples();
  const auto trajectory = MakeTrajectory(0.25);
  const offline_lc_minimal::GeoReference geo_reference(0.1, 0.2, 10.0);

  offline_lc_minimal::RtkHeadingAlignmentRequest request;
  request.gnss_samples = &gnss_samples;
  request.trajectory = &trajectory;
  request.geo_reference = &geo_reference;
  request.options.heading_window_s = 1.0;
  request.options.time_tolerance_s = 0.02;
  request.options.min_displacement_m = 0.2;
  request.options.dynamic_start_time_s = 0.0;
  request.options.end_time_s = 4.0;

  const auto estimate = offline_lc_minimal::RtkHeadingAlignmentEstimator(request).Estimate();
  ExpectTrue(estimate.valid, "heading estimate should be valid");
  ExpectTrue(estimate.valid_pair_count > 0U, "heading estimate should have valid pairs");
  ExpectNear(estimate.median_error_rad, 0.25, 1e-12, "median yaw/course error should match offset");

  request.options.min_displacement_m = 10.0;
  const auto low_displacement =
    offline_lc_minimal::RtkHeadingAlignmentEstimator(request).Estimate();
  ExpectTrue(!low_displacement.valid, "low-displacement samples should be skipped");
}

void TestRtkHeadingAlignmentEstimatorSkipsNonRtkFix() {
  auto gnss_samples = MakeEastboundRtkSamples();
  for (auto &sample : gnss_samples) {
    sample.gnssfgo_type_code = 2;
  }
  const auto trajectory = MakeTrajectory(0.25);
  const offline_lc_minimal::GeoReference geo_reference(0.1, 0.2, 10.0);

  offline_lc_minimal::RtkHeadingAlignmentRequest request;
  request.gnss_samples = &gnss_samples;
  request.trajectory = &trajectory;
  request.geo_reference = &geo_reference;

  const auto estimate = offline_lc_minimal::RtkHeadingAlignmentEstimator(request).Estimate();
  ExpectTrue(!estimate.valid, "non-RTKFIX samples should not produce heading pairs");
}

void TestStage1RunnerUpdatesYawOverrideAndConverges() {
  offline_lc_minimal::DataSet dataset;
  dataset.gnss_samples = MakeEastboundRtkSamples();

  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_yaw_refinement = true;
  config.stage1_yaw_refinement_max_iterations = 10;
  config.stage1_heading_noise_floor_rad = 1.0e-4;
  config.stage1_yaw_update_max_rad = M_PI / 2.0;

  int call_count = 0;
  bool second_call_used_override = false;
  double second_call_yaw_rad = std::numeric_limits<double>::quiet_NaN();
  offline_lc_minimal::Stage1YawRefinementRequest request;
  request.config = config;
  request.dataset = dataset;
  request.run_once = [&](
                       const offline_lc_minimal::OfflineRunnerConfig &run_config,
                       offline_lc_minimal::DataSet) {
    ++call_count;
    if (call_count == 2) {
      second_call_used_override = run_config.enable_initial_yaw_override;
      second_call_yaw_rad = run_config.initial_yaw_override_rad;
    }
    const double yaw_rad = run_config.enable_initial_yaw_override
                             ? run_config.initial_yaw_override_rad
                             : 0.5;

    offline_lc_minimal::OfflineRunResult result;
    result.trajectory = MakeTrajectory(yaw_rad);
    result.run_summary.origin_lat_rad = 0.1;
    result.run_summary.origin_lon_rad = 0.2;
    result.run_summary.origin_h_m = 10.0;
    result.run_summary.dynamic_start_time_s = 0.0;
    result.run_summary.processing_end_time_s = 4.0;
    result.run_summary.final_error = static_cast<double>(call_count);
    result.run_summary.gnss_nis_mean = 0.1 * static_cast<double>(call_count);
    return result;
  };

  const auto result = offline_lc_minimal::Stage1YawRefinementRunner(request).Run();
  ExpectTrue(call_count == 2, "stage1 should stop after the converged second run");
  ExpectTrue(second_call_used_override, "second run should enable yaw override");
  ExpectNear(second_call_yaw_rad, 0.0, 1e-12, "second run yaw override should remove heading offset");
  ExpectTrue(result.run_summary.stage1_yaw_refinement_enabled, "stage1 summary should be enabled");
  ExpectTrue(result.run_summary.stage1_yaw_refinement_converged, "stage1 should converge");
  ExpectTrue(
    result.stage1_yaw_refinement_diagnostics.size() == 2U,
    "stage1 should keep one diagnostic row per optimization run");
  ExpectNear(
    result.stage1_yaw_refinement_diagnostics.front().yaw_update_rad,
    -0.5,
    1e-12,
    "first update should subtract the nav-minus-RTK yaw error");
}

void TestStage1RunnerHonorsMaxIterations() {
  offline_lc_minimal::DataSet dataset;
  dataset.gnss_samples = MakeEastboundRtkSamples();

  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_stage1_yaw_refinement = true;
  config.stage1_yaw_refinement_max_iterations = 1;
  config.stage1_heading_noise_floor_rad = 1.0e-4;

  offline_lc_minimal::Stage1YawRefinementRequest request;
  request.config = config;
  request.dataset = dataset;
  request.run_once = [](
                       const offline_lc_minimal::OfflineRunnerConfig &,
                       offline_lc_minimal::DataSet) {
    offline_lc_minimal::OfflineRunResult result;
    result.trajectory = MakeTrajectory(0.5);
    result.run_summary.origin_lat_rad = 0.1;
    result.run_summary.origin_lon_rad = 0.2;
    result.run_summary.origin_h_m = 10.0;
    result.run_summary.dynamic_start_time_s = 0.0;
    result.run_summary.processing_end_time_s = 4.0;
    return result;
  };

  const auto result = offline_lc_minimal::Stage1YawRefinementRunner(request).Run();
  ExpectTrue(!result.run_summary.stage1_yaw_refinement_converged, "one-run stage1 should not converge");
  ExpectTrue(
    result.run_summary.stage1_yaw_refinement_stop_reason == "max_iterations",
    "stage1 should report max_iterations");
  ExpectTrue(
    result.stage1_yaw_refinement_diagnostics.size() == 1U,
    "stage1 should run only once when max_iterations is one");
}

}  // namespace

int main() {
  try {
    RunTest("TestStage1PolicyDisablesSecondStageConstraints", TestStage1PolicyDisablesSecondStageConstraints);
    RunTest("TestRtkHeadingAlignmentEstimatorComputesOffset", TestRtkHeadingAlignmentEstimatorComputesOffset);
    RunTest("TestRtkHeadingAlignmentEstimatorSkipsNonRtkFix", TestRtkHeadingAlignmentEstimatorSkipsNonRtkFix);
    RunTest("TestStage1RunnerUpdatesYawOverrideAndConverges", TestStage1RunnerUpdatesYawOverrideAndConverges);
    RunTest("TestStage1RunnerHonorsMaxIterations", TestStage1RunnerHonorsMaxIterations);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
