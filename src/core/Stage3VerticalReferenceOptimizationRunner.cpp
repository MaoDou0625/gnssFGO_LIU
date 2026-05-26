#include "offline_lc_minimal/core/Stage3VerticalReferenceOptimizationRunner.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"

namespace offline_lc_minimal {
namespace {

OfflineRunnerConfig MakeStage2SourceConfig(OfflineRunnerConfig config) {
  config.enable_stage3_vertical_reference_optimization = false;
  config.stage3_disable_stage2_vehicle_nhc_constraint = false;
  return config;
}

OfflineRunnerConfig MakeStage3Config(OfflineRunnerConfig config) {
  config = MakeStage2VelocityOptimizationConfig(config);
  config.enable_stage3_vertical_reference_optimization = false;
  config.enable_rtk_outage_segmented_batch = false;
  config.enable_rtk_vertical_drift_reference = false;
  config.enable_rtk_vertical_lowpass_reference = false;
  config.enable_rtk_outage_causal_drift_reference = false;
  config.enable_rtk_outage_preoutage_vertical_fence = false;
  config.enable_late_static_detection = false;
  if (config.stage3_disable_stage2_vehicle_nhc_constraint) {
    config.enable_stage2_vehicle_nhc_constraint = false;
  }
  config.stage3_disable_stage2_vehicle_nhc_constraint = false;
  return config;
}

std::shared_ptr<Stage2VelocityReference> MakeStage2ReferenceFromResult(
  const OfflineRunResult &stage2_result,
  const OfflineRunnerConfig &source_config) {
  if (stage2_result.trajectory.empty()) {
    throw std::runtime_error("Stage3 received an empty Stage2 trajectory");
  }
  auto reference = std::make_shared<Stage2VelocityReference>();
  reference->trajectory = stage2_result.trajectory;
  reference->source_config =
    std::make_shared<OfflineRunnerConfig>(source_config);
  return reference;
}

void CopySourceDiagnostics(
  const OfflineRunResult &stage2_result,
  OfflineRunResult &stage3_result) {
  stage3_result.stage1_yaw_refinement_diagnostics =
    stage2_result.stage1_yaw_refinement_diagnostics;
  stage3_result.stage1_outage_body_y_envelopes =
    stage2_result.stage1_outage_body_y_envelopes;
  stage3_result.stage1_outage_body_y_state_diagnostics =
    stage2_result.stage1_outage_body_y_state_diagnostics;
  stage3_result.run_summary.stage1_yaw_refinement_enabled =
    stage2_result.run_summary.stage1_yaw_refinement_enabled;
  stage3_result.run_summary.stage1_yaw_refinement_iteration_count =
    stage2_result.run_summary.stage1_yaw_refinement_iteration_count;
  stage3_result.run_summary.stage1_yaw_refinement_converged =
    stage2_result.run_summary.stage1_yaw_refinement_converged;
  stage3_result.run_summary.stage1_yaw_refinement_stop_reason =
    stage2_result.run_summary.stage1_yaw_refinement_stop_reason;
  stage3_result.run_summary.stage1_yaw_refinement_final_yaw_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_yaw_rad;
  stage3_result.run_summary.stage1_yaw_refinement_final_median_error_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_median_error_rad;
  stage3_result.run_summary.stage1_yaw_refinement_final_noise_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_noise_rad;
  stage3_result.run_summary.stage1_yaw_refinement_final_update_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_update_rad;
  stage3_result.run_summary.stage1_outage_body_y_envelope_enabled =
    stage2_result.run_summary.stage1_outage_body_y_envelope_enabled;
  stage3_result.run_summary.stage1_outage_body_y_envelope_count =
    stage2_result.run_summary.stage1_outage_body_y_envelope_count;
  stage3_result.run_summary.stage1_outage_body_y_envelope_valid_count =
    stage2_result.run_summary.stage1_outage_body_y_envelope_valid_count;
  stage3_result.run_summary.stage1_outage_body_y_velocity_factor_count =
    stage2_result.run_summary.stage1_outage_body_y_velocity_factor_count;
  stage3_result.run_summary.stage1_outage_body_y_mean_mps =
    stage2_result.run_summary.stage1_outage_body_y_mean_mps;
  stage3_result.run_summary.stage1_outage_body_y_rmse_mps =
    stage2_result.run_summary.stage1_outage_body_y_rmse_mps;
  stage3_result.run_summary.stage1_outage_body_y_deadband_mps =
    stage2_result.run_summary.stage1_outage_body_y_deadband_mps;
}

}  // namespace

Stage3VerticalReferenceOptimizationRunner::Stage3VerticalReferenceOptimizationRunner(
  Stage3VerticalReferenceOptimizationRequest request)
    : request_(std::move(request)) {}

OfflineRunResult Stage3VerticalReferenceOptimizationRunner::Run() const {
  if (!request_.run_once) {
    throw std::runtime_error("Stage3 vertical reference optimization requires a run_once callback");
  }

  OfflineRunnerConfig stage2_config = MakeStage2SourceConfig(request_.config);
  OfflineRunResult stage2_result =
    request_.run_once(stage2_config, nullptr, nullptr, request_.dataset);
  auto stage2_reference =
    MakeStage2ReferenceFromResult(stage2_result, request_.config);

  Stage3VerticalReferenceProfilePlanRequest plan_request;
  plan_request.config = &request_.config;
  plan_request.stage2_trajectory = &stage2_result.trajectory;
  auto stage3_reference =
    std::make_shared<Stage3VerticalReference>(
      Stage3VerticalReferenceProfilePlanner(std::move(plan_request)).Plan());

  OfflineRunnerConfig stage3_config = MakeStage3Config(request_.config);
  OfflineRunResult stage3_result =
    request_.run_once(
      stage3_config,
      std::move(stage2_reference),
      std::move(stage3_reference),
      request_.dataset);
  CopySourceDiagnostics(stage2_result, stage3_result);
  stage3_result.run_summary.stage3_vertical_reference_optimization_enabled = true;
  stage3_result.run_summary.stage3_vertical_reference_lowpass_cutoff_hz =
    request_.config.stage3_vertical_reference_lowpass_cutoff_hz;
  stage3_result.run_summary.stage3_vertical_anchor_sigma_m =
    request_.config.stage3_vertical_anchor_sigma_m;
  stage3_result.run_summary.stage3_vertical_reference_constraint_mode =
    ToString(request_.config.stage3_vertical_reference_constraint_mode);
  return stage3_result;
}

}  // namespace offline_lc_minimal
