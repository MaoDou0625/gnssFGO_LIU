#include "offline_lc_minimal/core/Stage3VerticalReferenceOptimizationRunner.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkOutageSegmentationPolicy.h"

namespace offline_lc_minimal {
namespace {

OfflineRunnerConfig MakeStage2SourceConfig(OfflineRunnerConfig config) {
  config.enable_stage3_vertical_reference_optimization = false;
  config.stage3_disable_stage2_vehicle_nhc_constraint = false;
  config.enable_stage3_jump_velocity_smoothness_regularizer = false;
  config.enable_stage3_jump_height_highfreq_deadband = false;
  return config;
}

OfflineRunnerConfig MakeStage3Config(OfflineRunnerConfig config) {
  const bool restore_base_tilt_attitude_reference =
    config.enable_base_graph_tilt_reference_constraint &&
    config.enable_attitude_reference_constraint;
  config = MakeStage2VelocityOptimizationConfig(config);
  if (restore_base_tilt_attitude_reference) {
    config.enable_attitude_reference_constraint = true;
  }
  config.enable_stage3_vertical_reference_optimization = false;
  config = DisableRtkOutageSegmentedBatchRecursion(std::move(config));
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
  reference->reference_states =
    !stage2_result.optimized_reference_states.empty()
      ? stage2_result.optimized_reference_states
      : BuildStage2ReferenceStatesFromTrajectory(stage2_result.trajectory);
  reference->source_config =
    std::make_shared<OfflineRunnerConfig>(source_config);
  return reference;
}

void CopySourceDiagnostics(
  const OfflineRunResult &stage2_result,
  OfflineRunResult &stage3_result) {
  stage3_result.stage1_yaw_refinement_diagnostics =
    stage2_result.stage1_yaw_refinement_diagnostics;
  stage3_result.stage1_yaw_residual_module_contributions =
    stage2_result.stage1_yaw_residual_module_contributions;
  stage3_result.stage1_yaw_residual_factor_contributions =
    stage2_result.stage1_yaw_residual_factor_contributions;
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
  stage3_result.run_summary.stage1_yaw_refinement_cycle_detected =
    stage2_result.run_summary.stage1_yaw_refinement_cycle_detected;
  stage3_result.run_summary.stage1_yaw_refinement_reference_valid =
    stage2_result.run_summary.stage1_yaw_refinement_reference_valid;
  stage3_result.run_summary.stage1_yaw_refinement_stop_reason =
    stage2_result.run_summary.stage1_yaw_refinement_stop_reason;
  stage3_result.run_summary.stage1_yaw_refinement_selected_iteration =
    stage2_result.run_summary.stage1_yaw_refinement_selected_iteration;
  stage3_result.run_summary.stage1_yaw_refinement_selection_reason =
    stage2_result.run_summary.stage1_yaw_refinement_selection_reason;
  stage3_result.run_summary.stage1_yaw_refinement_selected_branch_score =
    stage2_result.run_summary.stage1_yaw_refinement_selected_branch_score;
  stage3_result.run_summary.stage1_yaw_refinement_final_yaw_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_yaw_rad;
  stage3_result.run_summary.stage1_yaw_refinement_final_median_error_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_median_error_rad;
  stage3_result.run_summary.stage1_yaw_refinement_final_noise_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_noise_rad;
  stage3_result.run_summary.stage1_yaw_refinement_final_update_rad =
    stage2_result.run_summary.stage1_yaw_refinement_final_update_rad;
  stage3_result.run_summary.stage1_source_segmentation_context =
    stage2_result.run_summary.stage1_source_segmentation_context;
  stage3_result.run_summary.stage1_source_segmented_batch_requested =
    stage2_result.run_summary.stage1_source_segmented_batch_requested;
  stage3_result.run_summary.stage1_source_segmented_batch_enabled =
    stage2_result.run_summary.stage1_source_segmented_batch_enabled;
  stage3_result.run_summary.stage1_source_segment_count =
    stage2_result.run_summary.stage1_source_segment_count;
  stage3_result.run_summary.stage1_source_segmented_batch_run_count =
    stage2_result.run_summary.stage1_source_segmented_batch_run_count;
  stage3_result.run_summary.stage1_source_segmented_batch_disabled_reason =
    stage2_result.run_summary.stage1_source_segmented_batch_disabled_reason;
  stage3_result.run_summary.stage1_source_reference_evaluated =
    stage2_result.run_summary.stage1_source_reference_evaluated;
  stage3_result.run_summary.stage1_source_reference_valid =
    stage2_result.run_summary.stage1_source_reference_valid;
  stage3_result.run_summary.stage1_source_reference_reject_reason =
    stage2_result.run_summary.stage1_source_reference_reject_reason;
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
  if (stage2_result.run_summary.stage1_yaw_refinement_enabled &&
      !stage2_result.run_summary.stage1_yaw_refinement_reference_valid) {
    stage2_result.run_summary.stage3_vertical_reference_optimization_enabled = false;
    return stage2_result;
  }
  auto stage2_reference =
    MakeStage2ReferenceFromResult(stage2_result, request_.config);

  Stage3VerticalReferenceProfilePlanRequest plan_request;
  plan_request.config = &request_.config;
  plan_request.stage2_trajectory = &stage2_result.trajectory;
  plan_request.initial_dynamic_static_windows =
    &stage2_result.initial_dynamic_static_windows;
  plan_request.dynamic_start_index =
    stage2_result.run_summary.initial_static_state_count;
  plan_request.dynamic_start_time_s =
    stage2_result.run_summary.dynamic_start_time_s;
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
  stage3_result.run_summary.stage3_vertical_reference_smoothing_method =
    ToString(request_.config.stage3_vertical_reference_smoothing_method);
  stage3_result.run_summary.stage3_vertical_reference_lowpass_cutoff_hz =
    request_.config.stage3_vertical_reference_lowpass_cutoff_hz;
  stage3_result.run_summary.stage3_vertical_reference_spline_knot_spacing_m =
    request_.config.stage3_vertical_reference_spline_knot_spacing_m;
  stage3_result.run_summary.stage3_vertical_anchor_sigma_m =
    request_.config.stage3_vertical_anchor_sigma_m;
  stage3_result.run_summary.stage3_vertical_reference_constraint_mode =
    ToString(request_.config.stage3_vertical_reference_constraint_mode);
  return stage3_result;
}

}  // namespace offline_lc_minimal
