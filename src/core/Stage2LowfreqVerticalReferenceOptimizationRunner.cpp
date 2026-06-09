#include "offline_lc_minimal/core/Stage2LowfreqVerticalReferenceOptimizationRunner.h"

#include <memory>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"

namespace offline_lc_minimal {
namespace {

OfflineRunnerConfig MakeSourceConfig(OfflineRunnerConfig config) {
  config.enable_stage2_lowfreq_vertical_reference_optimization = false;
  config.enable_stage2_lowfreq_final_dvz_relaxation = false;
  config.enable_stage2_lowfreq_final_hold_relaxation = false;
  config = DisableStage3VerticalReferenceOptimization(std::move(config));
  config.gnss_vertical_reference_source = GnssVerticalReferenceSource::kRawRtk;
  config.enable_rtk_vertical_lowpass_reference = false;
  return config;
}

void ApplyFinalDvzRelaxation(OfflineRunnerConfig &config) {
  if (!config.enable_stage2_lowfreq_final_dvz_relaxation) {
    return;
  }
  const double scale = config.stage2_lowfreq_final_dvz_sigma_scale;
  if (config.enable_vertical_velocity_delta_context_sigma_scale) {
    config.vertical_velocity_delta_context_normal_sigma_scale *= scale;
    config.vertical_velocity_delta_context_rough_sigma_scale *= scale;
    config.vertical_velocity_delta_context_outage_sigma_scale *= scale;
    config.vertical_velocity_delta_context_jump_sigma_scale *= scale;
    return;
  }
  config.vertical_velocity_delta_sigma_scale *= scale;
}

void ApplyFinalHoldRelaxation(OfflineRunnerConfig &config) {
  if (!config.enable_stage2_lowfreq_final_hold_relaxation) {
    return;
  }
  config.stage2_attitude_hold_sigma_rad *=
    config.stage2_lowfreq_final_attitude_hold_sigma_scale;
  config.stage2_horizontal_position_hold_sigma_m *=
    config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale;
  config.stage2_horizontal_velocity_hold_sigma_mps *=
    config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale;
}

OfflineRunnerConfig MakeFinalConfig(OfflineRunnerConfig config) {
  config = DisableStage3VerticalReferenceOptimization(std::move(config));
  config.gnss_vertical_reference_source =
    config.stage2_lowfreq_vertical_reference_source;
  ApplyFinalDvzRelaxation(config);
  ApplyFinalHoldRelaxation(config);
  return config;
}

Stage3VerticalReference PlanLowpassReference(
  const OfflineRunnerConfig &config,
  const OfflineRunResult &source_result) {
  if (source_result.trajectory.empty()) {
    throw std::runtime_error(
      "Stage2 lowfreq vertical reference received an empty source trajectory");
  }
  OfflineRunnerConfig planner_config = config;
  planner_config.stage3_vertical_reference_lowpass_cutoff_hz =
    config.stage2_lowfreq_vertical_reference_cutoff_hz;
  Stage3VerticalReferenceProfilePlanRequest plan_request;
  plan_request.config = &planner_config;
  plan_request.stage2_trajectory = &source_result.trajectory;
  plan_request.initial_dynamic_static_windows =
    &source_result.initial_dynamic_static_windows;
  plan_request.dynamic_start_index =
    source_result.run_summary.initial_static_state_count;
  plan_request.dynamic_start_time_s =
    source_result.run_summary.dynamic_start_time_s;
  Stage3VerticalReference reference =
    Stage3VerticalReferenceProfilePlanner(std::move(plan_request)).Plan();
  reference.source_config =
    std::make_shared<OfflineRunnerConfig>(config);
  return reference;
}

}  // namespace

Stage2LowfreqVerticalReferenceOptimizationRunner::
  Stage2LowfreqVerticalReferenceOptimizationRunner(
    Stage2LowfreqVerticalReferenceOptimizationRequest request)
    : request_(std::move(request)) {}

OfflineRunResult Stage2LowfreqVerticalReferenceOptimizationRunner::Run() const {
  if (!request_.run_once) {
    throw std::runtime_error(
      "Stage2 lowfreq vertical reference optimization requires a run_once callback");
  }

  const OfflineRunnerConfig source_config = MakeSourceConfig(request_.config);
  OfflineRunResult source_result =
    request_.run_once(source_config, nullptr, request_.dataset);

  auto lowpass_reference =
    std::make_shared<Stage3VerticalReference>(
      PlanLowpassReference(request_.config, source_result));

  const OfflineRunnerConfig final_config = MakeFinalConfig(request_.config);
  OfflineRunResult final_result =
    request_.run_once(final_config, lowpass_reference, request_.dataset);
  final_result.run_summary.stage2_lowfreq_vertical_reference_optimization_enabled = true;
  final_result.run_summary.stage2_lowfreq_vertical_reference_source =
    ToString(request_.config.stage2_lowfreq_vertical_reference_source);
  final_result.run_summary.stage2_lowfreq_vertical_reference_cutoff_hz =
    request_.config.stage2_lowfreq_vertical_reference_cutoff_hz;
  final_result.run_summary.stage2_lowfreq_final_dvz_relaxation_enabled =
    request_.config.enable_stage2_lowfreq_final_dvz_relaxation;
  final_result.run_summary.stage2_lowfreq_final_dvz_sigma_scale =
    request_.config.stage2_lowfreq_final_dvz_sigma_scale;
  final_result.run_summary.stage2_lowfreq_final_hold_relaxation_enabled =
    request_.config.enable_stage2_lowfreq_final_hold_relaxation;
  final_result.run_summary.stage2_lowfreq_final_attitude_hold_sigma_scale =
    request_.config.stage2_lowfreq_final_attitude_hold_sigma_scale;
  final_result.run_summary.stage2_lowfreq_final_horizontal_position_hold_sigma_scale =
    request_.config.stage2_lowfreq_final_horizontal_position_hold_sigma_scale;
  final_result.run_summary.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale =
    request_.config.stage2_lowfreq_final_horizontal_velocity_hold_sigma_scale;
  final_result.stage2_lowfreq_vertical_reference_diagnostics =
    lowpass_reference->rows;
  return final_result;
}

}  // namespace offline_lc_minimal
