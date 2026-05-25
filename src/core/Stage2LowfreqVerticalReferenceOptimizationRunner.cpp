#include "offline_lc_minimal/core/Stage2LowfreqVerticalReferenceOptimizationRunner.h"

#include <memory>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

OfflineRunnerConfig MakeSourceConfig(OfflineRunnerConfig config) {
  config.enable_stage2_lowfreq_vertical_reference_optimization = false;
  config.enable_stage3_vertical_reference_optimization = false;
  config.gnss_vertical_reference_source = GnssVerticalReferenceSource::kRawRtk;
  config.enable_rtk_vertical_lowpass_reference = false;
  return config;
}

OfflineRunnerConfig MakeFinalConfig(OfflineRunnerConfig config) {
  config.enable_stage3_vertical_reference_optimization = false;
  config.gnss_vertical_reference_source =
    config.stage2_lowfreq_vertical_reference_source;
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
  final_result.stage2_lowfreq_vertical_reference_diagnostics =
    lowpass_reference->rows;
  return final_result;
}

}  // namespace offline_lc_minimal
