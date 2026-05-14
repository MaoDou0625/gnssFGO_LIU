#include "offline_lc_minimal/core/Stage2VelocityOptimizationRunner.h"

#include <cmath>
#include <memory>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/Stage1YawRefinementRunner.h"

namespace offline_lc_minimal {
namespace {

void CopyStage1Summary(
  const OfflineRunResult &stage1_result,
  OfflineRunResult &stage2_result) {
  stage2_result.stage1_yaw_refinement_diagnostics =
    stage1_result.stage1_yaw_refinement_diagnostics;
  stage2_result.run_summary.stage1_yaw_refinement_enabled =
    stage1_result.run_summary.stage1_yaw_refinement_enabled;
  stage2_result.run_summary.stage1_yaw_refinement_iteration_count =
    stage1_result.run_summary.stage1_yaw_refinement_iteration_count;
  stage2_result.run_summary.stage1_yaw_refinement_converged =
    stage1_result.run_summary.stage1_yaw_refinement_converged;
  stage2_result.run_summary.stage1_yaw_refinement_stop_reason =
    stage1_result.run_summary.stage1_yaw_refinement_stop_reason;
  stage2_result.run_summary.stage1_yaw_refinement_final_yaw_rad =
    stage1_result.run_summary.stage1_yaw_refinement_final_yaw_rad;
  stage2_result.run_summary.stage1_yaw_refinement_final_median_error_rad =
    stage1_result.run_summary.stage1_yaw_refinement_final_median_error_rad;
  stage2_result.run_summary.stage1_yaw_refinement_final_noise_rad =
    stage1_result.run_summary.stage1_yaw_refinement_final_noise_rad;
  stage2_result.run_summary.stage1_yaw_refinement_final_update_rad =
    stage1_result.run_summary.stage1_yaw_refinement_final_update_rad;
}

}  // namespace

Stage2VelocityOptimizationRunner::Stage2VelocityOptimizationRunner(
  Stage2VelocityOptimizationRequest request)
    : request_(std::move(request)) {}

OfflineRunResult Stage2VelocityOptimizationRunner::Run() const {
  if (!request_.run_once) {
    throw std::runtime_error("stage2 velocity optimization requires a run_once callback");
  }

  OfflineRunnerConfig stage1_config = request_.config;
  stage1_config.enable_stage2_velocity_optimization = false;
  stage1_config.enable_stage1_yaw_refinement = true;
  Stage1YawRefinementRequest stage1_request;
  stage1_request.config = stage1_config;
  stage1_request.dataset = request_.dataset;
  stage1_request.run_once = [&](const OfflineRunnerConfig &config, DataSet dataset) {
    return request_.run_once(config, nullptr, std::move(dataset));
  };
  OfflineRunResult stage1_result =
    Stage1YawRefinementRunner(std::move(stage1_request)).Run();
  if (stage1_result.trajectory.empty()) {
    throw std::runtime_error("stage2 velocity optimization received an empty stage1 trajectory");
  }

  auto reference = std::make_shared<Stage2VelocityReference>();
  reference->trajectory = stage1_result.trajectory;

  OfflineRunnerConfig stage2_config =
    MakeStage2VelocityOptimizationConfig(request_.config);
  if (std::isfinite(stage1_result.run_summary.stage1_yaw_refinement_final_yaw_rad)) {
    stage2_config.enable_initial_yaw_override = true;
    stage2_config.initial_yaw_override_rad =
      stage1_result.run_summary.stage1_yaw_refinement_final_yaw_rad;
  }

  OfflineRunResult stage2_result =
    request_.run_once(stage2_config, reference, request_.dataset);
  CopyStage1Summary(stage1_result, stage2_result);
  stage2_result.run_summary.stage2_velocity_optimization_enabled = true;
  return stage2_result;
}

}  // namespace offline_lc_minimal
