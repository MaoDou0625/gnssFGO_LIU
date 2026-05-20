#include "offline_lc_minimal/core/Stage2VelocityOptimizationRunner.h"

#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"
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

std::vector<double> CollectTrajectoryTimestamps(
  const std::vector<TrajectoryRow> &trajectory) {
  std::vector<double> timestamps;
  timestamps.reserve(trajectory.size());
  for (const auto &row : trajectory) {
    if (std::isfinite(row.time_s)) {
      timestamps.push_back(row.time_s);
    }
  }
  return timestamps;
}

std::optional<OfflineRunResult> TryRunSegmentedStage2(
  const Stage2VelocityOptimizationRequest &request,
  const OfflineRunnerConfig &stage2_config,
  std::shared_ptr<const Stage2VelocityReference> reference,
  const OfflineRunResult &stage1_result) {
  if (!request.config.enable_rtk_outage_segmented_batch ||
      !request.config.enable_rtk_outage_smoothing ||
      stage1_result.rtk_outage_windows.empty()) {
    return std::nullopt;
  }

  std::vector<double> state_timestamps =
    CollectTrajectoryTimestamps(stage1_result.trajectory);
  if (state_timestamps.empty()) {
    throw std::runtime_error(
      "global stage1 result did not produce trajectory timestamps for segmented stage2");
  }

  double processing_end_time_s =
    stage1_result.run_summary.processing_end_time_s;
  if (!std::isfinite(processing_end_time_s) ||
      processing_end_time_s <= stage1_result.run_summary.dynamic_start_time_s) {
    processing_end_time_s = state_timestamps.back();
  }

  RtkOutageSegmentedBatchRunRequest segmented_request;
  segmented_request.base_config = request.config;
  segmented_request.config = stage2_config;
  segmented_request.dataset = request.dataset;
  segmented_request.stage2_reference = std::move(reference);
  segmented_request.outage_windows = stage1_result.rtk_outage_windows;
  segmented_request.bias_reestimate_segments = stage1_result.body_z_bias_reestimate_segments;
  segmented_request.gnss_factor_records = stage1_result.gnss_factor_records;
  segmented_request.state_timestamps = std::move(state_timestamps);
  segmented_request.dynamic_start_time_s =
    stage1_result.run_summary.dynamic_start_time_s;
  segmented_request.processing_end_time_s = processing_end_time_s;
  segmented_request.run_once = request.run_once;
  return RtkOutageSegmentedBatchRunner(std::move(segmented_request)).Run();
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
  stage1_config.enable_rtk_outage_segmented_batch = false;
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
  reference->source_config =
    std::make_shared<OfflineRunnerConfig>(request_.config);

  OfflineRunnerConfig stage2_config =
    MakeStage2VelocityOptimizationConfig(request_.config);
  if (std::isfinite(stage1_result.run_summary.stage1_yaw_refinement_final_yaw_rad)) {
    stage2_config.enable_initial_yaw_override = true;
    stage2_config.initial_yaw_override_rad =
      stage1_result.run_summary.stage1_yaw_refinement_final_yaw_rad;
  }

  if (auto segmented_result =
        TryRunSegmentedStage2(request_, stage2_config, reference, stage1_result)) {
    segmented_result->run_summary.stage2_velocity_optimization_enabled = true;
    return std::move(*segmented_result);
  }

  OfflineRunResult stage2_result =
    request_.run_once(stage2_config, reference, request_.dataset);
  CopyStage1Summary(stage1_result, stage2_result);
  stage2_result.run_summary.stage2_velocity_optimization_enabled = true;
  return stage2_result;
}

}  // namespace offline_lc_minimal
