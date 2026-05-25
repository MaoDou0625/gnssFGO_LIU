#include "offline_lc_minimal/core/Stage2VelocityOptimizationRunner.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include "offline_lc_minimal/core/OptimizationStagePolicy.h"
#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"
#include "offline_lc_minimal/core/Stage1OutageLateralVelocityEnvelopeEstimator.h"
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
  stage2_result.stage1_outage_body_y_envelopes =
    stage1_result.stage1_outage_body_y_envelopes;
  stage2_result.stage1_outage_body_y_state_diagnostics =
    stage1_result.stage1_outage_body_y_state_diagnostics;
  stage2_result.run_summary.stage1_outage_body_y_envelope_enabled =
    stage1_result.run_summary.stage1_outage_body_y_envelope_enabled;
  stage2_result.run_summary.stage1_outage_body_y_envelope_count =
    stage1_result.run_summary.stage1_outage_body_y_envelope_count;
  stage2_result.run_summary.stage1_outage_body_y_envelope_valid_count =
    stage1_result.run_summary.stage1_outage_body_y_envelope_valid_count;
  stage2_result.run_summary.stage1_outage_body_y_velocity_factor_count =
    stage1_result.run_summary.stage1_outage_body_y_velocity_factor_count;
  stage2_result.run_summary.stage1_outage_body_y_mean_mps =
    stage1_result.run_summary.stage1_outage_body_y_mean_mps;
  stage2_result.run_summary.stage1_outage_body_y_rmse_mps =
    stage1_result.run_summary.stage1_outage_body_y_rmse_mps;
  stage2_result.run_summary.stage1_outage_body_y_deadband_mps =
    stage1_result.run_summary.stage1_outage_body_y_deadband_mps;
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
  stage1_config.enable_stage2_lowfreq_vertical_reference_optimization = false;
  stage1_config.enable_stage1_yaw_refinement = true;
  stage1_config.enable_rtk_outage_segmented_batch = false;
  stage1_config.gnss_vertical_reference_source =
    GnssVerticalReferenceSource::kRawRtk;
  Stage1YawRefinementRequest stage1_request;
  stage1_request.config = stage1_config;
  stage1_request.dataset = request_.dataset;
  stage1_request.run_once = [&](
                               const OfflineRunnerConfig &config,
                               std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> body_y_reference,
                               DataSet dataset) {
    return request_.run_once(config, nullptr, std::move(body_y_reference), std::move(dataset));
  };
  OfflineRunResult stage1_result =
    Stage1YawRefinementRunner(std::move(stage1_request)).Run();
  if (stage1_result.trajectory.empty()) {
    throw std::runtime_error("stage2 velocity optimization received an empty stage1 trajectory");
  }

  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> body_y_reference;
  if (request_.config.enable_stage1_outage_body_y_envelope &&
      !stage1_result.rtk_outage_windows.empty()) {
    Stage1OutageLateralVelocityEnvelopeEstimateRequest envelope_request;
    envelope_request.config = &request_.config;
    envelope_request.outage_windows = &stage1_result.rtk_outage_windows;
    envelope_request.trajectory = &stage1_result.trajectory;
    auto mutable_reference =
      std::make_shared<Stage1OutageBodyYEnvelopeReference>(
        Stage1OutageLateralVelocityEnvelopeEstimator(
          std::move(envelope_request)).Estimate());
    const bool has_valid_envelope =
      std::any_of(
        mutable_reference->envelopes.begin(),
        mutable_reference->envelopes.end(),
        [](const Stage1OutageBodyYEnvelopeRow &row) {
          return row.valid;
        });
    if (has_valid_envelope) {
      body_y_reference = mutable_reference;
      Stage1YawRefinementRequest constrained_stage1_request;
      constrained_stage1_request.config = stage1_config;
      constrained_stage1_request.dataset = request_.dataset;
      constrained_stage1_request.body_y_envelope_reference = body_y_reference;
      constrained_stage1_request.run_once = [&](
                                              const OfflineRunnerConfig &config,
                                              std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> reference,
                                              DataSet dataset) {
        return request_.run_once(config, nullptr, std::move(reference), std::move(dataset));
      };
      stage1_result =
        Stage1YawRefinementRunner(std::move(constrained_stage1_request)).Run();
      if (stage1_result.trajectory.empty()) {
        throw std::runtime_error(
          "stage2 velocity optimization received an empty constrained stage1 trajectory");
      }
    } else {
      stage1_result.stage1_outage_body_y_envelopes = mutable_reference->envelopes;
      stage1_result.run_summary.stage1_outage_body_y_envelope_enabled = true;
      stage1_result.run_summary.stage1_outage_body_y_envelope_count =
        mutable_reference->envelopes.size();
    }
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
    CopyStage1Summary(stage1_result, *segmented_result);
    segmented_result->run_summary.stage2_velocity_optimization_enabled = true;
    return std::move(*segmented_result);
  }

  OfflineRunResult stage2_result =
    request_.run_once(stage2_config, reference, nullptr, request_.dataset);
  CopyStage1Summary(stage1_result, stage2_result);
  stage2_result.run_summary.stage2_velocity_optimization_enabled = true;
  return stage2_result;
}

}  // namespace offline_lc_minimal
