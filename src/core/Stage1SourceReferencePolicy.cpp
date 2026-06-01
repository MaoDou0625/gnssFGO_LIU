#include "offline_lc_minimal/core/Stage1SourceReferencePolicy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "offline_lc_minimal/core/RtkOutageSegmentationPolicy.h"

namespace offline_lc_minimal {
namespace {

bool RequestsSegmentedBatch(const OfflineRunnerConfig &config) {
  return config.enable_rtk_outage_segmented_batch &&
         config.enable_rtk_outage_smoothing &&
         config.rtk_outage_segmented_batch_max_outages > 0;
}

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return (window.skip_reason == "PLANNED" || window.skip_reason == "ADDED") &&
         std::isfinite(window.start_time_s) &&
         std::isfinite(window.end_time_s) &&
         window.end_time_s > window.start_time_s;
}

bool HasPlannedOutage(const OfflineRunResult &result) {
  return std::any_of(
    result.rtk_outage_windows.begin(),
    result.rtk_outage_windows.end(),
    IsPlannedOutage);
}

std::string Stage1SourceSegmentationInactiveReason(
  const OfflineRunnerConfig &source_config,
  const OfflineRunResult &source_result) {
  if (source_result.run_summary.rtk_outage_segmented_batch_enabled) {
    return "";
  }
  if (!RequestsSegmentedBatch(source_config)) {
    return ExplainRtkOutageSegmentedBatchInactive(source_config);
  }
  if (!HasPlannedOutage(source_result)) {
    return "no_planned_outage";
  }
  return "segmented_batch_not_triggered";
}

std::string Stage1SourceReferenceRejectReason(
  const OfflineRunnerConfig &requested_config,
  const OfflineRunnerConfig &source_config,
  const OfflineRunResult &source_result) {
  if (source_result.run_summary.stage1_yaw_refinement_enabled &&
      !source_result.run_summary.stage1_yaw_refinement_reference_valid) {
    return "stage1_yaw_reference_invalid";
  }

  const bool top_level_requested_segmentation =
    RequestsSegmentedBatch(requested_config);
  if (top_level_requested_segmentation && HasPlannedOutage(source_result) &&
      !source_result.run_summary.rtk_outage_segmented_batch_enabled) {
    if (source_config.enable_rtk_outage_segmented_batch) {
      return "segmented_batch_requested_but_not_run";
    }
    return "segmented_batch_disabled_with_outage";
  }

  return "";
}

}  // namespace

void ApplyStage1SourceReferencePolicy(
  const Stage1SourceReferencePolicyRequest &request,
  OfflineRunResult &source_result) {
  if (request.requested_config == nullptr || request.source_config == nullptr) {
    throw std::runtime_error(
      "Stage1SourceReferencePolicy received an incomplete request");
  }

  RunSummary &summary = source_result.run_summary;
  summary.stage1_source_segmentation_context =
    ToString(RtkOutageSegmentationContext::kStage1Source);
  summary.stage1_source_segmented_batch_requested =
    RequestsSegmentedBatch(*request.requested_config);
  summary.stage1_source_segmented_batch_enabled =
    summary.rtk_outage_segmented_batch_enabled;
  summary.stage1_source_segment_count = summary.rtk_outage_batch_segment_count;
  summary.stage1_source_segmented_batch_run_count =
    summary.rtk_outage_segmented_batch_run_count;
  summary.stage1_source_segmented_batch_disabled_reason =
    Stage1SourceSegmentationInactiveReason(
      *request.source_config,
      source_result);

  const std::string reject_reason = Stage1SourceReferenceRejectReason(
    *request.requested_config,
    *request.source_config,
    source_result);
  summary.stage1_source_reference_evaluated = true;
  summary.stage1_source_reference_valid = reject_reason.empty();
  summary.stage1_source_reference_reject_reason = reject_reason;
  if (!reject_reason.empty()) {
    summary.stage1_yaw_refinement_reference_valid = false;
  }
}

}  // namespace offline_lc_minimal
