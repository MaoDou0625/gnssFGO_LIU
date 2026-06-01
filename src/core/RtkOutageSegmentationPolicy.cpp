#include "offline_lc_minimal/core/RtkOutageSegmentationPolicy.h"

namespace offline_lc_minimal {

const char *ToString(const RtkOutageSegmentationContext context) {
  switch (context) {
    case RtkOutageSegmentationContext::kStage1Source:
      return "stage1_source";
    case RtkOutageSegmentationContext::kSegmentChild:
      return "segment_child";
    case RtkOutageSegmentationContext::kStage3Final:
      return "stage3_final";
    case RtkOutageSegmentationContext::kPassthrough:
      return "passthrough";
  }
  return "unknown";
}

OfflineRunnerConfig DisableRtkOutageSegmentedBatchRecursion(
  OfflineRunnerConfig config) {
  config.enable_rtk_outage_segmented_batch = false;
  return config;
}

std::string ExplainRtkOutageSegmentedBatchInactive(
  const OfflineRunnerConfig &config) {
  if (!config.enable_rtk_outage_segmented_batch) {
    return "segmented_batch_disabled_by_config";
  }
  if (!config.enable_rtk_outage_smoothing) {
    return "rtk_outage_smoothing_disabled";
  }
  if (config.rtk_outage_segmented_batch_max_outages <= 0) {
    return "max_outages_nonpositive";
  }
  return "not_triggered";
}

}  // namespace offline_lc_minimal
