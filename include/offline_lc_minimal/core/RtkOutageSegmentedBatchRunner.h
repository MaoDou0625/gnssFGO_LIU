#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/core/Stage1OutageLateralVelocityEnvelopeEstimator.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

using SegmentedBatchRunOnce = std::function<OfflineRunResult(
  OfflineRunnerConfig,
  std::shared_ptr<const Stage2VelocityReference>,
  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference>,
  DataSet)>;

struct RtkOutageSegmentedBatchRunRequest {
  OfflineRunnerConfig base_config;
  OfflineRunnerConfig config;
  DataSet dataset;
  std::shared_ptr<const Stage2VelocityReference> stage2_reference;
  std::vector<RtkOutageWindowRow> outage_windows;
  std::vector<BodyZBiasReestimateSegmentRow> bias_reestimate_segments;
  std::vector<GnssFactorRecord> gnss_factor_records;
  std::vector<double> state_timestamps;
  double dynamic_start_time_s = 0.0;
  double processing_end_time_s = 0.0;
  SegmentedBatchRunOnce run_once;
};

class RtkOutageSegmentedBatchRunner {
 public:
  explicit RtkOutageSegmentedBatchRunner(RtkOutageSegmentedBatchRunRequest request);

  [[nodiscard]] OfflineRunResult Run() const;

 private:
  RtkOutageSegmentedBatchRunRequest request_;
};

}  // namespace offline_lc_minimal
