#pragma once

#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

struct RtkOutageBatchSegmentPlanRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
  const std::vector<GnssFactorRecord> *gnss_factor_records = nullptr;
  const std::vector<RtkOutageRecoveryReferenceRow> *recovery_references = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  double dynamic_start_time_s = 0.0;
  double final_end_time_s = 0.0;
};

class RtkOutageBatchSegmentPlanner {
 public:
  explicit RtkOutageBatchSegmentPlanner(RtkOutageBatchSegmentPlanRequest request);

  [[nodiscard]] std::vector<RtkOutageBatchSegmentRow> Plan() const;

 private:
  RtkOutageBatchSegmentPlanRequest request_;
};

}  // namespace offline_lc_minimal
