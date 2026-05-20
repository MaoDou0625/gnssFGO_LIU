#pragma once

#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"

namespace offline_lc_minimal {

struct RtkOutageBiasContinuityPolicyRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
  const std::vector<BodyZBiasReestimateSegmentRow> *bias_reestimate_segments = nullptr;
};

class RtkOutageBiasContinuityPolicy {
 public:
  explicit RtkOutageBiasContinuityPolicy(RtkOutageBiasContinuityPolicyRequest request);

  [[nodiscard]] std::vector<RtkOutageBiasContinuityPolicyRow> Build() const;

 private:
  RtkOutageBiasContinuityPolicyRequest request_;
};

[[nodiscard]] bool AllowsRtkOutageBazContinuity(
  const std::vector<RtkOutageBiasContinuityPolicyRow> &policy_rows,
  std::size_t window_index,
  const std::string &boundary_role);

}  // namespace offline_lc_minimal
