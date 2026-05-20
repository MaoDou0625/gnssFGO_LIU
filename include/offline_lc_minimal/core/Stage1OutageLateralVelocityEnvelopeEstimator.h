#pragma once

#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct Stage1OutageBodyYEnvelopeReference {
  std::vector<Stage1OutageBodyYEnvelopeRow> envelopes;
  std::vector<ReferenceNodeState> reference_states;
};

struct Stage1OutageLateralVelocityEnvelopeEstimateRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<RtkOutageWindowRow> *outage_windows = nullptr;
  const std::vector<TrajectoryRow> *trajectory = nullptr;
};

class Stage1OutageLateralVelocityEnvelopeEstimator {
 public:
  explicit Stage1OutageLateralVelocityEnvelopeEstimator(
    Stage1OutageLateralVelocityEnvelopeEstimateRequest request);

  [[nodiscard]] Stage1OutageBodyYEnvelopeReference Estimate() const;

 private:
  Stage1OutageLateralVelocityEnvelopeEstimateRequest request_;
};

}  // namespace offline_lc_minimal
