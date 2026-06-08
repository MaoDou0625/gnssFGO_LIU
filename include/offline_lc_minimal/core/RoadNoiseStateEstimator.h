#pragma once

#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"

namespace offline_lc_minimal {

struct RoadNoiseStateEstimatorRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<BodyZJumpSignalSample> *signal = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
};

class RoadNoiseStateEstimator {
 public:
  explicit RoadNoiseStateEstimator(RoadNoiseStateEstimatorRequest request);

  [[nodiscard]] std::vector<RoadNoiseStateSegmentRow> Estimate() const;

 private:
  RoadNoiseStateEstimatorRequest request_;
};

}  // namespace offline_lc_minimal
