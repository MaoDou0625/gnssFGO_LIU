#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/factor/BodyZLeakageCorrectedVelocityFactor.h"

namespace offline_lc_minimal {

struct BodyZHorizontalLeakageEstimate {
  bool valid = false;
  factor::BodyZHorizontalLeakageModel model;
  BodyZHorizontalLeakageDiagnosticRow diagnostic;
};

struct BodyZHorizontalLeakageEstimateRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<BodyZJumpConstraintWindow> *excluded_windows = nullptr;
  const gtsam::Values *initial_values = nullptr;
  const std::vector<ReferenceNodeState> *reference_states = nullptr;
  std::size_t dynamic_start_index = 0;
};

class BodyZHorizontalLeakageEstimator {
 public:
  explicit BodyZHorizontalLeakageEstimator(BodyZHorizontalLeakageEstimateRequest request);

  [[nodiscard]] BodyZHorizontalLeakageEstimate Estimate() const;

 private:
  BodyZHorizontalLeakageEstimateRequest request_;
};

}  // namespace offline_lc_minimal
