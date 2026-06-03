#pragma once

#include <cstddef>
#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

namespace offline_lc_minimal {

struct Stage3JumpContextEnvelopePlanRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const Stage3VerticalReference *reference = nullptr;
  const std::vector<BodyZJumpConstraintWindow> *windows = nullptr;
  const gtsam::Values *initial_values = nullptr;
  std::size_t dynamic_start_index = 0;
};

struct Stage3JumpContextEnvelopePlan {
  std::vector<Stage3JumpContextEnvelopeProfileRow> profiles;
};

class Stage3JumpContextEnvelopePlanner {
 public:
  explicit Stage3JumpContextEnvelopePlanner(
    Stage3JumpContextEnvelopePlanRequest request);

  [[nodiscard]] Stage3JumpContextEnvelopePlan Plan() const;

 private:
  void Validate() const;

  Stage3JumpContextEnvelopePlanRequest request_;
};

}  // namespace offline_lc_minimal
