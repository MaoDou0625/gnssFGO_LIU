#pragma once

#include <functional>
#include <memory>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

namespace offline_lc_minimal {

using Stage3RunOnce = std::function<OfflineRunResult(
  const OfflineRunnerConfig &,
  std::shared_ptr<const Stage2VelocityReference>,
  std::shared_ptr<const Stage3VerticalReference>,
  DataSet)>;

struct Stage3VerticalReferenceOptimizationRequest {
  OfflineRunnerConfig config;
  DataSet dataset;
  Stage3RunOnce run_once;
};

class Stage3VerticalReferenceOptimizationRunner {
 public:
  explicit Stage3VerticalReferenceOptimizationRunner(
    Stage3VerticalReferenceOptimizationRequest request);

  [[nodiscard]] OfflineRunResult Run() const;

 private:
  Stage3VerticalReferenceOptimizationRequest request_;
};

}  // namespace offline_lc_minimal
