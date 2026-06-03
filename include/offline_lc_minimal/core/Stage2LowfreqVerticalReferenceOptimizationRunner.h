#pragma once

#include <functional>
#include <memory>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"

namespace offline_lc_minimal {

using Stage2LowfreqVerticalReferenceRunOnce = std::function<OfflineRunResult(
  const OfflineRunnerConfig &,
  std::shared_ptr<const Stage3VerticalReference>,
  DataSet)>;

struct Stage2LowfreqVerticalReferenceOptimizationRequest {
  OfflineRunnerConfig config;
  DataSet dataset;
  Stage2LowfreqVerticalReferenceRunOnce run_once;
};

class Stage2LowfreqVerticalReferenceOptimizationRunner {
 public:
  explicit Stage2LowfreqVerticalReferenceOptimizationRunner(
    Stage2LowfreqVerticalReferenceOptimizationRequest request);

  [[nodiscard]] OfflineRunResult Run() const;

 private:
  Stage2LowfreqVerticalReferenceOptimizationRequest request_;
};

}  // namespace offline_lc_minimal
