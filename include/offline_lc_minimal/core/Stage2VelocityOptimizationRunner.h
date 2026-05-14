#pragma once

#include <functional>
#include <memory>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

using Stage2RunOnce = std::function<OfflineRunResult(
  const OfflineRunnerConfig &,
  std::shared_ptr<const Stage2VelocityReference>,
  DataSet)>;

struct Stage2VelocityOptimizationRequest {
  OfflineRunnerConfig config;
  DataSet dataset;
  Stage2RunOnce run_once;
};

class Stage2VelocityOptimizationRunner {
 public:
  explicit Stage2VelocityOptimizationRunner(Stage2VelocityOptimizationRequest request);

  [[nodiscard]] OfflineRunResult Run() const;

 private:
  Stage2VelocityOptimizationRequest request_;
};

}  // namespace offline_lc_minimal
