#pragma once

#include <functional>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

using Stage1RunOnce = std::function<OfflineRunResult(const OfflineRunnerConfig &, DataSet)>;

struct Stage1YawRefinementRequest {
  OfflineRunnerConfig config;
  DataSet dataset;
  Stage1RunOnce run_once;
};

class Stage1YawRefinementRunner {
 public:
  explicit Stage1YawRefinementRunner(Stage1YawRefinementRequest request);

  [[nodiscard]] OfflineRunResult Run() const;

 private:
  Stage1YawRefinementRequest request_;
};

}  // namespace offline_lc_minimal
