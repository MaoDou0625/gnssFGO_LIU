#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct Stage3VerticalReference {
  std::vector<Stage3VerticalReferenceDiagnosticRow> rows;
  std::shared_ptr<const OfflineRunnerConfig> source_config;
};

struct Stage3VerticalReferenceProfilePlanRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<TrajectoryRow> *stage2_trajectory = nullptr;
  std::size_t dynamic_start_index = std::numeric_limits<std::size_t>::max();
  double dynamic_start_time_s = std::numeric_limits<double>::quiet_NaN();
};

class Stage3VerticalReferenceProfilePlanner {
 public:
  explicit Stage3VerticalReferenceProfilePlanner(
    Stage3VerticalReferenceProfilePlanRequest request);

  [[nodiscard]] Stage3VerticalReference Plan() const;

 private:
  Stage3VerticalReferenceProfilePlanRequest request_;
};

}  // namespace offline_lc_minimal
