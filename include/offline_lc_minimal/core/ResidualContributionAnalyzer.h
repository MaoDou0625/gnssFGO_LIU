#pragma once

#include <string>
#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/DiagnosticsTypes.h"

namespace offline_lc_minimal {

struct ResidualContributionReport {
  std::vector<ResidualContributionRow> module_rows;
  std::vector<ResidualContributionRow> factor_rows;
};

[[nodiscard]] ResidualContributionReport AnalyzeResidualContributions(
  const gtsam::NonlinearFactorGraph &graph,
  const gtsam::Values &values,
  const std::string &stage_name = "final",
  int stage_iteration = 0);

}  // namespace offline_lc_minimal
