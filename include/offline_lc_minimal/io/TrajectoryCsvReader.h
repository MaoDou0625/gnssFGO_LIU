#pragma once

#include <filesystem>
#include <limits>
#include <memory>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

struct TrajectoryCsvRow {
  TrajectoryRow trajectory;
  double lat_rad = std::numeric_limits<double>::quiet_NaN();
  double lon_rad = std::numeric_limits<double>::quiet_NaN();
  double h_m = std::numeric_limits<double>::quiet_NaN();
  bool has_geodetic = false;
};

[[nodiscard]] std::vector<TrajectoryRow> ReadTrajectoryCsv(
  const std::filesystem::path &path);

[[nodiscard]] std::vector<TrajectoryCsvRow> ReadTrajectoryCsvRows(
  const std::filesystem::path &path);

[[nodiscard]] Stage2VelocityReference ReadStage2VelocityReferenceCsv(
  const std::filesystem::path &path,
  std::shared_ptr<const OfflineRunnerConfig> source_config = nullptr);

}  // namespace offline_lc_minimal
