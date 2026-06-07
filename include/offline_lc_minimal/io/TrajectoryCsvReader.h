#pragma once

#include <filesystem>
#include <memory>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

[[nodiscard]] std::vector<TrajectoryRow> ReadTrajectoryCsv(
  const std::filesystem::path &path);

[[nodiscard]] Stage2VelocityReference ReadStage2VelocityReferenceCsv(
  const std::filesystem::path &path,
  std::shared_ptr<const OfflineRunnerConfig> source_config = nullptr);

}  // namespace offline_lc_minimal
