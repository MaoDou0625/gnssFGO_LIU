#pragma once

#include <string>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

class ResultWriter {
 public:
  static void WriteOutputs(
    const std::string &output_dir,
    const OfflineRunnerConfig &config,
    const OfflineRunResult &result,
    const GeoReference &geo_reference);
};

}  // namespace offline_lc_minimal
