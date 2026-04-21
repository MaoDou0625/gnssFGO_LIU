#pragma once

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

class TextDatasetLoader {
 public:
  static DataSet Load(const OfflineRunnerConfig &config);
};

}  // namespace offline_lc_minimal
