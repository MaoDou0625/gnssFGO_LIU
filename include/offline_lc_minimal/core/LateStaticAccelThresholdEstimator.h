#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct LateStaticAccelThresholdRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<ImuSample> *imu_samples = nullptr;
  double alignment_start_time_s = 0.0;
  double alignment_end_time_s = 0.0;
};

struct LateStaticAccelThresholdEstimate {
  bool valid = false;
  double threshold_mps2 = std::numeric_limits<double>::quiet_NaN();
  double reference_std_mps2 = std::numeric_limits<double>::quiet_NaN();
  std::size_t sample_count = 0U;
  std::string method = "UNSET";
  std::string skip_reason = "UNSET";
};

[[nodiscard]] LateStaticAccelThresholdEstimate EstimateLateStaticAccelNormStdThreshold(
  const LateStaticAccelThresholdRequest &request);

[[nodiscard]] LateStaticThresholdDiagnosticRow MakeLateStaticAccelNormStdThresholdDiagnostic(
  const LateStaticAccelThresholdEstimate &estimate);

}  // namespace offline_lc_minimal
