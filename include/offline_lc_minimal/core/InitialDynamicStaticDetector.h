#pragma once

#include <functional>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct InitialDynamicStaticDetectionRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<ImuSample> *imu_samples = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<RtkOutageWindowRow> *rtk_outage_windows = nullptr;
  double alignment_start_time_s = 0.0;
  double alignment_end_time_s = 0.0;
  double dynamic_start_time_s = 0.0;
  double processing_end_time_s = 0.0;

  std::function<bool(const GnssSolutionSample &sample)> should_use_rtkfix_sample;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
};

struct InitialDynamicStaticDetectionResult {
  std::vector<LateStaticFeatureDiagnosticRow> feature_diagnostics;
  std::vector<LateStaticThresholdDiagnosticRow> threshold_diagnostics;
  std::vector<LateStaticWindowRow> windows;
};

class InitialDynamicStaticDetector {
 public:
  explicit InitialDynamicStaticDetector(InitialDynamicStaticDetectionRequest request);

  [[nodiscard]] InitialDynamicStaticDetectionResult Detect() const;

 private:
  InitialDynamicStaticDetectionRequest request_;
};

}  // namespace offline_lc_minimal
