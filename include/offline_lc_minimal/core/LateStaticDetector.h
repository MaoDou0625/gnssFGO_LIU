#pragma once

#include <functional>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct LateStaticDetectionRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<ImuSample> *imu_samples = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<RtkOutageWindowRow> *rtk_outage_windows = nullptr;
  double processing_start_time_s = 0.0;
  double processing_end_time_s = 0.0;
  double alignment_start_time_s = 0.0;
  double alignment_end_time_s = 0.0;

  std::function<bool(const GnssSolutionSample &sample)> should_use_rtkfix_sample;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
};

struct LateStaticThresholdSet {
  bool valid = false;
  double rtk_speed_rms_threshold_mps = 0.0;
  double rtk_horizontal_range_threshold_m = 0.0;
  double gyro_rms_threshold_radps = 0.0;
  double gyro_p95_threshold_radps = 0.0;
  std::vector<LateStaticThresholdDiagnosticRow> diagnostics;
};

class LateStaticFeatureExtractor {
 public:
  explicit LateStaticFeatureExtractor(LateStaticDetectionRequest request);

  [[nodiscard]] std::vector<LateStaticFeatureDiagnosticRow> Extract() const;

 private:
  LateStaticDetectionRequest request_;
};

class DataDrivenStaticThresholdEstimator {
 public:
  explicit DataDrivenStaticThresholdEstimator(const OfflineRunnerConfig &config);

  [[nodiscard]] LateStaticThresholdSet Estimate(
    const std::vector<LateStaticFeatureDiagnosticRow> &features) const;

 private:
  const OfflineRunnerConfig &config_;
};

class LateStaticWindowDetector {
 public:
  explicit LateStaticWindowDetector(const OfflineRunnerConfig &config);

  [[nodiscard]] std::vector<LateStaticWindowRow> Detect(
    const LateStaticThresholdSet &thresholds,
    std::vector<LateStaticFeatureDiagnosticRow> *features) const;

 private:
  const OfflineRunnerConfig &config_;
};

}  // namespace offline_lc_minimal
