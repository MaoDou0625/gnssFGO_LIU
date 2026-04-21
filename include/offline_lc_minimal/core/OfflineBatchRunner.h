#pragma once

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

class OfflineBatchRunner {
 public:
  explicit OfflineBatchRunner(OfflineRunnerConfig config);

  [[nodiscard]] OfflineRunResult Run(DataSet dataset) const;
  [[nodiscard]] const OfflineRunnerConfig &config() const { return config_; }

 private:
  [[nodiscard]] std::size_t FindOriginIndex(const std::vector<GnssSolutionSample> &gnss_samples) const;
  void PopulateEnuPositions(std::vector<GnssSolutionSample> &gnss_samples, const GeoReference &geo_reference) const;
  [[nodiscard]] bool ShouldUseGnssFactor(const GnssSolutionSample &sample, RunSummary &run_summary) const;
  [[nodiscard]] double GnssFixScale(GnssFixType fix_type) const;
  [[nodiscard]] Eigen::Vector3d ClampGnssSigma(const GnssSolutionSample &sample) const;

  OfflineRunnerConfig config_;
};

}  // namespace offline_lc_minimal
