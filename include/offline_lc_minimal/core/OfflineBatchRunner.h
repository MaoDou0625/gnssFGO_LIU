#pragma once

#include <memory>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/Stage1OutageLateralVelocityEnvelopeEstimator.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceProfilePlanner.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

class OfflineBatchRunner {
 public:
  explicit OfflineBatchRunner(OfflineRunnerConfig config);
  OfflineBatchRunner(
    OfflineRunnerConfig config,
    std::shared_ptr<const Stage2VelocityReference> stage2_reference);
  OfflineBatchRunner(
    OfflineRunnerConfig config,
    std::shared_ptr<const Stage2VelocityReference> stage2_reference,
    std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference);
  OfflineBatchRunner(
    OfflineRunnerConfig config,
    std::shared_ptr<const Stage2VelocityReference> stage2_reference,
    std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference,
    std::shared_ptr<const Stage3VerticalReference> stage3_vertical_reference);

  [[nodiscard]] OfflineRunResult Run(DataSet dataset) const;
  [[nodiscard]] const OfflineRunnerConfig &config() const { return config_; }

 private:
  [[nodiscard]] std::size_t FindOriginIndex(
    const std::vector<GnssSolutionSample> &gnss_samples,
    const std::vector<ImuSample> &imu_samples) const;
  [[nodiscard]] std::size_t FindNavigationStartIndex(
    const std::vector<GnssSolutionSample> &gnss_samples,
    const std::vector<ImuSample> &imu_samples,
    std::size_t origin_index,
    double navigation_start_min_time_s) const;
  [[nodiscard]] std::vector<std::size_t> CollectInitializationCandidateIndices(
    const std::vector<GnssSolutionSample> &gnss_samples,
    const std::vector<ImuSample> &imu_samples) const;
  void PopulateEnuPositions(std::vector<GnssSolutionSample> &gnss_samples, const GeoReference &geo_reference) const;
  [[nodiscard]] bool ShouldUseGnssFactor(const GnssSolutionSample &sample, RunSummary &run_summary) const;
  [[nodiscard]] bool CanUseGnssSampleForInitialization(
    const GnssSolutionSample &sample,
    const std::vector<ImuSample> &imu_samples) const;
  [[nodiscard]] bool PassesGnssQualityFilters(const GnssSolutionSample &sample) const;
  [[nodiscard]] bool IsAllowedGnssFixType(GnssFixType fix_type) const;
  [[nodiscard]] bool IsWithinImuCoverage(const std::vector<ImuSample> &imu_samples, double time_s) const;
  [[nodiscard]] double CorrectedGnssTime(const GnssSolutionSample &sample) const;
  [[nodiscard]] double GnssFixScale(GnssFixType fix_type) const;
  [[nodiscard]] Eigen::Vector3d ClampGnssSigma(const GnssSolutionSample &sample) const;

  OfflineRunnerConfig config_;
  std::shared_ptr<const Stage2VelocityReference> stage2_reference_;
  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference_;
  std::shared_ptr<const Stage3VerticalReference> stage3_vertical_reference_;
};

}  // namespace offline_lc_minimal
