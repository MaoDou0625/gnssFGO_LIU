#pragma once

#include <functional>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/DiagnosticsTypes.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkVerticalDriftReferenceEstimateRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const gtsam::Values *optimized_values = nullptr;
  double alignment_start_time_s = 0.0;
  double alignment_end_time_s = 0.0;
  double static_reference_up_m = 0.0;
  double dynamic_start_time_s = 0.0;
  int pass_index = 0;

  std::function<bool(const GnssSolutionSample &sample)> should_use_sample;
  std::function<bool(double corrected_time_s)> is_within_imu_coverage;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
  std::function<Eigen::Vector3d(const GnssSolutionSample &sample)> clamped_sigma_m;
  std::function<StateMeasSyncResult(double corrected_time_s)> find_state_for_time_s;
};

struct RtkVerticalDriftReferenceEstimateResult {
  std::vector<RtkVerticalDriftReferenceDiagnosticRow> profile;
  double max_abs_profile_delta_m = 0.0;
};

class RtkVerticalDriftReferenceEstimator {
 public:
  explicit RtkVerticalDriftReferenceEstimator(RtkVerticalDriftReferenceEstimateRequest request);

  [[nodiscard]] RtkVerticalDriftReferenceEstimateResult Estimate(
    const std::vector<RtkVerticalDriftReferenceDiagnosticRow> *previous_profile) const;

 private:
  RtkVerticalDriftReferenceEstimateRequest request_;
};

void PopulateRtkVerticalDriftReferenceSummary(
  const OfflineRunnerConfig &config,
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> &profile,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
