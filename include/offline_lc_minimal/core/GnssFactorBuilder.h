#pragma once

#include <cstddef>
#include <functional>
#include <map>
#include <optional>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/VerticalConstraintPolicy.h"

namespace offline_lc_minimal {

struct GnssFactorBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  std::size_t navigation_start_index = 0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  std::vector<TrajectoryRow> *trajectory = nullptr;
  RunSummary *run_summary = nullptr;
  std::vector<GnssFactorRecord> *factor_records = nullptr;
  std::vector<GnssConsistencyRecord> *consistency_records = nullptr;
  std::vector<VerticalEnvelopeDiagnosticRow> *vertical_envelope_diagnostics = nullptr;
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> *rtk_vertical_drift_reference_profile =
    nullptr;
  bool collect_consistency_records = false;
  double dynamic_start_time_s = 0.0;

  std::function<bool(const GnssSolutionSample &sample)> should_use_sample;
  std::function<bool(double corrected_time_s)> is_within_imu_coverage;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
  std::function<Eigen::Vector3d(const GnssSolutionSample &sample)> clamped_sigma_m;
  std::function<StateMeasSyncResult(double corrected_time_s)> find_state_for_time_s;
  std::function<long long(std::size_t state_index)> trajectory_row_index_for_state;
};

class GnssFactorBuilder {
 public:
  explicit GnssFactorBuilder(GnssFactorBuildRequest request);

  void Build() const;

 private:
  void Validate() const;
  void AddSynchronizedFactors(
    const GnssSolutionSample &sample,
    std::size_t sample_index,
    double corrected_time_s,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const VerticalConstraintPolicy &vertical_policy) const;
  void AddInterpolatedFactors(
    const GnssSolutionSample &sample,
    std::size_t sample_index,
    double corrected_time_s,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const VerticalConstraintPolicy &vertical_policy) const;
  void UpdateTrajectoryRows(const GnssSolutionSample &sample, const GnssFactorRecord &record) const;

  GnssFactorBuildRequest request_;
};

}  // namespace offline_lc_minimal
