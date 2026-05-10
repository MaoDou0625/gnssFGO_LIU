#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Core>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkVerticalLatentReferenceBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  std::size_t first_sample_index = 0;
  double reference_epoch_s = std::numeric_limits<double>::quiet_NaN();
  double dynamic_start_time_s = 0.0;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  gtsam::Values *initial_values = nullptr;
  RunSummary *run_summary = nullptr;

  std::function<bool(double corrected_time_s)> is_within_imu_coverage;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
  std::function<Eigen::Vector3d(const GnssSolutionSample &sample)> clamped_sigma_m;
};

struct RtkVerticalLatentReferenceBuildResult {
  std::vector<RtkVerticalLatentReferenceSampleReference> sample_references;
  std::vector<RtkVerticalLatentReferenceDiagnosticRow> diagnostics;
};

[[nodiscard]] gtsam::Key RtkVerticalLatentReferenceKey(std::size_t key_index);

class RtkVerticalLatentReferenceBuilder {
 public:
  explicit RtkVerticalLatentReferenceBuilder(RtkVerticalLatentReferenceBuildRequest request);

  [[nodiscard]] RtkVerticalLatentReferenceBuildResult Build() const;

 private:
  void Validate() const;

  RtkVerticalLatentReferenceBuildRequest request_;
};

}  // namespace offline_lc_minimal
