#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

#include <Eigen/Core>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkVerticalLowpassReferenceBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  std::size_t first_sample_index = 0;

  std::function<bool(double corrected_time_s)> is_within_imu_coverage;
  std::function<double(const GnssSolutionSample &sample)> corrected_time_s;
  std::function<Eigen::Vector3d(const GnssSolutionSample &sample)> clamped_sigma_m;
};

struct RtkVerticalLowpassReferenceBuildResult {
  std::vector<RtkVerticalLowpassReferenceRow> rows;
  std::size_t valid_count = 0;
  double raw_minus_lowpass_std_m = std::numeric_limits<double>::quiet_NaN();
  double raw_minus_lowpass_max_abs_m = std::numeric_limits<double>::quiet_NaN();
};

class RtkVerticalLowpassReferenceBuilder {
 public:
  explicit RtkVerticalLowpassReferenceBuilder(RtkVerticalLowpassReferenceBuildRequest request);

  [[nodiscard]] RtkVerticalLowpassReferenceBuildResult Build() const;

 private:
  void Validate() const;

  RtkVerticalLowpassReferenceBuildRequest request_;
};

}  // namespace offline_lc_minimal
