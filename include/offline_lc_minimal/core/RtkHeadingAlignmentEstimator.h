#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/GeoUtils.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct RtkHeadingAlignmentOptions {
  double heading_window_s = 1.0;
  double time_tolerance_s = 0.12;
  double min_displacement_m = 0.2;
  double gnss_time_offset_s = 0.0;
  double dynamic_start_time_s = 0.0;
  double end_time_s = std::numeric_limits<double>::infinity();
};

struct RtkHeadingAlignmentEstimate {
  bool valid = false;
  std::size_t valid_pair_count = 0;
  double median_error_rad = std::numeric_limits<double>::quiet_NaN();
  double heading_noise_rad = std::numeric_limits<double>::quiet_NaN();
  double mean_abs_error_rad = std::numeric_limits<double>::quiet_NaN();
  double rms_error_rad = std::numeric_limits<double>::quiet_NaN();
  double max_abs_error_rad = std::numeric_limits<double>::quiet_NaN();
  std::string stop_reason;
};

struct RtkHeadingAlignmentRequest {
  const std::vector<GnssSolutionSample> *gnss_samples = nullptr;
  const std::vector<TrajectoryRow> *trajectory = nullptr;
  const GeoReference *geo_reference = nullptr;
  RtkHeadingAlignmentOptions options;
};

[[nodiscard]] double NormalizeHeadingAngleRad(double angle_rad);

class RtkHeadingAlignmentEstimator {
 public:
  explicit RtkHeadingAlignmentEstimator(RtkHeadingAlignmentRequest request);

  [[nodiscard]] RtkHeadingAlignmentEstimate Estimate() const;

 private:
  RtkHeadingAlignmentRequest request_;
};

}  // namespace offline_lc_minimal
