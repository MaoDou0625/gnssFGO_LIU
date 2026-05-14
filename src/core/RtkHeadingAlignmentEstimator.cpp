#include "offline_lc_minimal/core/RtkHeadingAlignmentEstimator.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kMadToSigmaScale = 1.4826;

struct RtkHeadingSample {
  double time_s = 0.0;
  Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
};

bool IsFiniteVector(const Eigen::Vector3d &value) {
  return value.allFinite();
}

double CorrectedGnssTime(const GnssSolutionSample &sample, const RtkHeadingAlignmentOptions &options) {
  return sample.time_s - options.gnss_time_offset_s;
}

std::vector<RtkHeadingSample> CollectRtkFixSamples(const RtkHeadingAlignmentRequest &request) {
  std::vector<RtkHeadingSample> samples;
  if (request.gnss_samples == nullptr) {
    return samples;
  }
  samples.reserve(request.gnss_samples->size());
  for (const auto &sample : *request.gnss_samples) {
    if (sample.fix_type() != GnssFixType::kRtkFix) {
      continue;
    }
    const double corrected_time_s = CorrectedGnssTime(sample, request.options);
    if (!std::isfinite(corrected_time_s) ||
        corrected_time_s < request.options.dynamic_start_time_s - kTimeEpsilonS ||
        corrected_time_s > request.options.end_time_s + kTimeEpsilonS) {
      continue;
    }

    Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
    if (sample.has_enu_position && IsFiniteVector(sample.enu_position_m)) {
      position_m = sample.enu_position_m;
    } else if (sample.has_valid_position() && request.geo_reference != nullptr) {
      position_m = request.geo_reference->Forward(sample.lat_rad, sample.lon_rad, sample.h_m);
    } else {
      continue;
    }
    if (!IsFiniteVector(position_m)) {
      continue;
    }
    samples.push_back(RtkHeadingSample{corrected_time_s, position_m});
  }
  std::sort(samples.begin(), samples.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.time_s < rhs.time_s;
  });
  return samples;
}

std::size_t FindIndexAtOrBefore(const std::vector<RtkHeadingSample> &samples, const double time_s) {
  const auto it = std::upper_bound(
    samples.begin(),
    samples.end(),
    time_s,
    [](const double target_time_s, const RtkHeadingSample &sample) {
      return target_time_s < sample.time_s;
    });
  if (it == samples.begin()) {
    return samples.size();
  }
  return static_cast<std::size_t>(std::distance(samples.begin(), std::prev(it)));
}

std::size_t FindIndexAtOrAfter(const std::vector<RtkHeadingSample> &samples, const double time_s) {
  const auto it = std::lower_bound(
    samples.begin(),
    samples.end(),
    time_s,
    [](const RtkHeadingSample &sample, const double target_time_s) {
      return sample.time_s < target_time_s;
    });
  return static_cast<std::size_t>(std::distance(samples.begin(), it));
}

std::size_t FindNearestTrajectoryIndex(
  const std::vector<TrajectoryRow> &trajectory,
  const double target_time_s) {
  const auto it = std::lower_bound(
    trajectory.begin(),
    trajectory.end(),
    target_time_s,
    [](const TrajectoryRow &row, const double time_s) {
      return row.time_s < time_s;
    });
  std::size_t best_index = trajectory.size();
  double best_dt_s = std::numeric_limits<double>::infinity();
  if (it != trajectory.end()) {
    best_index = static_cast<std::size_t>(std::distance(trajectory.begin(), it));
    best_dt_s = std::abs(it->time_s - target_time_s);
  }
  if (it != trajectory.begin()) {
    const auto prev_it = std::prev(it);
    const double prev_dt_s = std::abs(prev_it->time_s - target_time_s);
    if (prev_dt_s < best_dt_s) {
      best_index = static_cast<std::size_t>(std::distance(trajectory.begin(), prev_it));
    }
  }
  return best_index;
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 1U) {
    return values[middle];
  }
  return 0.5 * (values[middle - 1U] + values[middle]);
}

double CircularMedian(const std::vector<double> &angles_rad) {
  if (angles_rad.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sin_sum = 0.0;
  double cos_sum = 0.0;
  for (const double angle_rad : angles_rad) {
    sin_sum += std::sin(angle_rad);
    cos_sum += std::cos(angle_rad);
  }
  double center_rad = angles_rad.front();
  if (std::hypot(sin_sum, cos_sum) > 1.0e-12) {
    center_rad = std::atan2(sin_sum, cos_sum);
  }

  std::vector<double> unwrapped;
  unwrapped.reserve(angles_rad.size());
  for (const double angle_rad : angles_rad) {
    unwrapped.push_back(center_rad + NormalizeHeadingAngleRad(angle_rad - center_rad));
  }
  return NormalizeHeadingAngleRad(Median(std::move(unwrapped)));
}

RtkHeadingAlignmentEstimate EstimateFromErrors(std::vector<double> errors_rad) {
  RtkHeadingAlignmentEstimate estimate;
  if (errors_rad.empty()) {
    estimate.stop_reason = "no_valid_heading_pairs";
    return estimate;
  }
  estimate.valid = true;
  estimate.valid_pair_count = errors_rad.size();
  estimate.median_error_rad = CircularMedian(errors_rad);

  std::vector<double> abs_centered;
  abs_centered.reserve(errors_rad.size());
  double abs_sum = 0.0;
  double sq_sum = 0.0;
  double max_abs = 0.0;
  for (const double error_rad : errors_rad) {
    const double normalized_error_rad = NormalizeHeadingAngleRad(error_rad);
    const double abs_error_rad = std::abs(normalized_error_rad);
    abs_sum += abs_error_rad;
    sq_sum += normalized_error_rad * normalized_error_rad;
    max_abs = std::max(max_abs, abs_error_rad);
    abs_centered.push_back(
      std::abs(NormalizeHeadingAngleRad(normalized_error_rad - estimate.median_error_rad)));
  }
  estimate.mean_abs_error_rad = abs_sum / static_cast<double>(errors_rad.size());
  estimate.rms_error_rad = std::sqrt(sq_sum / static_cast<double>(errors_rad.size()));
  estimate.max_abs_error_rad = max_abs;
  estimate.heading_noise_rad = kMadToSigmaScale * Median(std::move(abs_centered));
  estimate.stop_reason = "valid";
  return estimate;
}

}  // namespace

double NormalizeHeadingAngleRad(const double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

RtkHeadingAlignmentEstimator::RtkHeadingAlignmentEstimator(RtkHeadingAlignmentRequest request)
    : request_(std::move(request)) {}

RtkHeadingAlignmentEstimate RtkHeadingAlignmentEstimator::Estimate() const {
  RtkHeadingAlignmentEstimate estimate;
  if (request_.trajectory == nullptr || request_.trajectory->empty()) {
    estimate.stop_reason = "empty_trajectory";
    return estimate;
  }
  if (request_.gnss_samples == nullptr || request_.gnss_samples->empty()) {
    estimate.stop_reason = "empty_gnss";
    return estimate;
  }
  if (request_.options.heading_window_s <= 0.0 ||
      request_.options.time_tolerance_s <= 0.0 ||
      request_.options.min_displacement_m <= 0.0) {
    throw std::runtime_error("RTK heading alignment options must be positive");
  }

  std::vector<TrajectoryRow> trajectory = *request_.trajectory;
  std::sort(trajectory.begin(), trajectory.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.time_s < rhs.time_s;
  });
  const std::vector<RtkHeadingSample> samples = CollectRtkFixSamples(request_);
  if (samples.size() < 3U) {
    estimate.stop_reason = "insufficient_rtkfix_samples";
    return estimate;
  }

  const double half_window_s = 0.5 * request_.options.heading_window_s;
  std::vector<double> errors_rad;
  errors_rad.reserve(samples.size());
  for (std::size_t center_index = 0; center_index < samples.size(); ++center_index) {
    const double center_time_s = samples[center_index].time_s;
    const double left_time_s = center_time_s - half_window_s;
    const double right_time_s = center_time_s + half_window_s;
    if (left_time_s < samples.front().time_s || right_time_s > samples.back().time_s) {
      continue;
    }

    const std::size_t left_index = FindIndexAtOrBefore(samples, left_time_s);
    const std::size_t right_index = FindIndexAtOrAfter(samples, right_time_s);
    if (left_index >= samples.size() || right_index >= samples.size() ||
        left_index >= center_index || right_index <= center_index || left_index >= right_index) {
      continue;
    }

    const double left_gap_s = left_time_s - samples[left_index].time_s;
    const double right_gap_s = samples[right_index].time_s - right_time_s;
    if (left_gap_s > half_window_s + kTimeEpsilonS ||
        right_gap_s > half_window_s + kTimeEpsilonS) {
      continue;
    }

    const Eigen::Vector3d delta_m = samples[right_index].position_m - samples[left_index].position_m;
    const double displacement_m = std::hypot(delta_m.x(), delta_m.y());
    if (!std::isfinite(displacement_m) || displacement_m < request_.options.min_displacement_m) {
      continue;
    }

    const std::size_t trajectory_index = FindNearestTrajectoryIndex(trajectory, center_time_s);
    if (trajectory_index >= trajectory.size()) {
      continue;
    }
    if (std::abs(trajectory[trajectory_index].time_s - center_time_s) >
        request_.options.time_tolerance_s) {
      continue;
    }
    const double nav_yaw_rad = trajectory[trajectory_index].ypr_rad.x();
    if (!std::isfinite(nav_yaw_rad)) {
      continue;
    }
    const double rtk_course_rad = std::atan2(delta_m.y(), delta_m.x());
    errors_rad.push_back(NormalizeHeadingAngleRad(nav_yaw_rad - rtk_course_rad));
  }

  return EstimateFromErrors(std::move(errors_rad));
}

}  // namespace offline_lc_minimal
