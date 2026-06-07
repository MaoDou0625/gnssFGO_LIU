#include "offline_lc_minimal/core/Stage3SharedReferenceDeltaSmoother.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace offline_lc_minimal {
namespace {

constexpr double kDistanceEpsilonM = 1.0e-9;

double FallbackDeltaAt(
  const std::vector<Stage3SharedReferenceDeltaSample> &samples,
  const std::size_t index) {
  if (index < samples.size() && std::isfinite(samples[index].raw_delta_m)) {
    return samples[index].raw_delta_m;
  }
  for (const auto &sample : samples) {
    if (std::isfinite(sample.raw_delta_m)) {
      return sample.raw_delta_m;
    }
  }
  return 0.0;
}

}  // namespace

std::vector<double> SmoothStage3SharedReferenceDelta(
  const std::vector<Stage3SharedReferenceDeltaSample> &samples,
  Stage3SharedReferenceDeltaSmoothingOptions options) {
  if (samples.empty()) {
    return {};
  }
  const double radius_m =
    std::max(options.radius_m, kDistanceEpsilonM);
  const double sigma_m =
    std::max(0.5 * radius_m, kDistanceEpsilonM);

  std::vector<double> smoothed(
    samples.size(),
    std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const double center_s_m = samples[index].s_m;
    if (!std::isfinite(center_s_m)) {
      smoothed[index] = FallbackDeltaAt(samples, index);
      continue;
    }

    double weight_sum = 0.0;
    double weighted_y_sum = 0.0;
    double weighted_x_sum = 0.0;
    double weighted_xx_sum = 0.0;
    double weighted_xy_sum = 0.0;
    for (const auto &candidate : samples) {
      if (!std::isfinite(candidate.s_m) ||
          !std::isfinite(candidate.raw_delta_m)) {
        continue;
      }
      const double x_m = candidate.s_m - center_s_m;
      const double distance_m = std::abs(x_m);
      if (distance_m > radius_m) {
        continue;
      }
      const double weight =
        std::exp(-0.5 * std::pow(distance_m / sigma_m, 2.0));
      weight_sum += weight;
      weighted_y_sum += weight * candidate.raw_delta_m;
      weighted_x_sum += weight * x_m;
      weighted_xx_sum += weight * x_m * x_m;
      weighted_xy_sum += weight * x_m * candidate.raw_delta_m;
    }

    if (weight_sum <= 0.0) {
      smoothed[index] = FallbackDeltaAt(samples, index);
      continue;
    }
    const double determinant =
      weight_sum * weighted_xx_sum - weighted_x_sum * weighted_x_sum;
    smoothed[index] =
      determinant > kDistanceEpsilonM
        ? (weighted_y_sum * weighted_xx_sum -
           weighted_x_sum * weighted_xy_sum) / determinant
        : weighted_y_sum / weight_sum;
  }
  return smoothed;
}

}  // namespace offline_lc_minimal
