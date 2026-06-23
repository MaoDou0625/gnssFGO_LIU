#include "offline_lc_minimal/core/SharedVerticalReferenceComposer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace offline_lc_minimal {
namespace {

constexpr double kDistanceEpsilonM = 1.0e-6;
constexpr double kFinalReferenceSmoothingRadiusM = 4.0;

std::vector<double> FillFiniteSeriesByInterpolation(std::vector<double> values) {
  std::vector<std::size_t> finite_indices;
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (std::isfinite(values[index])) {
      finite_indices.push_back(index);
    }
  }
  if (finite_indices.empty()) {
    throw std::runtime_error("shared vertical reference has no finite Stage2 navigation observations");
  }
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (std::isfinite(values[index])) {
      continue;
    }
    const auto right_it =
      std::lower_bound(finite_indices.begin(), finite_indices.end(), index);
    if (right_it == finite_indices.begin()) {
      values[index] = values[*right_it];
    } else if (right_it == finite_indices.end()) {
      values[index] = values[finite_indices.back()];
    } else {
      const std::size_t right = *right_it;
      const std::size_t left = *(right_it - 1);
      const double alpha =
        static_cast<double>(index - left) /
        std::max(static_cast<double>(right - left), 1.0);
      values[index] = (1.0 - alpha) * values[left] + alpha * values[right];
    }
  }
  return values;
}

void SmoothReferenceRows(
  std::vector<SharedVerticalReferenceRow> &rows,
  const double grid_spacing_m) {
  if (rows.empty()) {
    return;
  }
  const double radius_m =
    std::max(kFinalReferenceSmoothingRadiusM, 2.0 * grid_spacing_m);
  const double sigma_m = std::max(0.5 * radius_m, grid_spacing_m);
  const int radius_bins =
    std::max(1, static_cast<int>(std::ceil(radius_m / grid_spacing_m)));
  std::vector<double> smoothed(rows.size(), std::numeric_limits<double>::quiet_NaN());
  for (std::size_t index = 0; index < rows.size(); ++index) {
    double weight_sum = 0.0;
    double weighted_sum = 0.0;
    double weighted_x_sum = 0.0;
    double weighted_xx_sum = 0.0;
    double weighted_xy_sum = 0.0;
    const int begin = std::max(0, static_cast<int>(index) - radius_bins);
    const int end =
      std::min(static_cast<int>(rows.size()) - 1, static_cast<int>(index) + radius_bins);
    for (int candidate = begin; candidate <= end; ++candidate) {
      const double value = rows[static_cast<std::size_t>(candidate)].reference_up_m;
      if (!std::isfinite(value)) {
        continue;
      }
      const double distance_m =
        std::abs(static_cast<double>(candidate) - static_cast<double>(index)) *
        grid_spacing_m;
      const double weight =
        std::exp(-0.5 * std::pow(distance_m / sigma_m, 2.0));
      const double x_m =
        (static_cast<double>(candidate) - static_cast<double>(index)) *
        grid_spacing_m;
      weight_sum += weight;
      weighted_sum += weight * value;
      weighted_x_sum += weight * x_m;
      weighted_xx_sum += weight * x_m * x_m;
      weighted_xy_sum += weight * x_m * value;
    }
    if (weight_sum > 0.0) {
      const double determinant =
        weight_sum * weighted_xx_sum - weighted_x_sum * weighted_x_sum;
      smoothed[index] =
        determinant > kDistanceEpsilonM
          ? (weighted_sum * weighted_xx_sum -
             weighted_x_sum * weighted_xy_sum) / determinant
          : weighted_sum / weight_sum;
    }
  }
  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (std::isfinite(smoothed[index])) {
      rows[index].reference_up_m = smoothed[index];
    }
  }
}

}  // namespace

std::vector<SharedVerticalReferenceRow> ComposeNavOnlySharedVerticalReference(
  SharedVerticalReferenceCompositionRequest request) {
  if (!std::isfinite(request.grid_spacing_m) || request.grid_spacing_m <= 0.0 ||
      !std::isfinite(request.sigma_m) || request.sigma_m <= 0.0) {
    throw std::runtime_error("shared vertical reference composition grid spacing and sigma must be positive");
  }
  if (request.nav_up_by_bin.empty()) {
    throw std::runtime_error("shared vertical reference composition requires navigation bins");
  }
  if (request.nav_sample_count_by_bin.size() != request.nav_up_by_bin.size()) {
    throw std::runtime_error("shared vertical reference composition count bins do not match height bins");
  }

  const std::vector<double> filled_nav_up_by_bin =
    FillFiniteSeriesByInterpolation(std::move(request.nav_up_by_bin));

  std::vector<SharedVerticalReferenceRow> rows;
  rows.reserve(filled_nav_up_by_bin.size());
  for (std::size_t bin = 0; bin < filled_nav_up_by_bin.size(); ++bin) {
    SharedVerticalReferenceRow row;
    row.s_m = static_cast<double>(bin) * request.grid_spacing_m;
    row.reference_up_m = filled_nav_up_by_bin[bin];
    row.sigma_m = request.sigma_m;
    row.source =
      request.nav_sample_count_by_bin[bin] > 0U ? "NAV_BRIDGE" : "NAV_BRIDGE_INTERPOLATED";
    row.rtk_weight = 0.0;
    row.nav_bridge_weight = 1.0;
    row.sample_count = request.nav_sample_count_by_bin[bin];
    rows.push_back(row);
  }
  SmoothReferenceRows(rows, request.grid_spacing_m);
  return rows;
}

}  // namespace offline_lc_minimal
