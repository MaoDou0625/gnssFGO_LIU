#include "offline_lc_minimal/core/Stage3SharedReferenceMapper.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "offline_lc_minimal/common/GeoUtils.h"

namespace offline_lc_minimal {
namespace {

struct InterpolatedSharedReference {
  double up_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_m = std::numeric_limits<double>::quiet_NaN();
  std::string source = "UNSET";
};

void ValidateSharedReferenceRows(
  const std::vector<SharedVerticalReferenceRow> &rows) {
  if (rows.empty()) {
    throw std::runtime_error("Stage3 shared reference requires non-empty z_shared rows");
  }
  double previous_s_m = -std::numeric_limits<double>::infinity();
  for (const auto &row : rows) {
    if (!std::isfinite(row.s_m) || !std::isfinite(row.reference_up_m)) {
      throw std::runtime_error("Stage3 shared reference rows must have finite s_m and reference_up_m");
    }
    if (!(row.s_m > previous_s_m)) {
      throw std::runtime_error("Stage3 shared reference rows must be strictly increasing in s_m");
    }
    previous_s_m = row.s_m;
  }
}

GeoReference MakeSharedReferenceGeoReference(
  const std::vector<SharedReferenceLinePoint> &line) {
  if (line.size() < 2U) {
    throw std::runtime_error("Stage3 shared reference line requires at least two points");
  }
  const auto &first = line.front();
  if (!std::isfinite(first.origin_lat_rad) ||
      !std::isfinite(first.origin_lon_rad) ||
      !std::isfinite(first.origin_h_m)) {
    throw std::runtime_error(
      "shared_reference_line.csv must include origin_lat_rad/origin_lon_rad/origin_h_m");
  }
  return GeoReference(first.origin_lat_rad, first.origin_lon_rad, first.origin_h_m);
}

InterpolatedSharedReference InterpolateSharedReference(
  const std::vector<SharedVerticalReferenceRow> &rows,
  const double s_m,
  const double fallback_sigma_m) {
  if (s_m <= rows.front().s_m) {
    return {
      rows.front().reference_up_m,
      std::isfinite(rows.front().sigma_m) ? rows.front().sigma_m : fallback_sigma_m,
      rows.front().source};
  }
  if (s_m >= rows.back().s_m) {
    return {
      rows.back().reference_up_m,
      std::isfinite(rows.back().sigma_m) ? rows.back().sigma_m : fallback_sigma_m,
      rows.back().source};
  }
  const auto right_it =
    std::lower_bound(
      rows.begin(),
      rows.end(),
      s_m,
      [](const SharedVerticalReferenceRow &row, const double value) {
        return row.s_m < value;
      });
  const auto left_it = right_it - 1;
  const double denominator = right_it->s_m - left_it->s_m;
  const double alpha = denominator > 0.0 ? (s_m - left_it->s_m) / denominator : 0.0;
  const double left_sigma =
    std::isfinite(left_it->sigma_m) ? left_it->sigma_m : fallback_sigma_m;
  const double right_sigma =
    std::isfinite(right_it->sigma_m) ? right_it->sigma_m : fallback_sigma_m;
  InterpolatedSharedReference reference;
  reference.up_m =
    (1.0 - alpha) * left_it->reference_up_m +
    alpha * right_it->reference_up_m;
  reference.sigma_m = (1.0 - alpha) * left_sigma + alpha * right_sigma;
  reference.source = alpha < 0.5 ? left_it->source : right_it->source;
  return reference;
}

}  // namespace

Stage3VerticalReference BuildStage3ReferenceFromSharedVerticalReference(
  Stage3SharedReferenceMapRequest request) {
  if (request.config == nullptr ||
      request.stage2_trajectory == nullptr ||
      request.shared_reference == nullptr ||
      request.shared_reference_line == nullptr) {
    throw std::runtime_error("Stage3 shared reference mapper received an incomplete request");
  }
  if (request.stage2_trajectory->empty()) {
    throw std::runtime_error("Stage3 shared reference mapper requires a non-empty Stage2 trajectory");
  }
  ValidateSharedReferenceRows(*request.shared_reference);
  const GeoReference geo_reference =
    MakeSharedReferenceGeoReference(*request.shared_reference_line);

  Stage3VerticalReference reference;
  reference.source_config = std::make_shared<OfflineRunnerConfig>(*request.config);
  reference.rows.reserve(request.stage2_trajectory->size());
  for (std::size_t index = 0; index < request.stage2_trajectory->size(); ++index) {
    const TrajectoryCsvRow &csv_row = (*request.stage2_trajectory)[index];
    if (!csv_row.has_geodetic) {
      throw std::runtime_error(
        "Stage3 shared reference mapper requires lat_rad/lon_rad/h_m in Stage2 trajectory");
    }
    const Eigen::Vector3d common_enu =
      geo_reference.Forward(csv_row.lat_rad, csv_row.lon_rad, csv_row.h_m);
    const SharedReferenceProjection projection =
      ProjectPointToSharedReferenceLine(
        common_enu.head<2>(),
        *request.shared_reference_line);
    if (!projection.valid) {
      throw std::runtime_error("failed to project Stage2 trajectory row to shared reference line");
    }
    const InterpolatedSharedReference shared =
      InterpolateSharedReference(
        *request.shared_reference,
        projection.s_m,
        request.config->stage3_vertical_anchor_sigma_m);

    Stage3VerticalReferenceDiagnosticRow row;
    row.state_index = index;
    row.time_s = csv_row.trajectory.time_s;
    row.stage2_up_m = csv_row.trajectory.enu_position_m.z();
    row.stage2_lowpass_up_m = shared.up_m;
    row.lowpass_delta_m = row.stage2_lowpass_up_m - row.stage2_up_m;
    row.sigma_m = shared.sigma_m;
    row.factor_added = false;
    row.skip_reason = "SHARED_" + shared.source;
    reference.rows.push_back(row);
  }
  return reference;
}

}  // namespace offline_lc_minimal
