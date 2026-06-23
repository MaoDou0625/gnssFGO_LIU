#include "offline_lc_minimal/core/VerticalVelocityDeltaBiasTargetAdjuster.h"

#include <cmath>
#include <limits>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsFiniteInterval(const double start_time_s, const double end_time_s) {
  return std::isfinite(start_time_s) &&
         std::isfinite(end_time_s) &&
         end_time_s > start_time_s + kTimeEpsilonS;
}

bool ContainsMidpoint(
  const BodyZBiasReestimateSegmentRow &segment,
  const double midpoint_time_s) {
  return std::isfinite(segment.start_time_s) &&
         std::isfinite(segment.end_time_s) &&
         segment.start_time_s <= midpoint_time_s + kTimeEpsilonS &&
         midpoint_time_s <= segment.end_time_s + kTimeEpsilonS;
}

const BodyZBiasReestimateSegmentRow *FindSegmentForInterval(
  const std::vector<BodyZBiasReestimateSegmentRow> &segments,
  const double start_time_s,
  const double end_time_s) {
  if (!IsFiniteInterval(start_time_s, end_time_s)) {
    return nullptr;
  }
  const double midpoint_time_s = 0.5 * (start_time_s + end_time_s);
  for (const auto &segment : segments) {
    if (!ContainsMidpoint(segment, midpoint_time_s)) {
      continue;
    }
    if (!std::isfinite(segment.prior_target_ba_z_mps2)) {
      continue;
    }
    return &segment;
  }
  return nullptr;
}

}  // namespace

VerticalVelocityDeltaBiasTargetAdjustment
AdjustVerticalVelocityDeltaTargetForBiasReestimate(
  const VerticalVelocityDeltaBiasTargetAdjustmentRequest &request) {
  VerticalVelocityDeltaBiasTargetAdjustment result;
  result.target_delta_vz_mps = request.target_delta_vz_mps;
  result.reference_ba_z_mps2 = request.reference_ba_z_mps2;

  if (request.bias_reestimate_segments == nullptr ||
      request.bias_reestimate_segments->empty() ||
      !IsFiniteInterval(request.start_time_s, request.end_time_s) ||
      !std::isfinite(request.target_delta_vz_mps) ||
      !std::isfinite(request.reference_ba_z_mps2)) {
    return result;
  }

  const auto *segment = FindSegmentForInterval(
    *request.bias_reestimate_segments,
    request.start_time_s,
    request.end_time_s);
  if (segment == nullptr) {
    return result;
  }

  const double dt_s = request.end_time_s - request.start_time_s;
  const double bias_delta_mps2 =
    segment->prior_target_ba_z_mps2 - request.reference_ba_z_mps2;
  if (!std::isfinite(bias_delta_mps2)) {
    return result;
  }

  result.target_delta_vz_mps =
    request.target_delta_vz_mps - bias_delta_mps2 * dt_s;
  result.reference_ba_z_mps2 = segment->prior_target_ba_z_mps2;
  result.applied = true;
  return result;
}

}  // namespace offline_lc_minimal
