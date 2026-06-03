#include "offline_lc_minimal/core/BodyZNHCWeightPlanner.h"

#include <algorithm>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool StrictlyOverlaps(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s < right_end_s - kTimeEpsilonS &&
         right_start_s < left_end_s - kTimeEpsilonS;
}

}  // namespace

BodyZNHCWindowWeightPlan BodyZNHCWeightPlanner::PlanWindow(
  const BodyZNHCWeightPlannerWindowRequest &request) {
  BodyZNHCWindowWeightPlan plan;
  plan.velocity_factor_used.resize(request.state_indices.size(), true);
  plan.applied_velocity_sigma_mps = request.target_velocity_sigma_mps;
  plan.applied_displacement_sigma_m = request.target_displacement_sigma_m;

  for (std::size_t offset = 0U; offset < request.state_indices.size(); ++offset) {
    const std::size_t state_index = request.state_indices[offset];
    const bool already_used =
      velocity_factor_state_indices_.find(state_index) != velocity_factor_state_indices_.end();
    if (request.strict_effective_weighting && already_used) {
      plan.velocity_factor_used[offset] = false;
      ++plan.velocity_state_duplicate_count;
      ++duplicate_velocity_state_count_;
      continue;
    }
    velocity_factor_state_indices_.insert(state_index);
  }

  for (const auto &[start_time_s, end_time_s] : displacement_intervals_s_) {
    if (StrictlyOverlaps(
          request.start_time_s,
          request.end_time_s,
          start_time_s,
          end_time_s)) {
      ++plan.interval_overlap_count;
    }
  }
  interval_overlap_count_ += plan.interval_overlap_count;
  displacement_intervals_s_.emplace_back(request.start_time_s, request.end_time_s);
  return plan;
}

std::size_t BodyZNHCWeightPlanner::uniqueVelocityFactorStateCount() const {
  return velocity_factor_state_indices_.size();
}

std::size_t BodyZNHCWeightPlanner::duplicateVelocityStateCount() const {
  return duplicate_velocity_state_count_;
}

std::size_t BodyZNHCWeightPlanner::intervalOverlapCount() const {
  return interval_overlap_count_;
}

}  // namespace offline_lc_minimal
