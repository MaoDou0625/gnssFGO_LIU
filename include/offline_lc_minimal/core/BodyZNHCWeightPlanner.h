#pragma once

#include <cstddef>
#include <unordered_set>
#include <utility>
#include <vector>

namespace offline_lc_minimal {

struct BodyZNHCWeightPlannerWindowRequest {
  bool strict_effective_weighting = false;
  std::vector<std::size_t> state_indices;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  double target_velocity_sigma_mps = 1.0;
  double target_displacement_sigma_m = 1.0;
};

struct BodyZNHCWindowWeightPlan {
  std::vector<bool> velocity_factor_used;
  std::size_t velocity_state_duplicate_count = 0;
  std::size_t interval_overlap_count = 0;
  double applied_velocity_sigma_mps = 1.0;
  double applied_displacement_sigma_m = 1.0;
};

class BodyZNHCWeightPlanner {
 public:
  [[nodiscard]] BodyZNHCWindowWeightPlan PlanWindow(
    const BodyZNHCWeightPlannerWindowRequest &request);

  [[nodiscard]] std::size_t uniqueVelocityFactorStateCount() const;
  [[nodiscard]] std::size_t duplicateVelocityStateCount() const;
  [[nodiscard]] std::size_t intervalOverlapCount() const;

 private:
  std::unordered_set<std::size_t> velocity_factor_state_indices_;
  std::vector<std::pair<double, double>> displacement_intervals_s_;
  std::size_t duplicate_velocity_state_count_ = 0;
  std::size_t interval_overlap_count_ = 0;
};

}  // namespace offline_lc_minimal
