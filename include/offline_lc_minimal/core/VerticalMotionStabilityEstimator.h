#pragma once

#include <vector>

#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/SensorTypes.h"
#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/core/VerticalAccelBiasGmConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalMotionStabilityProfile.h"

namespace offline_lc_minimal {

struct VerticalMotionStabilityEstimateRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  const std::vector<VerticalVelocityDeltaPropagationRecord> *propagation_records = nullptr;
  const std::vector<VerticalAccelBiasGmTransitionRecord> *bias_gm_records = nullptr;
  const std::vector<ImuSample> *imu_samples = nullptr;
  const std::vector<BodyZSeedJumpWindowRow> *jump_windows = nullptr;
  const gtsam::Values *optimized_values = nullptr;
  int outer_pass = 0;
};

class VerticalMotionStabilityEstimator {
 public:
  explicit VerticalMotionStabilityEstimator(VerticalMotionStabilityEstimateRequest request);

  [[nodiscard]] VerticalMotionStabilityProfile Estimate() const;

 private:
  [[nodiscard]] bool OverlapsJumpPadding(double start_time_s, double end_time_s) const;

  VerticalMotionStabilityEstimateRequest request_;
  std::vector<BodyZJumpConstraintWindow> jump_constraint_windows_;
};

void AccumulateAdaptiveReweightingSummary(
  const VerticalMotionStabilityProfile &profile,
  const std::vector<VerticalVelocityDeltaDiagnosticRow> &vertical_velocity_delta_diagnostics,
  double dynamic_start_time_s,
  RunSummary &run_summary);

}  // namespace offline_lc_minimal
