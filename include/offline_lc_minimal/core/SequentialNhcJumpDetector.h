#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include <Eigen/Core>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct NhcThresholdSnapshot {
  double body_vy_threshold_mps = std::numeric_limits<double>::quiet_NaN();
  double body_vz_threshold_mps = std::numeric_limits<double>::quiet_NaN();
  double body_vz_baseline_mps = std::numeric_limits<double>::quiet_NaN();
};

struct NhcStateEvaluation {
  double body_vy_mps = std::numeric_limits<double>::quiet_NaN();
  double body_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double body_vz_residual_mps = std::numeric_limits<double>::quiet_NaN();
  double body_vz_jump_mps = std::numeric_limits<double>::quiet_NaN();
  bool exceeds_threshold = false;
};

class SequentialNhcJumpDetector {
 public:
  explicit SequentialNhcJumpDetector(const OfflineRunnerConfig &config);

  void SeedWithConfirmedStates(
    const std::vector<ReferenceNodeState> &reference_states,
    std::size_t start_index,
    std::size_t end_index);

  void ObserveConfirmedWindow(
    const std::vector<ReferenceNodeState> &reference_states,
    std::size_t start_index,
    std::size_t end_index);

  [[nodiscard]] NhcThresholdSnapshot CurrentThresholds(double evaluation_time_s) const;

  [[nodiscard]] NhcStateEvaluation EvaluateState(
    const ReferenceNodeState &state,
    double evaluation_time_s) const;

  [[nodiscard]] NhcStateEvaluation EvaluateTransition(
    const ReferenceNodeState &previous_state,
    const ReferenceNodeState &state,
    double evaluation_time_s) const;

  [[nodiscard]] std::optional<std::size_t> FindJumpAnchor(
    const std::vector<ReferenceNodeState> &reference_states,
    std::size_t start_index,
    std::size_t end_index) const;

 private:
  struct AcceptedSample {
    std::size_t state_index = 0;
    double time_s = 0.0;
    double body_vy_mps = 0.0;
    double body_vz_mps = 0.0;
  };

  [[nodiscard]] Eigen::Vector3d ComputeBodyVelocity(const ReferenceNodeState &state) const;
  [[nodiscard]] NhcStateEvaluation EvaluateWithPreviousBodyVz(
    double body_vy_mps,
    double body_vz_mps,
    double previous_body_vz_mps,
    double evaluation_time_s) const;
  [[nodiscard]] double ComputeBodyVzBaseline(double evaluation_time_s) const;
  void AppendState(const ReferenceNodeState &state, std::size_t state_index);
  void PruneHistory(double evaluation_time_s);

  const OfflineRunnerConfig &config_;
  std::vector<AcceptedSample> history_;
};

}  // namespace offline_lc_minimal
