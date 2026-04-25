#pragma once

#include <cstddef>
#include <functional>
#include <limits>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct VerticalVzReferenceSample {
  double time_s = 0.0;
  bool valid = false;
  double vz_ref_global_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_ref_global_smoothed_mps = std::numeric_limits<double>::quiet_NaN();
};

struct SparseVerticalJumpCandidate {
  std::size_t state_index = 0;
  double time_s = 0.0;
  double vz_prefit_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_ref_global_smoothed_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_mismatch_mps = std::numeric_limits<double>::quiet_NaN();
  double vz_mismatch_jump_mps = std::numeric_limits<double>::quiet_NaN();
  double jump_step_threshold_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_init_mps = std::numeric_limits<double>::quiet_NaN();
  double score = std::numeric_limits<double>::quiet_NaN();
  bool nhc_supported = false;
};

class SparseVerticalJumpPlanner {
 public:
  explicit SparseVerticalJumpPlanner(const OfflineRunnerConfig &config);

  [[nodiscard]] std::vector<VerticalVzReferenceSample> BuildGlobalReference(
    const std::vector<GnssSolutionSample> &gnss_samples,
    const std::vector<double> &state_timestamps,
    double dynamic_start_time_s) const;

  void SeedWithConfirmedStates(
    const std::vector<ReferenceNodeState> &reference_states,
    const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
    std::size_t start_index,
    std::size_t end_index);

  void ObserveConfirmedWindow(
    const std::vector<ReferenceNodeState> &reference_states,
    const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
    std::size_t start_index,
    std::size_t end_index);

  [[nodiscard]] double CurrentJumpStepThreshold(double evaluation_time_s) const;

  [[nodiscard]] std::vector<SparseVerticalJumpCandidate> BuildCandidates(
    const std::vector<ReferenceNodeState> &reference_states,
    const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
    std::size_t start_index,
    std::size_t end_index,
    const std::function<bool(std::size_t)> &nhc_support) const;

 private:
  struct AcceptedMismatchJumpSample {
    std::size_t state_index = 0;
    double time_s = 0.0;
    double mismatch_jump_mps = 0.0;
  };

  void ObserveStateRange(
    const std::vector<ReferenceNodeState> &reference_states,
    const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
    std::size_t start_index,
    std::size_t end_index);
  void PruneHistory(double evaluation_time_s);

  const OfflineRunnerConfig &config_;
  std::vector<AcceptedMismatchJumpSample> mismatch_jump_history_;
};

}  // namespace offline_lc_minimal
