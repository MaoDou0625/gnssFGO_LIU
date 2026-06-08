#pragma once

#include <vector>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/VerticalJumpImuMasker.h"

namespace offline_lc_minimal {

struct BodyZBiasReestimateConstraintBuildRequest {
  const OfflineRunnerConfig *config = nullptr;
  const std::vector<double> *state_timestamps = nullptr;
  std::vector<BodyZBiasReestimateSegmentRow> *segments = nullptr;
  const std::vector<RtkOutageBoundaryReferenceRow> *boundary_references = nullptr;
  const std::vector<VerticalJumpImuIntervalRecord> *imu_intervals = nullptr;
  gtsam::NonlinearFactorGraph *graph = nullptr;
  gtsam::Values *initial_values = nullptr;
  RunSummary *run_summary = nullptr;
};

class BodyZBiasReestimateConstraintBuilder {
 public:
  explicit BodyZBiasReestimateConstraintBuilder(BodyZBiasReestimateConstraintBuildRequest request);

  void Apply() const;

 private:
  [[nodiscard]] std::vector<std::size_t> StateIndicesInSegment(
    const BodyZBiasReestimateSegmentRow &segment) const;
  [[nodiscard]] const BodyZBiasReestimateSegmentRow *SegmentForState(
    std::size_t state_index) const;
  [[nodiscard]] const RtkOutageBoundaryReferenceRow *PostStartBazReferenceForSegment(
    const BodyZBiasReestimateSegmentRow &segment) const;
  [[nodiscard]] long long SegmentIndexForState(std::size_t state_index) const;
  [[nodiscard]] bool CrossesReestimateBoundary(const VerticalJumpImuIntervalRecord &interval) const;
  [[nodiscard]] bool BoundaryTouchesRtkOutageSegment(
    const VerticalJumpImuIntervalRecord &interval) const;

  BodyZBiasReestimateConstraintBuildRequest request_;
};

}  // namespace offline_lc_minimal
