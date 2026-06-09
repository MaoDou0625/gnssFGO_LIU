#pragma once

#include <vector>

#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

struct TimedStage2UpSample {
  double time_s = 0.0;
  double up_m = 0.0;
};

[[nodiscard]] std::vector<TimedStage2UpSample> BuildSortedStage2UpSamples(
  const Stage2VelocityReference &reference);

[[nodiscard]] bool HasStage2UpCoverage(
  const std::vector<TimedStage2UpSample> &samples,
  double time_s);

[[nodiscard]] double InterpolateStage2UpAt(
  const std::vector<TimedStage2UpSample> &samples,
  double time_s);

[[nodiscard]] bool IntervalsOverlapWithTolerance(
  double left_start_s,
  double left_end_s,
  double right_start_s,
  double right_end_s);

[[nodiscard]] bool IntervalOverlapsJumpWindow(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  double start_time_s,
  double end_time_s);

}  // namespace offline_lc_minimal
