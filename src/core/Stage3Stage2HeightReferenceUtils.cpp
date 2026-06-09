#include "offline_lc_minimal/core/Stage3Stage2HeightReferenceUtils.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

}  // namespace

std::vector<TimedStage2UpSample> BuildSortedStage2UpSamples(
  const Stage2VelocityReference &reference) {
  std::vector<TimedStage2UpSample> samples;
  samples.reserve(reference.trajectory.size());
  for (const auto &row : reference.trajectory) {
    const double up_m = row.enu_position_m.z();
    if (!std::isfinite(row.time_s) || !std::isfinite(up_m)) {
      continue;
    }
    samples.push_back(TimedStage2UpSample{row.time_s, up_m});
  }
  std::sort(
    samples.begin(),
    samples.end(),
    [](const TimedStage2UpSample &lhs, const TimedStage2UpSample &rhs) {
      return lhs.time_s < rhs.time_s;
    });
  samples.erase(
    std::unique(
      samples.begin(),
      samples.end(),
      [](const TimedStage2UpSample &lhs, const TimedStage2UpSample &rhs) {
        return std::abs(lhs.time_s - rhs.time_s) <= kTimeEpsilonS;
      }),
    samples.end());
  return samples;
}

bool HasStage2UpCoverage(
  const std::vector<TimedStage2UpSample> &samples,
  const double time_s) {
  return !samples.empty() &&
         time_s + kTimeEpsilonS >= samples.front().time_s &&
         time_s <= samples.back().time_s + kTimeEpsilonS;
}

double InterpolateStage2UpAt(
  const std::vector<TimedStage2UpSample> &samples,
  const double time_s) {
  if (!HasStage2UpCoverage(samples, time_s)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (samples.size() == 1U) {
    return samples.front().up_m;
  }
  const auto upper = std::lower_bound(
    samples.begin(),
    samples.end(),
    time_s,
    [](const TimedStage2UpSample &sample, const double target_time_s) {
      return sample.time_s < target_time_s;
    });
  if (upper == samples.begin()) {
    return samples.front().up_m;
  }
  if (upper == samples.end()) {
    return samples.back().up_m;
  }
  const auto &rhs = *upper;
  const auto &lhs = *(upper - 1);
  const double dt_s = rhs.time_s - lhs.time_s;
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double alpha = std::clamp((time_s - lhs.time_s) / dt_s, 0.0, 1.0);
  return (1.0 - alpha) * lhs.up_m + alpha * rhs.up_m;
}

bool IntervalsOverlapWithTolerance(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

bool IntervalOverlapsJumpWindow(
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double start_time_s,
  const double end_time_s) {
  return std::any_of(
    windows.begin(),
    windows.end(),
    [start_time_s, end_time_s](const BodyZJumpConstraintWindow &window) {
      return IntervalsOverlapWithTolerance(
        start_time_s,
        end_time_s,
        window.start_time_s,
        window.end_time_s);
    });
}

}  // namespace offline_lc_minimal
