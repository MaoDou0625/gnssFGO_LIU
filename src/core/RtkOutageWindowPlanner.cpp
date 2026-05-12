#include "offline_lc_minimal/core/RtkOutageWindowPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool Overlaps(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

bool IsFiniteWindow(const BodyZSeedJumpWindowRow &window) {
  return std::isfinite(window.start_time_s) &&
         std::isfinite(window.end_time_s) &&
         window.end_time_s >= window.start_time_s;
}

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED";
}

}  // namespace

RtkOutageWindowPlanner::RtkOutageWindowPlanner(RtkOutageWindowPlanRequest request)
    : request_(std::move(request)) {}

std::vector<RtkOutageWindowRow> RtkOutageWindowPlanner::Plan() const {
  if (request_.config == nullptr || request_.gnss_samples == nullptr ||
      request_.state_timestamps == nullptr || request_.body_z_jump_windows == nullptr ||
      !request_.passes_gnss_quality_filters || !request_.corrected_time_s) {
    throw std::runtime_error("RtkOutageWindowPlanner received an incomplete request");
  }
  std::vector<RtkOutageWindowRow> windows;
  if (!request_.config->enable_rtk_outage_smoothing ||
      request_.gnss_samples->empty() ||
      request_.state_timestamps->empty()) {
    return windows;
  }

  std::size_t previous_sample_index = request_.gnss_samples->size();
  double previous_time_s = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t sample_index = request_.navigation_start_index;
       sample_index < request_.gnss_samples->size();
       ++sample_index) {
    const GnssSolutionSample &sample = (*request_.gnss_samples)[sample_index];
    if (!sample.has_enu_position || !request_.passes_gnss_quality_filters(sample)) {
      continue;
    }
    const double time_s = request_.corrected_time_s(sample);
    if (!std::isfinite(time_s)) {
      continue;
    }
    if (previous_sample_index != request_.gnss_samples->size()) {
      const double gap_s = time_s - previous_time_s;
      if (std::isfinite(gap_s) && gap_s >= request_.config->rtk_outage_min_gap_s) {
        RtkOutageWindowRow row;
        row.window_index = windows.size();
        row.pre_sample_index = previous_sample_index;
        row.post_sample_index = sample_index;
        row.pre_anchor_time_s = previous_time_s;
        row.post_anchor_time_s = time_s;
        row.start_time_s = previous_time_s;
        row.end_time_s = time_s;
        row.duration_s = gap_s;
        row.pre_anchor_state_index = NearestStateIndex(previous_time_s);
        row.post_anchor_state_index = NearestStateIndex(time_s);
        row.rejected_sample_count =
          CountRejectedSamples(previous_sample_index + 1U, sample_index);
        row.body_z_jump_overlap_count =
          CountBodyZOverlaps(row.start_time_s, row.end_time_s);
        if (row.post_anchor_state_index <= row.pre_anchor_state_index + 1U) {
          row.skip_reason = "NO_INTERIOR_STATE";
        } else {
          row.interior_state_count =
            row.post_anchor_state_index - row.pre_anchor_state_index - 1U;
          row.skip_reason = "PLANNED";
        }
        windows.push_back(row);
      }
    }
    previous_sample_index = sample_index;
    previous_time_s = time_s;
  }
  return windows;
}

std::size_t RtkOutageWindowPlanner::NearestStateIndex(const double time_s) const {
  const auto &times = *request_.state_timestamps;
  const auto it = std::lower_bound(times.begin(), times.end(), time_s);
  if (it == times.begin()) {
    return 0U;
  }
  if (it == times.end()) {
    return times.size() - 1U;
  }
  const std::size_t right_index = static_cast<std::size_t>(it - times.begin());
  const std::size_t left_index = right_index - 1U;
  return std::abs(times[right_index] - time_s) < std::abs(time_s - times[left_index])
    ? right_index
    : left_index;
}

std::size_t RtkOutageWindowPlanner::CountRejectedSamples(
  const std::size_t begin_index,
  const std::size_t end_index) const {
  if (begin_index >= end_index) {
    return 0U;
  }
  std::size_t count = 0;
  const std::size_t bounded_end = std::min(end_index, request_.gnss_samples->size());
  for (std::size_t sample_index = begin_index; sample_index < bounded_end; ++sample_index) {
    const GnssSolutionSample &sample = (*request_.gnss_samples)[sample_index];
    if (!sample.has_enu_position || !request_.passes_gnss_quality_filters(sample)) {
      ++count;
    }
  }
  return count;
}

std::size_t RtkOutageWindowPlanner::CountBodyZOverlaps(
  const double start_time_s,
  const double end_time_s) const {
  std::size_t count = 0;
  for (const auto &window : *request_.body_z_jump_windows) {
    if (!IsFiniteWindow(window)) {
      continue;
    }
    if (Overlaps(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      ++count;
    }
  }
  return count;
}

std::vector<BodyZSeedJumpWindowRow> BuildRtkOutageNHCWindows(
  const std::vector<BodyZSeedJumpWindowRow> &body_z_jump_windows,
  const std::vector<RtkOutageWindowRow> &rtk_outage_windows) {
  std::vector<BodyZSeedJumpWindowRow> windows = body_z_jump_windows;
  windows.reserve(body_z_jump_windows.size() + rtk_outage_windows.size());
  for (const auto &outage : rtk_outage_windows) {
    if (!IsPlannedOutage(outage)) {
      continue;
    }
    BodyZSeedJumpWindowRow row;
    row.direction = "RTK_OUTAGE";
    row.selection_level = 0;
    row.start_state_index = static_cast<long long>(outage.pre_anchor_state_index);
    row.center_state_index = static_cast<long long>(
      (outage.pre_anchor_state_index + outage.post_anchor_state_index) / 2U);
    row.end_state_index = static_cast<long long>(outage.post_anchor_state_index);
    row.start_time_s = outage.start_time_s;
    row.center_time_s = 0.5 * (outage.start_time_s + outage.end_time_s);
    row.end_time_s = outage.end_time_s;
    row.duration_s = outage.duration_s;
    windows.push_back(row);
  }
  return windows;
}

}  // namespace offline_lc_minimal
