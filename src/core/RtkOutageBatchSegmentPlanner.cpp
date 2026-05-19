#include "offline_lc_minimal/core/RtkOutageBatchSegmentPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED" &&
         std::isfinite(window.start_time_s) &&
         std::isfinite(window.end_time_s) &&
         window.end_time_s > window.start_time_s + kTimeEpsilonS;
}

RtkOutageBatchSegmentRow MakeSegment(
  const std::size_t segment_index,
  const char *role,
  const long long outage_index,
  const double start_time_s,
  const double end_time_s,
  const bool allow_vertical_boundary_jump,
  const char *start_boundary_source,
  const char *end_boundary_source) {
  RtkOutageBatchSegmentRow row;
  row.segment_index = segment_index;
  row.segment_role = role;
  row.source_outage_window_index = outage_index;
  row.start_time_s = start_time_s;
  row.end_time_s = end_time_s;
  row.duration_s = end_time_s - start_time_s;
  row.planned = row.duration_s > kTimeEpsilonS;
  row.vertical_boundary_jump_allowed = allow_vertical_boundary_jump;
  row.start_boundary_source = start_boundary_source;
  row.end_boundary_source = end_boundary_source;
  row.skip_reason = row.planned ? "PLANNED" : "EMPTY_DURATION";
  return row;
}

double BoundaryTimeFromStateIndex(
  const std::vector<double> *state_timestamps,
  const std::size_t state_index,
  const double fallback_time_s) {
  if (state_timestamps == nullptr || state_index >= state_timestamps->size()) {
    return fallback_time_s;
  }
  const double state_time_s = (*state_timestamps)[state_index];
  return std::isfinite(state_time_s) ? state_time_s : fallback_time_s;
}

}  // namespace

RtkOutageBatchSegmentPlanner::RtkOutageBatchSegmentPlanner(
  RtkOutageBatchSegmentPlanRequest request)
    : request_(std::move(request)) {}

std::vector<RtkOutageBatchSegmentRow> RtkOutageBatchSegmentPlanner::Plan() const {
  if (request_.config == nullptr || request_.outage_windows == nullptr) {
    throw std::runtime_error("RtkOutageBatchSegmentPlanner received an incomplete request");
  }
  std::vector<RtkOutageBatchSegmentRow> segments;
  if (!request_.config->enable_rtk_outage_segmented_batch ||
      !request_.config->enable_rtk_outage_smoothing ||
      request_.config->rtk_outage_segmented_batch_max_outages <= 0 ||
      !std::isfinite(request_.dynamic_start_time_s) ||
      !std::isfinite(request_.final_end_time_s) ||
      request_.final_end_time_s <= request_.dynamic_start_time_s + kTimeEpsilonS) {
    return segments;
  }

  const auto outage_it = std::find_if(
    request_.outage_windows->begin(),
    request_.outage_windows->end(),
    IsPlannedOutage);
  if (outage_it == request_.outage_windows->end()) {
    return segments;
  }

  const RtkOutageWindowRow &outage = *outage_it;
  const double pre_start_s = request_.dynamic_start_time_s;
  const double outage_start_boundary_s = BoundaryTimeFromStateIndex(
    request_.state_timestamps,
    outage.pre_anchor_state_index,
    outage.start_time_s);
  const double outage_end_boundary_s = BoundaryTimeFromStateIndex(
    request_.state_timestamps,
    outage.post_anchor_state_index,
    outage.end_time_s);
  const double pre_end_s = std::clamp(
    outage_start_boundary_s,
    request_.dynamic_start_time_s,
    request_.final_end_time_s);
  const double outage_start_s = pre_end_s;
  const double outage_end_s = std::clamp(
    outage_end_boundary_s,
    outage_start_s,
    request_.final_end_time_s);
  const double post_start_s = outage_end_s;
  const double post_end_s = request_.final_end_time_s;
  const bool allow_jump =
    request_.config->rtk_outage_segmented_batch_allow_vertical_boundary_jump;
  const long long outage_index = static_cast<long long>(outage.window_index);

  segments.push_back(MakeSegment(
    segments.size(),
    "PRE_RTK_VALID",
    outage_index,
    pre_start_s,
    pre_end_s,
    allow_jump,
    "DYNAMIC_START",
    "RTK_OUTAGE_START"));
  segments.push_back(MakeSegment(
    segments.size(),
    "RTK_OUTAGE",
    outage_index,
    outage_start_s,
    outage_end_s,
    allow_jump,
    "RTK_OUTAGE_START",
    "RTK_OUTAGE_END"));
  segments.push_back(MakeSegment(
    segments.size(),
    "POST_RTK_VALID",
    outage_index,
    post_start_s,
    post_end_s,
    allow_jump,
    "RTK_OUTAGE_END",
    "PROCESSING_END"));

  segments.erase(
    std::remove_if(
      segments.begin(),
      segments.end(),
      [](const RtkOutageBatchSegmentRow &row) { return !row.planned; }),
    segments.end());
  for (std::size_t index = 0; index < segments.size(); ++index) {
    segments[index].segment_index = index;
  }
  return segments;
}

}  // namespace offline_lc_minimal
