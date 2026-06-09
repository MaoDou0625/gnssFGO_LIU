#include "offline_lc_minimal/core/RtkOutageBatchSegmentPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kRecoveryStateAlignmentToleranceS = 5.0e-3;

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return (window.skip_reason == "PLANNED" || window.skip_reason == "ADDED") &&
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

const RtkOutageRecoveryReferenceRow *FindRecoveryReference(
  const std::vector<RtkOutageRecoveryReferenceRow> *rows,
  const std::size_t window_index) {
  if (rows == nullptr) {
    return nullptr;
  }
  const auto it = std::find_if(
    rows->begin(),
    rows->end(),
    [&](const RtkOutageRecoveryReferenceRow &row) {
      return row.window_index == window_index;
    });
  return it == rows->end() ? nullptr : &(*it);
}

std::optional<double> NearestStateTime(
  const std::vector<double> *state_timestamps,
  const double target_time_s) {
  if (state_timestamps == nullptr || state_timestamps->empty() ||
      !std::isfinite(target_time_s)) {
    return std::nullopt;
  }
  const auto right =
    std::lower_bound(state_timestamps->begin(), state_timestamps->end(), target_time_s);
  if (right == state_timestamps->begin()) {
    return std::isfinite(state_timestamps->front())
      ? std::optional<double>(state_timestamps->front())
      : std::nullopt;
  }
  if (right == state_timestamps->end()) {
    return std::isfinite(state_timestamps->back())
      ? std::optional<double>(state_timestamps->back())
      : std::nullopt;
  }
  const auto left = std::prev(right);
  const double left_time_s = *left;
  const double right_time_s = *right;
  if (!std::isfinite(left_time_s)) {
    return std::isfinite(right_time_s) ? std::optional<double>(right_time_s) : std::nullopt;
  }
  if (!std::isfinite(right_time_s)) {
    return left_time_s;
  }
  return std::abs(left_time_s - target_time_s) <=
         std::abs(right_time_s - target_time_s)
    ? left_time_s
    : right_time_s;
}

std::optional<double> RecoveryFirstFixBoundaryTime(
  const RtkOutageBatchSegmentPlanRequest &request,
  const RtkOutageWindowRow &outage) {
  const RtkOutageRecoveryReferenceRow *reference =
    FindRecoveryReference(request.recovery_references, outage.window_index);
  if (reference == nullptr ||
      reference->valid_fix_sample_count == 0U ||
      !std::isfinite(reference->first_sample_time_s)) {
    return std::nullopt;
  }
  const std::optional<double> nearest_state_time_s =
    NearestStateTime(request.state_timestamps, reference->first_sample_time_s);
  if (nearest_state_time_s.has_value() &&
      std::abs(*nearest_state_time_s - reference->first_sample_time_s) <=
        kRecoveryStateAlignmentToleranceS) {
    return nearest_state_time_s;
  }
  return reference->first_sample_time_s;
}

double SynchronizedFactorStateTime(const GnssFactorRecord &record) {
  if (record.sync_status == StateMeasSyncStatus::kSynchronizedI &&
      std::isfinite(record.state_time_i_s)) {
    return record.state_time_i_s;
  }
  if (record.sync_status == StateMeasSyncStatus::kSynchronizedJ &&
      std::isfinite(record.state_time_j_s)) {
    return record.state_time_j_s;
  }
  return record.corrected_time_s;
}

std::optional<double> FirstUsedRtkFixBoundaryTime(
  const RtkOutageBatchSegmentPlanRequest &request,
  const RtkOutageWindowRow &outage) {
  if (request.gnss_factor_records == nullptr) {
    return std::nullopt;
  }
  const double recovery_factor_start_time_s =
    outage.end_time_s +
    (request.config != nullptr
      ? std::max(request.config->state_meas_sync_upper_bound_s, kTimeEpsilonS)
      : kTimeEpsilonS);
  const GnssFactorRecord *first_record = nullptr;
  for (const auto &record : *request.gnss_factor_records) {
    if (!record.factor_used ||
        record.gnss_fix_type != GnssFixType::kRtkFix ||
        !std::isfinite(record.corrected_time_s) ||
        record.corrected_time_s < recovery_factor_start_time_s) {
      continue;
    }
    if (first_record == nullptr ||
        record.corrected_time_s < first_record->corrected_time_s) {
      first_record = &record;
    }
  }
  if (first_record == nullptr) {
    return std::nullopt;
  }
  const double boundary_time_s = SynchronizedFactorStateTime(*first_record);
  return std::isfinite(boundary_time_s)
    ? std::optional<double>(boundary_time_s)
    : std::nullopt;
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
  const std::optional<double> first_used_rtkfix_boundary_time_s =
    FirstUsedRtkFixBoundaryTime(request_, outage);
  const std::optional<double> recovery_first_fix_boundary_time_s =
    RecoveryFirstFixBoundaryTime(request_, outage);
  const double pre_end_s = std::clamp(
    outage_start_boundary_s,
    request_.dynamic_start_time_s,
    request_.final_end_time_s);
  const double outage_start_s = pre_end_s;
  const double outage_end_s = std::clamp(
    first_used_rtkfix_boundary_time_s.value_or(
      recovery_first_fix_boundary_time_s.value_or(outage_end_boundary_s)),
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
