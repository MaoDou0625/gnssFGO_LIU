#include "offline_lc_minimal/core/GraphTimelineBuilder.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1e-9;

}  // namespace

std::vector<double> BuildStateTimestamps(
  const double start_time_s,
  const double end_time_s,
  const double state_frequency_hz) {
  if (state_frequency_hz <= 0.0) {
    throw std::runtime_error("state_frequency_hz must be positive");
  }
  if (end_time_s <= start_time_s) {
    throw std::runtime_error("state timeline end must be after start");
  }

  const double dt_s = 1.0 / state_frequency_hz;
  std::vector<double> timestamps;
  timestamps.push_back(start_time_s);

  for (double next_time_s = start_time_s + dt_s; next_time_s < end_time_s - kTimeEpsilonS; next_time_s += dt_s) {
    timestamps.push_back(next_time_s);
  }

  if (timestamps.back() < end_time_s - kTimeEpsilonS) {
    timestamps.push_back(end_time_s);
  } else {
    timestamps.back() = end_time_s;
  }

  if (timestamps.size() < 2U) {
    throw std::runtime_error("state timeline must contain at least two states");
  }
  return timestamps;
}

GraphTimeline BuildGraphTimeline(
  const double alignment_start_time_s,
  const double alignment_end_time_s,
  const double navigation_start_time_s,
  const double end_time_s,
  const OfflineRunnerConfig &config) {
  GraphTimeline timeline;
  if (config.enable_initial_static_subgraph && alignment_end_time_s > alignment_start_time_s + kTimeEpsilonS) {
    timeline.timestamps_s = BuildStateTimestamps(
      alignment_start_time_s,
      alignment_end_time_s,
      config.initial_static_state_frequency_hz);
    timeline.initial_static_state_count = timeline.timestamps_s.size();
  }

  std::vector<double> dynamic_timestamps =
    BuildStateTimestamps(navigation_start_time_s, end_time_s, config.state_frequency_hz);
  const bool shares_boundary =
    !timeline.timestamps_s.empty() &&
    !dynamic_timestamps.empty() &&
    std::abs(dynamic_timestamps.front() - timeline.timestamps_s.back()) <= kTimeEpsilonS;
  if (shares_boundary) {
    dynamic_timestamps.erase(dynamic_timestamps.begin());
    timeline.dynamic_start_index = timeline.timestamps_s.size() - 1U;
  } else {
    timeline.dynamic_start_index = timeline.timestamps_s.size();
  }
  timeline.timestamps_s.insert(
    timeline.timestamps_s.end(),
    dynamic_timestamps.begin(),
    dynamic_timestamps.end());

  if (timeline.timestamps_s.size() < 2U) {
    throw std::runtime_error("graph timeline must contain at least two states");
  }
  return timeline;
}

StateMeasSyncResult FindStateForMeasurement(
  const std::map<std::size_t, double> &state_timestamp_map,
  const double corrected_timestamp_s,
  const OfflineRunnerConfig &config) {
  StateMeasSyncResult result;
  if (state_timestamp_map.empty()) {
    result.status = StateMeasSyncStatus::kDropped;
    return result;
  }

  if (state_timestamp_map.begin()->second > corrected_timestamp_s &&
      (corrected_timestamp_s - state_timestamp_map.begin()->second) < config.state_meas_sync_lower_bound_s) {
    result.status = StateMeasSyncStatus::kDropped;
    return result;
  }

  if (corrected_timestamp_s < state_timestamp_map.begin()->second) {
    result.status = StateMeasSyncStatus::kDropped;
    return result;
  }

  const auto last_state_it = std::prev(state_timestamp_map.end());
  if ((corrected_timestamp_s - last_state_it->second) > config.state_meas_sync_upper_bound_s) {
    result.status = StateMeasSyncStatus::kCached;
    return result;
  }

  const auto next_state_it = std::find_if(
    state_timestamp_map.begin(),
    state_timestamp_map.end(),
    [corrected_timestamp_s](const auto &entry) { return entry.second >= corrected_timestamp_s; });

  const bool has_next = next_state_it != state_timestamp_map.end();
  const bool has_prev = has_next ? next_state_it != state_timestamp_map.begin() : !state_timestamp_map.empty();
  auto prev_state_it = has_next ? next_state_it : last_state_it;
  if (has_next && has_prev) {
    --prev_state_it;
  }

  struct SyncCandidate {
    bool valid{false};
    double abs_dt_s{std::numeric_limits<double>::infinity()};
    bool synchronized_to_i{false};
    std::size_t key_index_i{0U};
    std::size_t key_index_j{0U};
    double timestamp_i_s{0.0};
    double timestamp_j_s{0.0};
    double duration_from_state_i_s{0.0};
  };

  const auto make_sync_candidate =
    [&](const std::map<std::size_t, double>::const_iterator state_it, const bool synchronized_to_i) {
      SyncCandidate candidate;
      if (state_it == state_timestamp_map.end()) {
        return candidate;
      }

      const double state_meas_dt_s = corrected_timestamp_s - state_it->second;
      if (state_meas_dt_s < config.state_meas_sync_lower_bound_s ||
          state_meas_dt_s > config.state_meas_sync_upper_bound_s) {
        return candidate;
      }

      candidate.valid = true;
      candidate.abs_dt_s = std::abs(state_meas_dt_s);
      candidate.synchronized_to_i = synchronized_to_i;

      if (synchronized_to_i) {
        candidate.key_index_i = state_it->first;
        candidate.timestamp_i_s = state_it->second;
        candidate.duration_from_state_i_s = std::max(0.0, corrected_timestamp_s - candidate.timestamp_i_s);

        if (std::next(state_it) != state_timestamp_map.end()) {
          candidate.key_index_j = std::next(state_it)->first;
          candidate.timestamp_j_s = std::next(state_it)->second;
        } else {
          candidate.key_index_j = candidate.key_index_i;
          candidate.timestamp_j_s = candidate.timestamp_i_s;
        }
      } else {
        if (state_it == state_timestamp_map.begin()) {
          candidate.valid = false;
          return candidate;
        }

        const auto previous_state_it = std::prev(state_it);
        candidate.key_index_j = state_it->first;
        candidate.timestamp_j_s = state_it->second;
        candidate.key_index_i = previous_state_it->first;
        candidate.timestamp_i_s = previous_state_it->second;
        candidate.duration_from_state_i_s = std::max(0.0, corrected_timestamp_s - candidate.timestamp_i_s);
      }

      return candidate;
    };

  SyncCandidate best_candidate;
  if (has_prev) {
    best_candidate = make_sync_candidate(prev_state_it, true);
  } else if (has_next) {
    best_candidate = make_sync_candidate(next_state_it, true);
  }
  if (has_next) {
    const SyncCandidate next_candidate = make_sync_candidate(next_state_it, false);
    const bool is_nearly_tied =
      std::abs(next_candidate.abs_dt_s - best_candidate.abs_dt_s) <= std::numeric_limits<double>::epsilon();
    if (next_candidate.valid &&
        (!best_candidate.valid ||
         next_candidate.abs_dt_s < best_candidate.abs_dt_s ||
         (is_nearly_tied && !best_candidate.synchronized_to_i))) {
      best_candidate = next_candidate;
    }
  }

  if (best_candidate.valid) {
    result.found_i = true;
    result.key_index_i = best_candidate.key_index_i;
    result.key_index_j = best_candidate.key_index_j;
    result.timestamp_i_s = best_candidate.timestamp_i_s;
    result.timestamp_j_s = best_candidate.timestamp_j_s;
    result.duration_from_state_i_s = best_candidate.duration_from_state_i_s;
    result.status = best_candidate.synchronized_to_i ? StateMeasSyncStatus::kSynchronizedI
                                                     : StateMeasSyncStatus::kSynchronizedJ;
    return result;
  }

  if (has_prev && has_next) {
    result.key_index_i = prev_state_it->first;
    result.timestamp_i_s = prev_state_it->second;
    result.duration_from_state_i_s = corrected_timestamp_s - result.timestamp_i_s;
    result.key_index_j = next_state_it->first;
    result.timestamp_j_s = next_state_it->second;
    result.found_i = true;
    result.status = StateMeasSyncStatus::kInterpolated;
    return result;
  }

  result.status = StateMeasSyncStatus::kDropped;
  return result;
}

}  // namespace offline_lc_minimal
