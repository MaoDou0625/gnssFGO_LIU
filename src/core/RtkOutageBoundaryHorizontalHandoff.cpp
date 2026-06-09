#include "offline_lc_minimal/core/RtkOutageBoundaryHorizontalHandoff.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <optional>

#include "offline_lc_minimal/core/ImuIntegrationUtils.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

std::optional<double> LastKeptOutageStateTime(
  const std::vector<double> &state_timestamps,
  const RtkOutageWindowRow &outage) {
  if (!std::isfinite(outage.start_time_s) || !std::isfinite(outage.end_time_s) ||
      outage.end_time_s <= outage.start_time_s + kTimeEpsilonS) {
    return std::nullopt;
  }
  const auto upper = std::lower_bound(
    state_timestamps.begin(),
    state_timestamps.end(),
    outage.end_time_s - kTimeEpsilonS);
  if (upper == state_timestamps.begin()) {
    return std::nullopt;
  }
  for (auto it = std::make_reverse_iterator(upper);
       it != state_timestamps.rend();
       ++it) {
    const double time_s = *it;
    if (!std::isfinite(time_s)) {
      continue;
    }
    if (time_s <= outage.start_time_s + kTimeEpsilonS) {
      break;
    }
    if (time_s < outage.end_time_s - kTimeEpsilonS) {
      return time_s;
    }
  }
  return std::nullopt;
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

const TrajectoryRow *NearestTrajectoryRow(
  const std::vector<TrajectoryRow> &trajectory,
  const double time_s) {
  if (trajectory.empty() || !std::isfinite(time_s)) {
    return nullptr;
  }
  const auto upper = std::lower_bound(
    trajectory.begin(),
    trajectory.end(),
    time_s,
    [](const TrajectoryRow &row, const double target_time_s) {
      return row.time_s < target_time_s;
    });
  if (upper == trajectory.begin()) {
    return &trajectory.front();
  }
  if (upper == trajectory.end()) {
    return &trajectory.back();
  }
  const auto &right = *upper;
  const auto &left = *std::prev(upper);
  return std::abs(left.time_s - time_s) <= std::abs(right.time_s - time_s)
    ? &left
    : &right;
}

double StatePeriodS(const double state_frequency_hz) {
  return state_frequency_hz > 0.0 && std::isfinite(state_frequency_hz)
    ? 1.0 / state_frequency_hz
    : 0.0;
}

template <typename Row>
Row *NearestMutableRow(std::vector<Row> &rows, const double time_s) {
  if (rows.empty() || !std::isfinite(time_s)) {
    return nullptr;
  }
  auto upper = std::lower_bound(
    rows.begin(),
    rows.end(),
    time_s,
    [](const Row &row, const double target_time_s) {
      return row.time_s < target_time_s;
    });
  if (upper == rows.begin()) {
    return &rows.front();
  }
  if (upper == rows.end()) {
    return &rows.back();
  }
  auto left = std::prev(upper);
  return std::abs(left->time_s - time_s) <= std::abs(upper->time_s - time_s)
    ? &(*left)
    : &(*upper);
}

Eigen::Vector2d ImuBackPropagatedHorizontalVelocity(
  const DataSet &dataset,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &imu_params,
  const ReferenceNodeState &target_state,
  const Eigen::Vector2d &post_horizontal_velocity_mps,
  const double post_first_time_s) {
  if (imu_params == nullptr ||
      dataset.imu_samples.empty() ||
      !std::isfinite(target_state.time_s) ||
      post_first_time_s <= target_state.time_s + kTimeEpsilonS) {
    return post_horizontal_velocity_mps;
  }
  try {
    const ImuWindowIntegration imu_window = IntegrateImuWindow(
      dataset.imu_samples,
      target_state.time_s,
      post_first_time_s,
      imu_params,
      target_state.bias);
    const gtsam::NavState target_nav_state(
      target_state.pose,
      target_state.velocity);
    const gtsam::NavState predicted_post_state =
      imu_window.preintegrated_measurements.predict(
        target_nav_state,
        target_state.bias);
    const gtsam::Vector3 delta_v_mps =
      predicted_post_state.v() - target_state.velocity;
    if (!delta_v_mps.allFinite()) {
      return post_horizontal_velocity_mps;
    }
    return post_horizontal_velocity_mps -
      Eigen::Vector2d(delta_v_mps.x(), delta_v_mps.y());
  } catch (const std::exception &) {
    return post_horizontal_velocity_mps;
  }
}

}  // namespace

std::vector<double> OutageEndHorizontalHandoffTargetTimes(
  const std::vector<double> &state_timestamps,
  const std::vector<GnssFactorRecord> &gnss_factor_records,
  const RtkOutageWindowRow &source_outage,
  const RtkOutageWindowRow &handoff_outage,
  const double state_frequency_hz,
  const double handoff_guard_duration_s) {
  std::vector<double> target_times;
  if (state_timestamps.empty() ||
      !std::isfinite(source_outage.start_time_s) ||
      !std::isfinite(source_outage.end_time_s) ||
      !std::isfinite(handoff_outage.end_time_s) ||
      handoff_outage.end_time_s <= source_outage.start_time_s + kTimeEpsilonS) {
    return target_times;
  }

  const auto append_unique = [&](const double time_s) {
    if (!std::isfinite(time_s) ||
        time_s <= source_outage.start_time_s + kTimeEpsilonS ||
        time_s >= handoff_outage.end_time_s - kTimeEpsilonS) {
      return;
    }
    const auto existing = std::find_if(
      target_times.begin(),
      target_times.end(),
      [&](const double existing_time_s) {
        return std::abs(existing_time_s - time_s) <= kTimeEpsilonS;
      });
    if (existing == target_times.end()) {
      target_times.push_back(time_s);
    }
  };

  const double state_period_s = StatePeriodS(state_frequency_hz);
  const double minimum_end_guard_duration_s =
    state_period_s > 0.0 ? 2.0 * state_period_s : 0.0;
  const double configured_end_guard_duration_s =
    handoff_guard_duration_s > 0.0 && std::isfinite(handoff_guard_duration_s)
      ? handoff_guard_duration_s
      : 0.0;
  const double end_guard_duration_s =
    std::max(minimum_end_guard_duration_s, configured_end_guard_duration_s);
  const double original_end_guard_start_s =
    source_outage.end_time_s - end_guard_duration_s;
  if (end_guard_duration_s > 0.0) {
    for (const double time_s : state_timestamps) {
      if (time_s >= original_end_guard_start_s - kTimeEpsilonS &&
          time_s < handoff_outage.end_time_s - kTimeEpsilonS) {
        append_unique(time_s);
      }
    }
  }

  const auto original_end_lower = std::lower_bound(
    state_timestamps.begin(),
    state_timestamps.end(),
    source_outage.end_time_s - kTimeEpsilonS);
  if (original_end_lower != state_timestamps.begin()) {
    for (auto it = std::make_reverse_iterator(original_end_lower);
         it != state_timestamps.rend();
         ++it) {
      const double time_s = *it;
      if (!std::isfinite(time_s)) {
        continue;
      }
      if (time_s <= source_outage.start_time_s + kTimeEpsilonS) {
        break;
      }
      append_unique(time_s);
      break;
    }
  }

  for (const double time_s : state_timestamps) {
    if (time_s >= source_outage.end_time_s - kTimeEpsilonS &&
        time_s < handoff_outage.end_time_s - kTimeEpsilonS) {
      append_unique(time_s);
    }
  }

  for (const auto &record : gnss_factor_records) {
    if (!record.factor_used ||
        record.gnss_fix_type != GnssFixType::kRtkFix ||
        !std::isfinite(record.corrected_time_s) ||
        record.corrected_time_s < source_outage.end_time_s - kTimeEpsilonS ||
        record.corrected_time_s >= handoff_outage.end_time_s - kTimeEpsilonS) {
      continue;
    }
    append_unique(SynchronizedFactorStateTime(record));
  }

  if (const std::optional<double> last_kept_time_s =
        LastKeptOutageStateTime(state_timestamps, handoff_outage);
      last_kept_time_s.has_value()) {
    append_unique(*last_kept_time_s);
  }

  std::sort(target_times.begin(), target_times.end());
  return target_times;
}

std::shared_ptr<Stage2VelocityReference> MutableStage2ReferenceCopy(
  const std::shared_ptr<const Stage2VelocityReference> &reference) {
  return reference != nullptr
    ? std::make_shared<Stage2VelocityReference>(*reference)
    : std::make_shared<Stage2VelocityReference>();
}

void ApplyOutageEndHorizontalHandoffToStage2Reference(
  Stage2VelocityReference &reference,
  const DataSet &dataset,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &imu_params,
  const OfflineRunResult &post_result,
  const double post_first_time_s,
  const std::vector<double> &target_times,
  const double state_frequency_hz) {
  if (target_times.empty() ||
      (reference.trajectory.empty() && reference.reference_states.empty())) {
    return;
  }
  const TrajectoryRow *post_first_row =
    NearestTrajectoryRow(post_result.trajectory, post_first_time_s);
  if (post_first_row == nullptr ||
      !post_first_row->enu_position_m.head<2>().allFinite() ||
      !post_first_row->enu_velocity_mps.head<2>().allFinite()) {
    return;
  }
  const Eigen::Vector2d post_position_m =
    post_first_row->enu_position_m.head<2>();
  const Eigen::Vector2d post_velocity_mps =
    post_first_row->enu_velocity_mps.head<2>();
  const double match_tolerance_s = std::max(
    state_frequency_hz > 0.0 ? 0.5 / state_frequency_hz : 0.0,
    5.0e-3);

  for (const double target_time_s : target_times) {
    if (!std::isfinite(target_time_s) ||
        target_time_s >= post_first_time_s - kTimeEpsilonS) {
      continue;
    }
    const double dt_s = post_first_time_s - target_time_s;
    ReferenceNodeState *state =
      NearestMutableRow(reference.reference_states, target_time_s);
    const bool state_matches =
      state != nullptr &&
      std::abs(state->time_s - target_time_s) <= match_tolerance_s;
    const Eigen::Vector2d target_velocity_mps = state_matches
      ? ImuBackPropagatedHorizontalVelocity(
          dataset,
          imu_params,
          *state,
          post_velocity_mps,
          post_first_time_s)
      : post_velocity_mps;
    const Eigen::Vector2d target_position_m =
      post_position_m - 0.5 * dt_s * (target_velocity_mps + post_velocity_mps);

    if (TrajectoryRow *row =
          NearestMutableRow(reference.trajectory, target_time_s);
        row != nullptr &&
        std::abs(row->time_s - target_time_s) <= match_tolerance_s) {
      row->enu_position_m.x() = target_position_m.x();
      row->enu_position_m.y() = target_position_m.y();
      row->enu_velocity_mps.x() = target_velocity_mps.x();
      row->enu_velocity_mps.y() = target_velocity_mps.y();
    }

    if (state_matches) {
      const gtsam::Point3 old_translation = state->pose.translation();
      state->pose = gtsam::Pose3(
        state->pose.rotation(),
        gtsam::Point3(
          target_position_m.x(),
          target_position_m.y(),
          old_translation.z()));
      state->velocity.x() = target_velocity_mps.x();
      state->velocity.y() = target_velocity_mps.y();
    }
  }
}

}  // namespace offline_lc_minimal
