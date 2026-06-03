#include "offline_lc_minimal/core/RtkOutageCausalReferenceBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/GraphTimelineBuilder.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kInterpolatorQcVariance = 10000.0;

gtsam::Pose3 PoseFromTrajectoryRow(const TrajectoryRow &row) {
  return gtsam::Pose3(
    gtsam::Rot3::Ypr(row.ypr_rad.x(), row.ypr_rad.y(), row.ypr_rad.z()),
    gtsam::Point3(
      row.enu_position_m.x(),
      row.enu_position_m.y(),
      row.enu_position_m.z()));
}

gtsam::Vector3 VectorFromEigen(const Eigen::Vector3d &vector) {
  return gtsam::Vector3(vector.x(), vector.y(), vector.z());
}

std::optional<RtkOutageWindowRow> EarliestOutageWindow(
  const std::vector<RtkOutageWindowRow> &windows) {
  std::optional<RtkOutageWindowRow> best;
  for (const auto &window : windows) {
    if (!std::isfinite(window.start_time_s) ||
        !std::isfinite(window.end_time_s) ||
        window.end_time_s <= window.start_time_s) {
      continue;
    }
    if (!best.has_value() || window.start_time_s < best->start_time_s) {
      best = window;
    }
  }
  return best;
}

std::vector<RtkOutageCausalStateReferenceRow> BuildStateReferenceRows(
  const std::vector<TrajectoryRow> &trajectory,
  const double boundary_time_s) {
  std::vector<RtkOutageCausalStateReferenceRow> rows;
  rows.reserve(trajectory.size());
  for (std::size_t state_index = 0; state_index < trajectory.size(); ++state_index) {
    const auto &trajectory_row = trajectory[state_index];
    if (trajectory_row.time_s > boundary_time_s + kTimeEpsilonS) {
      continue;
    }
    RtkOutageCausalStateReferenceRow row;
    row.state_index = state_index;
    row.time_s = trajectory_row.time_s;
    row.reference_up_m = trajectory_row.enu_position_m.z();
    row.reference_vz_mps = trajectory_row.enu_velocity_mps.z();
    row.source_processing_end_time_s = boundary_time_s;
    row.outage_boundary_time_s = boundary_time_s;
    row.valid = true;
    row.source_type = "PREFIX_SOLVE";
    row.skip_reason = "OK";
    rows.push_back(row);
  }
  return rows;
}

std::optional<std::pair<double, double>> NavReferenceFromPrefixTrajectory(
  const std::vector<TrajectoryRow> &trajectory,
  const std::map<std::size_t, double> &state_timestamp_map,
  const OfflineRunnerConfig &config,
  const double corrected_time_s,
  RtkOutageCausalNavReferenceRow *row) {
  const StateMeasSyncResult sync =
    FindStateForMeasurement(state_timestamp_map, corrected_time_s, config);
  row->sync_status = sync.status;
  row->source_state_index_i = sync.found_i ? static_cast<long long>(sync.key_index_i) : -1;
  row->source_state_index_j = sync.found_i ? static_cast<long long>(sync.key_index_j) : -1;
  row->state_time_i_s = sync.timestamp_i_s;
  row->state_time_j_s = sync.timestamp_j_s;
  row->duration_from_state_i_s = sync.duration_from_state_i_s;

  if (sync.status == StateMeasSyncStatus::kSynchronizedI ||
      sync.status == StateMeasSyncStatus::kSynchronizedJ) {
    const std::size_t state_index =
      sync.status == StateMeasSyncStatus::kSynchronizedI ? sync.key_index_i : sync.key_index_j;
    if (state_index >= trajectory.size()) {
      row->skip_reason = "missing_prefix_state";
      return std::nullopt;
    }
    return std::make_pair(
      trajectory[state_index].enu_position_m.z(),
      trajectory[state_index].enu_velocity_mps.z());
  }
  if (sync.status != StateMeasSyncStatus::kInterpolated ||
      sync.key_index_i >= trajectory.size() ||
      sync.key_index_j >= trajectory.size()) {
    row->skip_reason = "unsupported_prefix_sync";
    return std::nullopt;
  }

  const auto qc_model =
    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance));
  gp::GPWNOJInterpolator interpolator(
    qc_model,
    sync.timestamp_j_s - sync.timestamp_i_s,
    sync.duration_from_state_i_s);
  const auto &row_i = trajectory[sync.key_index_i];
  const auto &row_j = trajectory[sync.key_index_j];
  const gtsam::Pose3 pose = interpolator.InterpolatePose(
    PoseFromTrajectoryRow(row_i),
    VectorFromEigen(row_i.enu_velocity_mps),
    VectorFromEigen(row_i.omega_radps),
    PoseFromTrajectoryRow(row_j),
    VectorFromEigen(row_j.enu_velocity_mps),
    VectorFromEigen(row_j.omega_radps));
  const double dt_s = sync.timestamp_j_s - sync.timestamp_i_s;
  const double alpha =
    dt_s > kTimeEpsilonS ? std::clamp(sync.duration_from_state_i_s / dt_s, 0.0, 1.0) : 0.0;
  const double vz_mps =
    (1.0 - alpha) * row_i.enu_velocity_mps.z() + alpha * row_j.enu_velocity_mps.z();
  return std::make_pair(pose.translation().z(), vz_mps);
}

std::vector<RtkOutageCausalNavReferenceRow> BuildNavReferenceRows(
  const RtkOutageCausalReferenceBuildRequest &request,
  const OfflineRunResult &prefix_result,
  const double boundary_time_s) {
  std::vector<RtkOutageCausalNavReferenceRow> rows(request.dataset->gnss_samples.size());
  std::map<std::size_t, double> state_timestamp_map;
  for (std::size_t state_index = 0; state_index < prefix_result.trajectory.size(); ++state_index) {
    state_timestamp_map.emplace(state_index, prefix_result.trajectory[state_index].time_s);
  }

  for (std::size_t sample_index = 0; sample_index < request.dataset->gnss_samples.size(); ++sample_index) {
    const auto &sample = request.dataset->gnss_samples[sample_index];
    auto &row = rows[sample_index];
    row.sample_index = sample_index;
    row.raw_rtk_up_m = sample.enu_position_m.z();
    row.outage_boundary_time_s = boundary_time_s;
    row.source_processing_end_time_s = boundary_time_s;
    if (!request.corrected_time_s) {
      row.skip_reason = "missing_time_callback";
      continue;
    }
    const double corrected_time_s = request.corrected_time_s(sample);
    row.time_s = corrected_time_s;
    if (corrected_time_s > boundary_time_s + kTimeEpsilonS) {
      row.skip_reason = "after_causal_boundary";
      continue;
    }
    if (!sample.has_enu_position || !std::isfinite(sample.enu_position_m.z())) {
      row.skip_reason = "invalid_up";
      continue;
    }
    if (request.should_use_sample && !request.should_use_sample(sample)) {
      row.skip_reason = "filtered_gnss_sample";
      continue;
    }
    if (request.is_within_imu_coverage && !request.is_within_imu_coverage(corrected_time_s)) {
      row.skip_reason = "out_of_imu_coverage";
      continue;
    }
    const auto reference = NavReferenceFromPrefixTrajectory(
      prefix_result.trajectory,
      state_timestamp_map,
      *request.config,
      corrected_time_s,
      &row);
    if (!reference.has_value()) {
      if (row.skip_reason == "UNSET") {
        row.skip_reason = "missing_prefix_reference";
      }
      continue;
    }
    row.causal_nav_reference_up_m = reference->first;
    row.causal_up_m = reference->first;
    row.causal_vz_mps = reference->second;
    row.source_type = "PREFIX_SOLVE";
    row.valid = true;
    row.skip_reason = "OK";
  }
  return rows;
}

}  // namespace

RtkOutageCausalReferenceBuilder::RtkOutageCausalReferenceBuilder(
  RtkOutageCausalReferenceBuildRequest request)
    : request_(std::move(request)) {}

RtkOutageCausalReferenceResult RtkOutageCausalReferenceBuilder::Build() const {
  if (request_.config == nullptr || request_.dataset == nullptr ||
      request_.outage_windows == nullptr) {
    throw std::runtime_error("RtkOutageCausalReferenceBuilder received an incomplete request");
  }

  RtkOutageCausalReferenceResult result;
  if (!request_.config->enable_rtk_outage_causal_drift_reference ||
      request_.config->rtk_outage_causal_reference_max_prefix_runs <= 0 ||
      request_.outage_windows->empty()) {
    return result;
  }
  if (!request_.run_prefix) {
    throw std::runtime_error("RtkOutageCausalReferenceBuilder requires a prefix run callback");
  }

  const std::optional<RtkOutageWindowRow> first_outage =
    EarliestOutageWindow(*request_.outage_windows);
  if (!first_outage.has_value() ||
      first_outage->start_time_s <= request_.dynamic_start_time_s + kTimeEpsilonS) {
    return result;
  }

  OfflineRunnerConfig prefix_config =
    request_.prefix_base_config != nullptr
      ? *request_.prefix_base_config
      : *request_.config;
  prefix_config.processing_end_time_s = first_outage->start_time_s;
  prefix_config.enable_rtk_outage_causal_drift_reference = false;
  prefix_config.enable_rtk_outage_preoutage_vertical_fence = false;
  prefix_config.rtk_outage_causal_reference_max_prefix_runs = 0;

  OfflineRunResult prefix_result =
    request_.run_prefix(prefix_config, *request_.dataset);
  if (prefix_result.trajectory.empty()) {
    throw std::runtime_error("causal prefix run returned an empty trajectory");
  }

  result.valid = true;
  result.prefix_run_count = 1U;
  result.boundary_time_s = first_outage->start_time_s;
  result.state_reference_rows =
    BuildStateReferenceRows(prefix_result.trajectory, first_outage->start_time_s);
  result.nav_reference_rows =
    BuildNavReferenceRows(request_, prefix_result, first_outage->start_time_s);
  return result;
}

}  // namespace offline_lc_minimal
