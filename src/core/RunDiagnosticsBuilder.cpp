#include "offline_lc_minimal/core/RunDiagnosticsBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <unordered_map>

#include <gtsam/inference/Symbol.h>

#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/core/TrajectoryResultBuilder.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimeEpsilonS = 1e-9;

struct SegmentGnssStatsAccumulator {
  std::size_t gnss_factor_count = 0;
  double postfit_nis_sum = 0.0;
  std::size_t postfit_nis_count = 0;
};

struct ScalarSummaryStats {
  double mean = std::numeric_limits<double>::quiet_NaN();
  double median = std::numeric_limits<double>::quiet_NaN();
  double p95 = std::numeric_limits<double>::quiet_NaN();
};

Eigen::Vector3d ComputePositionResidualEnu(
  const gtsam::Pose3 &pose,
  const Eigen::Vector3d &measurement_enu_m) {
  return Eigen::Vector3d(
           pose.translation().x(),
           pose.translation().y(),
           pose.translation().z()) -
         measurement_enu_m;
}

double ComputeNis(const Eigen::Vector3d &residual_enu_m, const Eigen::Vector3d &sigma_m) {
  Eigen::Vector3d variances = sigma_m.array().square().matrix();
  for (int axis = 0; axis < variances.size(); ++axis) {
    variances[axis] = std::max(variances[axis], 1e-12);
  }
  return residual_enu_m.x() * residual_enu_m.x() / variances.x() +
         residual_enu_m.y() * residual_enu_m.y() / variances.y() +
         residual_enu_m.z() * residual_enu_m.z() / variances.z();
}

double ComputeHorizontalNis(const Eigen::Vector3d &residual_enu_m, const Eigen::Vector2d &sigma_m) {
  Eigen::Vector2d variances = sigma_m.array().square().matrix();
  for (int axis = 0; axis < variances.size(); ++axis) {
    variances[axis] = std::max(variances[axis], 1e-12);
  }
  return residual_enu_m.x() * residual_enu_m.x() / variances.x() +
         residual_enu_m.y() * residual_enu_m.y() / variances.y();
}

ScalarSummaryStats ComputeScalarSummaryStats(std::vector<double> values) {
  ScalarSummaryStats stats;
  values.erase(
    std::remove_if(values.begin(), values.end(), [](const double value) { return !std::isfinite(value); }),
    values.end());
  if (values.empty()) {
    return stats;
  }
  std::sort(values.begin(), values.end());
  const double sum = std::accumulate(values.begin(), values.end(), 0.0);
  stats.mean = sum / static_cast<double>(values.size());
  const std::size_t median_index = values.size() / 2U;
  if (values.size() % 2U == 0U) {
    stats.median = 0.5 * (values[median_index - 1U] + values[median_index]);
  } else {
    stats.median = values[median_index];
  }
  const double p95_index = 0.95 * static_cast<double>(values.size() - 1U);
  const auto lower_index = static_cast<std::size_t>(std::floor(p95_index));
  const auto upper_index = static_cast<std::size_t>(std::ceil(p95_index));
  const double alpha = p95_index - static_cast<double>(lower_index);
  stats.p95 = (1.0 - alpha) * values[lower_index] + alpha * values[upper_index];
  return stats;
}

double ComputeTotalVariation(const std::vector<double> &values) {
  if (values.size() < 2U) {
    return 0.0;
  }
  double total_variation = 0.0;
  for (std::size_t index = 0; index + 1U < values.size(); ++index) {
    total_variation += std::abs(values[index + 1U] - values[index]);
  }
  return total_variation;
}

double ComputePopulationStdDev(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double mean = std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
  double variance = 0.0;
  for (const double value : values) {
    const double centered = value - mean;
    variance += centered * centered;
  }
  return std::sqrt(variance / static_cast<double>(values.size()));
}

double ComputeBiasDecay(const double dt_s, const double tau_s) {
  return std::exp(-std::max(dt_s, 0.0) / std::max(tau_s, 1e-9));
}

gtsam::Matrix3 ComputeBiasPhi(const double dt_s, const double tau_s) {
  return ComputeBiasDecay(dt_s, tau_s) * gtsam::I_3x3;
}

bool IsSynchronizedStatus(const StateMeasSyncStatus status) {
  return status == StateMeasSyncStatus::kSynchronizedI || status == StateMeasSyncStatus::kSynchronizedJ;
}

std::optional<std::size_t> ResolveSegmentIndexForRecord(const GnssFactorRecord &record) {
  if (IsSynchronizedStatus(record.sync_status) && record.synchronized_state_index > 0U) {
    return record.synchronized_state_index - 1U;
  }
  if (record.sync_status == StateMeasSyncStatus::kInterpolated) {
    return record.state_index_i;
  }
  return std::nullopt;
}

}  // namespace

void PopulateGnssPostfitResiduals(
  const gtsam::Values &optimized_values,
  const gp::GPWNOJInterpolator &base_interpolator,
  const std::vector<std::optional<std::size_t>> &trajectory_row_index_by_state,
  std::vector<GnssFactorRecord> &gnss_factor_records,
  std::vector<GnssConsistencyRecord> *gnss_consistency_records,
  std::vector<TrajectoryRow> *trajectory_rows) {
  for (std::size_t record_index = 0; record_index < gnss_factor_records.size(); ++record_index) {
    auto &record = gnss_factor_records[record_index];
    GnssConsistencyRecord *consistency_record =
      gnss_consistency_records != nullptr && record_index < gnss_consistency_records->size()
        ? &(*gnss_consistency_records)[record_index]
        : nullptr;
    if (!record.factor_used) {
      continue;
    }

    if (record.sync_status == StateMeasSyncStatus::kSynchronizedI ||
        record.sync_status == StateMeasSyncStatus::kSynchronizedJ) {
      const auto pose = optimized_values.at<gtsam::Pose3>(X(record.synchronized_state_index));
      const Eigen::Vector3d residual_enu_m = ComputePositionResidualEnu(pose, record.measurement_enu_m);
      const bool use_vertical_direct_position =
        consistency_record != nullptr
          ? consistency_record->vertical_direct_position_factor_used
          : record.vertical_direct_position_factor_used;
      record.residual_m =
        use_vertical_direct_position ? residual_enu_m.norm() : residual_enu_m.head<2>().norm();
      if (consistency_record != nullptr) {
        consistency_record->postfit_residual_enu_m = residual_enu_m;
        const Eigen::Vector2d scaled_horizontal_sigma_m(
          consistency_record->sigma_e_m,
          consistency_record->sigma_n_m);
        if (use_vertical_direct_position) {
          const Eigen::Vector3d scaled_sigma_m(
            scaled_horizontal_sigma_m.x(),
            scaled_horizontal_sigma_m.y(),
            consistency_record->vertical_sigma_u_used_m);
          consistency_record->postfit_nis = ComputeNis(residual_enu_m, scaled_sigma_m);
        } else {
          consistency_record->postfit_nis = ComputeHorizontalNis(residual_enu_m, scaled_horizontal_sigma_m);
        }
      }
      if (trajectory_rows != nullptr &&
          record.synchronized_state_index < trajectory_row_index_by_state.size() &&
          trajectory_row_index_by_state[record.synchronized_state_index].has_value()) {
        auto &row = (*trajectory_rows)[*trajectory_row_index_by_state[record.synchronized_state_index]];
        if (!std::isfinite(row.gnss_residual_m)) {
          row.gnss_residual_m = record.residual_m;
        }
      }
    } else if (record.sync_status == StateMeasSyncStatus::kInterpolated) {
      gp::GPWNOJInterpolator interpolator = base_interpolator;
      interpolator.Recalculate(
        record.state_time_j_s - record.state_time_i_s,
        record.duration_from_state_i_s);
      const gtsam::Pose3 interpolated_pose = interpolator.InterpolatePose(
        optimized_values.at<gtsam::Pose3>(X(record.state_index_i)),
        optimized_values.at<gtsam::Vector3>(V(record.state_index_i)),
        optimized_values.at<gtsam::Vector3>(W(record.state_index_i)),
        optimized_values.at<gtsam::Pose3>(X(record.state_index_j)),
        optimized_values.at<gtsam::Vector3>(V(record.state_index_j)),
        optimized_values.at<gtsam::Vector3>(W(record.state_index_j)));
      const Eigen::Vector3d residual_enu_m = ComputePositionResidualEnu(interpolated_pose, record.measurement_enu_m);
      const bool use_vertical_direct_position =
        consistency_record != nullptr
          ? consistency_record->vertical_direct_position_factor_used
          : record.vertical_direct_position_factor_used;
      record.residual_m =
        use_vertical_direct_position ? residual_enu_m.norm() : residual_enu_m.head<2>().norm();
      if (consistency_record != nullptr) {
        consistency_record->postfit_residual_enu_m = residual_enu_m;
        const Eigen::Vector2d scaled_horizontal_sigma_m(
          consistency_record->sigma_e_m,
          consistency_record->sigma_n_m);
        if (use_vertical_direct_position) {
          const Eigen::Vector3d scaled_sigma_m(
            scaled_horizontal_sigma_m.x(),
            scaled_horizontal_sigma_m.y(),
            consistency_record->vertical_sigma_u_used_m);
          consistency_record->postfit_nis = ComputeNis(residual_enu_m, scaled_sigma_m);
        } else {
          consistency_record->postfit_nis = ComputeHorizontalNis(residual_enu_m, scaled_horizontal_sigma_m);
        }
      }
      if (trajectory_rows != nullptr &&
          record.state_index_i < trajectory_row_index_by_state.size() &&
          trajectory_row_index_by_state[record.state_index_i].has_value()) {
        auto &row_i = (*trajectory_rows)[*trajectory_row_index_by_state[record.state_index_i]];
        if (!std::isfinite(row_i.gnss_residual_m)) {
          row_i.gnss_residual_m = record.residual_m;
        }
      }
      if (trajectory_rows != nullptr &&
          record.state_index_j < trajectory_row_index_by_state.size() &&
          trajectory_row_index_by_state[record.state_index_j].has_value()) {
        auto &row_j = (*trajectory_rows)[*trajectory_row_index_by_state[record.state_index_j]];
        if (!std::isfinite(row_j.gnss_residual_m)) {
          row_j.gnss_residual_m = record.residual_m;
        }
      }
    }
  }
}

void PopulateVerticalEnvelopeDiagnostics(
  const gtsam::Values &optimized_values,
  const gp::GPWNOJInterpolator &base_interpolator,
  std::vector<VerticalEnvelopeDiagnosticRow> &vertical_envelope_diagnostics) {
  for (auto &row : vertical_envelope_diagnostics) {
    if (!row.factor_used) {
      continue;
    }

    std::optional<double> predicted_up_m;
    if (row.sync_status == StateMeasSyncStatus::kSynchronizedI ||
        row.sync_status == StateMeasSyncStatus::kSynchronizedJ) {
      const auto pose = optimized_values.at<gtsam::Pose3>(X(row.synchronized_state_index));
      predicted_up_m = pose.translation().z();
    } else if (row.sync_status == StateMeasSyncStatus::kInterpolated) {
      gp::GPWNOJInterpolator interpolator = base_interpolator;
      interpolator.Recalculate(
        row.state_time_j_s - row.state_time_i_s,
        row.duration_from_state_i_s);
      const gtsam::Pose3 interpolated_pose = interpolator.InterpolatePose(
        optimized_values.at<gtsam::Pose3>(X(row.state_index_i)),
        optimized_values.at<gtsam::Vector3>(V(row.state_index_i)),
        optimized_values.at<gtsam::Vector3>(W(row.state_index_i)),
        optimized_values.at<gtsam::Pose3>(X(row.state_index_j)),
        optimized_values.at<gtsam::Vector3>(V(row.state_index_j)),
        optimized_values.at<gtsam::Vector3>(W(row.state_index_j)));
      predicted_up_m = interpolated_pose.translation().z();
    }

    if (!predicted_up_m.has_value()) {
      continue;
    }
    row.predicted_up_m = *predicted_up_m;
    row.raw_residual_m = row.predicted_up_m - row.rtk_up_m;
    row.violation_m = factor::VerticalEnvelopeResidual(row.raw_residual_m, row.half_width_m);
    row.inside_envelope = std::abs(row.violation_m) <= 1e-12;
    if (row.center_pull_factor_used) {
      row.center_pull_residual_m =
        factor::VerticalEnvelopeCenterResidual(
          row.raw_residual_m,
          row.half_width_m,
          row.center_pull_deadband_m);
    }
  }
}

ForwardDriftSummary ComputeFeedbackForwardDriftSummary(
  const std::vector<ImuSample> &imu_samples,
  const gtsam::Pose3 &start_pose,
  const gtsam::Vector3 &start_velocity,
  const gtsam::imuBias::ConstantBias &bias,
  const double start_time_s,
  const double max_time_s,
  const double gravity_mps2) {
  ForwardDriftSummary summary;
  if (imu_samples.size() < 2U || max_time_s <= start_time_s + kTimeEpsilonS) {
    return summary;
  }

  const Eigen::Vector3d initial_position(
    start_pose.translation().x(),
    start_pose.translation().y(),
    start_pose.translation().z());
  Eigen::Vector3d current_position = initial_position;
  Eigen::Vector3d current_velocity(start_velocity.x(), start_velocity.y(), start_velocity.z());
  gtsam::Rot3 current_rotation = start_pose.rotation();
  const Eigen::Vector3d gravity_enu(0.0, 0.0, gravity_mps2);
  bool reached_10s = false;
  bool reached_30s = false;

  const auto fill_summary =
    [&](const double duration_s, const Eigen::Vector3d &position) {
      const Eigen::Vector3d delta = position - initial_position;
      if (std::abs(duration_s - 10.0) <= 1e-6) {
        summary.up_slope_10s = delta.z() / duration_s;
        summary.horizontal_slope_10s = delta.head<2>().norm() / duration_s;
      } else if (std::abs(duration_s - 30.0) <= 1e-6) {
        summary.up_slope_30s = delta.z() / duration_s;
        summary.horizontal_slope_30s = delta.head<2>().norm() / duration_s;
      }
    };

  for (std::size_t imu_index = 0; imu_index + 1U < imu_samples.size(); ++imu_index) {
    const auto &current_sample = imu_samples[imu_index];
    const auto &next_sample = imu_samples[imu_index + 1U];
    const double interval_start_s = std::max(current_sample.time_s, start_time_s);
    const double interval_end_s = std::min(next_sample.time_s, max_time_s);
    if (interval_end_s <= interval_start_s + kTimeEpsilonS) {
      continue;
    }

    const double delta_time_s = interval_end_s - interval_start_s;
    const Eigen::Vector3d corrected_gyro = current_sample.gyro_radps - bias.gyroscope();
    const Eigen::Vector3d corrected_acc = current_sample.accel_mps2 - bias.accelerometer();
    const Eigen::Vector3d nav_specific_force = current_rotation.matrix() * corrected_acc;
    const Eigen::Vector3d nav_acc = nav_specific_force - gravity_enu;

    current_position += current_velocity * delta_time_s + 0.5 * nav_acc * delta_time_s * delta_time_s;
    current_velocity += nav_acc * delta_time_s;
    current_rotation =
      current_rotation.compose(gtsam::Rot3::Expmap(gtsam::Vector3(
        corrected_gyro.x() * delta_time_s,
        corrected_gyro.y() * delta_time_s,
        corrected_gyro.z() * delta_time_s)));

    const double elapsed_s = interval_end_s - start_time_s;
    if (!reached_10s && elapsed_s >= 10.0 - kTimeEpsilonS) {
      fill_summary(10.0, current_position);
      reached_10s = true;
    }
    if (!reached_30s && elapsed_s >= 30.0 - kTimeEpsilonS) {
      fill_summary(30.0, current_position);
      reached_30s = true;
      break;
    }
  }

  return summary;
}

void AccumulateForwardTrajectoryVariationSummary(
  const std::vector<TrajectoryRow> &rows,
  double &up_total_variation_m,
  double &vz_total_variation_mps) {
  if (rows.empty()) {
    return;
  }

  std::vector<double> up_values;
  std::vector<double> vz_values;
  up_values.reserve(rows.size());
  vz_values.reserve(rows.size());
  for (const auto &row : rows) {
    up_values.push_back(row.enu_position_m.z());
    vz_values.push_back(row.enu_velocity_mps.z());
  }
  up_total_variation_m = ComputeTotalVariation(up_values);
  vz_total_variation_mps = ComputeTotalVariation(vz_values);
}

void AccumulateInitialDynamicConsistencyMetrics(
  const std::vector<TrajectoryRow> &trajectory_rows,
  const gtsam::imuBias::ConstantBias &initial_bias,
  const std::optional<TrajectoryRow> &optimized_last_static_row,
  const std::vector<TrajectoryRow> &optimized_static_terminal_forward_rows,
  RunSummary &run_summary) {
  run_summary.initial_baz_mps2 = initial_bias.accelerometer().z();
  run_summary.initial_bgz_radps = initial_bias.gyroscope().z();

  if (optimized_last_static_row.has_value()) {
    run_summary.static_baz_mps2 = optimized_last_static_row->bias_acc.z();
    run_summary.static_bgz_radps = optimized_last_static_row->bias_gyro.z();
    run_summary.optimized_last_static_baz_mps2 = optimized_last_static_row->bias_acc.z();
    run_summary.optimized_last_static_bgz_radps = optimized_last_static_row->bias_gyro.z();
  }

  if (trajectory_rows.empty()) {
    AccumulateForwardTrajectoryVariationSummary(
      optimized_static_terminal_forward_rows,
      run_summary.optimized_static_terminal_forward20s_up_total_variation_m,
      run_summary.optimized_static_terminal_forward20s_vz_total_variation_mps);
    return;
  }

  run_summary.optimized_first_dynamic_baz_mps2 = trajectory_rows.front().bias_acc.z();
  run_summary.optimized_first_dynamic_bgz_radps = trajectory_rows.front().bias_gyro.z();
  run_summary.bootstrap_to_optimized_first_dynamic_baz_delta_mps2 =
    run_summary.optimized_first_dynamic_baz_mps2 - run_summary.initial_baz_mps2;
  if (std::isfinite(run_summary.optimized_last_static_baz_mps2)) {
    run_summary.static_to_dynamic_baz_delta_mps2 =
      run_summary.optimized_first_dynamic_baz_mps2 - run_summary.optimized_last_static_baz_mps2;
  }
  const double start_time_s = trajectory_rows.front().time_s;
  const double end_time_s = start_time_s + 30.0;
  std::vector<double> baz_values;
  std::vector<double> bgz_values;
  std::vector<double> roll_values;
  std::vector<double> pitch_values;
  std::vector<double> yaw_values;
  std::vector<double> up_values;
  std::vector<double> vz_values;

  for (const auto &row : trajectory_rows) {
    if (row.time_s > end_time_s + kTimeEpsilonS) {
      break;
    }
    baz_values.push_back(row.bias_acc.z());
    bgz_values.push_back(row.bias_gyro.z());
    yaw_values.push_back(row.ypr_rad.x());
    pitch_values.push_back(row.ypr_rad.y());
    roll_values.push_back(row.ypr_rad.z());
    up_values.push_back(row.enu_position_m.z());
    vz_values.push_back(row.enu_velocity_mps.z());
  }

  if (!baz_values.empty()) {
    run_summary.optimized_first30s_mean_baz_mps2 =
      std::accumulate(baz_values.begin(), baz_values.end(), 0.0) / static_cast<double>(baz_values.size());
    run_summary.optimized_first30s_mean_bgz_radps =
      std::accumulate(bgz_values.begin(), bgz_values.end(), 0.0) / static_cast<double>(bgz_values.size());
    run_summary.optimized_first30s_mean_roll_rad =
      std::accumulate(roll_values.begin(), roll_values.end(), 0.0) / static_cast<double>(roll_values.size());
    run_summary.optimized_first30s_mean_pitch_rad =
      std::accumulate(pitch_values.begin(), pitch_values.end(), 0.0) / static_cast<double>(pitch_values.size());
    run_summary.optimized_first30s_mean_yaw_rad =
      std::accumulate(yaw_values.begin(), yaw_values.end(), 0.0) / static_cast<double>(yaw_values.size());
    run_summary.optimized_first30s_std_baz_mps2 = ComputePopulationStdDev(baz_values);
    run_summary.optimized_first30s_std_pitch_rad = ComputePopulationStdDev(pitch_values);
    run_summary.optimized_first30s_std_roll_rad = ComputePopulationStdDev(roll_values);
    run_summary.optimized_first30s_up_total_variation_m = ComputeTotalVariation(up_values);
    run_summary.optimized_first30s_vz_total_variation_mps = ComputeTotalVariation(vz_values);
  }
  AccumulateForwardTrajectoryVariationSummary(
    optimized_static_terminal_forward_rows,
    run_summary.optimized_static_terminal_forward20s_up_total_variation_m,
    run_summary.optimized_static_terminal_forward20s_vz_total_variation_mps);
}

std::vector<SegmentErrorDiagnostic> BuildSegmentErrorDiagnostics(
  const std::vector<double> &state_timestamps,
  const std::vector<ImuSample> &imu_samples,
  const boost::shared_ptr<gtsam::PreintegrationCombinedParams> &imu_params,
  const gtsam::Values &optimized_values,
  const std::vector<GnssFactorRecord> &gnss_factor_records,
  const std::vector<GnssConsistencyRecord> &gnss_consistency_records,
  const OfflineRunnerConfig &config) {
  if (state_timestamps.size() < 2U) {
    return {};
  }

  std::unordered_map<std::size_t, SegmentGnssStatsAccumulator> gnss_stats_by_segment;
  for (std::size_t record_index = 0; record_index < gnss_factor_records.size(); ++record_index) {
    const auto &record = gnss_factor_records[record_index];
    if (!record.factor_used) {
      continue;
    }

    const auto segment_index = ResolveSegmentIndexForRecord(record);
    if (!segment_index.has_value()) {
      continue;
    }

    auto &accumulator = gnss_stats_by_segment[*segment_index];
    ++accumulator.gnss_factor_count;
    if (record_index < gnss_consistency_records.size()) {
      const auto &consistency_record = gnss_consistency_records[record_index];
      if (std::isfinite(consistency_record.postfit_nis)) {
        accumulator.postfit_nis_sum += consistency_record.postfit_nis;
        ++accumulator.postfit_nis_count;
      }
    }
  }

  std::vector<SegmentErrorDiagnostic> diagnostics;
  diagnostics.reserve(state_timestamps.size() - 1U);
  for (std::size_t state_index = 1; state_index < state_timestamps.size(); ++state_index) {
    const std::size_t segment_index = state_index - 1U;
    const double start_time_s = state_timestamps[segment_index];
    const double end_time_s = state_timestamps[state_index];
    const auto start_pose = optimized_values.at<gtsam::Pose3>(X(segment_index));
    const auto start_velocity = optimized_values.at<gtsam::Vector3>(V(segment_index));
    const auto start_bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(segment_index));
    const auto end_pose = optimized_values.at<gtsam::Pose3>(X(state_index));
    const auto end_velocity = optimized_values.at<gtsam::Vector3>(V(state_index));
    const auto end_bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index));
    const auto imu_window =
      IntegrateImuWindow(imu_samples, start_time_s, end_time_s, imu_params, start_bias);
    const gtsam::NavState predicted_state =
      imu_window.preintegrated_measurements.predict(gtsam::NavState(start_pose, start_velocity), start_bias);
    SegmentErrorDiagnostic diagnostic;
    diagnostic.segment_index = segment_index;
    diagnostic.start_time_s = start_time_s;
    diagnostic.end_time_s = end_time_s;
    diagnostic.dtheta_rad = gtsam::Rot3::Logmap(predicted_state.pose().rotation().between(end_pose.rotation()));
    diagnostic.dv_mps = end_velocity - predicted_state.v();
    diagnostic.dp_m = Eigen::Vector3d(
      end_pose.translation().x() - predicted_state.position().x(),
      end_pose.translation().y() - predicted_state.position().y(),
      end_pose.translation().z() - predicted_state.position().z());
    const double delta_time_s = end_time_s - start_time_s;
    diagnostic.dba_mps2 =
      end_bias.accelerometer() - ComputeBiasPhi(delta_time_s, config.tau_acc_bias_s) * start_bias.accelerometer();
    diagnostic.dbg_radps =
      end_bias.gyroscope() - ComputeBiasPhi(delta_time_s, config.tau_gyro_bias_s) * start_bias.gyroscope();

    if (const auto stats_it = gnss_stats_by_segment.find(segment_index); stats_it != gnss_stats_by_segment.end()) {
      const auto &stats = stats_it->second;
      diagnostic.gnss_factor_count = stats.gnss_factor_count;
      diagnostic.mean_postfit_nis =
        stats.postfit_nis_count > 0U
          ? stats.postfit_nis_sum / static_cast<double>(stats.postfit_nis_count)
          : std::numeric_limits<double>::quiet_NaN();
    }

    diagnostics.push_back(diagnostic);
  }
  return diagnostics;
}

void AccumulateStaticConsistencyMetrics(
  const gtsam::Values &optimized_values,
  const GraphTimeline &graph_timeline,
  RunSummary &run_summary) {
  if (graph_timeline.initial_static_state_count == 0U) {
    return;
  }

  const std::size_t last_static_index = std::min(
    graph_timeline.initial_static_state_count - 1U,
    graph_timeline.timestamps_s.size() - 1U);
  if (last_static_index >= graph_timeline.timestamps_s.size()) {
    return;
  }

  const gtsam::Pose3 reference_pose = optimized_values.at<gtsam::Pose3>(X(0));
  const auto first_static_bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(0));
  run_summary.optimized_first_static_baz_mps2 = first_static_bias.accelerometer().z();
  run_summary.optimized_first_static_bgz_radps = first_static_bias.gyroscope().z();
  const Eigen::Vector3d reference_position(
    reference_pose.translation().x(),
    reference_pose.translation().y(),
    reference_pose.translation().z());

  std::vector<double> velocity_norms;
  velocity_norms.reserve(last_static_index + 1U);
  double max_horizontal_drift_m = 0.0;
  double max_up_drift_m = 0.0;
  double max_3d_drift_m = 0.0;

  for (std::size_t state_index = 0; state_index <= last_static_index; ++state_index) {
    const auto pose = optimized_values.at<gtsam::Pose3>(X(state_index));
    const auto velocity = optimized_values.at<gtsam::Vector3>(V(state_index));
    const Eigen::Vector3d position(
      pose.translation().x(),
      pose.translation().y(),
      pose.translation().z());
    const Eigen::Vector3d delta = position - reference_position;

    max_horizontal_drift_m = std::max(max_horizontal_drift_m, delta.head<2>().norm());
    max_up_drift_m = std::max(max_up_drift_m, std::abs(delta.z()));
    max_3d_drift_m = std::max(max_3d_drift_m, delta.norm());
    velocity_norms.push_back(velocity.norm());
  }

  if (!velocity_norms.empty()) {
    const double mean_velocity_norm =
      std::accumulate(velocity_norms.begin(), velocity_norms.end(), 0.0) /
      static_cast<double>(velocity_norms.size());
    double variance = 0.0;
    for (const double value : velocity_norms) {
      const double delta = value - mean_velocity_norm;
      variance += delta * delta;
    }
    variance /= static_cast<double>(velocity_norms.size());
    run_summary.initial_static_velocity_norm_mean_mps = mean_velocity_norm;
    run_summary.initial_static_velocity_norm_std_mps = std::sqrt(variance);
    run_summary.initial_static_velocity_norm_max_mps =
      *std::max_element(velocity_norms.begin(), velocity_norms.end());
  }
  run_summary.initial_static_horizontal_drift_max_m = max_horizontal_drift_m;
  run_summary.initial_static_up_drift_max_m = max_up_drift_m;
  run_summary.initial_static_3d_drift_max_m = max_3d_drift_m;
}

void AccumulateStaticSpecificForceWindowMetrics(
  const std::vector<ImuSample> &imu_samples,
  const double start_time_s,
  const double end_time_s,
  const double window_duration_s,
  RunSummary &run_summary) {
  if (imu_samples.empty() || end_time_s <= start_time_s + kTimeEpsilonS || window_duration_s <= 0.0) {
    return;
  }

  const auto imu_begin = std::lower_bound(
    imu_samples.begin(),
    imu_samples.end(),
    start_time_s,
    [](const ImuSample &sample, const double time_s) { return sample.time_s < time_s; });
  std::size_t sample_index = static_cast<std::size_t>(std::distance(imu_samples.begin(), imu_begin));
  std::vector<Eigen::Vector3d> window_means;

  for (double window_start_s = start_time_s; window_start_s < end_time_s - kTimeEpsilonS;
       window_start_s += window_duration_s) {
    const double window_end_s = std::min(window_start_s + window_duration_s, end_time_s);
    const bool is_last_window = window_end_s >= end_time_s - kTimeEpsilonS;
    Eigen::Vector3d sum_acc_mps2 = Eigen::Vector3d::Zero();
    std::size_t sample_count = 0U;

    while (sample_index < imu_samples.size() && imu_samples[sample_index].time_s < window_start_s - kTimeEpsilonS) {
      ++sample_index;
    }

    std::size_t scan_index = sample_index;
    while (scan_index < imu_samples.size()) {
      const double sample_time_s = imu_samples[scan_index].time_s;
      if ((!is_last_window && sample_time_s >= window_end_s - kTimeEpsilonS) ||
          (is_last_window && sample_time_s > window_end_s + kTimeEpsilonS)) {
        break;
      }
      if (sample_time_s >= window_start_s - kTimeEpsilonS) {
        sum_acc_mps2 += imu_samples[scan_index].accel_mps2;
        ++sample_count;
      }
      ++scan_index;
    }

    if (sample_count > 0U) {
      window_means.push_back(sum_acc_mps2 / static_cast<double>(sample_count));
    }
    sample_index = scan_index;
  }

  if (window_means.empty()) {
    return;
  }

  Eigen::Vector3d mean_specific_force_mps2 = Eigen::Vector3d::Zero();
  for (const auto &window_mean : window_means) {
    mean_specific_force_mps2 += window_mean;
  }
  mean_specific_force_mps2 /= static_cast<double>(window_means.size());

  Eigen::Vector3d variance_mps4 = Eigen::Vector3d::Zero();
  double squared_norm_sum = 0.0;
  for (const auto &window_mean : window_means) {
    const Eigen::Vector3d centered = window_mean - mean_specific_force_mps2;
    variance_mps4 += centered.cwiseProduct(centered);
    squared_norm_sum += centered.squaredNorm();
  }
  variance_mps4 /= static_cast<double>(window_means.size());
  squared_norm_sum /= static_cast<double>(window_means.size());

  run_summary.static_specific_force_window_std_x_mps2 = std::sqrt(std::max(variance_mps4.x(), 0.0));
  run_summary.static_specific_force_window_std_y_mps2 = std::sqrt(std::max(variance_mps4.y(), 0.0));
  run_summary.static_specific_force_window_std_z_mps2 = std::sqrt(std::max(variance_mps4.z(), 0.0));
  run_summary.static_specific_force_window_rms_xyz_mps2 = std::sqrt(std::max(squared_norm_sum, 0.0));
}

std::vector<StaticAlignmentValidationRow> BuildStaticAlignmentValidation(
  const gtsam::Values &optimized_values,
  const GraphTimeline &graph_timeline,
  const gtsam::Key global_acc_bias_key,
  const double vertical_acc_bias_tau_s,
  RunSummary &run_summary) {
  std::vector<StaticAlignmentValidationRow> rows;
  if (graph_timeline.initial_static_state_count == 0U || graph_timeline.timestamps_s.empty()) {
    return rows;
  }

  const std::size_t last_static_index = std::min(
    graph_timeline.initial_static_state_count - 1U,
    graph_timeline.timestamps_s.size() - 1U);
  rows.reserve(last_static_index + 1U);

  const gtsam::Pose3 reference_pose = optimized_values.at<gtsam::Pose3>(X(0));
  const double reference_time_s = graph_timeline.timestamps_s.front();
  const double reference_up_m = reference_pose.translation().z();
  const bool has_global_acc_bias = optimized_values.exists(global_acc_bias_key);
  const double global_baz_mps2 =
    has_global_acc_bias
      ? optimized_values.at<gtsam::Vector3>(global_acc_bias_key).z()
      : std::numeric_limits<double>::quiet_NaN();

  std::vector<double> up_values;
  std::vector<double> vz_abs_values;
  std::vector<double> baz_ug_values;
  std::vector<double> baz_minus_global_abs_ug_values;
  up_values.reserve(last_static_index + 1U);
  vz_abs_values.reserve(last_static_index + 1U);
  baz_ug_values.reserve(last_static_index + 1U);
  baz_minus_global_abs_ug_values.reserve(last_static_index + 1U);

  for (std::size_t state_index = 0; state_index <= last_static_index; ++state_index) {
    const double time_s = graph_timeline.timestamps_s[state_index];
    const auto pose = optimized_values.at<gtsam::Pose3>(X(state_index));
    const auto velocity = optimized_values.at<gtsam::Vector3>(V(state_index));
    const auto bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index));
    const double baz_mps2 = bias.accelerometer().z();
    const double baz_ug = Mps2ToMicroG(baz_mps2);

    StaticAlignmentValidationRow row;
    row.time_s = time_s;
    row.relative_time_s = time_s - reference_time_s;
    row.up_delta_m = pose.translation().z() - reference_up_m;
    row.vz_mps = velocity.z();
    row.ba_z_ug = baz_ug;
    row.global_ba_z_ug = Mps2ToMicroG(global_baz_mps2);
    row.ba_z_minus_global_ug = Mps2ToMicroG(baz_mps2 - global_baz_mps2);
    row.static_height_residual_m = row.up_delta_m;

    if (state_index > 0U && std::isfinite(global_baz_mps2)) {
      const double previous_time_s = graph_timeline.timestamps_s[state_index - 1U];
      const auto previous_bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index - 1U));
      const double phi = ComputeBiasDecay(time_s - previous_time_s, vertical_acc_bias_tau_s);
      const double predicted_baz =
        global_baz_mps2 + phi * (previous_bias.accelerometer().z() - global_baz_mps2);
      row.static_bias_gm_residual_ug = Mps2ToMicroG(baz_mps2 - predicted_baz);
    }

    rows.push_back(row);
    up_values.push_back(row.up_delta_m);
    vz_abs_values.push_back(std::abs(row.vz_mps));
    baz_ug_values.push_back(row.ba_z_ug);
    if (std::isfinite(row.ba_z_minus_global_ug)) {
      baz_minus_global_abs_ug_values.push_back(std::abs(row.ba_z_minus_global_ug));
    }
  }

  if (!up_values.empty()) {
    const auto [up_min_it, up_max_it] = std::minmax_element(up_values.begin(), up_values.end());
    run_summary.static_alignment_up_drift_m = up_values.back() - up_values.front();
    run_summary.static_alignment_up_range_m = *up_max_it - *up_min_it;
    run_summary.static_alignment_vz_max_abs_mps =
      *std::max_element(vz_abs_values.begin(), vz_abs_values.end());
  }
  if (!baz_ug_values.empty()) {
    const auto [baz_min_it, baz_max_it] = std::minmax_element(baz_ug_values.begin(), baz_ug_values.end());
    run_summary.static_alignment_baz_range_ug = *baz_max_it - *baz_min_it;
  }
  if (!baz_minus_global_abs_ug_values.empty()) {
    run_summary.static_alignment_baz_minus_global_max_abs_ug =
      *std::max_element(
        baz_minus_global_abs_ug_values.begin(),
        baz_minus_global_abs_ug_values.end());
  }

  return rows;
}

void AccumulateGnssConsistencySummary(
  const std::vector<GnssConsistencyRecord> &gnss_consistency_records,
  const OfflineRunnerConfig &config,
  RunSummary &run_summary) {
  std::vector<double> postfit_nis_values;
  std::size_t axis_pass_count = 0U;
  std::size_t axis_total_count = 0U;
  for (const auto &record : gnss_consistency_records) {
    if (!record.factor_used || !std::isfinite(record.postfit_nis)) {
      continue;
    }
    postfit_nis_values.push_back(record.postfit_nis);
    const Eigen::Vector3d scaled_sigma_m(
      record.sigma_e_m,
      record.sigma_n_m,
      record.vertical_sigma_u_used_m);
    const int axis_limit = record.vertical_direct_position_factor_used ? 3 : 2;
    for (int axis = 0; axis < axis_limit; ++axis) {
      if (!std::isfinite(scaled_sigma_m[axis]) || scaled_sigma_m[axis] <= 0.0 ||
          !std::isfinite(record.postfit_residual_enu_m[axis])) {
        continue;
      }
      ++axis_total_count;
      if (std::abs(record.postfit_residual_enu_m[axis]) <= config.gnss_axis_sigma_multiple * scaled_sigma_m[axis]) {
        ++axis_pass_count;
      }
    }
  }
  const ScalarSummaryStats nis_stats = ComputeScalarSummaryStats(std::move(postfit_nis_values));
  run_summary.gnss_nis_mean = nis_stats.mean;
  run_summary.gnss_nis_median = nis_stats.median;
  run_summary.gnss_nis_p95 = nis_stats.p95;
  run_summary.axis_2sigma_pass_rate =
    axis_total_count > 0U
      ? static_cast<double>(axis_pass_count) / static_cast<double>(axis_total_count)
      : std::numeric_limits<double>::quiet_NaN();
}

std::vector<VerticalStateCorrectionRow> BuildVerticalStateCorrections(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values,
  const std::vector<GnssFactorRecord> &gnss_factor_records,
  const std::vector<GnssConsistencyRecord> &gnss_consistency_records,
  const bool collect_gnss_consistency) {
  std::vector<VerticalStateCorrectionRow> rows;
  rows.reserve(gnss_factor_records.size());
  const std::vector<ReferenceNodeState> optimized_reference_states =
    BuildReferenceStatesFromOptimizedValues(state_timestamps, optimized_values);
  for (std::size_t record_index = 0; record_index < gnss_factor_records.size(); ++record_index) {
    const auto &record = gnss_factor_records[record_index];
    if (!record.factor_used) {
      continue;
    }

    std::optional<std::size_t> correction_state_index;
    if (record.sync_status == StateMeasSyncStatus::kSynchronizedI ||
        record.sync_status == StateMeasSyncStatus::kSynchronizedJ) {
      correction_state_index = record.synchronized_state_index;
    } else if (record.sync_status == StateMeasSyncStatus::kInterpolated) {
      correction_state_index = record.state_index_j;
    }
    if (!correction_state_index.has_value() || *correction_state_index >= state_timestamps.size()) {
      continue;
    }

    VerticalStateCorrectionRow row;
    row.sample_index = record.sample_index;
    row.raw_time_s = record.raw_time_s;
    row.corrected_time_s = record.corrected_time_s;
    row.sync_status = record.sync_status;
    row.state_index = *correction_state_index;
    row.state_time_s = state_timestamps[*correction_state_index];
    row.factor_used = record.factor_used;
    row.measurement_up_m = record.measurement_enu_m.z();

    if (collect_gnss_consistency && record_index < gnss_consistency_records.size()) {
      const auto &consistency_record = gnss_consistency_records[record_index];
      row.vertical_direct_position_factor_used = consistency_record.vertical_direct_position_factor_used;
      row.postfit_residual_u_m = consistency_record.postfit_residual_enu_m.z();
    }

    const ReferenceNodeState optimized_state =
      InterpolateReferenceState(optimized_reference_states, record.corrected_time_s);
    const Eigen::Vector3d optimized_ypr = Rot3ToYpr(optimized_state.pose.rotation());
    row.optimized_up_m = optimized_state.pose.translation().z();
    row.optimized_vz_mps = optimized_state.velocity.z();
    row.optimized_pitch_rad = optimized_ypr.y();
    row.optimized_roll_rad = optimized_ypr.z();
    row.optimized_baz_mps2 = optimized_state.bias.accelerometer().z();

    rows.push_back(row);
  }
  return rows;
}

}  // namespace offline_lc_minimal
