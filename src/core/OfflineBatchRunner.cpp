#include "offline_lc_minimal/core/OfflineBatchRunner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <iterator>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/BodyZWindowPipeline.h"
#include "offline_lc_minimal/core/GnssFactorBuilder.h"
#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/core/InitialStaticConstraintBuilder.h"
#include "offline_lc_minimal/core/ImuRateAvpReconstructor.h"
#include "offline_lc_minimal/core/TrajectoryInitializer.h"
#include "offline_lc_minimal/factor/AngularRateFactor.h"
#include "offline_lc_minimal/factor/BiasGmTransitionFactor.h"
#include "offline_lc_minimal/factor/GlobalAccelBiasFactor.h"
#include "offline_lc_minimal/factor/GlobalGyroBiasFactor.h"
#include "offline_lc_minimal/factor/GlobalPlanarAccelBiasFactor.h"
#include "offline_lc_minimal/factor/SegmentBiasFeedbackFactor.h"
#include "offline_lc_minimal/factor/StaticZeroAngularRateFactor.h"
#include "offline_lc_minimal/factor/StaticSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticAttitudeDriftFactor.h"
#include "offline_lc_minimal/factor/VerticalAccelBiasGmTransitionFactor.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {

namespace {

constexpr double kNumericalSigmaFloorM = 1e-4;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimeEpsilonS = 1e-9;
constexpr double kAngularRateSigmaRadps = 0.1;
constexpr double kInterpolatorQcVariance = 10000.0;
constexpr double kDisabledAccBiasPriorSigmaMps2 = 1e6;
constexpr double kDisabledGyroBiasPriorSigmaRadps = 1e6;

struct GraphTimeline {
  std::vector<double> timestamps_s;
  std::size_t dynamic_start_index = 0;
  std::size_t initial_static_state_count = 0;
};

struct SegmentGnssStatsAccumulator {
  std::size_t gnss_factor_count = 0;
  double prefit_nis_sum = 0.0;
  double postfit_nis_sum = 0.0;
  double covariance_scale_sum = 0.0;
  double vertical_rtk_residual_sum = 0.0;
  double vertical_gate_inside_sum = 0.0;
  double target_baz_sum = 0.0;
  double feedback_attitude_scale_sum = 0.0;
  std::size_t prefit_nis_count = 0;
  std::size_t postfit_nis_count = 0;
  std::size_t covariance_scale_count = 0;
  std::size_t vertical_rtk_residual_count = 0;
  std::size_t vertical_gate_inside_count = 0;
  std::size_t target_baz_count = 0;
  std::size_t feedback_attitude_scale_count = 0;
};

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

ReferenceNodeState MakeReferenceNodeState(
  const double time_s,
  const gtsam::NavState &nav_state,
  const gtsam::imuBias::ConstantBias &bias,
  const gtsam::Vector3 &omega) {
  ReferenceNodeState state;
  state.time_s = time_s;
  state.pose = nav_state.pose();
  state.velocity = nav_state.v();
  state.bias = bias;
  state.omega = omega;
  return state;
}

ReferenceNodeRow MakeReferenceNodeRow(const ReferenceNodeState &state) {
  ReferenceNodeRow row;
  row.time_s = state.time_s;
  row.enu_position_m =
    Eigen::Vector3d(state.pose.translation().x(), state.pose.translation().y(), state.pose.translation().z());
  row.enu_velocity_mps = Eigen::Vector3d(state.velocity.x(), state.velocity.y(), state.velocity.z());
  row.ypr_rad = Rot3ToYpr(state.pose.rotation());
  row.bias_acc = state.bias.accelerometer();
  row.bias_gyro = state.bias.gyroscope();
  return row;
}

double ComputeBiasDecay(const double dt_s, const double tau_s) {
  return std::exp(-std::max(dt_s, 0.0) / std::max(tau_s, 1e-9));
}

gtsam::Matrix3 ComputeBiasPhi(const double dt_s, const double tau_s) {
  return ComputeBiasDecay(dt_s, tau_s) * gtsam::I_3x3;
}

double ResolveVerticalAccBiasSigmaMps2(const OfflineRunnerConfig &config) {
  return config.vertical_acc_bias_sigma_mps2 > 0.0 ? config.vertical_acc_bias_sigma_mps2 : config.bias_acc_sigma;
}

double ComputeVerticalAccBiasProcessVariance(const double dt_s, const OfflineRunnerConfig &config) {
  const double bounded_dt_s = std::max(dt_s, 1e-6);
  return std::pow(ResolveVerticalAccBiasSigmaMps2(config), 2.0) * config.vertical_acc_bias_process_noise_scale *
         std::max(1.0 - std::exp(-2.0 * bounded_dt_s / std::max(config.vertical_acc_bias_tau_s, 1e-9)), 1e-9);
}

gtsam::Matrix66 ComputeBiasProcessCovariance(const double dt_s, const OfflineRunnerConfig &config) {
  const double bounded_dt_s = std::max(dt_s, 1e-6);
  const double acc_variance =
    std::pow(config.bias_acc_sigma, 2.0) * config.bias_process_noise_acc_scale *
    std::max(1.0 - std::exp(-2.0 * bounded_dt_s / std::max(config.tau_acc_bias_s, 1e-9)), 1e-9);
  const double gyro_variance =
    std::pow(config.bias_gyro_sigma, 2.0) * config.bias_process_noise_gyro_scale *
    std::max(1.0 - std::exp(-2.0 * bounded_dt_s / std::max(config.tau_gyro_bias_s, 1e-9)), 1e-12);

  gtsam::Matrix66 covariance = gtsam::Matrix66::Zero();
  covariance.block<3, 3>(0, 0) = acc_variance * gtsam::I_3x3;
  covariance.block<3, 3>(3, 3) = gyro_variance * gtsam::I_3x3;
  return covariance;
}

std::vector<ReferenceNodeState> BuildReferenceStatesFromOptimizedValues(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values) {
  std::vector<ReferenceNodeState> states;
  states.reserve(state_timestamps.size());
  for (std::size_t state_index = 0; state_index < state_timestamps.size(); ++state_index) {
    states.push_back(MakeReferenceNodeState(
      state_timestamps[state_index],
      gtsam::NavState(
        optimized_values.at<gtsam::Pose3>(X(state_index)),
        optimized_values.at<gtsam::Vector3>(V(state_index))),
      optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index)),
      optimized_values.at<gtsam::Vector3>(W(state_index))));
  }
  return states;
}

gtsam::Rot3 InterpolateRotation(const gtsam::Rot3 &left, const gtsam::Rot3 &right, const double alpha) {
  const gtsam::Vector3 delta = gtsam::Rot3::Logmap(left.between(right));
  return left.compose(gtsam::Rot3::Expmap(alpha * delta));
}

ReferenceNodeState InterpolateReferenceState(
  const std::vector<ReferenceNodeState> &reference_states,
  const double time_s) {
  if (reference_states.empty()) {
    throw std::runtime_error("reference node state sequence is empty");
  }
  if (reference_states.size() == 1U || time_s <= reference_states.front().time_s + kTimeEpsilonS) {
    return reference_states.front();
  }
  if (time_s >= reference_states.back().time_s - kTimeEpsilonS) {
    return reference_states.back();
  }

  const auto upper_it = std::upper_bound(
    reference_states.begin(),
    reference_states.end(),
    time_s,
    [](const double timestamp_s, const ReferenceNodeState &state) { return timestamp_s < state.time_s; });
  const std::size_t right_index = static_cast<std::size_t>(std::distance(reference_states.begin(), upper_it));
  const std::size_t left_index = right_index - 1U;
  const auto &left_state = reference_states[left_index];
  const auto &right_state = reference_states[right_index];
  const double alpha =
    std::clamp((time_s - left_state.time_s) / (right_state.time_s - left_state.time_s), 0.0, 1.0);
  const double dt_s = std::max(right_state.time_s - left_state.time_s, kTimeEpsilonS);

  const double alpha2 = alpha * alpha;
  const double alpha3 = alpha2 * alpha;
  const double h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
  const double h10 = alpha3 - 2.0 * alpha2 + alpha;
  const double h01 = -2.0 * alpha3 + 3.0 * alpha2;
  const double h11 = alpha3 - alpha2;
  const double interpolated_up_m =
    h00 * left_state.pose.translation().z() +
    h10 * dt_s * left_state.velocity.z() +
    h01 * right_state.pose.translation().z() +
    h11 * dt_s * right_state.velocity.z();
  Eigen::Vector3d interpolated_velocity =
    (1.0 - alpha) * left_state.velocity + alpha * right_state.velocity;
  interpolated_velocity.z() =
    ((6.0 * alpha2 - 6.0 * alpha) * left_state.pose.translation().z() +
     (-6.0 * alpha2 + 6.0 * alpha) * right_state.pose.translation().z()) /
      dt_s +
    (3.0 * alpha2 - 4.0 * alpha + 1.0) * left_state.velocity.z() +
    (3.0 * alpha2 - 2.0 * alpha) * right_state.velocity.z();

  ReferenceNodeState interpolated;
  interpolated.time_s = time_s;
  interpolated.pose = gtsam::Pose3(
    InterpolateRotation(left_state.pose.rotation(), right_state.pose.rotation(), alpha),
    gtsam::Point3(
      (1.0 - alpha) * left_state.pose.translation().x() + alpha * right_state.pose.translation().x(),
      (1.0 - alpha) * left_state.pose.translation().y() + alpha * right_state.pose.translation().y(),
      interpolated_up_m));
  interpolated.velocity = interpolated_velocity;
  interpolated.bias = gtsam::imuBias::ConstantBias(
    (1.0 - alpha) * left_state.bias.accelerometer() + alpha * right_state.bias.accelerometer(),
    (1.0 - alpha) * left_state.bias.gyroscope() + alpha * right_state.bias.gyroscope());
  interpolated.omega = (1.0 - alpha) * left_state.omega + alpha * right_state.omega;
  return interpolated;
}

Eigen::Vector3d ComputePositionResidualEnu(
  const gtsam::Pose3 &pose,
  const Eigen::Vector3d &measurement_enu_m) {
  return Eigen::Vector3d(
           pose.translation().x(),
           pose.translation().y(),
           pose.translation().z()) -
         measurement_enu_m;
}

double ComputeNis(const Eigen::Vector3d &residual_enu_m, const Eigen::Vector3d &sigma_m);
double ComputeHorizontalNis(const Eigen::Vector3d &residual_enu_m, const Eigen::Vector2d &sigma_m);

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
        consistency_record == nullptr || consistency_record->vertical_direct_position_factor_used;
      record.residual_m =
        use_vertical_direct_position ? residual_enu_m.norm() : residual_enu_m.head<2>().norm();
      if (consistency_record != nullptr) {
        consistency_record->postfit_residual_enu_m = residual_enu_m;
        const Eigen::Vector2d scaled_horizontal_sigma_m(
          consistency_record->sigma_e_m * consistency_record->covariance_scale_e,
          consistency_record->sigma_n_m * consistency_record->covariance_scale_n);
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
        consistency_record == nullptr || consistency_record->vertical_direct_position_factor_used;
      record.residual_m =
        use_vertical_direct_position ? residual_enu_m.norm() : residual_enu_m.head<2>().norm();
      if (consistency_record != nullptr) {
        consistency_record->postfit_residual_enu_m = residual_enu_m;
        const Eigen::Vector2d scaled_horizontal_sigma_m(
          consistency_record->sigma_e_m * consistency_record->covariance_scale_e,
          consistency_record->sigma_n_m * consistency_record->covariance_scale_n);
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

struct ScalarSummaryStats {
  double mean = std::numeric_limits<double>::quiet_NaN();
  double median = std::numeric_limits<double>::quiet_NaN();
  double p95 = std::numeric_limits<double>::quiet_NaN();
};

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

struct ForwardDriftSummary {
  double up_slope_10s = std::numeric_limits<double>::quiet_NaN();
  double up_slope_30s = std::numeric_limits<double>::quiet_NaN();
  double horizontal_slope_10s = std::numeric_limits<double>::quiet_NaN();
  double horizontal_slope_30s = std::numeric_limits<double>::quiet_NaN();
};

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

std::vector<TrajectoryRow> BuildForwardTrajectoryRows(
  const std::vector<ImuSample> &imu_samples,
  const gtsam::Pose3 &start_pose,
  const gtsam::Vector3 &start_velocity,
  const gtsam::imuBias::ConstantBias &bias,
  const double start_time_s,
  const double duration_s,
  const double gravity_mps2) {
  std::vector<TrajectoryRow> rows;
  if (imu_samples.size() < 2U || duration_s <= 0.0) {
    return rows;
  }

  const double end_time_s = start_time_s + duration_s;
  gtsam::Rot3 current_rotation = start_pose.rotation();
  Eigen::Vector3d current_position(
    start_pose.translation().x(),
    start_pose.translation().y(),
    start_pose.translation().z());
  Eigen::Vector3d current_velocity(start_velocity.x(), start_velocity.y(), start_velocity.z());
  const Eigen::Vector3d bias_acc = bias.accelerometer();
  const Eigen::Vector3d bias_gyro = bias.gyroscope();
  const Eigen::Vector3d gravity_enu(0.0, 0.0, gravity_mps2);

  auto append_row = [&](const double time_s, const Eigen::Vector3d &omega_radps) {
    TrajectoryRow row;
    row.time_s = time_s;
    row.enu_position_m = current_position;
    row.enu_velocity_mps = current_velocity;
    row.ypr_rad = Rot3ToYpr(current_rotation);
    row.omega_radps = omega_radps;
    row.bias_acc = bias_acc;
    row.bias_gyro = bias_gyro;
    rows.push_back(row);
  };

  append_row(start_time_s, Eigen::Vector3d::Zero());
  for (std::size_t imu_index = 0; imu_index + 1U < imu_samples.size(); ++imu_index) {
    const auto &current_sample = imu_samples[imu_index];
    const auto &next_sample = imu_samples[imu_index + 1U];
    const double interval_start_s = std::max(current_sample.time_s, start_time_s);
    const double interval_end_s = std::min(next_sample.time_s, end_time_s);
    if (interval_end_s <= interval_start_s + kTimeEpsilonS) {
      continue;
    }

    const double delta_time_s = interval_end_s - interval_start_s;
    const Eigen::Vector3d corrected_gyro = current_sample.gyro_radps - bias_gyro;
    const Eigen::Vector3d corrected_acc = current_sample.accel_mps2 - bias_acc;
    const Eigen::Vector3d nav_specific_force = current_rotation.matrix() * corrected_acc;
    const Eigen::Vector3d nav_acc = nav_specific_force - gravity_enu;

    current_position += current_velocity * delta_time_s + 0.5 * nav_acc * delta_time_s * delta_time_s;
    current_velocity += nav_acc * delta_time_s;
    current_rotation =
      current_rotation.compose(gtsam::Rot3::Expmap(gtsam::Vector3(
        corrected_gyro.x() * delta_time_s,
        corrected_gyro.y() * delta_time_s,
        corrected_gyro.z() * delta_time_s)));

    append_row(interval_end_s, corrected_gyro);
    if (interval_end_s >= end_time_s - kTimeEpsilonS) {
      break;
    }
  }

  return rows;
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
      if (std::isfinite(consistency_record.prefit_nis)) {
        accumulator.prefit_nis_sum += consistency_record.prefit_nis;
        ++accumulator.prefit_nis_count;
      }
      if (std::isfinite(consistency_record.postfit_nis)) {
        accumulator.postfit_nis_sum += consistency_record.postfit_nis;
        ++accumulator.postfit_nis_count;
      }
      if (std::isfinite(consistency_record.covariance_scale)) {
        accumulator.covariance_scale_sum += consistency_record.covariance_scale;
        ++accumulator.covariance_scale_count;
      }
      if (std::isfinite(consistency_record.prefit_residual_enu_m.z())) {
        accumulator.vertical_rtk_residual_sum += consistency_record.prefit_residual_enu_m.z();
        ++accumulator.vertical_rtk_residual_count;
      }
      if (std::isfinite(consistency_record.vertical_gate_inside)) {
        accumulator.vertical_gate_inside_sum += consistency_record.vertical_gate_inside;
        ++accumulator.vertical_gate_inside_count;
      }
      if (std::isfinite(consistency_record.vertical_feedback_target_baz_mps2)) {
        accumulator.target_baz_sum += consistency_record.vertical_feedback_target_baz_mps2;
        ++accumulator.target_baz_count;
      }
      if (std::isfinite(consistency_record.vertical_feedback_attitude_scale)) {
        accumulator.feedback_attitude_scale_sum += consistency_record.vertical_feedback_attitude_scale;
        ++accumulator.feedback_attitude_scale_count;
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
      diagnostic.mean_prefit_nis =
        stats.prefit_nis_count > 0U
          ? stats.prefit_nis_sum / static_cast<double>(stats.prefit_nis_count)
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.mean_postfit_nis =
        stats.postfit_nis_count > 0U
          ? stats.postfit_nis_sum / static_cast<double>(stats.postfit_nis_count)
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.mean_covariance_scale =
        stats.covariance_scale_count > 0U
          ? stats.covariance_scale_sum / static_cast<double>(stats.covariance_scale_count)
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.segment_vertical_rtk_residual_m =
        stats.vertical_rtk_residual_count > 0U
          ? stats.vertical_rtk_residual_sum / static_cast<double>(stats.vertical_rtk_residual_count)
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.segment_vertical_gate_inside =
        stats.vertical_gate_inside_count > 0U
          ? stats.vertical_gate_inside_sum / static_cast<double>(stats.vertical_gate_inside_count)
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.segment_target_baz_mps2 =
        stats.target_baz_count > 0U
          ? stats.target_baz_sum / static_cast<double>(stats.target_baz_count)
          : std::numeric_limits<double>::quiet_NaN();
      diagnostic.segment_feedback_attitude_scale =
        stats.feedback_attitude_scale_count > 0U
          ? stats.feedback_attitude_scale_sum / static_cast<double>(stats.feedback_attitude_scale_count)
          : std::numeric_limits<double>::quiet_NaN();
    }

    diagnostics.push_back(diagnostic);
  }
  return diagnostics;
}

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

}  // namespace

OfflineBatchRunner::OfflineBatchRunner(OfflineRunnerConfig config)
    : config_(std::move(config)) {}

double OfflineBatchRunner::CorrectedGnssTime(const GnssSolutionSample &sample) const {
  return sample.time_s - config_.gnss_time_offset_s;
}

bool OfflineBatchRunner::IsAllowedGnssFixType(const GnssFixType fix_type) const {
  if (!config_.drop_non_rtkfix) {
    return true;
  }
  return fix_type == GnssFixType::kRtkFix;
}

bool OfflineBatchRunner::PassesGnssQualityFilters(const GnssSolutionSample &sample) const {
  if (!sample.has_valid_position()) {
    return false;
  }
  if (config_.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config_.required_best_sol_status_code) {
    return false;
  }
  if (config_.drop_no_solution && sample.fix_type() == GnssFixType::kNoSolution) {
    return false;
  }
  if (!IsAllowedGnssFixType(sample.fix_type())) {
    return false;
  }
  if (config_.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return true;
}

bool OfflineBatchRunner::IsWithinImuCoverage(const std::vector<ImuSample> &imu_samples, const double time_s) const {
  if (imu_samples.empty()) {
    return false;
  }
  return time_s >= imu_samples.front().time_s - kTimeEpsilonS &&
         time_s <= imu_samples.back().time_s + kTimeEpsilonS;
}

bool OfflineBatchRunner::CanUseGnssSampleForInitialization(
  const GnssSolutionSample &sample,
  const std::vector<ImuSample> &imu_samples) const {
  if (!PassesGnssQualityFilters(sample)) {
    return false;
  }
  return IsWithinImuCoverage(imu_samples, CorrectedGnssTime(sample));
}

std::size_t OfflineBatchRunner::FindOriginIndex(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<ImuSample> &imu_samples) const {
  for (std::size_t index = 0; index < gnss_samples.size(); ++index) {
    if (CanUseGnssSampleForInitialization(gnss_samples[index], imu_samples)) {
      return index;
    }
  }
  throw std::runtime_error("failed to find a GNSS origin sample that also passes quality and IMU coverage checks");
}

std::size_t OfflineBatchRunner::FindNavigationStartIndex(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<ImuSample> &imu_samples,
  const std::size_t origin_index,
  const double navigation_start_min_time_s) const {
  for (std::size_t index = origin_index; index < gnss_samples.size(); ++index) {
    if (!CanUseGnssSampleForInitialization(gnss_samples[index], imu_samples)) {
      continue;
    }
    if (CorrectedGnssTime(gnss_samples[index]) + kTimeEpsilonS < navigation_start_min_time_s) {
      continue;
    }
    return index;
  }
  throw std::runtime_error(
    "failed to find a navigation start GNSS sample after the requested static alignment duration");
}

std::vector<std::size_t> OfflineBatchRunner::CollectInitializationCandidateIndices(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<ImuSample> &imu_samples) const {
  std::vector<std::size_t> indices;
  indices.reserve(gnss_samples.size());
  for (std::size_t index = 0; index < gnss_samples.size(); ++index) {
    if (CanUseGnssSampleForInitialization(gnss_samples[index], imu_samples)) {
      indices.push_back(index);
    }
  }
  return indices;
}

void OfflineBatchRunner::PopulateEnuPositions(
  std::vector<GnssSolutionSample> &gnss_samples,
  const GeoReference &geo_reference) const {
  for (auto &sample : gnss_samples) {
    if (!sample.has_valid_position()) {
      continue;
    }
    sample.enu_position_m = geo_reference.Forward(sample.lat_rad, sample.lon_rad, sample.h_m);
    sample.has_enu_position = true;
  }
}

bool OfflineBatchRunner::ShouldUseGnssFactor(const GnssSolutionSample &sample, RunSummary &run_summary) const {
  if (!sample.has_valid_position() || !sample.has_enu_position) {
    ++run_summary.gnss_dropped_count;
    return false;
  }
  if (config_.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config_.required_best_sol_status_code) {
    ++run_summary.dropped_bad_status_count;
    ++run_summary.gnss_dropped_count;
    return false;
  }
  if (config_.drop_no_solution && sample.fix_type() == GnssFixType::kNoSolution) {
    ++run_summary.dropped_no_solution_count;
    ++run_summary.gnss_dropped_count;
    return false;
  }
  if (!IsAllowedGnssFixType(sample.fix_type())) {
    ++run_summary.dropped_non_rtkfix_count;
    ++run_summary.gnss_dropped_count;
    return false;
  }
  if (config_.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    ++run_summary.dropped_nonfinite_sigma_count;
    ++run_summary.gnss_dropped_count;
    return false;
  }
  return true;
}

double OfflineBatchRunner::GnssFixScale(const GnssFixType fix_type) const {
  switch (fix_type) {
    case GnssFixType::kRtkFix:
      return config_.rtkfix_scale;
    case GnssFixType::kRtkFloat:
      return config_.rtkfloat_scale;
    case GnssFixType::kSingle:
      return config_.single_scale;
    case GnssFixType::kNoSolution:
    default:
      return std::max(config_.single_scale * 10.0, config_.single_scale);
  }
}

Eigen::Vector3d OfflineBatchRunner::ClampGnssSigma(const GnssSolutionSample &sample) const {
  Eigen::Vector3d sigma(sample.sigma_lon_m, sample.sigma_lat_m, sample.sigma_h_m);
  if (!std::isfinite(sigma.x())) {
    sigma.x() = config_.position_sigma_ceiling_m;
  }
  if (!std::isfinite(sigma.y())) {
    sigma.y() = config_.position_sigma_ceiling_m;
  }
  if (config_.gnss_vertical_sigma_mode == GnssVerticalSigmaMode::kFixed) {
    sigma.z() = config_.gnss_vertical_fixed_sigma_m;
  } else if (!std::isfinite(sigma.z())) {
    sigma.z() = config_.position_sigma_ceiling_m;
  }
  sigma.x() *= config_.gnss_sigma_scale_horizontal;
  sigma.y() *= config_.gnss_sigma_scale_horizontal;
  sigma.z() *= config_.gnss_sigma_scale_up;
  sigma.x() = std::clamp(
    sigma.x(),
    std::max(config_.position_sigma_floor_horizontal_m, kNumericalSigmaFloorM),
    config_.position_sigma_ceiling_m);
  sigma.y() = std::clamp(
    sigma.y(),
    std::max(config_.position_sigma_floor_horizontal_m, kNumericalSigmaFloorM),
    config_.position_sigma_ceiling_m);
  sigma.z() = std::clamp(
    sigma.z(),
    std::max(config_.position_sigma_floor_up_m, kNumericalSigmaFloorM),
    config_.position_sigma_ceiling_m);
  sigma *= GnssFixScale(sample.fix_type());
  return sigma;
}

std::vector<double> OfflineBatchRunner::BuildGnssVerticalReferenceUpBySample(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<ImuSample> &imu_samples,
  const std::size_t navigation_start_index) const {
  std::vector<double> references(gnss_samples.size(), std::numeric_limits<double>::quiet_NaN());
  if (!config_.enable_gnss_vertical_drift_model || gnss_samples.empty()) {
    return references;
  }

  const double window_s = config_.gnss_vertical_drift_window_s;
  const double half_window_s = 0.5 * window_s;
  if (window_s <= 0.0) {
    return references;
  }

  struct VerticalReferenceSample {
    std::size_t sample_index = 0;
    double corrected_time_s = 0.0;
    double up_m = 0.0;
  };

  std::vector<VerticalReferenceSample> valid_samples;
  valid_samples.reserve(gnss_samples.size());
  for (std::size_t sample_index = 0; sample_index < gnss_samples.size(); ++sample_index) {
    const auto &sample = gnss_samples[sample_index];
    if (sample_index <= navigation_start_index) {
      continue;
    }
    if (!sample.has_valid_position()) {
      continue;
    }
    if (!PassesGnssQualityFilters(sample)) {
      continue;
    }
    if (!IsWithinImuCoverage(imu_samples, CorrectedGnssTime(sample))) {
      continue;
    }
    if (!sample.has_enu_position || !std::isfinite(sample.enu_position_m.z())) {
      continue;
    }
    valid_samples.push_back(VerticalReferenceSample{
      sample_index,
      CorrectedGnssTime(sample),
      sample.enu_position_m.z(),
    });
  }
  if (valid_samples.empty()) {
    return references;
  }

  std::size_t left_index = 0U;
  std::size_t right_index = 0U;
  double up_sum = 0.0;
  for (std::size_t center_index = 0; center_index < valid_samples.size(); ++center_index) {
    const double center_time_s = valid_samples[center_index].corrected_time_s;
    while (left_index < valid_samples.size() &&
           valid_samples[left_index].corrected_time_s < center_time_s - half_window_s) {
      up_sum -= valid_samples[left_index].up_m;
      ++left_index;
    }
    while (right_index < valid_samples.size() &&
           valid_samples[right_index].corrected_time_s <= center_time_s + half_window_s) {
      up_sum += valid_samples[right_index].up_m;
      ++right_index;
    }
    const std::size_t count = right_index - left_index;
    if (count == 0U) {
      references[valid_samples[center_index].sample_index] = valid_samples[center_index].up_m;
    } else {
      references[valid_samples[center_index].sample_index] = up_sum / static_cast<double>(count);
    }
  }

  return references;
}

OfflineRunResult OfflineBatchRunner::Run(DataSet dataset) const {
  if (dataset.imu_samples.empty()) {
    throw std::runtime_error("offline runner received an empty IMU data set");
  }
  if (dataset.gnss_samples.empty()) {
    throw std::runtime_error("offline runner received an empty GNSS data set");
  }

  OfflineRunResult run_result;
  run_result.data_summary = dataset.summary;
  run_result.run_summary.gnss_enabled = config_.enable_gnss;
  run_result.run_summary.initial_static_constraints_enabled =
    config_.enable_initial_static_zupt_zaru ||
    config_.enable_initial_static_zero_specific_force ||
    config_.enable_initial_static_vertical_specific_force;
  run_result.run_summary.initial_static_subgraph_enabled = config_.enable_initial_static_subgraph;

  const std::size_t origin_index = FindOriginIndex(dataset.gnss_samples, dataset.imu_samples);
  const auto &origin_sample = dataset.gnss_samples[origin_index];
  const double alignment_start_time_s = CorrectedGnssTime(origin_sample);
  const double navigation_start_min_time_s = alignment_start_time_s + config_.static_alignment_duration_s;
  const std::size_t navigation_start_index =
    FindNavigationStartIndex(dataset.gnss_samples, dataset.imu_samples, origin_index, navigation_start_min_time_s);
  const auto &navigation_start_sample = dataset.gnss_samples[navigation_start_index];
  const double alignment_end_time_s =
    config_.static_alignment_duration_s > 0.0 ? navigation_start_min_time_s : CorrectedGnssTime(navigation_start_sample);
  GeoReference geo_reference(origin_sample.lat_rad, origin_sample.lon_rad, origin_sample.h_m);
  PopulateEnuPositions(dataset.gnss_samples, geo_reference);
  const std::vector<double> gnss_vertical_reference_up_by_sample =
    BuildGnssVerticalReferenceUpBySample(dataset.gnss_samples, dataset.imu_samples, navigation_start_index);

  run_result.run_summary.origin_lat_rad = geo_reference.origin_lat_rad();
  run_result.run_summary.origin_lon_rad = geo_reference.origin_lon_rad();
  run_result.run_summary.origin_h_m = geo_reference.origin_h_m();
  run_result.run_summary.alignment_start_time_s = alignment_start_time_s;
  run_result.run_summary.static_alignment_duration_s = config_.static_alignment_duration_s;

  const std::vector<std::size_t> initialization_candidate_indices =
    CollectInitializationCandidateIndices(dataset.gnss_samples, dataset.imu_samples);
  const double dynamic_start_time_s = CorrectedGnssTime(navigation_start_sample);
  run_result.run_summary.navigation_start_time_s = alignment_start_time_s;
  run_result.run_summary.dynamic_start_time_s = dynamic_start_time_s;
  const InitialPoseEstimate initial_pose =
    TrajectoryInitializer::Estimate(
      dataset.imu_samples,
      dataset.gnss_samples,
      navigation_start_index,
      alignment_start_time_s,
      alignment_end_time_s,
      dynamic_start_time_s,
      geo_reference.EarthRateEnu(),
      initialization_candidate_indices,
      config_);
  run_result.run_summary.yaw_source = initial_pose.yaw_source;
  const InitialStaticConstraintData initial_static_constraint_data =
    InitialStaticConstraintBuilder::Collect(
      dataset.imu_samples,
      alignment_start_time_s,
      alignment_end_time_s,
      config_);
  run_result.run_summary.initial_static_constraint_sample_count =
    initial_static_constraint_data.window_summary.sample_count;
  AccumulateStaticSpecificForceWindowMetrics(
    dataset.imu_samples,
    alignment_start_time_s,
    alignment_end_time_s,
    1.0 / std::max(config_.state_frequency_hz, 1e-9),
    run_result.run_summary);

  double end_time_s = std::numeric_limits<double>::lowest();
  for (const auto &sample : dataset.gnss_samples) {
    if (!sample.has_valid_position()) {
      continue;
    }
    if (config_.drop_non_rtkfix && !IsAllowedGnssFixType(sample.fix_type())) {
      continue;
    }
    const double corrected_time_s = CorrectedGnssTime(sample);
    if (IsWithinImuCoverage(dataset.imu_samples, corrected_time_s)) {
      end_time_s = std::max(end_time_s, corrected_time_s);
    }
  }
  if (!std::isfinite(end_time_s) || end_time_s <= dynamic_start_time_s) {
    throw std::runtime_error("failed to find a valid offline processing time range");
  }

  const GraphTimeline graph_timeline = BuildGraphTimeline(
    alignment_start_time_s,
    alignment_end_time_s,
    dynamic_start_time_s,
    end_time_s,
    config_);
  const std::vector<double> &state_timestamps = graph_timeline.timestamps_s;
  run_result.run_summary.initial_static_state_count = graph_timeline.initial_static_state_count;
  std::map<std::size_t, double> state_timestamp_map;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    state_timestamp_map.emplace(index, state_timestamps[index]);
  }
  const bool collect_error_diagnostics = config_.write_error_diagnostics;
  const bool collect_segment_error_diagnostics =
    config_.write_segment_error_diagnostics ||
    config_.enable_segment_error_feedback;
  const bool collect_reference_states =
    collect_error_diagnostics ||
    collect_segment_error_diagnostics ||
    config_.gnss_consistency_gate_mode != GnssConsistencyGateMode::kNone ||
    config_.enable_body_z_seed_jump_windows;
  run_result.run_summary.error_state_count = 0;

  const auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(config_.gravity_mps2);
  imu_params->accelerometerCovariance = std::pow(config_.imu_sigma_acc, 2.0) * gtsam::I_3x3;
  imu_params->gyroscopeCovariance = std::pow(config_.imu_sigma_gyro, 2.0) * gtsam::I_3x3;
  imu_params->integrationCovariance = std::pow(config_.integration_sigma, 2.0) * gtsam::I_3x3;
  imu_params->biasAccCovariance = std::pow(config_.bias_acc_sigma, 2.0) * gtsam::I_3x3;
  if (config_.enable_vertical_acc_bias_gm_process) {
    // In the Stage 2 route, let the explicit ba_z GM factor be the dominant
    // cross-node model for vertical accelerometer bias instead of stacking two
    // equally strong z-bias evolution assumptions.
    imu_params->biasAccCovariance(2, 2) *= 1e6;
  }
  imu_params->biasOmegaCovariance = std::pow(config_.bias_gyro_sigma, 2.0) * gtsam::I_3x3;
  imu_params->setOmegaCoriolis(geo_reference.EarthRateEnu());
  imu_params->setUse2ndOrderCoriolis(true);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const Eigen::Vector3d initial_position_enu =
    graph_timeline.dynamic_start_index > 0U ? origin_sample.enu_position_m : navigation_start_sample.enu_position_m;
  const gtsam::Pose3 initial_pose_world(
    initial_pose.orientation,
    gtsam::Point3(
      initial_position_enu.x(),
      initial_position_enu.y(),
      initial_position_enu.z()));
  const gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias initial_bias = initial_pose.imu_bias;
  const std::size_t initial_imu_index = FindNearestImuIndex(dataset.imu_samples, state_timestamps.front());
  const gtsam::Vector3 initial_omega =
    initial_bias.correctGyroscope(dataset.imu_samples[initial_imu_index].gyro_radps);
  const gtsam::Key global_acc_bias_key = gtsam::Symbol('a', 0);
  const gtsam::Key global_gyro_bias_key = gtsam::Symbol('g', 0);

  const gtsam::Vector6 pose_sigmas =
    (gtsam::Vector6() << config_.initial_roll_pitch_sigma_rad,
      config_.initial_roll_pitch_sigma_rad,
      config_.initial_yaw_sigma_rad,
      config_.initial_position_sigma_m,
      config_.initial_position_sigma_m,
      config_.initial_position_sigma_m).finished();

  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    X(0),
    initial_pose_world,
    gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas)));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
    V(0),
    initial_velocity,
    gtsam::noiseModel::Isotropic::Sigma(3, config_.initial_velocity_sigma_mps)));
  graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
    B(0),
    initial_bias,
    gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << (config_.enable_global_acc_bias ? kDisabledAccBiasPriorSigmaMps2 : config_.bias_acc_prior_sigma),
        (config_.enable_global_acc_bias ? kDisabledAccBiasPriorSigmaMps2 : config_.bias_acc_prior_sigma),
        (config_.enable_global_acc_bias ? kDisabledAccBiasPriorSigmaMps2 : config_.bias_acc_prior_sigma),
        (config_.enable_global_gyro_bias ? kDisabledGyroBiasPriorSigmaRadps : config_.bias_gyro_prior_sigma),
        (config_.enable_global_gyro_bias ? kDisabledGyroBiasPriorSigmaRadps : config_.bias_gyro_prior_sigma),
        (config_.enable_global_gyro_bias ? kDisabledGyroBiasPriorSigmaRadps : config_.bias_gyro_prior_sigma)).finished())));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
    W(0),
    initial_omega,
    gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));
  graph.add(factor::AngularRateFactor(
    W(0),
    B(0),
    dataset.imu_samples[initial_imu_index].gyro_radps,
    gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));
  if (config_.enable_global_acc_bias) {
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      global_acc_bias_key,
      initial_bias.accelerometer(),
      gtsam::noiseModel::Isotropic::Sigma(3, config_.bias_acc_prior_sigma)));
    if (config_.enable_vertical_acc_bias_gm_process) {
      graph.add(factor::GlobalPlanarAccelBiasFactor(
        B(0),
        global_acc_bias_key,
        gtsam::noiseModel::Isotropic::Sigma(2, config_.global_acc_bias_tie_sigma_xy_mps2)));
    } else {
      graph.add(factor::GlobalAccelBiasFactor(
        B(0),
        global_acc_bias_key,
        gtsam::noiseModel::Isotropic::Sigma(3, config_.global_acc_bias_tie_sigma_mps2)));
    }
  }
  if (config_.enable_global_gyro_bias) {
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      global_gyro_bias_key,
      initial_bias.gyroscope(),
      gtsam::noiseModel::Isotropic::Sigma(3, config_.bias_gyro_prior_sigma)));
    graph.add(factor::GlobalGyroBiasFactor(
      B(0),
      global_gyro_bias_key,
      gtsam::noiseModel::Isotropic::Sigma(3, config_.global_gyro_bias_tie_sigma_radps)));
  }
  InitialStaticConstraintBuilder::AddFactors(
    initial_static_constraint_data,
    geo_reference.EarthRateEnu(),
    config_,
    graph,
    X(0),
    V(0),
    B(0));

  initial_values.insert(X(0), initial_pose_world);
  initial_values.insert(V(0), initial_velocity);
  initial_values.insert(B(0), initial_bias);
  initial_values.insert(W(0), initial_omega);
  if (config_.enable_global_acc_bias) {
    initial_values.insert(global_acc_bias_key, initial_bias.accelerometer());
  }
  if (config_.enable_global_gyro_bias) {
    initial_values.insert(global_gyro_bias_key, initial_bias.gyroscope());
  }

  std::vector<ReferenceNodeState> reference_node_states;
  if (collect_reference_states) {
    reference_node_states.reserve(state_timestamps.size());
    reference_node_states.push_back(MakeReferenceNodeState(
      state_timestamps.front(),
      gtsam::NavState(initial_pose_world, initial_velocity),
      initial_bias,
      initial_omega));
  }

  gtsam::NavState previous_nav_state(initial_pose_world, initial_velocity);
  gtsam::imuBias::ConstantBias previous_bias = initial_bias;
  double previous_time_s = state_timestamps.front();
  std::vector<std::optional<std::size_t>> trajectory_row_index_by_state(state_timestamps.size(), std::nullopt);
  run_result.trajectory.reserve(state_timestamps.size());
  const auto graph_state_to_trajectory_row = [&](const std::size_t state_index) -> long long {
    if (state_index >= trajectory_row_index_by_state.size()) {
      return -1;
    }
    if (!trajectory_row_index_by_state[state_index].has_value()) {
      return -1;
    }
    return static_cast<long long>(*trajectory_row_index_by_state[state_index]);
  };
  const auto append_trajectory_row =
    [&](const std::size_t state_index,
        const gtsam::NavState &nav_state,
        const gtsam::imuBias::ConstantBias &bias,
        const gtsam::Vector3 &omega,
        const double time_s) {
      TrajectoryRow row;
      row.time_s = time_s;
      row.enu_position_m =
        Eigen::Vector3d(nav_state.position().x(), nav_state.position().y(), nav_state.position().z());
      row.enu_velocity_mps = Eigen::Vector3d(nav_state.v().x(), nav_state.v().y(), nav_state.v().z());
      row.ypr_rad = Rot3ToYpr(nav_state.pose().rotation());
      row.omega_radps = Eigen::Vector3d(omega.x(), omega.y(), omega.z());
      row.bias_acc = bias.accelerometer();
      row.bias_gyro = bias.gyroscope();
      trajectory_row_index_by_state[state_index] = run_result.trajectory.size();
      run_result.trajectory.push_back(row);
    };
  append_trajectory_row(0U, previous_nav_state, initial_bias, initial_omega, state_timestamps.front());

  for (std::size_t state_index = 1; state_index < state_timestamps.size(); ++state_index) {
    const double current_time_s = state_timestamps[state_index];
    const auto imu_window =
      IntegrateImuWindow(dataset.imu_samples, previous_time_s, current_time_s, imu_params, previous_bias);

    if (config_.enable_segment_error_feedback) {
      graph.add(gtsam::ImuFactor(
        X(state_index - 1U),
        V(state_index - 1U),
        X(state_index),
        V(state_index),
        B(state_index - 1U),
        imu_window.preintegrated_imu_measurements));
    } else {
      graph.add(gtsam::CombinedImuFactor(
        X(state_index - 1U),
        V(state_index - 1U),
        X(state_index),
        V(state_index),
        B(state_index - 1U),
        B(state_index),
        imu_window.preintegrated_measurements));
    }

    const gtsam::NavState predicted_state =
      config_.enable_segment_error_feedback
        ? imu_window.preintegrated_imu_measurements.predict(previous_nav_state, previous_bias)
        : imu_window.preintegrated_measurements.predict(previous_nav_state, previous_bias);
    initial_values.insert(X(state_index), predicted_state.pose());
    initial_values.insert(V(state_index), predicted_state.v());
    initial_values.insert(B(state_index), previous_bias);
    initial_values.insert(W(state_index), imu_window.end_gyro_radps);
    if (collect_reference_states) {
      reference_node_states.push_back(MakeReferenceNodeState(
        current_time_s,
        predicted_state,
        previous_bias,
        imu_window.end_gyro_radps));
    }
    if (config_.enable_global_acc_bias) {
      if (config_.enable_vertical_acc_bias_gm_process) {
        graph.add(factor::GlobalPlanarAccelBiasFactor(
          B(state_index),
          global_acc_bias_key,
          gtsam::noiseModel::Isotropic::Sigma(2, config_.global_acc_bias_tie_sigma_xy_mps2)));
      } else {
        graph.add(factor::GlobalAccelBiasFactor(
          B(state_index),
          global_acc_bias_key,
          gtsam::noiseModel::Isotropic::Sigma(3, config_.global_acc_bias_tie_sigma_mps2)));
      }
    }
    if (config_.enable_global_gyro_bias) {
      graph.add(factor::GlobalGyroBiasFactor(
        B(state_index),
        global_gyro_bias_key,
        gtsam::noiseModel::Isotropic::Sigma(3, config_.global_gyro_bias_tie_sigma_radps)));
    }
    if (config_.enable_vertical_acc_bias_gm_process) {
      const double delta_time_s = current_time_s - previous_time_s;
      const double phi_vertical_acc = ComputeBiasDecay(delta_time_s, config_.vertical_acc_bias_tau_s);
      const double vertical_acc_variance = ComputeVerticalAccBiasProcessVariance(delta_time_s, config_);
      graph.add(factor::VerticalAccelBiasGmTransitionFactor(
        B(state_index - 1U),
        B(state_index),
        global_acc_bias_key,
        phi_vertical_acc,
        gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(std::max(vertical_acc_variance, 1e-12)))));
    }
    if (config_.enable_segment_error_feedback) {
      const double delta_time_s = current_time_s - previous_time_s;
      const gtsam::Matrix3 phi_acc = ComputeBiasPhi(delta_time_s, config_.tau_acc_bias_s);
      const gtsam::Matrix3 phi_gyro = ComputeBiasPhi(delta_time_s, config_.tau_gyro_bias_s);
      graph.add(factor::BiasGmTransitionFactor(
        B(state_index - 1U),
        B(state_index),
        phi_acc,
        phi_gyro,
        gtsam::noiseModel::Gaussian::Covariance(ComputeBiasProcessCovariance(delta_time_s, config_))));
      if (config_.enable_segment_local_error_feedback) {
        const gtsam::Vector6 segment_feedback_sigmas =
          (gtsam::Vector6() << config_.segment_feedback_acc_sigma_mps2,
            config_.segment_feedback_acc_sigma_mps2,
            config_.segment_feedback_acc_sigma_mps2,
            config_.segment_feedback_gyro_sigma_radps,
            config_.segment_feedback_gyro_sigma_radps,
            config_.segment_feedback_gyro_sigma_radps).finished();
        graph.add(factor::SegmentBiasFeedbackFactor(
          X(state_index - 1U),
          V(state_index - 1U),
          B(state_index - 1U),
          X(state_index),
          V(state_index),
          B(state_index),
          imu_window.preintegrated_imu_measurements,
          phi_acc,
          phi_gyro,
          delta_time_s,
          config_.segment_feedback_attitude_gain,
          config_.segment_feedback_velocity_gain,
          config_.segment_feedback_position_gain,
          gtsam::noiseModel::Diagonal::Sigmas(segment_feedback_sigmas)));
      }
    }

    graph.add(factor::AngularRateFactor(
      W(state_index),
      B(state_index),
      imu_window.end_gyro_radps,
      gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));
    if (config_.enable_initial_static_subgraph && state_timestamps[state_index] <= alignment_end_time_s + kTimeEpsilonS) {
      if (config_.enable_initial_static_zupt_zaru && initial_static_constraint_data.valid) {
        double static_zaru_sigma = config_.initial_static_zaru_sigma_radps;
        if (imu_window.imu_segments > 0U && initial_static_constraint_data.window_summary.sample_count > 0U) {
          static_zaru_sigma *= std::sqrt(
            static_cast<double>(initial_static_constraint_data.window_summary.sample_count) /
            static_cast<double>(imu_window.imu_segments));
        }
        graph.add(gtsam::PriorFactor<gtsam::Vector3>(
          V(state_index),
          gtsam::Vector3::Zero(),
          gtsam::noiseModel::Isotropic::Sigma(3, config_.initial_static_zupt_velocity_sigma_mps)));
        graph.add(factor::StaticZeroAngularRateFactor(
          X(state_index),
          B(state_index),
          imu_window.mean_gyro_radps,
          geo_reference.EarthRateEnu(),
          gtsam::noiseModel::Isotropic::Sigma(3, static_zaru_sigma)));
      }
      if (config_.enable_initial_static_zero_specific_force && initial_static_constraint_data.valid) {
        double static_specific_force_sigma = config_.initial_static_specific_force_sigma_mps2;
        if (imu_window.imu_segments > 0U && initial_static_constraint_data.window_summary.sample_count > 0U) {
          static_specific_force_sigma *= std::sqrt(
            static_cast<double>(initial_static_constraint_data.window_summary.sample_count) /
            static_cast<double>(imu_window.imu_segments));
        }
        graph.add(factor::StaticSpecificForceFactor(
          X(state_index),
          B(state_index),
          imu_window.mean_acc_mps2,
          Eigen::Vector3d(0.0, 0.0, config_.gravity_mps2),
          gtsam::noiseModel::Isotropic::Sigma(3, static_specific_force_sigma)));
      }
      if (config_.enable_initial_static_vertical_specific_force && initial_static_constraint_data.valid) {
        double static_vertical_specific_force_sigma =
          config_.initial_static_vertical_specific_force_sigma_mps2;
        if (imu_window.imu_segments > 0U && initial_static_constraint_data.window_summary.sample_count > 0U) {
          static_vertical_specific_force_sigma *= std::sqrt(
            static_cast<double>(initial_static_constraint_data.window_summary.sample_count) /
            static_cast<double>(imu_window.imu_segments));
        }
        graph.add(factor::StaticVerticalSpecificForceFactor(
          X(state_index),
          B(state_index),
          imu_window.mean_acc_mps2.z(),
          Eigen::Vector3d(0.0, 0.0, config_.gravity_mps2),
          gtsam::noiseModel::Isotropic::Sigma(1, static_vertical_specific_force_sigma)));
      }
      graph.add(factor::StaticAttitudeDriftFactor(
        X(state_index - 1U),
        X(state_index),
        gtsam::noiseModel::Isotropic::Sigma(3, config_.initial_static_attitude_drift_sigma_rad)));
    }
    append_trajectory_row(state_index, predicted_state, previous_bias, imu_window.end_gyro_radps, current_time_s);

    previous_nav_state = predicted_state;
    previous_time_s = current_time_s;
  }

  run_result.run_summary.state_count = run_result.trajectory.size();
  if (collect_reference_states) {
    run_result.reference_node_trajectory.reserve(reference_node_states.size());
    for (const auto &reference_state : reference_node_states) {
      run_result.reference_node_trajectory.push_back(MakeReferenceNodeRow(reference_state));
    }
  }

  const bool collect_gnss_consistency =
    config_.enable_gnss &&
    (collect_error_diagnostics || collect_segment_error_diagnostics ||
     config_.gnss_consistency_gate_mode != GnssConsistencyGateMode::kNone);
  const gp::GPWNOJInterpolator base_interpolator(
    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance)));
  const gtsam::NonlinearFactorGraph base_graph = graph;
  const gtsam::Values base_initial_values = initial_values;
  const std::vector<TrajectoryRow> base_dynamic_trajectory = run_result.trajectory;
  const RunSummary base_run_summary = run_result.run_summary;
  gtsam::LevenbergMarquardtParams optimizer_params;
  optimizer_params.maxIterations = config_.lm_max_iterations;
  optimizer_params.lambdaInitial = config_.lm_lambda_initial;
  optimizer_params.setVerbosity(config_.verbose ? "ERROR" : "SILENT");
  optimizer_params.setVerbosityLM(config_.verbose ? "TRYLAMBDA" : "SILENT");

  if (config_.enable_body_z_seed_jump_windows) {
    BodyZWindowPipelineRequest body_z_request;
    body_z_request.config = &config_;
    body_z_request.imu_samples = &dataset.imu_samples;
    body_z_request.gnss_samples = &dataset.gnss_samples;
    body_z_request.state_timestamps = &state_timestamps;
    body_z_request.base_graph = &base_graph;
    body_z_request.base_initial_values = &base_initial_values;
    body_z_request.optimizer_params = optimizer_params;
    body_z_request.navigation_start_index = navigation_start_index;
    body_z_request.dynamic_start_time_s = dynamic_start_time_s;
    body_z_request.end_time_s = state_timestamps.back();
    body_z_request.passes_gnss_quality_filters = [&](const GnssSolutionSample &sample) {
      return PassesGnssQualityFilters(sample);
    };
    body_z_request.is_within_imu_coverage = [&](const double corrected_time_s) {
      return IsWithinImuCoverage(dataset.imu_samples, corrected_time_s);
    };
    body_z_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
      return CorrectedGnssTime(sample);
    };
    body_z_request.clamped_sigma_m = [&](const GnssSolutionSample &sample) {
      return ClampGnssSigma(sample);
    };
    body_z_request.find_state_for_time_s = [&](const double corrected_time_s) {
      return FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
    };
    const BodyZWindowPipelineResult body_z_result = BodyZWindowPipeline(std::move(body_z_request)).Run();
    run_result.seed_body_z_acc_diagnostics = body_z_result.imu_diagnostics;
    run_result.body_z_seed_jump_windows = body_z_result.jump_windows;
  }

  run_result.run_summary = base_run_summary;
  run_result.run_summary.gnss_factor_count = 0;
  run_result.run_summary.gnss_synced_factor_count = 0;
  run_result.run_summary.gnss_interpolated_factor_count = 0;
  run_result.run_summary.gnss_dropped_count = 0;
  run_result.run_summary.gnss_cached_count = 0;
  run_result.run_summary.dropped_non_rtkfix_count = 0;
  run_result.run_summary.dropped_no_solution_count = 0;
  run_result.run_summary.dropped_nonfinite_sigma_count = 0;
  run_result.run_summary.dropped_bad_status_count = 0;
  run_result.run_summary.dropped_out_of_imu_coverage_count = 0;
  run_result.run_summary.vertical_gate_inside_count = 0;
  run_result.run_summary.vertical_gate_outside_count = 0;
  run_result.run_summary.gnss_nis_mean = std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.gnss_nis_median = std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.gnss_nis_p95 = std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.axis_2sigma_pass_rate = std::numeric_limits<double>::quiet_NaN();
  run_result.trajectory = base_dynamic_trajectory;
  run_result.gnss_factor_records.clear();
  run_result.gnss_consistency_records.clear();
  run_result.vertical_local_recovery_iterations.clear();
  run_result.vertical_state_corrections.clear();

  gtsam::NonlinearFactorGraph graph_with_gnss = base_graph;
  GnssFactorBuildRequest gnss_request;
  gnss_request.config = &config_;
  gnss_request.gnss_samples = &dataset.gnss_samples;
  gnss_request.navigation_start_index = navigation_start_index;
  gnss_request.graph = &graph_with_gnss;
  gnss_request.trajectory = &run_result.trajectory;
  gnss_request.run_summary = &run_result.run_summary;
  gnss_request.factor_records = &run_result.gnss_factor_records;
  gnss_request.consistency_records = &run_result.gnss_consistency_records;
  gnss_request.collect_consistency_records = collect_gnss_consistency;
  gnss_request.dynamic_start_time_s = dynamic_start_time_s;
  gnss_request.should_use_sample = [&](const GnssSolutionSample &sample) {
    return ShouldUseGnssFactor(sample, run_result.run_summary);
  };
  gnss_request.is_within_imu_coverage = [&](const double corrected_time_s) {
    return IsWithinImuCoverage(dataset.imu_samples, corrected_time_s);
  };
  gnss_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
    return CorrectedGnssTime(sample);
  };
  gnss_request.clamped_sigma_m = [&](const GnssSolutionSample &sample) {
    return ClampGnssSigma(sample);
  };
  gnss_request.find_state_for_time_s = [&](const double corrected_time_s) {
    return FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
  };
  gnss_request.trajectory_row_index_for_state = graph_state_to_trajectory_row;
  GnssFactorBuilder(std::move(gnss_request)).Build();

  gtsam::LevenbergMarquardtOptimizer optimizer(
    graph_with_gnss,
    base_initial_values,
    optimizer_params);
  run_result.run_summary.initial_error = optimizer.error();
  const gtsam::Values optimized_values = optimizer.optimize();
  run_result.run_summary.final_error = graph_with_gnss.error(optimized_values);

  PopulateGnssPostfitResiduals(
    optimized_values,
    base_interpolator,
    trajectory_row_index_by_state,
    run_result.gnss_factor_records,
    collect_gnss_consistency ? &run_result.gnss_consistency_records : nullptr,
    &run_result.trajectory);

  if (collect_reference_states) {
    const std::vector<ReferenceNodeState> optimized_reference_states =
      BuildReferenceStatesFromOptimizedValues(state_timestamps, optimized_values);
    run_result.reference_node_trajectory.clear();
    run_result.reference_node_trajectory.reserve(optimized_reference_states.size());
    for (const auto &reference_state : optimized_reference_states) {
      run_result.reference_node_trajectory.push_back(MakeReferenceNodeRow(reference_state));
    }
  }
  AccumulateStaticConsistencyMetrics(optimized_values, graph_timeline, run_result.run_summary);
  run_result.error_state_trajectory.clear();
  if (graph_timeline.dynamic_start_index < state_timestamps.size()) {
    const std::size_t first_dynamic_index = graph_timeline.dynamic_start_index;
    const auto first_dynamic_pose = optimized_values.at<gtsam::Pose3>(X(first_dynamic_index));
    const auto first_dynamic_velocity = optimized_values.at<gtsam::Vector3>(V(first_dynamic_index));
    const auto first_dynamic_bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(first_dynamic_index));
    const ForwardDriftSummary drift_summary = ComputeFeedbackForwardDriftSummary(
      dataset.imu_samples,
      first_dynamic_pose,
      first_dynamic_velocity,
      first_dynamic_bias,
      state_timestamps[first_dynamic_index],
      state_timestamps.back(),
      config_.gravity_mps2);
    run_result.run_summary.feedback_forward_up_slope_10s = drift_summary.up_slope_10s;
    run_result.run_summary.feedback_forward_up_slope_30s = drift_summary.up_slope_30s;
    run_result.run_summary.feedback_forward_horizontal_slope_10s = drift_summary.horizontal_slope_10s;
    run_result.run_summary.feedback_forward_horizontal_slope_30s = drift_summary.horizontal_slope_30s;
    const auto first_dynamic_forward_rows = BuildForwardTrajectoryRows(
      dataset.imu_samples,
      first_dynamic_pose,
      first_dynamic_velocity,
      first_dynamic_bias,
      state_timestamps[first_dynamic_index],
      30.0,
      config_.gravity_mps2);
    AccumulateForwardTrajectoryVariationSummary(
      first_dynamic_forward_rows,
      run_result.run_summary.forward_first30s_up_total_variation_m,
      run_result.run_summary.forward_first30s_vz_total_variation_mps);
  }

  for (std::size_t index = 0; index < run_result.trajectory.size(); ++index) {
    const std::size_t graph_index = index;
    const auto pose = optimized_values.at<gtsam::Pose3>(X(graph_index));
    const auto velocity = optimized_values.at<gtsam::Vector3>(V(graph_index));
    const auto bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(graph_index));
    const auto omega = optimized_values.at<gtsam::Vector3>(W(graph_index));

    auto &row = run_result.trajectory[index];
    row.enu_position_m = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
    row.enu_velocity_mps = Eigen::Vector3d(velocity.x(), velocity.y(), velocity.z());
    row.ypr_rad = Rot3ToYpr(pose.rotation());
    row.omega_radps = Eigen::Vector3d(omega.x(), omega.y(), omega.z());
    row.bias_acc = bias.accelerometer();
    row.bias_gyro = bias.gyroscope();
  }

  run_result.initial_static_trajectory.clear();
  if (graph_timeline.initial_static_state_count > 0U) {
    run_result.initial_static_trajectory.reserve(graph_timeline.initial_static_state_count);
    for (std::size_t graph_index = 0; graph_index < graph_timeline.initial_static_state_count; ++graph_index) {
      TrajectoryRow row;
      row.time_s = state_timestamps[graph_index];
      const auto pose = optimized_values.at<gtsam::Pose3>(X(graph_index));
      const auto velocity = optimized_values.at<gtsam::Vector3>(V(graph_index));
      const auto bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(graph_index));
      const auto omega = optimized_values.at<gtsam::Vector3>(W(graph_index));
      row.enu_position_m = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
      row.enu_velocity_mps = Eigen::Vector3d(velocity.x(), velocity.y(), velocity.z());
      row.ypr_rad = Rot3ToYpr(pose.rotation());
      row.omega_radps = Eigen::Vector3d(omega.x(), omega.y(), omega.z());
      row.bias_acc = bias.accelerometer();
      row.bias_gyro = bias.gyroscope();
      run_result.initial_static_trajectory.push_back(row);
    }
  }
  run_result.run_summary.initial_static_trajectory_count = run_result.initial_static_trajectory.size();
  run_result.optimized_static_terminal_forward_trajectory.clear();
  std::optional<TrajectoryRow> optimized_last_static_row;
  if (!run_result.initial_static_trajectory.empty()) {
    const std::size_t static_terminal_graph_index = graph_timeline.initial_static_state_count - 1U;
    const auto static_terminal_pose = optimized_values.at<gtsam::Pose3>(X(static_terminal_graph_index));
    const auto static_terminal_velocity = optimized_values.at<gtsam::Vector3>(V(static_terminal_graph_index));
    const auto static_terminal_bias =
      optimized_values.at<gtsam::imuBias::ConstantBias>(B(static_terminal_graph_index));
    optimized_last_static_row = run_result.initial_static_trajectory.back();
    const auto &row = *optimized_last_static_row;
    run_result.optimized_static_terminal_forward_trajectory = BuildForwardTrajectoryRows(
      dataset.imu_samples,
      static_terminal_pose,
      static_terminal_velocity,
      static_terminal_bias,
      row.time_s,
      20.0,
      config_.gravity_mps2);
  }

  std::vector<TrajectoryRow> dynamic_trajectory_rows;
  if (graph_timeline.dynamic_start_index < run_result.trajectory.size()) {
    dynamic_trajectory_rows.assign(
      run_result.trajectory.begin() + static_cast<std::ptrdiff_t>(graph_timeline.dynamic_start_index),
      run_result.trajectory.end());
  }
  AccumulateInitialDynamicConsistencyMetrics(
    dynamic_trajectory_rows,
    initial_bias,
    optimized_last_static_row,
    run_result.optimized_static_terminal_forward_trajectory,
    run_result.run_summary);

  if (config_.write_imu_rate_avp) {
    std::vector<OptimizedNodeState> optimized_node_states;
    optimized_node_states.reserve(run_result.trajectory.size());
    for (std::size_t index = graph_timeline.dynamic_start_index; index < state_timestamps.size(); ++index) {
      OptimizedNodeState node_state;
      node_state.time_s = state_timestamps[index];
      node_state.pose = optimized_values.at<gtsam::Pose3>(X(index));
      node_state.velocity = optimized_values.at<gtsam::Vector3>(V(index));
      node_state.bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(index));
      optimized_node_states.push_back(node_state);
    }

    ImuRateAvpReconstructionResult imu_rate_result =
      ImuRateAvpReconstructor::Reconstruct(
        dataset.imu_samples,
        optimized_node_states,
        imu_params,
        config_.verbose);
    run_result.imu_rate_avp = std::move(imu_rate_result.rows);
    run_result.imu_rate_interval_diagnostics = std::move(imu_rate_result.diagnostics);
    run_result.run_summary.imu_rate_avp_count = run_result.imu_rate_avp.size();
    run_result.run_summary.imu_rate_interval_count =
      optimized_node_states.size() > 1U ? optimized_node_states.size() - 1U : 0U;
    run_result.run_summary.imu_rate_skipped_interval_count =
      std::count_if(
        run_result.imu_rate_interval_diagnostics.begin(),
        run_result.imu_rate_interval_diagnostics.end(),
        [](const ImuRateIntervalDiagnostic &diagnostic) { return !diagnostic.used_interval; });
  }

  if (config_.enable_gnss) {
    if (collect_gnss_consistency) {
      std::vector<double> postfit_nis_values;
      std::size_t axis_pass_count = 0U;
      std::size_t axis_total_count = 0U;
      for (const auto &record : run_result.gnss_consistency_records) {
        if (!record.factor_used || !std::isfinite(record.postfit_nis)) {
          continue;
        }
        postfit_nis_values.push_back(record.postfit_nis);
        const Eigen::Vector3d scaled_sigma_m(
          record.sigma_e_m * record.covariance_scale_e,
          record.sigma_n_m * record.covariance_scale_n,
          record.vertical_sigma_u_used_m);
        const int axis_limit = record.vertical_direct_position_factor_used ? 3 : 2;
        for (int axis = 0; axis < axis_limit; ++axis) {
          if (!std::isfinite(scaled_sigma_m[axis]) || scaled_sigma_m[axis] <= 0.0 ||
              !std::isfinite(record.postfit_residual_enu_m[axis])) {
            continue;
          }
          ++axis_total_count;
          if (std::abs(record.postfit_residual_enu_m[axis]) <= config_.gnss_axis_sigma_multiple * scaled_sigma_m[axis]) {
            ++axis_pass_count;
          }
        }
      }
      const ScalarSummaryStats nis_stats = ComputeScalarSummaryStats(std::move(postfit_nis_values));
      run_result.run_summary.gnss_nis_mean = nis_stats.mean;
      run_result.run_summary.gnss_nis_median = nis_stats.median;
      run_result.run_summary.gnss_nis_p95 = nis_stats.p95;
      run_result.run_summary.axis_2sigma_pass_rate =
        axis_total_count > 0U
          ? static_cast<double>(axis_pass_count) / static_cast<double>(axis_total_count)
          : std::numeric_limits<double>::quiet_NaN();
    }

    run_result.vertical_state_corrections.clear();
    run_result.vertical_state_corrections.reserve(run_result.gnss_factor_records.size());
    const std::vector<ReferenceNodeState> optimized_reference_states =
      BuildReferenceStatesFromOptimizedValues(state_timestamps, optimized_values);
    for (std::size_t record_index = 0; record_index < run_result.gnss_factor_records.size(); ++record_index) {
      const auto &record = run_result.gnss_factor_records[record_index];
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

      if (collect_gnss_consistency && record_index < run_result.gnss_consistency_records.size()) {
        const auto &consistency_record = run_result.gnss_consistency_records[record_index];
        row.vertical_gate_inside = consistency_record.vertical_gate_inside;
        row.vertical_direct_position_factor_used = consistency_record.vertical_direct_position_factor_used;
        row.prefit_residual_u_m = consistency_record.prefit_residual_enu_m.z();
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

      run_result.vertical_state_corrections.push_back(row);
    }
  }

  if (collect_segment_error_diagnostics) {
    run_result.segment_error_diagnostics = BuildSegmentErrorDiagnostics(
      state_timestamps,
      dataset.imu_samples,
      imu_params,
      optimized_values,
      run_result.gnss_factor_records,
      run_result.gnss_consistency_records,
      config_);
    run_result.run_summary.segment_error_count = run_result.segment_error_diagnostics.size();
  }

  return run_result;
}

}  // namespace offline_lc_minimal
