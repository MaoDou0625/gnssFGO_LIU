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
#include <unordered_set>
#include <vector>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/core/InitialStaticConstraintBuilder.h"
#include "offline_lc_minimal/core/BodyZBidirectionalJumpDetector.h"
#include "offline_lc_minimal/core/ImuRateAvpReconstructor.h"
#include "offline_lc_minimal/core/SequentialNhcJumpDetector.h"
#include "offline_lc_minimal/core/SparseVerticalJumpPlanner.h"
#include "offline_lc_minimal/core/TrajectoryInitializer.h"
#include "offline_lc_minimal/core/VerticalInsideBiasAdapter.h"
#include "offline_lc_minimal/factor/AngularRateFactor.h"
#include "offline_lc_minimal/factor/BiasGmTransitionFactor.h"
#include "offline_lc_minimal/factor/GPInterpolatedGPSFactor.h"
#include "offline_lc_minimal/factor/GPInterpolatedHorizontalPositionFactor.h"
#include "offline_lc_minimal/factor/GlobalAccelBiasFactor.h"
#include "offline_lc_minimal/factor/GlobalGyroBiasFactor.h"
#include "offline_lc_minimal/factor/GlobalPlanarAccelBiasFactor.h"
#include "offline_lc_minimal/factor/HorizontalPositionFactor.h"
#include "offline_lc_minimal/factor/ReweightedCombinedImuFactor.h"
#include "offline_lc_minimal/factor/SegmentBiasFeedbackFactor.h"
#include "offline_lc_minimal/factor/StaticZeroAngularRateFactor.h"
#include "offline_lc_minimal/factor/StaticSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticAttitudeDriftFactor.h"
#include "offline_lc_minimal/factor/VerticalAccelBiasGmTransitionFactor.h"
#include "offline_lc_minimal/factor/VerticalInsideKinematicFactor.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {

namespace {

constexpr double kNumericalSigmaFloorM = 1e-4;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimeEpsilonS = 1e-9;
constexpr double kChiSquareDegreesOfFreedom = 3.0;
constexpr double kAngularRateSigmaRadps = 0.1;
constexpr double kInterpolatorQcVariance = 10000.0;
constexpr double kDisabledAccBiasPriorSigmaMps2 = 1e6;
constexpr double kDisabledGyroBiasPriorSigmaRadps = 1e6;

struct GraphTimeline {
  std::vector<double> timestamps_s;
  std::size_t dynamic_start_index = 0;
  std::size_t initial_static_state_count = 0;
};

struct VerticalHoldWindowSpec {
  std::size_t sample_index = 0;
  double corrected_time_s = 0.0;
  Eigen::Vector3d measurement_enu_m = Eigen::Vector3d::Zero();
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  std::size_t reference_state_index = 0;
  bool interpolated = false;
};

struct VerticalHoldWindowEvaluation {
  bool current_inside = false;
  bool hold_window_passed = false;
  double gate_excess_cost = std::numeric_limits<double>::infinity();
  double current_local_postfit_u_m = std::numeric_limits<double>::quiet_NaN();
  double max_up_from_vz_error_m = std::numeric_limits<double>::quiet_NaN();
};

struct VerticalFutureTrendEvaluation {
  bool valid = false;
  std::size_t fix_count = 0;
  double residual_mean_m = std::numeric_limits<double>::quiet_NaN();
  double residual_slope_mps = std::numeric_limits<double>::quiet_NaN();
  double cost = 0.0;
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

struct VerticalRtkFeedbackWindowAccumulator {
  std::optional<std::size_t> start_state_index;
  double start_vertical_residual_m = 0.0;
  double end_vertical_residual_m = 0.0;
  double feedback_gain_scale_sum = 0.0;
  std::size_t sample_count = 0;

  void Start(const std::size_t anchor_state_index, const double vertical_residual_m, const double feedback_gain_scale) {
    start_state_index = anchor_state_index;
    start_vertical_residual_m = vertical_residual_m;
    end_vertical_residual_m = vertical_residual_m;
    feedback_gain_scale_sum = feedback_gain_scale;
    sample_count = 1;
  }

  void UpdateStartAnchor(const double vertical_residual_m, const double feedback_gain_scale) {
    start_vertical_residual_m = vertical_residual_m;
    end_vertical_residual_m = vertical_residual_m;
    feedback_gain_scale_sum = feedback_gain_scale;
    sample_count = 1;
  }

  void AccumulateFutureAnchor(const double vertical_residual_m, const double feedback_gain_scale) {
    end_vertical_residual_m = vertical_residual_m;
    feedback_gain_scale_sum += feedback_gain_scale;
    ++sample_count;
  }

  [[nodiscard]] bool HasSamples() const {
    return sample_count > 0U;
  }

  [[nodiscard]] double ResidualDriftM() const {
    return sample_count > 0U
             ? end_vertical_residual_m - start_vertical_residual_m
             : std::numeric_limits<double>::quiet_NaN();
  }

  [[nodiscard]] double MeanFeedbackGainScale() const {
    return sample_count > 0U
             ? feedback_gain_scale_sum / static_cast<double>(sample_count)
             : std::numeric_limits<double>::quiet_NaN();
  }
};

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

double WrapAngleRad(const double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
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

Eigen::Vector3d ComputePositionResidualEnu(
  const gtsam::Pose3 &pose,
  const Eigen::Vector3d &measurement_enu_m);
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

double ComputeVerticalNis(const double residual_u_m, const double sigma_u_m) {
  const double variance = std::max(sigma_u_m * sigma_u_m, 1e-12);
  return residual_u_m * residual_u_m / variance;
}

double ComputeUpFromVzConsistencyError(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::size_t start_index,
  const std::size_t end_index) {
  if (reference_states.empty() || start_index >= reference_states.size() || end_index >= reference_states.size() ||
      start_index >= end_index) {
    return 0.0;
  }
  double integrated_up_m = reference_states[start_index].pose.translation().z();
  double max_abs_error_m = 0.0;
  for (std::size_t state_index = start_index + 1U; state_index <= end_index; ++state_index) {
    const double dt_s = std::max(reference_states[state_index].time_s - reference_states[state_index - 1U].time_s, 0.0);
    const double average_vz_mps =
      0.5 * (reference_states[state_index - 1U].velocity.z() + reference_states[state_index].velocity.z());
    integrated_up_m += average_vz_mps * dt_s;
    const double error_m = reference_states[state_index].pose.translation().z() - integrated_up_m;
    max_abs_error_m = std::max(max_abs_error_m, std::abs(error_m));
  }
  return max_abs_error_m;
}

ReferenceNodeState ResolveReferenceStateForHoldWindowSpec(
  const std::vector<ReferenceNodeState> &reference_states,
  const VerticalHoldWindowSpec &spec) {
  return spec.interpolated
           ? InterpolateReferenceState(reference_states, spec.corrected_time_s)
           : reference_states[spec.reference_state_index];
}

std::vector<VerticalHoldWindowSpec> FilterVerticalHoldWindowSpecsAfterState(
  const std::vector<VerticalHoldWindowSpec> &hold_window_specs,
  const std::size_t state_index) {
  std::vector<VerticalHoldWindowSpec> filtered_specs;
  filtered_specs.reserve(hold_window_specs.size());
  for (const auto &spec : hold_window_specs) {
    if (spec.reference_state_index > state_index) {
      filtered_specs.push_back(spec);
    }
  }
  return filtered_specs;
}

VerticalHoldWindowEvaluation EvaluateVerticalHoldWindow(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalHoldWindowSpec> &hold_window_specs,
  const std::size_t up_anchor_state_index,
  const double vertical_gate_nis_threshold) {
  VerticalHoldWindowEvaluation evaluation;
  evaluation.gate_excess_cost = 0.0;
  evaluation.current_inside = false;
  evaluation.hold_window_passed = false;
  if (hold_window_specs.empty() || reference_states.empty()) {
    return evaluation;
  }

  std::size_t max_reference_state_index = up_anchor_state_index;
  bool all_inside = true;
  for (std::size_t spec_index = 0; spec_index < hold_window_specs.size(); ++spec_index) {
    const auto &spec = hold_window_specs[spec_index];
    const ReferenceNodeState state = ResolveReferenceStateForHoldWindowSpec(reference_states, spec);
    const Eigen::Vector3d residual_enu_m = ComputePositionResidualEnu(state.pose, spec.measurement_enu_m);
    const double vertical_nis = ComputeVerticalNis(residual_enu_m.z(), spec.sigma_u_m);
    const double gate_threshold_m = std::sqrt(vertical_gate_nis_threshold) * spec.sigma_u_m;
    const double excess_m = std::max(0.0, std::abs(residual_enu_m.z()) - gate_threshold_m);
    evaluation.gate_excess_cost += excess_m * excess_m;
    if (spec_index == 0U) {
      evaluation.current_local_postfit_u_m = residual_enu_m.z();
      evaluation.current_inside = vertical_nis <= vertical_gate_nis_threshold;
    }
    if (vertical_nis > vertical_gate_nis_threshold) {
      all_inside = false;
    }
    max_reference_state_index = std::max(max_reference_state_index, spec.reference_state_index);
  }
  evaluation.max_up_from_vz_error_m =
    ComputeUpFromVzConsistencyError(reference_states, up_anchor_state_index, max_reference_state_index);
  evaluation.hold_window_passed =
    evaluation.current_inside && all_inside && evaluation.max_up_from_vz_error_m <= 0.03;
  return evaluation;
}

VerticalFutureTrendEvaluation EvaluateVerticalFutureTrend(
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalHoldWindowSpec> &future_trend_specs,
  const int minimum_fix_count,
  const double mean_weight,
  const double slope_weight) {
  VerticalFutureTrendEvaluation evaluation;
  evaluation.cost = 0.0;
  if (future_trend_specs.empty() || reference_states.empty() ||
      minimum_fix_count <= 0 || (mean_weight <= 0.0 && slope_weight <= 0.0)) {
    return evaluation;
  }

  std::vector<double> times_s;
  std::vector<double> residuals_m;
  times_s.reserve(future_trend_specs.size());
  residuals_m.reserve(future_trend_specs.size());
  for (const auto &spec : future_trend_specs) {
    const ReferenceNodeState state = ResolveReferenceStateForHoldWindowSpec(reference_states, spec);
    const Eigen::Vector3d residual_enu_m = ComputePositionResidualEnu(state.pose, spec.measurement_enu_m);
    if (!std::isfinite(spec.corrected_time_s) || !std::isfinite(residual_enu_m.z())) {
      continue;
    }
    times_s.push_back(spec.corrected_time_s);
    residuals_m.push_back(residual_enu_m.z());
  }
  evaluation.fix_count = residuals_m.size();
  if (residuals_m.size() < static_cast<std::size_t>(minimum_fix_count)) {
    return evaluation;
  }

  const double time_origin_s = times_s.front();
  double time_sum_s = 0.0;
  double residual_sum_m = 0.0;
  for (std::size_t sample_index = 0; sample_index < residuals_m.size(); ++sample_index) {
    time_sum_s += times_s[sample_index] - time_origin_s;
    residual_sum_m += residuals_m[sample_index];
  }
  const double inv_count = 1.0 / static_cast<double>(residuals_m.size());
  const double mean_time_s = time_sum_s * inv_count;
  const double mean_residual_m = residual_sum_m * inv_count;

  double time_variance_sum = 0.0;
  double covariance_sum = 0.0;
  for (std::size_t sample_index = 0; sample_index < residuals_m.size(); ++sample_index) {
    const double centered_time_s = times_s[sample_index] - time_origin_s - mean_time_s;
    const double centered_residual_m = residuals_m[sample_index] - mean_residual_m;
    time_variance_sum += centered_time_s * centered_time_s;
    covariance_sum += centered_time_s * centered_residual_m;
  }

  const double slope_mps =
    time_variance_sum > 1e-12 ? covariance_sum / time_variance_sum : 0.0;
  evaluation.valid = true;
  evaluation.residual_mean_m = mean_residual_m;
  evaluation.residual_slope_mps = slope_mps;
  evaluation.cost =
    mean_weight * mean_residual_m * mean_residual_m +
    slope_weight * slope_mps * slope_mps;
  return evaluation;
}

double InverseNormalCdf(const double probability) {
  if (probability <= 0.0 || probability >= 1.0) {
    throw std::runtime_error("inverse normal CDF requires probability in (0, 1)");
  }

  constexpr double a1 = -3.969683028665376e+01;
  constexpr double a2 = 2.209460984245205e+02;
  constexpr double a3 = -2.759285104469687e+02;
  constexpr double a4 = 1.383577518672690e+02;
  constexpr double a5 = -3.066479806614716e+01;
  constexpr double a6 = 2.506628277459239e+00;
  constexpr double b1 = -5.447609879822406e+01;
  constexpr double b2 = 1.615858368580409e+02;
  constexpr double b3 = -1.556989798598866e+02;
  constexpr double b4 = 6.680131188771972e+01;
  constexpr double b5 = -1.328068155288572e+01;
  constexpr double c1 = -7.784894002430293e-03;
  constexpr double c2 = -3.223964580411365e-01;
  constexpr double c3 = -2.400758277161838e+00;
  constexpr double c4 = -2.549732539343734e+00;
  constexpr double c5 = 4.374664141464968e+00;
  constexpr double c6 = 2.938163982698783e+00;
  constexpr double d1 = 7.784695709041462e-03;
  constexpr double d2 = 3.224671290700398e-01;
  constexpr double d3 = 2.445134137142996e+00;
  constexpr double d4 = 3.754408661907416e+00;
  constexpr double p_low = 0.02425;
  constexpr double p_high = 1.0 - p_low;

  if (probability < p_low) {
    const double q = std::sqrt(-2.0 * std::log(probability));
    return (((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
           ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
  }
  if (probability > p_high) {
    const double q = std::sqrt(-2.0 * std::log(1.0 - probability));
    return -(((((c1 * q + c2) * q + c3) * q + c4) * q + c5) * q + c6) /
           ((((d1 * q + d2) * q + d3) * q + d4) * q + 1.0);
  }

  const double q = probability - 0.5;
  const double r = q * q;
  return (((((a1 * r + a2) * r + a3) * r + a4) * r + a5) * r + a6) * q /
         (((((b1 * r + b2) * r + b3) * r + b4) * r + b5) * r + 1.0);
}

double ChiSquareQuantile1D(const double confidence) {
  const double z = InverseNormalCdf(0.5 * (1.0 + confidence));
  return z * z;
}

double ChiSquareQuantile3D(const double confidence) {
  const double z = InverseNormalCdf(confidence);
  const double term =
    1.0 - (2.0 / (9.0 * kChiSquareDegreesOfFreedom)) +
    z * std::sqrt(2.0 / (9.0 * kChiSquareDegreesOfFreedom));
  return kChiSquareDegreesOfFreedom * term * term * term;
}

double ComputeConsistencyScale(
  const GnssConsistencyGateMode gate_mode,
  const double nis,
  const double nis_threshold,
  const double relaxed_threshold_ratio,
  const double max_scale) {
  if (gate_mode != GnssConsistencyGateMode::kNis || !std::isfinite(nis) || !std::isfinite(nis_threshold) ||
      nis <= 0.0) {
    return 1.0;
  }
  const double relaxed_threshold = std::max(relaxed_threshold_ratio * nis_threshold, 1e-6);
  if (nis >= relaxed_threshold) {
    return nis > nis_threshold
             ? std::clamp(std::sqrt(nis / std::max(nis_threshold, 1e-9)), 1.0, max_scale)
             : 1.0;
  }
  return std::clamp(std::sqrt(relaxed_threshold / std::max(nis, 1e-9)), 1.0, max_scale);
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

std::optional<std::size_t> ResolveVerticalFeedbackAnchorState(
  const GnssFactorRecord &record,
  const std::size_t dynamic_start_index) {
  if (IsSynchronizedStatus(record.sync_status)) {
    if (record.synchronized_state_index < dynamic_start_index) {
      return std::nullopt;
    }
    return record.synchronized_state_index;
  }
  if (record.sync_status == StateMeasSyncStatus::kInterpolated &&
      record.state_index_j > record.state_index_i) {
    if (record.state_index_j < dynamic_start_index) {
      return std::nullopt;
    }
    return record.state_index_j;
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

gtsam::SharedNoiseModel WrapGnssNoiseModel(
  const OfflineRunnerConfig &config,
  const gtsam::SharedNoiseModel &gaussian_model) {
  switch (config.gnss_position_noise_model) {
    case GnssNoiseModel::kGaussian:
      return gaussian_model;
    case GnssNoiseModel::kCauchy:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kHuber:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kDcs:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::DCS::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kTukey:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Tukey::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kGemanMcClure:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::GemanMcClure::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kWelsch:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Welsch::Create(config.gnss_position_robust_param),
        gaussian_model);
    default:
      return gaussian_model;
  }
}

gtsam::SharedNoiseModel MakeGnssNoiseModel(const OfflineRunnerConfig &config, const Eigen::Vector3d &sigma_m) {
  const gtsam::Vector3 variances(
    sigma_m.x() * sigma_m.x(),
    sigma_m.y() * sigma_m.y(),
    sigma_m.z() * sigma_m.z());
  return WrapGnssNoiseModel(config, gtsam::noiseModel::Diagonal::Variances(variances));
}

gtsam::SharedNoiseModel MakeHorizontalGnssNoiseModel(const OfflineRunnerConfig &config, const Eigen::Vector2d &sigma_m) {
  const gtsam::Vector2 variances(
    sigma_m.x() * sigma_m.x(),
    sigma_m.y() * sigma_m.y());
  return WrapGnssNoiseModel(config, gtsam::noiseModel::Diagonal::Variances(variances));
}

void UpsertReferenceStateInitialValues(
  const std::size_t state_index,
  const ReferenceNodeState &state,
  gtsam::Values *values) {
  const auto upsert_pose = [&](const gtsam::Key key, const gtsam::Pose3 &pose) {
    if (values->exists(key)) {
      values->update(key, pose);
    } else {
      values->insert(key, pose);
    }
  };
  const auto upsert_vector3 = [&](const gtsam::Key key, const gtsam::Vector3 &vector) {
    if (values->exists(key)) {
      values->update(key, vector);
    } else {
      values->insert(key, vector);
    }
  };
  const auto upsert_bias = [&](const gtsam::Key key, const gtsam::imuBias::ConstantBias &bias) {
    if (values->exists(key)) {
      values->update(key, bias);
    } else {
      values->insert(key, bias);
    }
  };

  upsert_pose(X(state_index), state.pose);
  upsert_vector3(V(state_index), state.velocity);
  upsert_bias(B(state_index), state.bias);
  upsert_vector3(W(state_index), state.omega);
}

void EnsureReferenceStatesValidUntil(
  const std::vector<double> &state_timestamps,
  const std::vector<ImuSample> &imu_samples,
  const boost::shared_ptr<gtsam::PreintegrationCombinedParams> &imu_params,
  std::vector<ReferenceNodeState> *reference_states,
  std::size_t *valid_until_index,
  const std::size_t required_index,
  gtsam::Values *initial_values) {
  if (reference_states == nullptr || valid_until_index == nullptr) {
    throw std::runtime_error("reference state propagation requires valid storage");
  }
  if (reference_states->empty() || required_index >= reference_states->size()) {
    throw std::runtime_error("reference state propagation requested an invalid index");
  }
  if (*valid_until_index >= required_index) {
    return;
  }

  gtsam::NavState previous_nav_state(
    (*reference_states)[*valid_until_index].pose,
    (*reference_states)[*valid_until_index].velocity);
  gtsam::imuBias::ConstantBias previous_bias = (*reference_states)[*valid_until_index].bias;
  double previous_time_s = (*reference_states)[*valid_until_index].time_s;

  for (std::size_t state_index = *valid_until_index + 1U; state_index <= required_index; ++state_index) {
    const double current_time_s = state_timestamps[state_index];
    const auto imu_window =
      IntegrateImuWindow(imu_samples, previous_time_s, current_time_s, imu_params, previous_bias);
    const gtsam::NavState predicted_state =
      imu_window.preintegrated_measurements.predict(previous_nav_state, previous_bias);
    (*reference_states)[state_index] = MakeReferenceNodeState(
      current_time_s,
      predicted_state,
      previous_bias,
      imu_window.end_gyro_radps);
    if (initial_values != nullptr) {
      UpsertReferenceStateInitialValues(state_index, (*reference_states)[state_index], initial_values);
    }
    previous_nav_state = predicted_state;
    previous_time_s = current_time_s;
  }

  *valid_until_index = required_index;
}

struct VerticalLocalRecoveryResult {
  ReferenceNodeState recovered_anchor_state;
  double local_postfit_u_m = std::numeric_limits<double>::quiet_NaN();
  double required_up_anchor_correction_m = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_applied_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_up_anchor_applied_m = std::numeric_limits<double>::quiet_NaN();
  double delta_roll_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_pitch_applied_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_baz_applied_mps2 = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_state_index = -1;
  double selected_jump_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double jump_candidate_score = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_window_start_state_index = -1;
  long long selected_jump_window_center_state_index = -1;
  long long selected_jump_window_end_state_index = -1;
  double selected_jump_window_duration_s = std::numeric_limits<double>::quiet_NaN();
  long long selected_jump_window_point_count = 0;
  double selected_jump_delta_vz_tail_mps = std::numeric_limits<double>::quiet_NaN();
  double window_velocity_smooth_cost = std::numeric_limits<double>::quiet_NaN();
  double window_height_integral_delta_m = std::numeric_limits<double>::quiet_NaN();
  double future_trend_residual_mean_m = std::numeric_limits<double>::quiet_NaN();
  double future_trend_residual_slope_mps = std::numeric_limits<double>::quiet_NaN();
  double future_trend_cost = std::numeric_limits<double>::quiet_NaN();
  long long future_trend_fix_count = 0;
  std::string recovery_mode = "NONE";
  bool hold_window_passed = false;
};

ReferenceNodeState ApplyVerticalUpAnchorCorrection(
  const ReferenceNodeState &anchor_state,
  const double delta_up_anchor_m) {
  ReferenceNodeState corrected_anchor_state = anchor_state;
  corrected_anchor_state.pose = gtsam::Pose3(
    anchor_state.pose.rotation(),
    gtsam::Point3(
      anchor_state.pose.translation().x(),
      anchor_state.pose.translation().y(),
      anchor_state.pose.translation().z() + delta_up_anchor_m));
  return corrected_anchor_state;
}

ReferenceNodeState ApplyInsideLowFrequencyStateCorrection(
  const ReferenceNodeState &anchor_state,
  const VerticalInsideBiasUpdate &update) {
  ReferenceNodeState corrected_anchor_state = anchor_state;
  const Eigen::Vector3d anchor_ypr = Rot3ToYpr(anchor_state.pose.rotation());
  corrected_anchor_state.pose = gtsam::Pose3(
    gtsam::Rot3::Ypr(
      anchor_ypr.x(),
      anchor_ypr.y() + update.delta_pitch_rad,
      anchor_ypr.z() + update.delta_roll_rad),
    anchor_state.pose.translation());
  corrected_anchor_state.bias = gtsam::imuBias::ConstantBias(
    gtsam::Vector3(
      anchor_state.bias.accelerometer().x(),
      anchor_state.bias.accelerometer().y(),
      anchor_state.bias.accelerometer().z() + update.delta_baz_mps2),
    anchor_state.bias.gyroscope());
  return corrected_anchor_state;
}

double ComputeRequiredUpAnchorCorrectionM(
  const gtsam::Pose3 &propagated_pose,
  const Eigen::Vector3d &measurement_enu_m) {
  return measurement_enu_m.z() - propagated_pose.translation().z();
}

std::size_t ResolvePrefitReferenceRightIndex(const StateMeasSyncResult &sync_result) {
  switch (sync_result.status) {
    case StateMeasSyncStatus::kSynchronizedI:
      return sync_result.key_index_i;
    case StateMeasSyncStatus::kSynchronizedJ:
    case StateMeasSyncStatus::kInterpolated:
      return sync_result.key_index_j;
    case StateMeasSyncStatus::kCached:
    case StateMeasSyncStatus::kDropped:
    default:
      return sync_result.key_index_i;
  }
}

struct VerticalVelocityWindowCorrection {
  std::size_t start_state_index = 0;
  std::size_t center_state_index = 0;
  std::size_t end_state_index = 0;
  double target_tail_up_m = std::numeric_limits<double>::quiet_NaN();
  double target_tail_vz_mps = std::numeric_limits<double>::quiet_NaN();
  double delta_up_tail_m = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_tail_mps = std::numeric_limits<double>::quiet_NaN();
  double velocity_smooth_cost = std::numeric_limits<double>::quiet_NaN();
  double height_integral_delta_m = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> corrected_up_m;
  std::vector<double> corrected_vz_mps;
};

void PushUniqueCandidateValue(std::vector<double> *values, const double value) {
  const bool value_is_nan = std::isnan(value);
  const auto duplicate = std::find_if(
    values->begin(),
    values->end(),
    [&](const double existing_value) {
      if (value_is_nan || std::isnan(existing_value)) {
        return value_is_nan && std::isnan(existing_value);
      }
      return std::abs(existing_value - value) <= 1e-9;
    });
  if (duplicate == values->end()) {
    values->push_back(value);
  }
}

double BodyZWindowCurrentTailOffsetM(
  const std::vector<ReferenceNodeState> &reference_states,
  const SparseVerticalJumpWindowCandidate &window_candidate) {
  if (window_candidate.start_state_index == 0U ||
      window_candidate.end_state_index >= reference_states.size()) {
    return 0.0;
  }
  const auto &pre_window_state = reference_states[window_candidate.start_state_index - 1U];
  const auto &tail_state = reference_states[window_candidate.end_state_index];
  const double window_duration_s = tail_state.time_s - pre_window_state.time_s;
  const double continuity_tail_up_m =
    pre_window_state.pose.translation().z() + pre_window_state.velocity.z() * window_duration_s;
  return tail_state.pose.translation().z() - continuity_tail_up_m;
}

std::vector<double> BuildBodyZTailVelocityTargetsMps(
  const std::vector<ReferenceNodeState> &reference_states,
  const SparseVerticalJumpWindowCandidate &window_candidate,
  const bool velocity_already_corrected,
  const bool velocity_feedback_requested_for_window,
  const double velocity_feedback_delta_mps) {
  std::vector<double> targets;
  if (window_candidate.end_state_index >= reference_states.size()) {
    targets.push_back(std::numeric_limits<double>::quiet_NaN());
    return targets;
  }

  if (!velocity_feedback_requested_for_window || !velocity_already_corrected) {
    PushUniqueCandidateValue(
      &targets,
      velocity_already_corrected
        ? reference_states[window_candidate.end_state_index].velocity.z()
        : std::numeric_limits<double>::quiet_NaN());
    return targets;
  }

  const double feedback_base_vz_mps =
    reference_states[window_candidate.end_state_index].velocity.z();
  PushUniqueCandidateValue(&targets, feedback_base_vz_mps + velocity_feedback_delta_mps);
  return targets;
}

std::vector<double> BuildBodyZTailPositionOffsetsM(
  const std::vector<ReferenceNodeState> &reference_states,
  const SparseVerticalJumpWindowCandidate &window_candidate,
  const bool position_offset_already_corrected) {
  std::vector<double> offsets;
  PushUniqueCandidateValue(
    &offsets,
    position_offset_already_corrected
      ? BodyZWindowCurrentTailOffsetM(reference_states, window_candidate)
      : 0.0);
  return offsets;
}

double SmoothStep01(const double alpha) {
  const double bounded_alpha = std::clamp(alpha, 0.0, 1.0);
  return bounded_alpha * bounded_alpha * (3.0 - 2.0 * bounded_alpha);
}

double MedianFinite(std::vector<double> values) {
  values.erase(
    std::remove_if(values.begin(), values.end(), [](const double value) { return !std::isfinite(value); }),
    values.end());
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle_index = values.size() / 2U;
  if (values.size() % 2U == 0U) {
    return 0.5 * (values[middle_index - 1U] + values[middle_index]);
  }
  return values[middle_index];
}

std::optional<VerticalVelocityWindowCorrection> BuildVerticalVelocityWindowCorrection(
  const OfflineRunnerConfig &config,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const SparseVerticalJumpWindowCandidate &window_candidate,
  const std::size_t segment_end_state_index,
  const double tail_delta_scale = 1.0,
  const double forced_tail_delta_mps = std::numeric_limits<double>::quiet_NaN(),
  const double forced_tail_delta_up_m = std::numeric_limits<double>::quiet_NaN()) {
  if (reference_states.empty() || vertical_vz_reference.size() != reference_states.size() ||
      window_candidate.start_state_index == 0U ||
      window_candidate.start_state_index > window_candidate.end_state_index ||
      window_candidate.end_state_index >= reference_states.size()) {
    return std::nullopt;
  }

  const bool body_z_seed_candidate =
    window_candidate.center_candidate.source == "BODY_Z_SEED_WINDOW";
  const std::size_t previous_state_index = window_candidate.start_state_index - 1U;
  const double previous_up_m = reference_states[previous_state_index].pose.translation().z();
  const double previous_vz_mps = reference_states[previous_state_index].velocity.z();
  const double original_tail_vz_mps = reference_states[window_candidate.end_state_index].velocity.z();
  const std::size_t tail_end_state_index = std::min(segment_end_state_index, reference_states.size() - 1U);
  const double tail_end_time_s =
    reference_states[window_candidate.end_state_index].time_s +
    std::max(config.vertical_jump_window_tail_target_s, 1e-3);
  std::vector<double> tail_reference_values_mps;
  std::size_t tail_reference_start_state_index = window_candidate.end_state_index;
  if (!body_z_seed_candidate) {
    for (std::size_t state_index = tail_reference_start_state_index;
         state_index <= tail_end_state_index;
         ++state_index) {
      if (reference_states[state_index].time_s > tail_end_time_s + kTimeEpsilonS) {
        break;
      }
      if (state_index < vertical_vz_reference.size() &&
          vertical_vz_reference[state_index].valid &&
          std::isfinite(vertical_vz_reference[state_index].vz_ref_global_smoothed_mps)) {
        tail_reference_values_mps.push_back(vertical_vz_reference[state_index].vz_ref_global_smoothed_mps);
      }
    }
  }

  double target_tail_vz_mps =
    body_z_seed_candidate ? previous_vz_mps : MedianFinite(std::move(tail_reference_values_mps));
  if (!std::isfinite(target_tail_vz_mps) && !body_z_seed_candidate &&
      std::isfinite(window_candidate.center_candidate.delta_vz_init_mps)) {
    target_tail_vz_mps =
      reference_states[window_candidate.end_state_index].velocity.z() +
      window_candidate.center_candidate.delta_vz_init_mps;
  }
  if (!std::isfinite(target_tail_vz_mps)) {
    return std::nullopt;
  }
  if (std::isfinite(forced_tail_delta_mps)) {
    target_tail_vz_mps = body_z_seed_candidate
                           ? forced_tail_delta_mps
                           : original_tail_vz_mps +
                               std::max(tail_delta_scale, 0.0) * forced_tail_delta_mps;
  } else if (!body_z_seed_candidate) {
    target_tail_vz_mps =
      original_tail_vz_mps +
      std::max(tail_delta_scale, 0.0) * (target_tail_vz_mps - original_tail_vz_mps);
  }
  if (!std::isfinite(target_tail_vz_mps)) {
    return std::nullopt;
  }

  double integrated_tail_up_m = previous_up_m;
  double integrated_previous_vz_mps = previous_vz_mps;
  std::vector<double> integrated_window_up_m;
  std::vector<double> integrated_window_vz_mps;
  integrated_window_up_m.reserve(window_candidate.end_state_index - window_candidate.start_state_index + 1U);
  integrated_window_vz_mps.reserve(window_candidate.end_state_index - window_candidate.start_state_index + 1U);
  const double denominator = static_cast<double>(window_candidate.end_state_index - previous_state_index);
  for (std::size_t state_index = window_candidate.start_state_index;
       state_index <= window_candidate.end_state_index;
       ++state_index) {
    const double alpha = static_cast<double>(state_index - previous_state_index) / std::max(denominator, 1.0);
    const double smooth_alpha = SmoothStep01(alpha);
    const double smooth_vz_mps =
      (1.0 - smooth_alpha) * previous_vz_mps + smooth_alpha * target_tail_vz_mps;
    const double dt_s =
      std::max(reference_states[state_index].time_s - reference_states[state_index - 1U].time_s, 0.0);
    integrated_tail_up_m += 0.5 * (integrated_previous_vz_mps + smooth_vz_mps) * dt_s;
    integrated_previous_vz_mps = smooth_vz_mps;
    integrated_window_up_m.push_back(integrated_tail_up_m);
    integrated_window_vz_mps.push_back(smooth_vz_mps);
  }

  const double original_tail_up_m = reference_states[window_candidate.end_state_index].pose.translation().z();
  const double window_duration_s =
    reference_states[window_candidate.end_state_index].time_s - reference_states[previous_state_index].time_s;
  if (window_duration_s <= kTimeEpsilonS) {
    return std::nullopt;
  }
  const double continuity_tail_up_m = previous_up_m + previous_vz_mps * window_duration_s;
  double target_tail_up_m = integrated_tail_up_m;
  if (std::isfinite(forced_tail_delta_up_m)) {
    // Body-z seed windows discard the abnormal IMU height inside the window, so the
    // height search is a bounded offset around the pre-window kinematic continuation.
    // Other fallback windows keep the legacy "original tail plus delta" meaning.
    target_tail_up_m =
      body_z_seed_candidate ? continuity_tail_up_m + forced_tail_delta_up_m
                            : original_tail_up_m + forced_tail_delta_up_m;
  }
  if (!std::isfinite(target_tail_up_m)) {
    return std::nullopt;
  }

  VerticalVelocityWindowCorrection correction;
  correction.start_state_index = window_candidate.start_state_index;
  correction.center_state_index = window_candidate.center_state_index;
  correction.end_state_index = window_candidate.end_state_index;
  correction.target_tail_up_m = target_tail_up_m;
  correction.target_tail_vz_mps = target_tail_vz_mps;
  correction.delta_up_tail_m = target_tail_up_m - original_tail_up_m;
  correction.corrected_up_m.reserve(
    correction.end_state_index - correction.start_state_index + 1U);
  correction.corrected_vz_mps.reserve(
    correction.end_state_index - correction.start_state_index + 1U);

  const double decoupled_tail_up_offset_m =
    body_z_seed_candidate ? target_tail_up_m - integrated_tail_up_m : 0.0;
  for (std::size_t state_index = correction.start_state_index;
       state_index <= correction.end_state_index;
       ++state_index) {
    const double alpha =
      std::clamp(
        (reference_states[state_index].time_s - reference_states[previous_state_index].time_s) /
          window_duration_s,
        0.0,
        1.0);
    const std::size_t window_offset = state_index - correction.start_state_index;
    if (body_z_seed_candidate) {
      const double smooth_alpha = SmoothStep01(alpha);
      correction.corrected_up_m.push_back(
        integrated_window_up_m[window_offset] + smooth_alpha * decoupled_tail_up_offset_m);
      correction.corrected_vz_mps.push_back(integrated_window_vz_mps[window_offset]);
      continue;
    }
    const double alpha2 = alpha * alpha;
    const double alpha3 = alpha2 * alpha;
    const double h00 = 2.0 * alpha3 - 3.0 * alpha2 + 1.0;
    const double h10 = alpha3 - 2.0 * alpha2 + alpha;
    const double h01 = -2.0 * alpha3 + 3.0 * alpha2;
    const double h11 = alpha3 - alpha2;
    const double corrected_up_m =
      h00 * previous_up_m +
      h10 * window_duration_s * previous_vz_mps +
      h01 * target_tail_up_m +
      h11 * window_duration_s * target_tail_vz_mps;
    const double corrected_vz_mps =
      ((6.0 * alpha2 - 6.0 * alpha) * previous_up_m +
       (-6.0 * alpha2 + 6.0 * alpha) * target_tail_up_m) /
        window_duration_s +
      (3.0 * alpha2 - 4.0 * alpha + 1.0) * previous_vz_mps +
      (3.0 * alpha2 - 2.0 * alpha) * target_tail_vz_mps;
    correction.corrected_up_m.push_back(corrected_up_m);
    correction.corrected_vz_mps.push_back(corrected_vz_mps);
  }

  correction.delta_vz_tail_mps =
    correction.corrected_vz_mps.back() -
    reference_states[correction.end_state_index].velocity.z();

  double smooth_cost = 0.0;
  std::size_t smooth_count = 0U;
  std::vector<double> smooth_sequence;
  smooth_sequence.reserve(correction.corrected_vz_mps.size() + 1U);
  smooth_sequence.push_back(previous_vz_mps);
  smooth_sequence.insert(
    smooth_sequence.end(),
    correction.corrected_vz_mps.begin(),
    correction.corrected_vz_mps.end());
  for (std::size_t index = 1U; index < smooth_sequence.size(); ++index) {
    const double first_difference_mps = smooth_sequence[index] - smooth_sequence[index - 1U];
    smooth_cost += first_difference_mps * first_difference_mps;
    ++smooth_count;
  }
  for (std::size_t index = 2U; index < smooth_sequence.size(); ++index) {
    const double second_difference_mps =
      smooth_sequence[index] - 2.0 * smooth_sequence[index - 1U] + smooth_sequence[index - 2U];
    smooth_cost += second_difference_mps * second_difference_mps;
    ++smooth_count;
  }
  correction.velocity_smooth_cost =
    smooth_count > 0U ? smooth_cost / static_cast<double>(smooth_count) : 0.0;

  correction.height_integral_delta_m = correction.delta_up_tail_m;
  return correction;
}

std::optional<SparseVerticalJumpWindowCandidate> BuildBodyZSeedSparseWindowCandidate(
  const BodyZJumpWindowCandidate &body_z_window,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const std::size_t dynamic_start_index,
  const std::size_t feedback_anchor_state_index,
  const std::optional<std::size_t> &required_center_state_index,
  const std::unordered_set<std::size_t> &nhc_supported_state_indices) {
  if (state_timestamps.empty() || reference_states.empty() ||
      body_z_window.end_state_index < dynamic_start_index + 1U ||
      body_z_window.start_state_index > feedback_anchor_state_index) {
    return std::nullopt;
  }

  const std::size_t last_state_index =
    std::min(state_timestamps.size(), reference_states.size()) - 1U;
  const std::size_t window_start_state_index =
    std::clamp(body_z_window.start_state_index, dynamic_start_index + 1U, last_state_index);
  const std::size_t window_end_state_index =
    std::clamp(body_z_window.end_state_index, window_start_state_index, last_state_index);
  const std::size_t window_center_state_index =
    std::clamp(body_z_window.center_state_index, window_start_state_index, window_end_state_index);
  if (required_center_state_index.has_value() &&
      window_center_state_index != *required_center_state_index) {
    return std::nullopt;
  }

  SparseVerticalJumpCandidate center_candidate;
  center_candidate.state_index = window_center_state_index;
  center_candidate.time_s = state_timestamps[window_center_state_index];
  center_candidate.vz_prefit_mps = reference_states[window_center_state_index].velocity.z();
  if (window_center_state_index < vertical_vz_reference.size() &&
      vertical_vz_reference[window_center_state_index].valid &&
      std::isfinite(vertical_vz_reference[window_center_state_index].vz_ref_global_smoothed_mps)) {
    center_candidate.vz_ref_global_smoothed_mps =
      vertical_vz_reference[window_center_state_index].vz_ref_global_smoothed_mps;
    center_candidate.vz_mismatch_mps =
      center_candidate.vz_prefit_mps - center_candidate.vz_ref_global_smoothed_mps;
  }
  center_candidate.vz_mismatch_jump_mps =
    body_z_window.signed_delta_velocity_mps * body_z_window.body_z_axis_nav_z;
  center_candidate.jump_step_threshold_mps = body_z_window.level_threshold_mps;
  // Body-z integration localizes the anomaly window and provides a diagnostic
  // first guess. Tail speed feedback is still estimated from post-window height
  // residual stability, not from RTK-derived velocity.
  center_candidate.delta_vz_init_mps = body_z_window.delta_vz_init_mps;
  center_candidate.score = body_z_window.direction_score_mps;
  center_candidate.nhc_supported = nhc_supported_state_indices.contains(window_center_state_index);
  center_candidate.source = "BODY_Z_SEED_WINDOW";
  center_candidate.body_z_direction = body_z_window.direction;
  center_candidate.body_z_signed_delta_velocity_mps = body_z_window.signed_delta_velocity_mps;
  center_candidate.body_z_direction_score_mps = body_z_window.direction_score_mps;
  center_candidate.body_z_axis_nav_z = body_z_window.body_z_axis_nav_z;

  SparseVerticalJumpWindowCandidate window_candidate;
  window_candidate.center_candidate = center_candidate;
  window_candidate.start_state_index = window_start_state_index;
  window_candidate.center_state_index = window_center_state_index;
  window_candidate.end_state_index = window_end_state_index;
  window_candidate.duration_s =
    state_timestamps[window_end_state_index] - state_timestamps[window_start_state_index];
  window_candidate.point_count =
    window_candidate.end_state_index - window_candidate.start_state_index + 1U;
  return window_candidate;
}

std::optional<SparseVerticalJumpWindowCandidate> FindLatestEndedBodyZSeedWindowCandidate(
  const std::vector<BodyZJumpWindowCandidate> &body_z_windows,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const std::size_t dynamic_start_index,
  const std::size_t feedback_anchor_state_index) {
  std::optional<SparseVerticalJumpWindowCandidate> latest_window_candidate;
  const std::unordered_set<std::size_t> no_nhc_support;
  for (const auto &body_z_window : body_z_windows) {
    if (body_z_window.end_state_index >= feedback_anchor_state_index) {
      continue;
    }
    const auto window_candidate = BuildBodyZSeedSparseWindowCandidate(
      body_z_window,
      state_timestamps,
      reference_states,
      vertical_vz_reference,
      dynamic_start_index,
      feedback_anchor_state_index,
      std::nullopt,
      no_nhc_support);
    if (!window_candidate.has_value()) {
      continue;
    }
    if (!latest_window_candidate.has_value() ||
        window_candidate->end_state_index > latest_window_candidate->end_state_index) {
      latest_window_candidate = *window_candidate;
    }
  }
  return latest_window_candidate;
}

void ApplyVerticalVelocityWindowCorrection(
  const VerticalVelocityWindowCorrection &correction,
  std::vector<ReferenceNodeState> *reference_states,
  gtsam::Values *initial_values) {
  if (reference_states == nullptr || reference_states->empty() ||
      correction.start_state_index == 0U ||
      correction.end_state_index >= reference_states->size() ||
      correction.corrected_up_m.size() != correction.end_state_index - correction.start_state_index + 1U ||
      correction.corrected_vz_mps.size() != correction.end_state_index - correction.start_state_index + 1U) {
    return;
  }

  for (std::size_t offset = 0U; offset < correction.corrected_vz_mps.size(); ++offset) {
    const std::size_t state_index = correction.start_state_index + offset;
    auto corrected_state = (*reference_states)[state_index];
    corrected_state.velocity = gtsam::Vector3(
      corrected_state.velocity.x(),
      corrected_state.velocity.y(),
      correction.corrected_vz_mps[offset]);
    corrected_state.pose = gtsam::Pose3(
      corrected_state.pose.rotation(),
      gtsam::Point3(
        corrected_state.pose.translation().x(),
        corrected_state.pose.translation().y(),
        correction.corrected_up_m[offset]));
    (*reference_states)[state_index] = corrected_state;
    if (initial_values != nullptr) {
      UpsertReferenceStateInitialValues(state_index, corrected_state, initial_values);
    }
  }
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
  SparseVerticalJumpPlanner sparse_vertical_jump_planner(config_);
  const std::vector<VerticalVzReferenceSample> vertical_vz_reference_by_state =
    sparse_vertical_jump_planner.BuildGlobalReference(
      dataset.gnss_samples,
      state_timestamps,
      dynamic_start_time_s);
  const bool collect_error_diagnostics = config_.write_error_diagnostics;
  const bool collect_segment_error_diagnostics =
    config_.write_segment_error_diagnostics ||
    config_.enable_segment_error_feedback ||
    config_.enable_vertical_rtk_preintegration_feedback;
  const bool collect_reference_states =
    collect_error_diagnostics ||
    collect_segment_error_diagnostics ||
    config_.gnss_consistency_gate_mode != GnssConsistencyGateMode::kNone ||
    config_.enable_vertical_rtk_preintegration_feedback;
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
    } else if (config_.enable_reweighted_combined_imu_factor) {
      graph.add(factor::MakeReweightedCombinedImuFactor(
        X(state_index - 1U),
        V(state_index - 1U),
        X(state_index),
        V(state_index),
        B(state_index - 1U),
        B(state_index),
        imu_window.preintegrated_measurements,
        config_.reweighted_combined_imu_attitude_sigma_rad,
        gtsam::Vector3(
          config_.reweighted_combined_imu_specific_force_sigma_x_mps2,
          config_.reweighted_combined_imu_specific_force_sigma_y_mps2,
          config_.reweighted_combined_imu_specific_force_sigma_z_mps2)));
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
     config_.gnss_consistency_gate_mode != GnssConsistencyGateMode::kNone ||
     config_.enable_vertical_rtk_preintegration_feedback);
  const bool use_vertical_rtk_1d_nis_gate = config_.enable_vertical_rtk_preintegration_feedback;
  const double gnss_nis_threshold =
    config_.gnss_consistency_gate_mode == GnssConsistencyGateMode::kNis
      ? ChiSquareQuantile3D(config_.gnss_nis_confidence)
      : std::numeric_limits<double>::quiet_NaN();
  const double vertical_gate_nis_threshold =
    use_vertical_rtk_1d_nis_gate
      ? ChiSquareQuantile1D(config_.gnss_nis_confidence)
      : std::numeric_limits<double>::quiet_NaN();
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
  gtsam::Values gate_seed_values = base_initial_values;
  std::vector<ReferenceNodeState> gate_seed_reference_states = reference_node_states;
  if (use_vertical_rtk_1d_nis_gate) {
    gtsam::LevenbergMarquardtOptimizer seed_optimizer(base_graph, base_initial_values, optimizer_params);
    gate_seed_values = seed_optimizer.optimize();
    gate_seed_reference_states = BuildReferenceStatesFromOptimizedValues(state_timestamps, gate_seed_values);
  }
  std::vector<ReferenceNodeState> body_z_seed_reference_states;
  BodyZJumpDetectionResult body_z_seed_detection;
  if (config_.enable_vertical_rtk_seed_pass) {
    auto rtk_seed_graph = base_graph;
    std::size_t rtk_seed_factor_count = 0U;
    const auto rtk_seed_noise_model = [&](const GnssSolutionSample &sample) {
      const Eigen::Vector3d sigma_m = ClampGnssSigma(sample);
      return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sigma_m.x(), sigma_m.y(), sigma_m.z()));
    };
    for (std::size_t sample_index = 0; sample_index < dataset.gnss_samples.size(); ++sample_index) {
      if (sample_index <= navigation_start_index) {
        continue;
      }
      const auto &gnss_sample = dataset.gnss_samples[sample_index];
      if (config_.body_z_seed_jump_use_fix_only && gnss_sample.fix_type() != GnssFixType::kRtkFix) {
        continue;
      }
      if (!PassesGnssQualityFilters(gnss_sample) || !gnss_sample.has_enu_position) {
        continue;
      }
      const double corrected_time_s = CorrectedGnssTime(gnss_sample);
      if (!IsWithinImuCoverage(dataset.imu_samples, corrected_time_s)) {
        continue;
      }
      const StateMeasSyncResult sync_result =
        FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
      if (sync_result.status == StateMeasSyncStatus::kSynchronizedI ||
          sync_result.status == StateMeasSyncStatus::kSynchronizedJ) {
        const std::size_t sync_state_index =
          sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i : sync_result.key_index_j;
        rtk_seed_graph.add(gtsam::GPSFactor(
          X(sync_state_index),
          gtsam::Point3(
            gnss_sample.enu_position_m.x(),
            gnss_sample.enu_position_m.y(),
            gnss_sample.enu_position_m.z()),
          rtk_seed_noise_model(gnss_sample)));
        ++rtk_seed_factor_count;
      } else if (
        sync_result.status == StateMeasSyncStatus::kInterpolated &&
        config_.enable_gp_interpolated_gnss &&
        sync_result.state_j_exists()) {
        gp::GPWNOJInterpolator interpolator = base_interpolator;
        interpolator.Recalculate(
          sync_result.timestamp_j_s - sync_result.timestamp_i_s,
          sync_result.duration_from_state_i_s);
        rtk_seed_graph.add(factor::GPInterpolatedGPSFactor(
          X(sync_result.key_index_i),
          V(sync_result.key_index_i),
          W(sync_result.key_index_i),
          X(sync_result.key_index_j),
          V(sync_result.key_index_j),
          W(sync_result.key_index_j),
          gtsam::Point3(
            gnss_sample.enu_position_m.x(),
            gnss_sample.enu_position_m.y(),
            gnss_sample.enu_position_m.z()),
          gtsam::Vector3::Zero(),
          rtk_seed_noise_model(gnss_sample),
          interpolator));
        ++rtk_seed_factor_count;
      }
    }
    if (rtk_seed_factor_count > 0U) {
      gtsam::LevenbergMarquardtOptimizer rtk_seed_optimizer(
        rtk_seed_graph,
        base_initial_values,
        optimizer_params);
      const gtsam::Values rtk_seed_values = rtk_seed_optimizer.optimize();
      body_z_seed_reference_states = BuildReferenceStatesFromOptimizedValues(state_timestamps, rtk_seed_values);
    }
  }
  if (config_.enable_body_z_seed_jump_windows && !body_z_seed_reference_states.empty()) {
    BodyZBidirectionalJumpDetector body_z_detector(config_);
    body_z_seed_detection = body_z_detector.Detect(
      dataset.imu_samples,
      body_z_seed_reference_states,
      state_timestamps,
      dynamic_start_time_s,
      state_timestamps.back());
    run_result.seed_body_z_acc_diagnostics.reserve(body_z_seed_detection.signal.size());
    for (const auto &sample : body_z_seed_detection.signal) {
      BodyZSeedImuDiagnosticRow row;
      row.time_s = sample.time_s;
      row.relative_time_s = sample.relative_time_s;
      row.body_z_specific_force_mps2 = sample.body_z_specific_force_mps2;
      row.gravity_projection_z_mps2 = sample.gravity_projection_z_mps2;
      row.body_z_acc_mps2 = sample.body_z_acc_mps2;
      row.body_z_acc_1s_smooth_mps2 = sample.body_z_acc_1s_smooth_mps2;
      row.integrated_body_z_velocity_mps = sample.integrated_body_z_velocity_mps;
      row.integrated_body_z_velocity_0p2s_smooth_mps = sample.integrated_body_z_velocity_0p2s_smooth_mps;
      row.integrated_body_z_velocity_1s_smooth_mps = sample.integrated_body_z_velocity_1s_smooth_mps;
      row.signed_step_metric_mps = sample.signed_step_metric_mps;
      row.downward_score_mps = sample.downward_score_mps;
      row.upward_score_mps = sample.upward_score_mps;
      row.body_z_axis_nav_z = sample.body_z_axis_nav_z;
      run_result.seed_body_z_acc_diagnostics.push_back(row);
    }
    run_result.body_z_seed_jump_windows.reserve(body_z_seed_detection.windows.size());
    for (const auto &window : body_z_seed_detection.windows) {
      BodyZSeedJumpWindowRow row;
      row.direction = window.direction;
      row.selection_level = window.selection_level;
      row.start_state_index = static_cast<long long>(window.start_state_index);
      row.center_state_index = static_cast<long long>(window.center_state_index);
      row.end_state_index = static_cast<long long>(window.end_state_index);
      row.start_time_s = window.start_time_s;
      row.center_time_s = window.center_time_s;
      row.end_time_s = window.end_time_s;
      row.start_relative_time_s = window.start_relative_time_s;
      row.center_relative_time_s = window.center_relative_time_s;
      row.end_relative_time_s = window.end_relative_time_s;
      row.duration_s = window.duration_s;
      row.pre_velocity_mps = window.pre_velocity_mps;
      row.post_velocity_mps = window.post_velocity_mps;
      row.signed_delta_velocity_mps = window.signed_delta_velocity_mps;
      row.direction_score_mps = window.direction_score_mps;
      row.signed_step_metric_mps = window.signed_step_metric_mps;
      row.level_threshold_mps = window.level_threshold_mps;
      row.level_max_peak_mps = window.level_max_peak_mps;
      row.level_noise_floor_mps = window.level_noise_floor_mps;
      row.min_acc_mps2 = window.min_acc_mps2;
      row.max_acc_mps2 = window.max_acc_mps2;
      row.mean_acc_mps2 = window.mean_acc_mps2;
      row.body_z_axis_nav_z = window.body_z_axis_nav_z;
      row.delta_vz_init_mps = window.delta_vz_init_mps;
      run_result.body_z_seed_jump_windows.push_back(row);
    }
  }
  struct VerticalGateIterationResult {
    gtsam::Values optimized_values;
    std::vector<GnssFactorRecord> gnss_factor_records;
    std::vector<GnssConsistencyRecord> gnss_consistency_records;
    std::vector<VerticalLocalRecoveryIterationRow> vertical_local_recovery_iterations;
    std::vector<TrajectoryRow> trajectory;
    std::vector<ReferenceNodeState> gate_reference_states;
    RunSummary run_summary;
    bool failed = false;
    std::string failure_message;
  };

  const auto run_vertical_gate_iteration =
    [&](const std::vector<ReferenceNodeState> &gate_reference_states,
        const gtsam::Values &optimizer_initial_values) -> VerticalGateIterationResult {
      VerticalGateIterationResult iteration;
      iteration.run_summary = base_run_summary;
      iteration.run_summary.gnss_factor_count = 0;
      iteration.run_summary.gnss_synced_factor_count = 0;
      iteration.run_summary.gnss_interpolated_factor_count = 0;
      iteration.run_summary.gnss_dropped_count = 0;
      iteration.run_summary.gnss_cached_count = 0;
      iteration.run_summary.dropped_non_rtkfix_count = 0;
      iteration.run_summary.dropped_no_solution_count = 0;
      iteration.run_summary.dropped_nonfinite_sigma_count = 0;
      iteration.run_summary.dropped_bad_status_count = 0;
      iteration.run_summary.dropped_out_of_imu_coverage_count = 0;
      iteration.run_summary.vertical_gate_inside_count = 0;
      iteration.run_summary.vertical_gate_outside_count = 0;
      iteration.run_summary.gnss_nis_mean = std::numeric_limits<double>::quiet_NaN();
      iteration.run_summary.gnss_nis_median = std::numeric_limits<double>::quiet_NaN();
      iteration.run_summary.gnss_nis_p95 = std::numeric_limits<double>::quiet_NaN();
      iteration.run_summary.axis_2sigma_pass_rate = std::numeric_limits<double>::quiet_NaN();
      iteration.trajectory = base_dynamic_trajectory;
      iteration.gate_reference_states = gate_reference_states;
      gtsam::Values sequential_initial_values = optimizer_initial_values;
      std::size_t sequential_reference_valid_until_index =
        use_vertical_rtk_1d_nis_gate
          ? std::min(graph_timeline.dynamic_start_index, iteration.gate_reference_states.size() > 0U
                                                       ? iteration.gate_reference_states.size() - 1U
                                                       : 0U)
          : (iteration.gate_reference_states.empty() ? 0U : iteration.gate_reference_states.size() - 1U);
      std::vector<ReferenceNodeState> inside_reference_states = iteration.gate_reference_states;
      SequentialNhcJumpDetector nhc_jump_detector(config_);
      SparseVerticalJumpPlanner iteration_sparse_jump_planner(config_);
      VerticalInsideBiasAdapter inside_bias_adapter(config_);
      if (use_vertical_rtk_1d_nis_gate && !iteration.gate_reference_states.empty() &&
          graph_timeline.dynamic_start_index > 0U) {
        nhc_jump_detector.SeedWithConfirmedStates(
          iteration.gate_reference_states,
          0U,
          graph_timeline.dynamic_start_index);
        iteration_sparse_jump_planner.SeedWithConfirmedStates(
          iteration.gate_reference_states,
          vertical_vz_reference_by_state,
          0U,
          graph_timeline.dynamic_start_index);
      }
      std::optional<std::size_t> confirmed_inside_state_index;
      std::optional<std::size_t> history_reobserve_begin_state_index;
      std::optional<std::size_t> inside_bias_adapter_reobserve_begin_state_index;
      std::optional<SparseVerticalJumpWindowCandidate> active_interval_feedback_window;
      std::unordered_set<std::size_t> body_z_windows_with_velocity_correction;
      std::unordered_set<std::size_t> body_z_windows_with_position_offset_correction;
      std::unordered_map<std::size_t, int> body_z_window_velocity_feedback_counts;
      double last_interval_feedback_time_s = -std::numeric_limits<double>::infinity();
      bool vertical_initial_anchor_applied = false;

      const auto build_vertical_window_specs = [&](const std::size_t current_sample_index, const double window_s) {
        std::vector<VerticalHoldWindowSpec> specs;
        if (window_s <= 0.0) {
          return specs;
        }
        const double window_end_time_s =
          CorrectedGnssTime(dataset.gnss_samples[current_sample_index]) + window_s;
        for (std::size_t future_sample_index = current_sample_index; future_sample_index < dataset.gnss_samples.size();
             ++future_sample_index) {
          if (future_sample_index <= navigation_start_index) {
            continue;
          }
          const auto &future_sample = dataset.gnss_samples[future_sample_index];
          const double corrected_time_s = CorrectedGnssTime(future_sample);
          if (corrected_time_s > window_end_time_s + kTimeEpsilonS) {
            break;
          }
          if (!PassesGnssQualityFilters(future_sample) || !future_sample.has_enu_position ||
              !IsWithinImuCoverage(dataset.imu_samples, corrected_time_s)) {
            continue;
          }
          const StateMeasSyncResult future_sync_result =
            FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
          if (!IsSynchronizedStatus(future_sync_result.status) &&
              future_sync_result.status != StateMeasSyncStatus::kInterpolated) {
            continue;
          }
          Eigen::Vector3d measurement_enu_m = future_sample.enu_position_m;
          if (config_.enable_gnss_vertical_drift_model &&
              future_sample_index < gnss_vertical_reference_up_by_sample.size()) {
            const double vertical_reference_up_m = gnss_vertical_reference_up_by_sample[future_sample_index];
            if (std::isfinite(vertical_reference_up_m)) {
              measurement_enu_m.z() = vertical_reference_up_m;
            }
          }
          const Eigen::Vector3d hold_sigma_m = ClampGnssSigma(future_sample);
          specs.push_back(VerticalHoldWindowSpec{
            future_sample_index,
            corrected_time_s,
            measurement_enu_m,
            hold_sigma_m.z(),
            ResolvePrefitReferenceRightIndex(future_sync_result),
            future_sync_result.status == StateMeasSyncStatus::kInterpolated,
          });
        }
        return specs;
      };
      const auto build_vertical_window_specs_between_times = [&](const double window_start_time_s, const double window_end_time_s) {
        std::vector<VerticalHoldWindowSpec> specs;
        if (!std::isfinite(window_start_time_s) || !std::isfinite(window_end_time_s) ||
            window_end_time_s < window_start_time_s - kTimeEpsilonS) {
          return specs;
        }
        for (std::size_t future_sample_index = navigation_start_index + 1U;
             future_sample_index < dataset.gnss_samples.size();
             ++future_sample_index) {
          const auto &future_sample = dataset.gnss_samples[future_sample_index];
          const double corrected_time_s = CorrectedGnssTime(future_sample);
          if (corrected_time_s < window_start_time_s - kTimeEpsilonS) {
            continue;
          }
          if (corrected_time_s > window_end_time_s + kTimeEpsilonS) {
            break;
          }
          if (future_sample.fix_type() != GnssFixType::kRtkFix ||
              !PassesGnssQualityFilters(future_sample) ||
              !future_sample.has_enu_position ||
              !IsWithinImuCoverage(dataset.imu_samples, corrected_time_s)) {
            continue;
          }
          const StateMeasSyncResult future_sync_result =
            FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
          if (!IsSynchronizedStatus(future_sync_result.status) &&
              future_sync_result.status != StateMeasSyncStatus::kInterpolated) {
            continue;
          }
          Eigen::Vector3d measurement_enu_m = future_sample.enu_position_m;
          if (config_.enable_gnss_vertical_drift_model &&
              future_sample_index < gnss_vertical_reference_up_by_sample.size()) {
            const double vertical_reference_up_m = gnss_vertical_reference_up_by_sample[future_sample_index];
            if (std::isfinite(vertical_reference_up_m)) {
              measurement_enu_m.z() = vertical_reference_up_m;
            }
          }
          const Eigen::Vector3d hold_sigma_m = ClampGnssSigma(future_sample);
          specs.push_back(VerticalHoldWindowSpec{
            future_sample_index,
            corrected_time_s,
            measurement_enu_m,
            hold_sigma_m.z(),
            ResolvePrefitReferenceRightIndex(future_sync_result),
            future_sync_result.status == StateMeasSyncStatus::kInterpolated,
          });
        }
        return specs;
      };
      const auto build_hold_window_specs = [&](const std::size_t current_sample_index) {
        return build_vertical_window_specs(current_sample_index, config_.vertical_jump_hold_window_s);
      };
      const auto build_future_trend_specs = [&](const std::size_t current_sample_index) {
        return build_vertical_window_specs(current_sample_index, config_.vertical_jump_future_trend_window_s);
      };

      const auto refresh_local_postfit_residual =
        [&](const GnssFactorRecord &gnss_record, GnssConsistencyRecord *gnss_consistency) {
          if (gnss_consistency == nullptr ||
              (!IsSynchronizedStatus(gnss_record.sync_status) &&
               gnss_record.sync_status != StateMeasSyncStatus::kInterpolated)) {
            return;
          }
          const ReferenceNodeState refreshed_state =
            InterpolateReferenceState(iteration.gate_reference_states, gnss_record.corrected_time_s);
          const Eigen::Vector3d refreshed_residual_enu_m =
            ComputePositionResidualEnu(refreshed_state.pose, gnss_record.measurement_enu_m);
          gnss_consistency->postfit_residual_enu_m = refreshed_residual_enu_m;
          gnss_consistency->local_postfit_residual_u_m = refreshed_residual_enu_m.z();
          gnss_consistency->prefit_residual_u_after_local_recovery_m = refreshed_residual_enu_m.z();
          const double sigma_u_m =
            std::isfinite(gnss_consistency->effective_sigma_u_m)
              ? gnss_consistency->effective_sigma_u_m
              : gnss_consistency->sigma_u_m;
          if (std::isfinite(sigma_u_m) && sigma_u_m > 0.0) {
            gnss_consistency->postfit_nis =
              ComputeVerticalNis(refreshed_residual_enu_m.z(), sigma_u_m);
            gnss_consistency->vertical_gate_inside =
              gnss_consistency->postfit_nis <= vertical_gate_nis_threshold ? 1.0 : 0.0;
          }
        };

      const auto refresh_local_postfit_residuals_between_times =
        [&](const double window_start_time_s, const double window_end_time_s) {
          if (!collect_gnss_consistency ||
              iteration.gnss_factor_records.empty() ||
              iteration.gnss_factor_records.size() != iteration.gnss_consistency_records.size()) {
            return;
          }
          for (std::size_t record_index = 0; record_index < iteration.gnss_factor_records.size(); ++record_index) {
            const auto &existing_record = iteration.gnss_factor_records[record_index];
            auto &existing_consistency = iteration.gnss_consistency_records[record_index];
            if (!existing_record.factor_used ||
                existing_record.corrected_time_s < window_start_time_s - kTimeEpsilonS ||
                existing_record.corrected_time_s > window_end_time_s + kTimeEpsilonS) {
              continue;
            }
            refresh_local_postfit_residual(existing_record, &existing_consistency);
          }
        };

      const auto compute_vz_reference_rms = [&](
                                               const std::vector<ReferenceNodeState> &reference_states,
                                               const std::size_t start_index,
                                              const std::size_t end_index) {
        if (reference_states.empty() || start_index >= reference_states.size() || end_index >= reference_states.size() ||
            start_index > end_index) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        double squared_error_sum = 0.0;
        std::size_t sample_count = 0U;
        for (std::size_t state_index = start_index; state_index <= end_index; ++state_index) {
          if (state_index >= vertical_vz_reference_by_state.size() ||
              !vertical_vz_reference_by_state[state_index].valid ||
              !std::isfinite(vertical_vz_reference_by_state[state_index].vz_ref_global_smoothed_mps)) {
            continue;
          }
          const double error_mps =
            reference_states[state_index].velocity.z() -
            vertical_vz_reference_by_state[state_index].vz_ref_global_smoothed_mps;
          squared_error_sum += error_mps * error_mps;
          ++sample_count;
        }
        if (sample_count == 0U) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        return std::sqrt(squared_error_sum / static_cast<double>(sample_count));
      };

      auto graph_with_gnss = base_graph;
      if (config_.enable_gnss) {
        for (std::size_t sample_index = 0; sample_index < dataset.gnss_samples.size(); ++sample_index) {
          if (sample_index <= navigation_start_index) {
            continue;
          }
          const auto &gnss_sample = dataset.gnss_samples[sample_index];
          GnssFactorRecord record;
          record.sample_index = sample_index;
          record.raw_time_s = gnss_sample.time_s;
          record.corrected_time_s = CorrectedGnssTime(gnss_sample);
          record.gnss_fix_type = gnss_sample.fix_type();
          GnssConsistencyRecord consistency_record;
          consistency_record.sample_index = sample_index;
          consistency_record.raw_time_s = gnss_sample.time_s;
          consistency_record.corrected_time_s = record.corrected_time_s;
          consistency_record.gnss_fix_type = gnss_sample.fix_type();
          consistency_record.raw_sigma_h_m = gnss_sample.sigma_h_m;
          consistency_record.prefit_residual_enu_m = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
          consistency_record.postfit_residual_enu_m = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
          if (gnss_sample.has_enu_position) {
            record.measurement_enu_m = gnss_sample.enu_position_m;
          }

          if (!ShouldUseGnssFactor(gnss_sample, iteration.run_summary)) {
            record.sync_status = StateMeasSyncStatus::kDropped;
            iteration.gnss_factor_records.push_back(record);
            if (collect_gnss_consistency) {
              consistency_record.sync_status = StateMeasSyncStatus::kDropped;
              iteration.gnss_consistency_records.push_back(consistency_record);
            }
            continue;
          }
          if (!IsWithinImuCoverage(dataset.imu_samples, record.corrected_time_s)) {
            ++iteration.run_summary.dropped_out_of_imu_coverage_count;
            ++iteration.run_summary.gnss_dropped_count;
            record.sync_status = StateMeasSyncStatus::kDropped;
            iteration.gnss_factor_records.push_back(record);
            if (collect_gnss_consistency) {
              consistency_record.sync_status = StateMeasSyncStatus::kDropped;
              iteration.gnss_consistency_records.push_back(consistency_record);
            }
            continue;
          }

          const StateMeasSyncResult sync_result =
            FindStateForMeasurement(state_timestamp_map, record.corrected_time_s, config_);
          record.sync_status = sync_result.status;
          consistency_record.sync_status = sync_result.status;
          record.state_index_i = sync_result.key_index_i;
          record.state_index_j = sync_result.key_index_j;
          record.trajectory_row_index_i = graph_state_to_trajectory_row(sync_result.key_index_i);
          record.trajectory_row_index_j = graph_state_to_trajectory_row(sync_result.key_index_j);
          record.state_time_i_s = sync_result.timestamp_i_s;
          record.state_time_j_s = sync_result.timestamp_j_s;
          record.duration_from_state_i_s = sync_result.duration_from_state_i_s;
          if (sync_result.status == StateMeasSyncStatus::kSynchronizedI ||
              sync_result.status == StateMeasSyncStatus::kSynchronizedJ) {
            record.synchronized_state_index =
              sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i : sync_result.key_index_j;
            record.synchronized_trajectory_row_index = graph_state_to_trajectory_row(record.synchronized_state_index);
          }

          const double elapsed_s = record.corrected_time_s - dynamic_start_time_s;
          if (config_.enable_gnss_vertical_drift_model && sample_index < gnss_vertical_reference_up_by_sample.size()) {
            const double vertical_reference_up_m = gnss_vertical_reference_up_by_sample[sample_index];
            if (std::isfinite(vertical_reference_up_m)) {
              record.measurement_enu_m.z() = vertical_reference_up_m;
              consistency_record.vertical_reference_up_m = vertical_reference_up_m;
              consistency_record.vertical_reference_used = true;
            }
          }
          const Eigen::Vector3d base_sigma_m = ClampGnssSigma(gnss_sample);
          Eigen::Vector3d sigma_m = base_sigma_m;
          if (!use_vertical_rtk_1d_nis_gate &&
              config_.early_gnss_relaxation_duration_s > 0.0 &&
              elapsed_s < config_.early_gnss_relaxation_duration_s) {
            const double alpha = 1.0 - (elapsed_s / config_.early_gnss_relaxation_duration_s);
            sigma_m *= 1.0 + alpha * (config_.early_gnss_relaxation_scale - 1.0);
          }
          consistency_record.sigma_e_m = sigma_m.x();
          consistency_record.sigma_n_m = sigma_m.y();
          consistency_record.sigma_u_m = sigma_m.z();
          const Eigen::Vector3d consistency_base_sigma_m = sigma_m;
          std::optional<gtsam::Pose3> prefit_pose;
          bool inside_gate = false;
          const auto feedback_anchor_state =
            ResolveVerticalFeedbackAnchorState(record, graph_timeline.dynamic_start_index);
          const bool apply_initial_vertical_anchor =
            use_vertical_rtk_1d_nis_gate && !vertical_initial_anchor_applied && feedback_anchor_state.has_value();
          consistency_record.confirmed_inside_before_sample = confirmed_inside_state_index.has_value() ? 1.0 : 0.0;

          if (use_vertical_rtk_1d_nis_gate) {
            sigma_m = consistency_base_sigma_m;
            consistency_record.effective_sigma_u_m = consistency_base_sigma_m.z();
            consistency_record.vertical_gate_threshold_m =
              std::sqrt(vertical_gate_nis_threshold) * consistency_record.effective_sigma_u_m;
            consistency_record.vertical_direct_position_factor_used = apply_initial_vertical_anchor;
            consistency_record.vertical_sigma_u_used_m =
              apply_initial_vertical_anchor ? consistency_base_sigma_m.z() : std::numeric_limits<double>::quiet_NaN();
            if (feedback_anchor_state.has_value()) {
              const std::size_t required_prefit_index = ResolvePrefitReferenceRightIndex(sync_result);
              const std::size_t propagation_anchor_index =
                confirmed_inside_state_index.value_or(graph_timeline.dynamic_start_index);
              if (sequential_reference_valid_until_index > propagation_anchor_index) {
                sequential_reference_valid_until_index = propagation_anchor_index;
              }
              EnsureReferenceStatesValidUntil(
                state_timestamps,
                dataset.imu_samples,
                imu_params,
                &iteration.gate_reference_states,
                &sequential_reference_valid_until_index,
                required_prefit_index,
                &sequential_initial_values);

              const ReferenceNodeState prefit_reference_state =
                sync_result.status == StateMeasSyncStatus::kInterpolated
                  ? InterpolateReferenceState(iteration.gate_reference_states, record.corrected_time_s)
                  : iteration.gate_reference_states[required_prefit_index];
              prefit_pose = prefit_reference_state.pose;
              const Eigen::Vector3d prefit_residual_enu_m =
                ComputePositionResidualEnu(prefit_reference_state.pose, record.measurement_enu_m);
              consistency_record.prefit_residual_enu_m = prefit_residual_enu_m;
              consistency_record.local_prefit_residual_u_m = prefit_residual_enu_m.z();
              consistency_record.local_postfit_residual_u_m = prefit_residual_enu_m.z();
              consistency_record.prefit_residual_u_before_local_recovery_m = prefit_residual_enu_m.z();
              consistency_record.prefit_residual_u_after_local_recovery_m = prefit_residual_enu_m.z();
              consistency_record.prefit_nis = ComputeVerticalNis(prefit_residual_enu_m.z(), consistency_base_sigma_m.z());

              const NhcThresholdSnapshot nhc_thresholds = nhc_jump_detector.CurrentThresholds(prefit_reference_state.time_s);
              const ReferenceNodeState nhc_previous_reference_state =
                required_prefit_index > propagation_anchor_index
                  ? iteration.gate_reference_states[required_prefit_index - 1U]
                  : prefit_reference_state;
              const NhcStateEvaluation nhc_evaluation =
                nhc_jump_detector.EvaluateTransition(
                  nhc_previous_reference_state,
                  prefit_reference_state,
                  prefit_reference_state.time_s);
              consistency_record.nhc_body_vy_mps = nhc_evaluation.body_vy_mps;
              consistency_record.nhc_body_vz_mps = nhc_evaluation.body_vz_mps;
              consistency_record.nhc_body_vz_baseline_mps = nhc_thresholds.body_vz_baseline_mps;
              consistency_record.nhc_body_vz_residual_mps = nhc_evaluation.body_vz_residual_mps;
              consistency_record.nhc_body_vz_jump_mps = nhc_evaluation.body_vz_jump_mps;
              consistency_record.nhc_body_vy_threshold_mps = nhc_thresholds.body_vy_threshold_mps;
              consistency_record.nhc_body_vz_threshold_mps = nhc_thresholds.body_vz_threshold_mps;
              if (required_prefit_index < vertical_vz_reference_by_state.size() &&
                  vertical_vz_reference_by_state[required_prefit_index].valid &&
                  std::isfinite(vertical_vz_reference_by_state[required_prefit_index].vz_ref_global_smoothed_mps)) {
                consistency_record.vz_ref_global_smoothed_mps =
                  vertical_vz_reference_by_state[required_prefit_index].vz_ref_global_smoothed_mps;
                consistency_record.vz_prefit_mps = prefit_reference_state.velocity.z();
                consistency_record.vz_mismatch_mps =
                  prefit_reference_state.velocity.z() -
                  vertical_vz_reference_by_state[required_prefit_index].vz_ref_global_smoothed_mps;
                if (required_prefit_index > propagation_anchor_index &&
                    required_prefit_index - 1U < vertical_vz_reference_by_state.size() &&
                    vertical_vz_reference_by_state[required_prefit_index - 1U].valid &&
                    std::isfinite(vertical_vz_reference_by_state[required_prefit_index - 1U].vz_ref_global_smoothed_mps)) {
                  const double previous_vz_mismatch_mps =
                    iteration.gate_reference_states[required_prefit_index - 1U].velocity.z() -
                    vertical_vz_reference_by_state[required_prefit_index - 1U].vz_ref_global_smoothed_mps;
                  consistency_record.vz_mismatch_jump_mps =
                    consistency_record.vz_mismatch_mps - previous_vz_mismatch_mps;
                }
              }

              if (apply_initial_vertical_anchor) {
                inside_gate = true;
                consistency_record.local_postfit_residual_u_m = 0.0;
                consistency_record.required_up_anchor_correction_m = 0.0;
              } else {
                inside_gate = consistency_record.prefit_nis <= vertical_gate_nis_threshold;
                bool interval_feedback_requested = false;
                std::optional<std::size_t> interval_feedback_window_center_state_index;
                double interval_feedback_delta_vz_mps = std::numeric_limits<double>::quiet_NaN();
                std::optional<SparseVerticalJumpWindowCandidate> interval_feedback_window =
                  active_interval_feedback_window;
                if (config_.enable_body_z_seed_jump_windows && !body_z_seed_detection.windows.empty()) {
                  const auto pending_body_z_window = FindLatestEndedBodyZSeedWindowCandidate(
                    body_z_seed_detection.windows,
                    state_timestamps,
                    iteration.gate_reference_states,
                    vertical_vz_reference_by_state,
                    graph_timeline.dynamic_start_index,
                    *feedback_anchor_state);
                  if (pending_body_z_window.has_value() &&
                      (!interval_feedback_window.has_value() ||
                       pending_body_z_window->end_state_index >= interval_feedback_window->end_state_index)) {
                    interval_feedback_window = *pending_body_z_window;
                  }
                }
                constexpr double kIntervalFeedbackNearGateToleranceM = 0.03;
                const bool interval_feedback_gate_eligible =
                  inside_gate ||
                  std::abs(prefit_residual_enu_m.z()) <=
                    consistency_record.vertical_gate_threshold_m + kIntervalFeedbackNearGateToleranceM ||
                  active_interval_feedback_window.has_value();
                if (interval_feedback_gate_eligible && !apply_initial_vertical_anchor &&
                    interval_feedback_window.has_value() &&
                    record.corrected_time_s - last_interval_feedback_time_s >= 1.0 &&
                    interval_feedback_window->end_state_index < iteration.gate_reference_states.size() &&
                    *feedback_anchor_state > interval_feedback_window->end_state_index) {
                  const double active_window_end_time_s =
                    iteration.gate_reference_states[interval_feedback_window->end_state_index].time_s;
                  double next_body_z_start_time_s = std::numeric_limits<double>::infinity();
                  for (const auto &body_z_window : body_z_seed_detection.windows) {
                    if (body_z_window.start_state_index > interval_feedback_window->end_state_index &&
                        body_z_window.start_state_index < iteration.gate_reference_states.size()) {
                      next_body_z_start_time_s = std::min(
                        next_body_z_start_time_s,
                        iteration.gate_reference_states[body_z_window.start_state_index].time_s);
                    }
                  }
                  const bool safely_before_next_body_z_window =
                    !std::isfinite(next_body_z_start_time_s) ||
                    record.corrected_time_s < next_body_z_start_time_s - kTimeEpsilonS;
                  if (safely_before_next_body_z_window) {
                    auto interval_specs =
                      build_vertical_window_specs_between_times(active_window_end_time_s, record.corrected_time_s);
                    interval_specs =
                      FilterVerticalHoldWindowSpecsAfterState(
                        interval_specs,
                        interval_feedback_window->end_state_index);
                    if (interval_specs.size() >=
                        static_cast<std::size_t>(std::max(config_.vertical_jump_future_trend_min_fix_count, 1))) {
                      std::vector<double> interval_times_s;
                      std::vector<double> interval_residuals_m;
                      interval_times_s.reserve(interval_specs.size());
                      interval_residuals_m.reserve(interval_specs.size());
                      double max_abs_interval_residual_m = 0.0;
                      bool interval_all_inside = true;
                      for (const auto &interval_spec : interval_specs) {
                        const ReferenceNodeState interval_state =
                          ResolveReferenceStateForHoldWindowSpec(iteration.gate_reference_states, interval_spec);
                        const Eigen::Vector3d residual_enu_m =
                          ComputePositionResidualEnu(interval_state.pose, interval_spec.measurement_enu_m);
                        const double tau_s = interval_spec.corrected_time_s - active_window_end_time_s;
                        if (!std::isfinite(residual_enu_m.z()) || tau_s <= 0.1) {
                          continue;
                        }
                        const double gate_threshold_m =
                          std::sqrt(vertical_gate_nis_threshold) * interval_spec.sigma_u_m;
                        if (std::abs(residual_enu_m.z()) >
                            gate_threshold_m + kIntervalFeedbackNearGateToleranceM) {
                          interval_all_inside = false;
                        }
                        max_abs_interval_residual_m =
                          std::max(max_abs_interval_residual_m, std::abs(residual_enu_m.z()));
                        interval_times_s.push_back(tau_s);
                        interval_residuals_m.push_back(residual_enu_m.z());
                      }
                      const double interval_duration_s =
                        interval_specs.back().corrected_time_s - interval_specs.front().corrected_time_s;
                      const auto interval_trend = EvaluateVerticalFutureTrend(
                        iteration.gate_reference_states,
                        interval_specs,
                        config_.vertical_jump_future_trend_min_fix_count,
                        0.0,
                        1.0);
                      double residual_noise_m = std::numeric_limits<double>::quiet_NaN();
                      double residual_trend_signal_m = std::numeric_limits<double>::quiet_NaN();
                      double residual_trend_snr = std::numeric_limits<double>::quiet_NaN();
                      if (interval_trend.valid &&
                          interval_residuals_m.size() >=
                            static_cast<std::size_t>(std::max(config_.vertical_jump_future_trend_min_fix_count, 1))) {
                        const double mean_time_s =
                          std::accumulate(interval_times_s.begin(), interval_times_s.end(), 0.0) /
                          static_cast<double>(interval_times_s.size());
                        const double mean_residual_m =
                          std::accumulate(interval_residuals_m.begin(), interval_residuals_m.end(), 0.0) /
                          static_cast<double>(interval_residuals_m.size());
                        std::vector<double> detrended_residuals_m;
                        detrended_residuals_m.reserve(interval_residuals_m.size());
                        double residual_variance_sum_m2 = 0.0;
                        for (std::size_t residual_index = 0U;
                             residual_index < interval_residuals_m.size();
                             ++residual_index) {
                          const double fitted_residual_m =
                            mean_residual_m +
                            interval_trend.residual_slope_mps *
                              (interval_times_s[residual_index] - mean_time_s);
                          const double detrended_residual_m =
                            interval_residuals_m[residual_index] - fitted_residual_m;
                          detrended_residuals_m.push_back(detrended_residual_m);
                          residual_variance_sum_m2 += detrended_residual_m * detrended_residual_m;
                        }
                        const double residual_std_m =
                          interval_residuals_m.size() > 1U
                            ? std::sqrt(
                                residual_variance_sum_m2 /
                                static_cast<double>(interval_residuals_m.size() - 1U))
                            : 0.0;
                        const double median_detrended_residual_m =
                          MedianFinite(detrended_residuals_m);
                        std::vector<double> abs_mad_residuals_m;
                        abs_mad_residuals_m.reserve(detrended_residuals_m.size());
                        for (const double detrended_residual_m : detrended_residuals_m) {
                          abs_mad_residuals_m.push_back(
                            std::abs(detrended_residual_m - median_detrended_residual_m));
                        }
                        const double residual_mad_sigma_m = 1.4826 * MedianFinite(abs_mad_residuals_m);
                        residual_noise_m =
                          std::max(
                            {residual_std_m,
                             residual_mad_sigma_m,
                             config_.vertical_interval_feedback_noise_floor_m});
                        residual_trend_signal_m =
                          std::abs(interval_trend.residual_slope_mps) * interval_duration_s;
                        residual_trend_snr =
                          residual_noise_m > 1e-12 ? residual_trend_signal_m / residual_noise_m
                                                    : std::numeric_limits<double>::infinity();
                      }
                      if ((interval_all_inside || !inside_gate) &&
                          interval_duration_s >= config_.vertical_interval_feedback_min_duration_s &&
                          interval_trend.valid &&
                          std::isfinite(interval_trend.residual_slope_mps) &&
                          std::isfinite(residual_trend_snr) &&
                          residual_trend_snr >= config_.vertical_interval_feedback_snr_threshold &&
                          std::abs(interval_trend.residual_slope_mps) >=
                            config_.vertical_interval_feedback_min_slope_mps &&
                          residual_trend_signal_m >= config_.vertical_interval_feedback_min_drift_m &&
                          max_abs_interval_residual_m >=
                            config_.vertical_interval_feedback_min_residual_m) {
                        interval_feedback_requested = true;
                        interval_feedback_window_center_state_index =
                          interval_feedback_window->center_state_index;
                        interval_feedback_delta_vz_mps =
                          std::clamp(
                              -config_.vertical_interval_feedback_gain *
                                interval_trend.residual_slope_mps,
                              -config_.vertical_interval_feedback_max_delta_vz_mps,
                              config_.vertical_interval_feedback_max_delta_vz_mps);
                      }
                    }
                  }
                }

                if (!inside_gate || interval_feedback_requested) {
                  std::size_t recovery_anchor_state_index =
                    confirmed_inside_state_index.value_or(graph_timeline.dynamic_start_index);
                  std::size_t nhc_search_start_state_index = recovery_anchor_state_index;
                  const double nhc_search_start_time_s =
                    record.corrected_time_s -
                    std::max(
                      {config_.vertical_jump_hold_window_s,
                       config_.nhc_jump_recovery_lookback_s,
                       1e-3});
                  while (nhc_search_start_state_index > graph_timeline.dynamic_start_index &&
                         state_timestamps[nhc_search_start_state_index] > nhc_search_start_time_s) {
                    --nhc_search_start_state_index;
                  }
                  const double preferred_nhc_jump_sign = prefit_residual_enu_m.z();
                  auto nhc_jump_anchor_state =
                    nhc_jump_detector.FindRecentJumpAnchor(
                      nhc_search_start_state_index,
                      *feedback_anchor_state,
                      preferred_nhc_jump_sign);
                  if (!nhc_jump_anchor_state.has_value()) {
                    nhc_jump_anchor_state =
                      nhc_jump_detector.FindJumpAnchor(
                        iteration.gate_reference_states,
                        nhc_search_start_state_index,
                        *feedback_anchor_state,
                        preferred_nhc_jump_sign);
                  }
                  if (nhc_jump_anchor_state.has_value()) {
                    consistency_record.nhc_jump_anchor_state_index =
                      static_cast<long long>(*nhc_jump_anchor_state);
                    std::size_t nhc_recovery_anchor_state_index = *nhc_jump_anchor_state;
                    if (*nhc_jump_anchor_state > graph_timeline.dynamic_start_index + 1U) {
                      nhc_recovery_anchor_state_index = *nhc_jump_anchor_state - 2U;
                    } else if (*nhc_jump_anchor_state > graph_timeline.dynamic_start_index) {
                      nhc_recovery_anchor_state_index = *nhc_jump_anchor_state - 1U;
                    }
                    recovery_anchor_state_index = nhc_recovery_anchor_state_index;
                  }
                  consistency_record.recovery_anchor_state_index =
                    static_cast<long long>(recovery_anchor_state_index);
                  const auto hold_window_specs = build_hold_window_specs(sample_index);
                  const auto future_trend_specs = build_future_trend_specs(sample_index);
                  const bool use_body_z_seed_window_candidates =
                    config_.enable_body_z_seed_jump_windows && !body_z_seed_detection.windows.empty();
                  const std::size_t hold_window_end_state_index =
                    hold_window_specs.empty()
                      ? *feedback_anchor_state
                      : std::max_element(
                          hold_window_specs.begin(),
                          hold_window_specs.end(),
                          [](const VerticalHoldWindowSpec &left, const VerticalHoldWindowSpec &right) {
                            return left.reference_state_index < right.reference_state_index;
                          })
                          ->reference_state_index;
                  const std::size_t future_trend_end_state_index =
                    future_trend_specs.empty()
                      ? hold_window_end_state_index
                      : std::max_element(
                          future_trend_specs.begin(),
                          future_trend_specs.end(),
                          [](const VerticalHoldWindowSpec &left, const VerticalHoldWindowSpec &right) {
                            return left.reference_state_index < right.reference_state_index;
                          })
                          ->reference_state_index;
                  const std::size_t recovery_scoring_end_state_index =
                    std::max(hold_window_end_state_index, future_trend_end_state_index);
                  const double local_gate_threshold_m =
                    std::sqrt(vertical_gate_nis_threshold) * consistency_base_sigma_m.z();
                  constexpr double kRecoveredGateToleranceM = 0.005;
                  const auto recovered_current_inside = [&](const VerticalHoldWindowEvaluation &evaluation) {
                    return evaluation.current_inside ||
                           std::abs(evaluation.current_local_postfit_u_m) <=
                             local_gate_threshold_m + kRecoveredGateToleranceM;
                  };
                  double iteration_prefit_u_m = prefit_residual_enu_m.z();
                  long long pure_delta_up_anchor_start_iteration = -1;
                  int local_recovery_attempt_count = 0;
                  std::optional<VerticalLocalRecoveryResult> last_recovery_result;
                  double last_velocity_recovery_postfit_u_m = std::numeric_limits<double>::quiet_NaN();
                  bool hold_window_passed = false;
                  constexpr double kNearGateNoRecoveryToleranceM = 0.0;
                  bool defer_body_z_window_gate_until_post_window = false;
                  bool pending_body_z_interval_feedback = false;
                  bool accepted_body_z_velocity_feedback_pending = false;
                  if (!inside_gate && use_body_z_seed_window_candidates) {
                    std::optional<std::size_t> covering_body_z_window_start_state_index;
                    std::optional<std::size_t> covering_body_z_window_end_state_index;
                    for (const auto &body_z_window : body_z_seed_detection.windows) {
                      const bool feedback_state_inside_window =
                        body_z_window.start_state_index <= *feedback_anchor_state &&
                        *feedback_anchor_state <= body_z_window.end_state_index;
                      const bool gnss_time_inside_window =
                        record.corrected_time_s >= body_z_window.start_time_s - kTimeEpsilonS &&
                        record.corrected_time_s <=
                          body_z_window.end_time_s +
                            std::max(config_.state_meas_sync_upper_bound_s, kTimeEpsilonS);
                      if ((feedback_state_inside_window || gnss_time_inside_window) &&
                          body_z_window.start_state_index >= graph_timeline.dynamic_start_index + 1U) {
                        if (!covering_body_z_window_start_state_index.has_value() ||
                            body_z_window.start_state_index > *covering_body_z_window_start_state_index) {
                          covering_body_z_window_start_state_index = body_z_window.start_state_index;
                          covering_body_z_window_end_state_index = body_z_window.end_state_index;
                        }
                      }
                    }
                    if (covering_body_z_window_end_state_index.has_value()) {
                      defer_body_z_window_gate_until_post_window = true;
                    }
                  }
                  if (!inside_gate && !interval_feedback_requested &&
                      !defer_body_z_window_gate_until_post_window &&
                      interval_feedback_window.has_value() &&
                      interval_feedback_window->end_state_index < iteration.gate_reference_states.size() &&
                      *feedback_anchor_state > interval_feedback_window->end_state_index) {
                    const double interval_window_end_time_s =
                      iteration.gate_reference_states[interval_feedback_window->end_state_index].time_s;
                    double next_body_z_start_time_s = std::numeric_limits<double>::infinity();
                    for (const auto &body_z_window : body_z_seed_detection.windows) {
                      if (body_z_window.start_state_index > interval_feedback_window->end_state_index &&
                          body_z_window.start_state_index < iteration.gate_reference_states.size()) {
                        next_body_z_start_time_s = std::min(
                          next_body_z_start_time_s,
                          iteration.gate_reference_states[body_z_window.start_state_index].time_s);
                      }
                    }
                    auto pending_interval_specs =
                      build_vertical_window_specs_between_times(interval_window_end_time_s, record.corrected_time_s);
                    pending_interval_specs =
                      FilterVerticalHoldWindowSpecsAfterState(
                        pending_interval_specs,
                        interval_feedback_window->end_state_index);
                    const double observed_interval_duration_s =
                      pending_interval_specs.size() >= 2U
                        ? pending_interval_specs.back().corrected_time_s -
                            pending_interval_specs.front().corrected_time_s
                        : 0.0;
                    const bool next_body_z_window_should_merge =
                      std::isfinite(next_body_z_start_time_s) &&
                      next_body_z_start_time_s - interval_window_end_time_s <=
                        std::max(config_.body_z_jump_redundant_padding_s, config_.body_z_jump_merge_gap_s) +
                          kTimeEpsilonS;
                    const bool waiting_for_interval_feedback_cooldown =
                      std::isfinite(last_interval_feedback_time_s) &&
                      record.corrected_time_s - last_interval_feedback_time_s < 1.0;
                    pending_body_z_interval_feedback =
                      waiting_for_interval_feedback_cooldown ||
                      observed_interval_duration_s < config_.vertical_interval_feedback_min_duration_s ||
                      (next_body_z_window_should_merge &&
                       record.corrected_time_s < next_body_z_start_time_s - kTimeEpsilonS);
                    if (pending_body_z_interval_feedback) {
                      defer_body_z_window_gate_until_post_window = true;
                    }
                  }
                  if (std::abs(iteration_prefit_u_m) <= local_gate_threshold_m + kNearGateNoRecoveryToleranceM) {
                    inside_gate = true;
                    consistency_record.local_postfit_residual_u_m = iteration_prefit_u_m;
                    consistency_record.prefit_residual_u_after_local_recovery_m = iteration_prefit_u_m;
                    consistency_record.required_up_anchor_correction_m = 0.0;
                  } else if (defer_body_z_window_gate_until_post_window) {
                    consistency_record.local_postfit_residual_u_m = iteration_prefit_u_m;
                    consistency_record.prefit_residual_u_after_local_recovery_m = iteration_prefit_u_m;
                    consistency_record.required_up_anchor_correction_m = 0.0;
                    consistency_record.recovery_mode =
                      pending_body_z_interval_feedback ? "BODY_Z_WINDOW_PENDING"
                                                       : "BODY_Z_WINDOW_DEFERRED";
                  }
                  std::unordered_set<std::size_t> nhc_supported_state_indices;
                  for (std::size_t state_index = std::max<std::size_t>(recovery_anchor_state_index + 1U, 1U);
                       state_index <= *feedback_anchor_state;
                       ++state_index) {
                    const NhcStateEvaluation nhc_candidate_evaluation =
                      nhc_jump_detector.EvaluateTransition(
                        iteration.gate_reference_states[state_index - 1U],
                        iteration.gate_reference_states[state_index],
                        iteration.gate_reference_states[state_index].time_s);
                    if (nhc_candidate_evaluation.exceeds_threshold) {
                      nhc_supported_state_indices.insert(state_index);
                    }
                  }
                  if (nhc_jump_anchor_state.has_value()) {
                    nhc_supported_state_indices.insert(*nhc_jump_anchor_state);
                  }
                  std::vector<std::size_t> selected_jump_window_center_indices;
                  const int max_accepted_window_corrections =
                    std::max(
                      1,
                      std::min(
                        config_.vertical_jump_max_selected_points_per_segment,
                        config_.vertical_jump_window_max_correction_attempts));

                  for (int jump_selection_index = 0;
                       jump_selection_index < max_accepted_window_corrections &&
                       (!inside_gate || interval_feedback_requested) &&
                       !defer_body_z_window_gate_until_post_window;
                       ++jump_selection_index) {
                    auto current_scoring_reference_states = iteration.gate_reference_states;
                    std::size_t current_scoring_valid_until_index = sequential_reference_valid_until_index;
                    std::size_t current_scoring_required_index = recovery_scoring_end_state_index;
                    if (config_.enable_body_z_seed_jump_windows && !body_z_seed_detection.windows.empty()) {
                      for (const auto &body_z_window : body_z_seed_detection.windows) {
                        if (body_z_window.end_state_index < graph_timeline.dynamic_start_index + 1U ||
                            body_z_window.start_state_index > *feedback_anchor_state ||
                            body_z_window.end_state_index >= state_timestamps.size()) {
                          continue;
                        }
                        current_scoring_required_index =
                          std::max(current_scoring_required_index, body_z_window.end_state_index);
                      }
                    }
                    EnsureReferenceStatesValidUntil(
                      state_timestamps,
                      dataset.imu_samples,
                      imu_params,
                      &current_scoring_reference_states,
                      &current_scoring_valid_until_index,
                      current_scoring_required_index,
                      nullptr);
                    const auto current_hold_evaluation = EvaluateVerticalHoldWindow(
                      current_scoring_reference_states,
                      hold_window_specs,
                      recovery_anchor_state_index,
                      vertical_gate_nis_threshold);
                    const double current_vz_reference_rms = compute_vz_reference_rms(
                      current_scoring_reference_states,
                      recovery_anchor_state_index,
                      hold_window_end_state_index);
                    const auto current_future_trend_evaluation = EvaluateVerticalFutureTrend(
                      current_scoring_reference_states,
                      future_trend_specs,
                      config_.vertical_jump_future_trend_min_fix_count,
                      config_.vertical_jump_future_trend_mean_weight,
                      config_.vertical_jump_future_trend_slope_weight);
                    const double current_objective =
                      use_body_z_seed_window_candidates
                        ? current_hold_evaluation.gate_excess_cost +
                            0.02 * static_cast<double>(selected_jump_window_center_indices.size())
                        : current_hold_evaluation.gate_excess_cost +
                            config_.vertical_jump_window_ref_weight *
                              (std::isfinite(current_vz_reference_rms)
                                 ? current_vz_reference_rms * current_vz_reference_rms
                                 : 0.0) +
                            current_future_trend_evaluation.cost +
                            0.02 * static_cast<double>(selected_jump_window_center_indices.size());

                    std::vector<SparseVerticalJumpWindowCandidate> sparse_jump_windows;
                    std::optional<std::size_t> covering_body_z_window_start_state_index;
                    if (use_body_z_seed_window_candidates) {
                      for (const auto &body_z_window : body_z_seed_detection.windows) {
                        if (body_z_window.start_state_index <= *feedback_anchor_state &&
                            *feedback_anchor_state <= body_z_window.end_state_index &&
                            body_z_window.start_state_index >= graph_timeline.dynamic_start_index + 1U) {
                          covering_body_z_window_start_state_index =
                            covering_body_z_window_start_state_index.has_value()
                              ? std::max(
                                  *covering_body_z_window_start_state_index,
                                  body_z_window.start_state_index)
                              : std::optional<std::size_t>(body_z_window.start_state_index);
                        }
                      }
                    }
                    if (use_body_z_seed_window_candidates) {
                      for (const auto &body_z_window : body_z_seed_detection.windows) {
                        if (covering_body_z_window_start_state_index.has_value() &&
                            body_z_window.start_state_index < *covering_body_z_window_start_state_index) {
                          continue;
                        }
                        const auto window_candidate = BuildBodyZSeedSparseWindowCandidate(
                          body_z_window,
                          state_timestamps,
                          current_scoring_reference_states,
                          vertical_vz_reference_by_state,
                          graph_timeline.dynamic_start_index,
                          *feedback_anchor_state,
                          interval_feedback_window_center_state_index,
                          nhc_supported_state_indices);
                        if (window_candidate.has_value()) {
                          sparse_jump_windows.push_back(*window_candidate);
                        }
                      }
                    }
                    if (!use_body_z_seed_window_candidates) {
                      auto fallback_sparse_jump_windows = iteration_sparse_jump_planner.BuildWindowCandidates(
                        iteration.gate_reference_states,
                        vertical_vz_reference_by_state,
                        recovery_anchor_state_index,
                        *feedback_anchor_state,
                        [&](const std::size_t state_index) {
                          return nhc_supported_state_indices.contains(state_index);
                        });
                      sparse_jump_windows.insert(
                        sparse_jump_windows.end(),
                        std::make_move_iterator(fallback_sparse_jump_windows.begin()),
                        std::make_move_iterator(fallback_sparse_jump_windows.end()));
                    }
                    sparse_jump_windows.erase(
                      std::remove_if(
                        sparse_jump_windows.begin(),
                        sparse_jump_windows.end(),
                        [&](const SparseVerticalJumpWindowCandidate &window_candidate) {
                          return !std::isfinite(window_candidate.center_candidate.vz_mismatch_jump_mps) ||
                                 std::find(
                                   selected_jump_window_center_indices.begin(),
                                   selected_jump_window_center_indices.end(),
                                   window_candidate.center_state_index) != selected_jump_window_center_indices.end();
                        }),
                      sparse_jump_windows.end());
                    const bool has_body_z_seed_window_candidate =
                      std::any_of(
                        sparse_jump_windows.begin(),
                        sparse_jump_windows.end(),
                        [](const SparseVerticalJumpWindowCandidate &window_candidate) {
                          return window_candidate.center_candidate.source == "BODY_Z_SEED_WINDOW";
                        });
                    if (sparse_jump_windows.empty()) {
                      break;
                    }
                    std::stable_sort(
                      sparse_jump_windows.begin(),
                      sparse_jump_windows.end(),
                      [&](const SparseVerticalJumpWindowCandidate &left,
                          const SparseVerticalJumpWindowCandidate &right) {
                        if (interval_feedback_window_center_state_index.has_value()) {
                          const bool left_is_active =
                            left.center_state_index == *interval_feedback_window_center_state_index;
                          const bool right_is_active =
                            right.center_state_index == *interval_feedback_window_center_state_index;
                          if (left_is_active != right_is_active) {
                            return left_is_active;
                          }
                        }
                        return left.end_state_index > right.end_state_index;
                      });

                    double best_objective = std::numeric_limits<double>::infinity();
                    std::optional<SparseVerticalJumpWindowCandidate> best_window;
                    std::optional<VerticalVelocityWindowCorrection> best_window_correction;
                    std::optional<VerticalLocalRecoveryResult> best_recovery_result;
                    std::vector<ReferenceNodeState> best_reference_states;
                    gtsam::Values best_initial_values;
                    std::size_t best_valid_until_index = sequential_reference_valid_until_index;
                    VerticalHoldWindowEvaluation best_hold_evaluation;

                    for (const auto &window_candidate : sparse_jump_windows) {
                      const bool body_z_seed_candidate =
                        window_candidate.center_candidate.source == "BODY_Z_SEED_WINDOW";
                      const bool current_sample_covered_by_body_z_window =
                        body_z_seed_candidate &&
                        (*feedback_anchor_state <= window_candidate.end_state_index ||
                         record.corrected_time_s <=
                           state_timestamps[window_candidate.end_state_index] +
                             std::max(config_.state_meas_sync_upper_bound_s, kTimeEpsilonS));
                      const bool body_z_position_offset_already_used =
                        body_z_seed_candidate &&
                        body_z_windows_with_position_offset_correction.contains(
                          window_candidate.center_state_index);
                      const bool body_z_velocity_already_used =
                        body_z_seed_candidate &&
                        body_z_windows_with_velocity_correction.contains(
                          window_candidate.center_state_index);
                      const int body_z_velocity_feedback_count =
                        body_z_seed_candidate &&
                            body_z_window_velocity_feedback_counts.contains(window_candidate.center_state_index)
                          ? body_z_window_velocity_feedback_counts[window_candidate.center_state_index]
                          : 0;
                      const bool body_z_velocity_feedback_limit_reached =
                        body_z_seed_candidate &&
                        body_z_velocity_feedback_count >=
                          std::max(config_.vertical_local_recovery_max_iterations - 1, 1);
                      const bool body_z_velocity_feedback_for_window =
                        body_z_seed_candidate &&
                        interval_feedback_requested &&
                        interval_feedback_window_center_state_index.has_value() &&
                        window_candidate.center_state_index == *interval_feedback_window_center_state_index &&
                        std::isfinite(interval_feedback_delta_vz_mps) &&
                        !body_z_velocity_feedback_limit_reached;
                      std::vector<double> forced_tail_delta_options_mps;
                      if (!body_z_seed_candidate) {
                        forced_tail_delta_options_mps.push_back(
                          std::numeric_limits<double>::quiet_NaN());
                        forced_tail_delta_options_mps.push_back(
                          window_candidate.center_candidate.delta_vz_init_mps);
                      } else {
                        forced_tail_delta_options_mps = BuildBodyZTailVelocityTargetsMps(
                          current_scoring_reference_states,
                          window_candidate,
                          body_z_velocity_already_used,
                          body_z_velocity_feedback_for_window,
                          interval_feedback_delta_vz_mps);
                      }
                      if (!body_z_seed_candidate &&
                          !current_sample_covered_by_body_z_window &&
                          window_candidate.end_state_index < current_scoring_reference_states.size() &&
                          std::isfinite(iteration_prefit_u_m)) {
                        const double feedback_dt_s = std::max(
                          std::abs(
                            record.corrected_time_s -
                            state_timestamps[window_candidate.end_state_index]),
                          0.05);
                        const double current_tail_vz_mps =
                          current_scoring_reference_states[window_candidate.end_state_index].velocity.z();
                        const double residual_feedback_delta_vz_mps =
                          std::clamp(-iteration_prefit_u_m / feedback_dt_s, -0.5, 0.5);
                        for (const double feedback_scale : {1.0, 0.75, 1.25, 0.5, 1.5}) {
                          forced_tail_delta_options_mps.push_back(
                            current_tail_vz_mps + feedback_scale * residual_feedback_delta_vz_mps);
                        }
                      }
                      std::vector<double> forced_tail_delta_up_options_m =
                        body_z_seed_candidate
                          ? BuildBodyZTailPositionOffsetsM(
                              current_scoring_reference_states,
                              window_candidate,
                              body_z_position_offset_already_used)
                          : std::vector<double>{std::numeric_limits<double>::quiet_NaN()};
                      if (!body_z_seed_candidate &&
                          window_candidate.start_state_index > 0U &&
                          window_candidate.start_state_index - 1U < iteration.gate_reference_states.size() &&
                          std::isfinite(iteration_prefit_u_m)) {
                        const double residual_correction_dt_s = std::max(
                          record.corrected_time_s -
                            state_timestamps[window_candidate.start_state_index - 1U],
                          0.05);
                        forced_tail_delta_options_mps.push_back(
                          std::clamp(-iteration_prefit_u_m / residual_correction_dt_s, -2.0, 2.0));
                        for (const double up_delta_scale : {0.5, 0.75, 1.0, 1.25, 1.5, 2.0}) {
                          forced_tail_delta_up_options_m.push_back(
                            std::clamp(-iteration_prefit_u_m * up_delta_scale, -5.0, 5.0));
                        }
                      }
                      if (!body_z_seed_candidate &&
                          current_future_trend_evaluation.valid &&
                          std::isfinite(current_future_trend_evaluation.residual_mean_m)) {
                        for (const double up_delta_scale : {0.5, 1.0, 1.5}) {
                          forced_tail_delta_up_options_m.push_back(
                            std::clamp(
                              -current_future_trend_evaluation.residual_mean_m * up_delta_scale,
                              -5.0,
                              5.0));
                        }
                      }

                      std::size_t candidate_scoring_end_state_index = recovery_scoring_end_state_index;
                      std::vector<VerticalHoldWindowSpec> body_z_segment_specs;
                      std::vector<VerticalHoldWindowSpec> candidate_acceptance_hold_specs = hold_window_specs;
                      if (current_sample_covered_by_body_z_window) {
                        // The triggering RTK sample is synchronized inside the detected acceleration anomaly.
                        // Do not reject the correction only because the abnormal window's internal height is
                        // outside gate; score the candidate from the first post-window RTK evidence instead.
                        candidate_acceptance_hold_specs =
                          FilterVerticalHoldWindowSpecsAfterState(hold_window_specs, window_candidate.end_state_index);
                      }
                      if (body_z_seed_candidate) {
                        std::size_t body_z_segment_end_state_index = recovery_scoring_end_state_index;
                        std::optional<std::size_t> next_body_z_start_state_index;
                        for (const auto &body_z_window : body_z_seed_detection.windows) {
                          if (body_z_window.start_state_index <= window_candidate.end_state_index) {
                            continue;
                          }
                          next_body_z_start_state_index =
                            next_body_z_start_state_index.has_value()
                              ? std::min(*next_body_z_start_state_index, body_z_window.start_state_index)
                              : std::optional<std::size_t>(body_z_window.start_state_index);
                        }
                        if (next_body_z_start_state_index.has_value() &&
                            *next_body_z_start_state_index > 0U) {
                          body_z_segment_end_state_index =
                            std::max(
                              body_z_segment_end_state_index,
                              *next_body_z_start_state_index - 1U);
                        }
                        body_z_segment_end_state_index =
                          std::min(body_z_segment_end_state_index, iteration.gate_reference_states.size() - 1U);
                        candidate_scoring_end_state_index =
                          std::max(candidate_scoring_end_state_index, body_z_segment_end_state_index);
                        body_z_segment_specs = build_vertical_window_specs_between_times(
                          state_timestamps[window_candidate.end_state_index],
                          state_timestamps[body_z_segment_end_state_index]);
                        body_z_segment_specs = FilterVerticalHoldWindowSpecsAfterState(
                          body_z_segment_specs, window_candidate.end_state_index);
                        if (current_sample_covered_by_body_z_window &&
                            candidate_acceptance_hold_specs.empty()) {
                          candidate_acceptance_hold_specs = body_z_segment_specs;
                        }
                      }
                      candidate_scoring_end_state_index =
                        std::min(
                          std::max(candidate_scoring_end_state_index, window_candidate.end_state_index),
                          iteration.gate_reference_states.size() - 1U);
                      if (current_sample_covered_by_body_z_window &&
                          candidate_acceptance_hold_specs.empty()) {
                        continue;
                      }
                      const std::vector<double> tail_delta_scales =
                        body_z_seed_candidate
                          ? std::vector<double>{1.0}
                          : std::vector<double>{0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0};
                      for (const double forced_tail_delta_mps : forced_tail_delta_options_mps) {
                      for (const double tail_delta_scale : tail_delta_scales) {
                      std::vector<double> candidate_forced_tail_delta_up_options_m =
                        forced_tail_delta_up_options_m;
                      if (body_z_seed_candidate &&
                          !current_sample_covered_by_body_z_window &&
                          !body_z_velocity_feedback_for_window) {
                        const double base_tail_up_offset_m =
                          candidate_forced_tail_delta_up_options_m.empty()
                            ? 0.0
                            : candidate_forced_tail_delta_up_options_m.front();
                        if (body_z_velocity_feedback_limit_reached &&
                            std::isfinite(iteration_prefit_u_m)) {
                          PushUniqueCandidateValue(
                            &candidate_forced_tail_delta_up_options_m,
                            base_tail_up_offset_m - iteration_prefit_u_m);
                        }
                        auto provisional_reference_states = iteration.gate_reference_states;
                        gtsam::Values provisional_initial_values = sequential_initial_values;
                        std::size_t provisional_valid_until_index = sequential_reference_valid_until_index;
                        EnsureReferenceStatesValidUntil(
                          state_timestamps,
                          dataset.imu_samples,
                          imu_params,
                          &provisional_reference_states,
                          &provisional_valid_until_index,
                          candidate_scoring_end_state_index,
                          &provisional_initial_values);
                        const auto provisional_window_correction = BuildVerticalVelocityWindowCorrection(
                          config_,
                          provisional_reference_states,
                          vertical_vz_reference_by_state,
                          window_candidate,
                          candidate_scoring_end_state_index,
                          tail_delta_scale,
                          forced_tail_delta_mps,
                          base_tail_up_offset_m);
                        if (provisional_window_correction.has_value() &&
                            std::isfinite(provisional_window_correction->delta_vz_tail_mps)) {
                          ApplyVerticalVelocityWindowCorrection(
                            *provisional_window_correction,
                            &provisional_reference_states,
                            &provisional_initial_values);
                          provisional_valid_until_index =
                            provisional_window_correction->end_state_index;
                          EnsureReferenceStatesValidUntil(
                            state_timestamps,
                            dataset.imu_samples,
                            imu_params,
                            &provisional_reference_states,
                            &provisional_valid_until_index,
                            candidate_scoring_end_state_index,
                            &provisional_initial_values);
                          auto stable_offset_specs =
                            build_vertical_window_specs_between_times(
                              state_timestamps[window_candidate.end_state_index],
                              record.corrected_time_s);
                          stable_offset_specs =
                            FilterVerticalHoldWindowSpecsAfterState(
                              stable_offset_specs,
                              window_candidate.end_state_index);
                          if (stable_offset_specs.empty()) {
                            stable_offset_specs = candidate_acceptance_hold_specs;
                          }
                          const auto provisional_stable_offset =
                            EvaluateVerticalFutureTrend(
                              provisional_reference_states,
                              stable_offset_specs,
                              config_.vertical_jump_future_trend_min_fix_count,
                              1.0,
                              1.0);
                          const double stable_offset_slope_limit_mps =
                            std::max(
                              2.0 * config_.vertical_interval_feedback_max_delta_vz_mps,
                              0.01);
                          const bool stable_offset_observable =
                            provisional_stable_offset.valid &&
                            std::isfinite(provisional_stable_offset.residual_mean_m) &&
                            std::isfinite(provisional_stable_offset.residual_slope_mps) &&
                            std::abs(provisional_stable_offset.residual_mean_m) >=
                              config_.vertical_interval_feedback_min_residual_m &&
                            (body_z_velocity_feedback_limit_reached ||
                             std::abs(provisional_stable_offset.residual_slope_mps) <=
                               stable_offset_slope_limit_mps);
                          if (stable_offset_observable) {
                            const double requested_tail_up_delta_m =
                              base_tail_up_offset_m - provisional_stable_offset.residual_mean_m;
                            for (const double up_delta_scale : {0.5, 0.75, 1.0, 1.25}) {
                              const double scaled_tail_up_delta_m =
                                base_tail_up_offset_m +
                                up_delta_scale *
                                  (requested_tail_up_delta_m - base_tail_up_offset_m);
                              if (std::abs(scaled_tail_up_delta_m) > 1e-4) {
                                PushUniqueCandidateValue(
                                  &candidate_forced_tail_delta_up_options_m,
                                  scaled_tail_up_delta_m);
                              }
                            }
                          }
                        }
                      }
                      for (const double forced_tail_delta_up_m :
                           candidate_forced_tail_delta_up_options_m) {
                      auto candidate_reference_states = iteration.gate_reference_states;
                      gtsam::Values candidate_initial_values = sequential_initial_values;
                      std::size_t candidate_valid_until_index = sequential_reference_valid_until_index;
                      EnsureReferenceStatesValidUntil(
                        state_timestamps,
                        dataset.imu_samples,
                        imu_params,
                        &candidate_reference_states,
                        &candidate_valid_until_index,
                        candidate_scoring_end_state_index,
                        &candidate_initial_values);
                      const auto window_correction = BuildVerticalVelocityWindowCorrection(
                        config_,
                        candidate_reference_states,
                        vertical_vz_reference_by_state,
                        window_candidate,
                        candidate_scoring_end_state_index,
                        tail_delta_scale,
                        forced_tail_delta_mps,
                        forced_tail_delta_up_m);
                      if (!window_correction.has_value() ||
                          !std::isfinite(window_correction->delta_vz_tail_mps) ||
                          !std::isfinite(window_correction->delta_up_tail_m) ||
                          (std::abs(window_correction->delta_vz_tail_mps) <= 1e-9 &&
                           std::abs(window_correction->delta_up_tail_m) <= 1e-9)) {
                        continue;
                      }
                      ApplyVerticalVelocityWindowCorrection(
                        *window_correction,
                        &candidate_reference_states,
                        &candidate_initial_values);
                      candidate_valid_until_index = window_correction->end_state_index;
                      EnsureReferenceStatesValidUntil(
                        state_timestamps,
                        dataset.imu_samples,
                        imu_params,
                        &candidate_reference_states,
                        &candidate_valid_until_index,
                        candidate_scoring_end_state_index,
                        &candidate_initial_values);

                      const auto candidate_hold_evaluation = EvaluateVerticalHoldWindow(
                        candidate_reference_states,
                        candidate_acceptance_hold_specs,
                        recovery_anchor_state_index,
                        vertical_gate_nis_threshold);
                      if (body_z_seed_candidate) {
                        const bool velocity_feedback_candidate =
                          body_z_velocity_feedback_for_window &&
                          std::isfinite(forced_tail_delta_mps);
                        const bool position_feedback_candidate =
                          std::isfinite(forced_tail_delta_up_m) &&
                          std::abs(forced_tail_delta_up_m) > 1e-4;
                        const bool initial_body_z_window_candidate =
                          !body_z_velocity_already_used &&
                          std::isnan(forced_tail_delta_mps);
                        if (!candidate_hold_evaluation.hold_window_passed &&
                            !recovered_current_inside(candidate_hold_evaluation) &&
                            !velocity_feedback_candidate &&
                            !position_feedback_candidate &&
                            !initial_body_z_window_candidate) {
                          continue;
                        }
                      } else if (!recovered_current_inside(candidate_hold_evaluation)) {
                        continue;
                      }
                      const double candidate_vz_reference_rms = compute_vz_reference_rms(
                        candidate_reference_states,
                        window_correction->end_state_index,
                        hold_window_end_state_index);
                      const auto candidate_future_trend_evaluation = EvaluateVerticalFutureTrend(
                        candidate_reference_states,
                        future_trend_specs,
                        config_.vertical_jump_future_trend_min_fix_count,
                        config_.vertical_jump_future_trend_mean_weight,
                        config_.vertical_jump_future_trend_slope_weight);
                      double body_z_segment_gate_cost = 0.0;
                      if (!body_z_segment_specs.empty()) {
                        const auto candidate_body_z_segment_evaluation = EvaluateVerticalHoldWindow(
                          candidate_reference_states,
                          body_z_segment_specs,
                          window_correction->end_state_index,
                          vertical_gate_nis_threshold);
                        body_z_segment_gate_cost =
                          100.0 * candidate_body_z_segment_evaluation.gate_excess_cost +
                          (candidate_body_z_segment_evaluation.hold_window_passed ? 0.0 : 1.0);
                      }
                      // Body-z seed windows are pre-localized by the RTK seed attitude pass, so they should not
                      // lose to later GNSS/NHC mismatch windows while any body-z candidate remains viable.
                      const double candidate_objective =
                        (has_body_z_seed_window_candidate &&
                             window_candidate.center_candidate.source != "BODY_Z_SEED_WINDOW"
                           ? 1.0e6
                           : 0.0) +
                        candidate_hold_evaluation.gate_excess_cost +
                        (candidate_hold_evaluation.hold_window_passed ? 0.0 : 1.0) +
                        (body_z_seed_candidate
                           ? 0.0
                           : config_.vertical_jump_window_ref_weight *
                               (std::isfinite(candidate_vz_reference_rms)
                                  ? candidate_vz_reference_rms * candidate_vz_reference_rms
                                  : 0.0)) +
                        config_.vertical_jump_window_velocity_smoothness_weight *
                          window_correction->velocity_smooth_cost +
                        config_.vertical_jump_window_height_integral_weight *
                          window_correction->height_integral_delta_m *
                          window_correction->height_integral_delta_m +
                        body_z_segment_gate_cost +
                        (body_z_seed_candidate ? 0.0 : candidate_future_trend_evaluation.cost) +
                        0.02 * static_cast<double>(selected_jump_window_center_indices.size() + 1U);
                      if (!std::isfinite(candidate_objective) || candidate_objective >= best_objective) {
                        continue;
                      }

                      VerticalLocalRecoveryResult scored_result;
                      scored_result.recovered_anchor_state =
                        candidate_reference_states[recovery_anchor_state_index];
                      scored_result.local_postfit_u_m = candidate_hold_evaluation.current_local_postfit_u_m;
                      scored_result.required_up_anchor_correction_m =
                        ComputeRequiredUpAnchorCorrectionM(
                          ResolveReferenceStateForHoldWindowSpec(
                            candidate_reference_states,
                            candidate_acceptance_hold_specs.front())
                            .pose,
                          candidate_acceptance_hold_specs.front().measurement_enu_m);
                      scored_result.delta_vz_applied_mps = window_correction->delta_vz_tail_mps;
                      scored_result.delta_roll_applied_rad = 0.0;
                      scored_result.delta_pitch_applied_rad = 0.0;
                      scored_result.delta_baz_applied_mps2 = 0.0;
                      scored_result.selected_jump_state_index =
                        static_cast<long long>(window_candidate.center_state_index);
                      scored_result.selected_jump_delta_vz_mps =
                        window_candidate.center_candidate.delta_vz_init_mps;
                      scored_result.jump_candidate_score = window_candidate.center_candidate.score;
                      scored_result.selected_jump_window_start_state_index =
                        static_cast<long long>(window_candidate.start_state_index);
                      scored_result.selected_jump_window_center_state_index =
                        static_cast<long long>(window_candidate.center_state_index);
                      scored_result.selected_jump_window_end_state_index =
                        static_cast<long long>(window_candidate.end_state_index);
                      scored_result.selected_jump_window_duration_s = window_candidate.duration_s;
                      scored_result.selected_jump_window_point_count =
                        static_cast<long long>(window_candidate.point_count);
                      scored_result.selected_jump_delta_vz_tail_mps =
                        window_correction->delta_vz_tail_mps;
                      scored_result.window_velocity_smooth_cost =
                        window_correction->velocity_smooth_cost;
                      scored_result.window_height_integral_delta_m =
                        window_correction->height_integral_delta_m;
                      scored_result.future_trend_residual_mean_m =
                        candidate_future_trend_evaluation.residual_mean_m;
                      scored_result.future_trend_residual_slope_mps =
                        candidate_future_trend_evaluation.residual_slope_mps;
                      scored_result.future_trend_cost = candidate_future_trend_evaluation.cost;
                      scored_result.future_trend_fix_count =
                        static_cast<long long>(candidate_future_trend_evaluation.fix_count);
                      if (body_z_seed_candidate &&
                          std::abs(window_correction->delta_vz_tail_mps) > 1e-6) {
                        scored_result.recovery_mode = "SPARSE_WINDOW_VELOCITY";
                      } else if (body_z_seed_candidate &&
                                 std::isfinite(forced_tail_delta_up_m) &&
                                 std::abs(forced_tail_delta_up_m) > 1e-4) {
                        scored_result.recovery_mode = "SPARSE_WINDOW_POSITION";
                      } else {
                        scored_result.recovery_mode = "SPARSE_WINDOW";
                      }
                      scored_result.hold_window_passed = candidate_hold_evaluation.hold_window_passed;
                      best_objective = candidate_objective;
                      best_window = window_candidate;
                      best_window_correction = window_correction;
                      best_recovery_result = scored_result;
                      best_reference_states = std::move(candidate_reference_states);
                      best_initial_values = std::move(candidate_initial_values);
                      best_valid_until_index = candidate_valid_until_index;
                      best_hold_evaluation = candidate_hold_evaluation;
                      }
                      }
                      }
                      if (body_z_seed_candidate &&
                          best_window.has_value() &&
                          best_window->center_state_index == window_candidate.center_state_index &&
                          recovered_current_inside(best_hold_evaluation)) {
                        break;
                      }
                    }

                    if (!best_window.has_value() || !best_window_correction.has_value() ||
                        !best_recovery_result.has_value()) {
                      break;
                    }
                    const bool best_objective_improved = best_objective + 1e-9 < current_objective;
                    const bool best_height_improved =
                      std::abs(best_hold_evaluation.current_local_postfit_u_m) + 1e-4 <
                      std::abs(iteration_prefit_u_m);
                    if (!interval_feedback_requested && !best_objective_improved && !best_height_improved) {
                      break;
                    }
                    if (!interval_feedback_requested &&
                        !recovered_current_inside(best_hold_evaluation) &&
                        std::abs(best_hold_evaluation.current_local_postfit_u_m) >=
                          std::abs(iteration_prefit_u_m)) {
                      break;
                    }

                    ++local_recovery_attempt_count;
                    selected_jump_window_center_indices.push_back(best_window->center_state_index);
                    iteration.gate_reference_states = std::move(best_reference_states);
                    sequential_initial_values = std::move(best_initial_values);
                    sequential_reference_valid_until_index = best_valid_until_index;
                    inside_reference_states = iteration.gate_reference_states;
                    history_reobserve_begin_state_index =
                      history_reobserve_begin_state_index.has_value()
                        ? std::min(*history_reobserve_begin_state_index, best_window->start_state_index)
                        : std::optional<std::size_t>(best_window->start_state_index);
                    inside_bias_adapter_reobserve_begin_state_index =
                      inside_bias_adapter_reobserve_begin_state_index.has_value()
                        ? std::min(*inside_bias_adapter_reobserve_begin_state_index, best_window->start_state_index)
                        : std::optional<std::size_t>(best_window->start_state_index);
                    last_recovery_result = best_recovery_result;
                    last_velocity_recovery_postfit_u_m = best_hold_evaluation.current_local_postfit_u_m;
                    hold_window_passed = best_hold_evaluation.hold_window_passed;
                    inside_gate = recovered_current_inside(best_hold_evaluation);
                    if (best_window->center_candidate.source == "BODY_Z_SEED_WINDOW") {
                      active_interval_feedback_window = *best_window;
                      if (best_window_correction.has_value() &&
                          std::isfinite(best_window_correction->delta_vz_tail_mps) &&
                          std::abs(best_window_correction->delta_vz_tail_mps) > 1e-6) {
                        body_z_windows_with_velocity_correction.insert(
                          best_window->center_state_index);
                        ++body_z_window_velocity_feedback_counts[best_window->center_state_index];
                        if (!inside_gate) {
                          accepted_body_z_velocity_feedback_pending = true;
                        }
                      }
                      if (best_window_correction.has_value() &&
                          std::isfinite(best_window_correction->delta_up_tail_m) &&
                          std::abs(best_window_correction->delta_up_tail_m) > 1e-4) {
                        body_z_windows_with_position_offset_correction.insert(
                          best_window->center_state_index);
                        if (!inside_gate) {
                          accepted_body_z_velocity_feedback_pending = true;
                        }
                      }
                    }
                    consistency_record.prefit_residual_u_after_local_recovery_m =
                      best_hold_evaluation.current_local_postfit_u_m;
                    consistency_record.local_postfit_residual_u_m =
                      best_hold_evaluation.current_local_postfit_u_m;
                    consistency_record.required_up_anchor_correction_m =
                      best_recovery_result->required_up_anchor_correction_m;
                    consistency_record.delta_vz_applied_mps = best_recovery_result->delta_vz_applied_mps;
                    consistency_record.delta_up_anchor_applied_m = std::numeric_limits<double>::quiet_NaN();
                    consistency_record.delta_roll_applied_rad = best_recovery_result->delta_roll_applied_rad;
                    consistency_record.delta_pitch_applied_rad = best_recovery_result->delta_pitch_applied_rad;
                    consistency_record.delta_baz_applied_mps2 = best_recovery_result->delta_baz_applied_mps2;
                    consistency_record.vz_ref_global_smoothed_mps =
                      best_window->center_candidate.vz_ref_global_smoothed_mps;
                    consistency_record.vz_prefit_mps = best_window->center_candidate.vz_prefit_mps;
                    consistency_record.vz_mismatch_mps = best_window->center_candidate.vz_mismatch_mps;
                    consistency_record.vz_mismatch_jump_mps = best_window->center_candidate.vz_mismatch_jump_mps;
                    consistency_record.jump_candidate_score = best_window->center_candidate.score;
                    consistency_record.candidate_source = best_window->center_candidate.source;
                    consistency_record.body_z_jump_direction = best_window->center_candidate.body_z_direction;
                    consistency_record.body_z_signed_delta_velocity_mps =
                      best_window->center_candidate.body_z_signed_delta_velocity_mps;
                    consistency_record.body_z_direction_score_mps =
                      best_window->center_candidate.body_z_direction_score_mps;
                    consistency_record.body_z_axis_nav_z = best_window->center_candidate.body_z_axis_nav_z;
                    consistency_record.selected_jump_state_index =
                      static_cast<long long>(best_window->center_state_index);
                    consistency_record.selected_jump_delta_vz_mps =
                      best_recovery_result->selected_jump_delta_vz_mps;
                    consistency_record.selected_jump_window_start_state_index =
                      best_recovery_result->selected_jump_window_start_state_index;
                    consistency_record.selected_jump_window_center_state_index =
                      best_recovery_result->selected_jump_window_center_state_index;
                    consistency_record.selected_jump_window_end_state_index =
                      best_recovery_result->selected_jump_window_end_state_index;
                    consistency_record.selected_jump_window_duration_s =
                      best_recovery_result->selected_jump_window_duration_s;
                    consistency_record.selected_jump_window_point_count =
                      best_recovery_result->selected_jump_window_point_count;
                    consistency_record.selected_jump_delta_vz_tail_mps =
                      best_recovery_result->selected_jump_delta_vz_tail_mps;
                    consistency_record.window_velocity_smooth_cost =
                      best_recovery_result->window_velocity_smooth_cost;
                    consistency_record.window_height_integral_delta_m =
                      best_recovery_result->window_height_integral_delta_m;
                    consistency_record.future_trend_residual_mean_m =
                      best_recovery_result->future_trend_residual_mean_m;
                    consistency_record.future_trend_residual_slope_mps =
                      best_recovery_result->future_trend_residual_slope_mps;
                    consistency_record.future_trend_cost = best_recovery_result->future_trend_cost;
                    consistency_record.future_trend_fix_count = best_recovery_result->future_trend_fix_count;
                    consistency_record.recovery_mode = best_recovery_result->recovery_mode;
                    consistency_record.hold_window_passed = hold_window_passed;
                    if (best_window->start_state_index < iteration.gate_reference_states.size() &&
                        best_window->end_state_index < iteration.gate_reference_states.size()) {
                      refresh_local_postfit_residuals_between_times(
                        iteration.gate_reference_states[best_window->start_state_index].time_s,
                        std::max(
                          iteration.gate_reference_states[best_window->end_state_index].time_s,
                          record.corrected_time_s));
                    }
                    refresh_local_postfit_residual(record, &consistency_record);

                    VerticalLocalRecoveryIterationRow iteration_row;
                    iteration_row.sample_index = sample_index;
                    iteration_row.corrected_time_s = record.corrected_time_s;
                    iteration_row.recovery_anchor_state_index =
                      static_cast<long long>(recovery_anchor_state_index);
                    iteration_row.feedback_anchor_state_index =
                      static_cast<long long>(*feedback_anchor_state);
                    iteration_row.nhc_jump_anchor_state_index =
                      nhc_jump_anchor_state.has_value() ? static_cast<long long>(*nhc_jump_anchor_state) : -1;
                    iteration_row.iteration_index = static_cast<long long>(local_recovery_attempt_count);
                    iteration_row.prefit_u_before_iteration_m = iteration_prefit_u_m;
                    iteration_row.postfit_u_after_velocity_recovery_m =
                      best_hold_evaluation.current_local_postfit_u_m;
                    iteration_row.postfit_u_after_iteration_m =
                      best_hold_evaluation.current_local_postfit_u_m;
                    iteration_row.delta_vz_applied_mps = best_recovery_result->delta_vz_applied_mps;
                    iteration_row.delta_up_anchor_applied_m = std::numeric_limits<double>::quiet_NaN();
                    iteration_row.delta_roll_applied_rad = best_recovery_result->delta_roll_applied_rad;
                    iteration_row.delta_pitch_applied_rad = best_recovery_result->delta_pitch_applied_rad;
                    iteration_row.delta_baz_applied_mps2 = best_recovery_result->delta_baz_applied_mps2;
                    iteration_row.required_up_anchor_correction_m =
                      best_recovery_result->required_up_anchor_correction_m;
                    iteration_row.vz_ref_global_smoothed_mps =
                      best_window->center_candidate.vz_ref_global_smoothed_mps;
                    iteration_row.vz_prefit_mps = best_window->center_candidate.vz_prefit_mps;
                    iteration_row.vz_mismatch_mps = best_window->center_candidate.vz_mismatch_mps;
                    iteration_row.vz_mismatch_jump_mps =
                      best_window->center_candidate.vz_mismatch_jump_mps;
                    iteration_row.jump_candidate_score = best_window->center_candidate.score;
                    iteration_row.candidate_source = best_window->center_candidate.source;
                    iteration_row.body_z_jump_direction = best_window->center_candidate.body_z_direction;
                    iteration_row.body_z_signed_delta_velocity_mps =
                      best_window->center_candidate.body_z_signed_delta_velocity_mps;
                    iteration_row.body_z_direction_score_mps =
                      best_window->center_candidate.body_z_direction_score_mps;
                    iteration_row.body_z_axis_nav_z = best_window->center_candidate.body_z_axis_nav_z;
                    iteration_row.selected_jump_state_index =
                      static_cast<long long>(best_window->center_state_index);
                    iteration_row.selected_jump_delta_vz_mps =
                      best_recovery_result->selected_jump_delta_vz_mps;
                    iteration_row.selected_jump_window_start_state_index =
                      best_recovery_result->selected_jump_window_start_state_index;
                    iteration_row.selected_jump_window_center_state_index =
                      best_recovery_result->selected_jump_window_center_state_index;
                    iteration_row.selected_jump_window_end_state_index =
                      best_recovery_result->selected_jump_window_end_state_index;
                    iteration_row.selected_jump_window_duration_s =
                      best_recovery_result->selected_jump_window_duration_s;
                    iteration_row.selected_jump_window_point_count =
                      best_recovery_result->selected_jump_window_point_count;
                    iteration_row.selected_jump_delta_vz_tail_mps =
                      best_recovery_result->selected_jump_delta_vz_tail_mps;
                    iteration_row.window_velocity_smooth_cost =
                      best_recovery_result->window_velocity_smooth_cost;
                    iteration_row.window_height_integral_delta_m =
                      best_recovery_result->window_height_integral_delta_m;
                    iteration_row.future_trend_residual_mean_m =
                      best_recovery_result->future_trend_residual_mean_m;
                    iteration_row.future_trend_residual_slope_mps =
                      best_recovery_result->future_trend_residual_slope_mps;
                    iteration_row.future_trend_cost = best_recovery_result->future_trend_cost;
                    iteration_row.future_trend_fix_count = best_recovery_result->future_trend_fix_count;
                    iteration_row.recovery_mode = best_recovery_result->recovery_mode;
                    iteration_row.hold_window_passed = hold_window_passed;
                    iteration_row.used_up_anchor_fallback = false;
                    iteration_row.pure_delta_up_anchor_only = false;
                    iteration_row.inside_after_velocity_recovery = best_hold_evaluation.current_inside;
                    iteration_row.inside_after_iteration = inside_gate;
                    iteration.vertical_local_recovery_iterations.push_back(iteration_row);
                    iteration_prefit_u_m = best_hold_evaluation.current_local_postfit_u_m;
                    if (interval_feedback_requested) {
                      last_interval_feedback_time_s = record.corrected_time_s;
                      interval_feedback_requested = false;
                    }
                  }

                  if (!iteration.failed && !inside_gate &&
                      config_.enable_vertical_local_up_anchor_fallback &&
                      last_recovery_result.has_value() &&
                      std::isfinite(last_recovery_result->required_up_anchor_correction_m) &&
                      std::abs(last_recovery_result->required_up_anchor_correction_m) > 0.0) {
                    const auto corrected_anchor_state = ApplyVerticalUpAnchorCorrection(
                      iteration.gate_reference_states[recovery_anchor_state_index],
                      last_recovery_result->required_up_anchor_correction_m);
                    iteration.gate_reference_states[recovery_anchor_state_index] = corrected_anchor_state;
                    UpsertReferenceStateInitialValues(
                      recovery_anchor_state_index,
                      corrected_anchor_state,
                      &sequential_initial_values);
                    sequential_reference_valid_until_index = recovery_anchor_state_index;
                    EnsureReferenceStatesValidUntil(
                      state_timestamps,
                      dataset.imu_samples,
                      imu_params,
                      &iteration.gate_reference_states,
                      &sequential_reference_valid_until_index,
                      hold_window_end_state_index,
                      &sequential_initial_values);
                    const auto corrected_hold_evaluation = EvaluateVerticalHoldWindow(
                      iteration.gate_reference_states,
                      hold_window_specs,
                      recovery_anchor_state_index,
                      vertical_gate_nis_threshold);
                    consistency_record.prefit_residual_u_after_local_recovery_m =
                      corrected_hold_evaluation.current_local_postfit_u_m;
                    consistency_record.local_postfit_residual_u_m =
                      corrected_hold_evaluation.current_local_postfit_u_m;
                    consistency_record.delta_up_anchor_applied_m =
                      last_recovery_result->required_up_anchor_correction_m;
                    consistency_record.recovery_mode = "UP_ANCHOR";
                    hold_window_passed = corrected_hold_evaluation.hold_window_passed;
                    consistency_record.hold_window_passed = hold_window_passed;
                    inside_gate = corrected_hold_evaluation.current_inside;

                    VerticalLocalRecoveryIterationRow fallback_iteration_row;
                    fallback_iteration_row.sample_index = sample_index;
                    fallback_iteration_row.corrected_time_s = record.corrected_time_s;
                    fallback_iteration_row.recovery_anchor_state_index =
                      static_cast<long long>(recovery_anchor_state_index);
                    fallback_iteration_row.feedback_anchor_state_index =
                      static_cast<long long>(*feedback_anchor_state);
                    fallback_iteration_row.nhc_jump_anchor_state_index =
                      nhc_jump_anchor_state.has_value() ? static_cast<long long>(*nhc_jump_anchor_state) : -1;
                    fallback_iteration_row.iteration_index =
                      static_cast<long long>(local_recovery_attempt_count + 1);
                    fallback_iteration_row.prefit_u_before_iteration_m = iteration_prefit_u_m;
                    fallback_iteration_row.postfit_u_after_velocity_recovery_m =
                      last_velocity_recovery_postfit_u_m;
                    fallback_iteration_row.postfit_u_after_iteration_m =
                      corrected_hold_evaluation.current_local_postfit_u_m;
                    fallback_iteration_row.delta_vz_applied_mps = 0.0;
                    fallback_iteration_row.delta_up_anchor_applied_m =
                      last_recovery_result->required_up_anchor_correction_m;
                    fallback_iteration_row.delta_roll_applied_rad = 0.0;
                    fallback_iteration_row.delta_pitch_applied_rad = 0.0;
                    fallback_iteration_row.delta_baz_applied_mps2 = 0.0;
                    fallback_iteration_row.required_up_anchor_correction_m =
                      last_recovery_result->required_up_anchor_correction_m;
                    fallback_iteration_row.vz_ref_global_smoothed_mps =
                      consistency_record.vz_ref_global_smoothed_mps;
                    fallback_iteration_row.vz_prefit_mps = consistency_record.vz_prefit_mps;
                    fallback_iteration_row.vz_mismatch_mps = consistency_record.vz_mismatch_mps;
                    fallback_iteration_row.vz_mismatch_jump_mps = consistency_record.vz_mismatch_jump_mps;
                    fallback_iteration_row.jump_candidate_score = consistency_record.jump_candidate_score;
                    fallback_iteration_row.candidate_source = consistency_record.candidate_source;
                    fallback_iteration_row.body_z_jump_direction = consistency_record.body_z_jump_direction;
                    fallback_iteration_row.body_z_signed_delta_velocity_mps =
                      consistency_record.body_z_signed_delta_velocity_mps;
                    fallback_iteration_row.body_z_direction_score_mps =
                      consistency_record.body_z_direction_score_mps;
                    fallback_iteration_row.body_z_axis_nav_z = consistency_record.body_z_axis_nav_z;
                    fallback_iteration_row.selected_jump_state_index =
                      consistency_record.selected_jump_state_index;
                    fallback_iteration_row.selected_jump_delta_vz_mps =
                      consistency_record.selected_jump_delta_vz_mps;
                    fallback_iteration_row.recovery_mode = "UP_ANCHOR";
                    fallback_iteration_row.hold_window_passed = hold_window_passed;
                    fallback_iteration_row.used_up_anchor_fallback = true;
                    fallback_iteration_row.pure_delta_up_anchor_only = true;
                    fallback_iteration_row.inside_after_velocity_recovery = false;
                    fallback_iteration_row.inside_after_iteration = inside_gate;
                    iteration.vertical_local_recovery_iterations.push_back(fallback_iteration_row);
                    pure_delta_up_anchor_start_iteration =
                      static_cast<long long>(local_recovery_attempt_count + 1);
                  }

                  consistency_record.local_recovery_iteration_count =
                    static_cast<long long>(local_recovery_attempt_count);
                  consistency_record.pure_delta_up_anchor_start_iteration = pure_delta_up_anchor_start_iteration;
                  if (accepted_body_z_velocity_feedback_pending && !inside_gate) {
                    defer_body_z_window_gate_until_post_window = true;
                  }

                  if (!iteration.failed &&
                      !defer_body_z_window_gate_until_post_window &&
                      (!inside_gate || (consistency_record.recovery_mode == "UP_ANCHOR" && !hold_window_passed))) {
                    iteration.failed = true;
                    iteration.failure_message =
                      "vertical local recovery failed at sample " + std::to_string(sample_index) +
                      ", required_up_anchor_correction_m=" +
                      std::to_string(consistency_record.required_up_anchor_correction_m);
                  }
                }
              }
            }

            consistency_record.vertical_gate_inside = inside_gate ? 1.0 : 0.0;
            if (!iteration.failed && inside_gate && !apply_initial_vertical_anchor &&
                feedback_anchor_state.has_value() &&
                *feedback_anchor_state < iteration.gate_reference_states.size() &&
                config_.enable_vertical_inside_bias_adaptation) {
              const auto &inside_feedback_state = iteration.gate_reference_states[*feedback_anchor_state];
              const Eigen::Vector3d inside_feedback_ypr =
                Rot3ToYpr(inside_feedback_state.pose.rotation());
              const auto inside_bias_update = inside_bias_adapter.ObserveInsideResidual(
                *feedback_anchor_state,
                record.corrected_time_s,
                consistency_record.local_postfit_residual_u_m,
                consistency_base_sigma_m.z(),
                consistency_record.vertical_gate_threshold_m,
                inside_feedback_ypr.y(),
                inside_feedback_ypr.z());
              if (inside_bias_update.has_value() &&
                  inside_bias_update->anchor_state_index < iteration.gate_reference_states.size()) {
                auto candidate_reference_states = iteration.gate_reference_states;
                gtsam::Values candidate_initial_values = sequential_initial_values;
                const std::size_t bias_anchor_state_index = inside_bias_update->anchor_state_index;
                const auto corrected_bias_anchor_state = ApplyInsideLowFrequencyStateCorrection(
                  candidate_reference_states[bias_anchor_state_index],
                  *inside_bias_update);
                candidate_reference_states[bias_anchor_state_index] = corrected_bias_anchor_state;
                UpsertReferenceStateInitialValues(
                  bias_anchor_state_index,
                  corrected_bias_anchor_state,
                  &candidate_initial_values);

                std::size_t candidate_valid_until_index = bias_anchor_state_index;
                const std::size_t required_prefit_index = ResolvePrefitReferenceRightIndex(sync_result);
                EnsureReferenceStatesValidUntil(
                  state_timestamps,
                  dataset.imu_samples,
                  imu_params,
                  &candidate_reference_states,
                  &candidate_valid_until_index,
                  required_prefit_index,
                  &candidate_initial_values);

                const ReferenceNodeState corrected_prefit_reference_state =
                  sync_result.status == StateMeasSyncStatus::kInterpolated
                    ? InterpolateReferenceState(candidate_reference_states, record.corrected_time_s)
                    : candidate_reference_states[required_prefit_index];
                const Eigen::Vector3d corrected_residual_enu_m =
                  ComputePositionResidualEnu(corrected_prefit_reference_state.pose, record.measurement_enu_m);
                const double corrected_vertical_nis =
                  ComputeVerticalNis(corrected_residual_enu_m.z(), consistency_base_sigma_m.z());
                const bool correction_keeps_inside = corrected_vertical_nis <= vertical_gate_nis_threshold;
                if (correction_keeps_inside) {
                  inside_bias_adapter.AcceptUpdate(*inside_bias_update);
                  iteration.gate_reference_states = std::move(candidate_reference_states);
                  sequential_initial_values = std::move(candidate_initial_values);
                  sequential_reference_valid_until_index = candidate_valid_until_index;
                  inside_reference_states = iteration.gate_reference_states;
                  history_reobserve_begin_state_index =
                    history_reobserve_begin_state_index.has_value()
                      ? std::min(*history_reobserve_begin_state_index, bias_anchor_state_index)
                      : std::optional<std::size_t>(bias_anchor_state_index);
                  const std::size_t inside_bias_reobserve_begin_state_index = bias_anchor_state_index + 1U;
                  inside_bias_adapter_reobserve_begin_state_index =
                    inside_bias_adapter_reobserve_begin_state_index.has_value()
                      ? std::min(
                          *inside_bias_adapter_reobserve_begin_state_index,
                          inside_bias_reobserve_begin_state_index)
                      : std::optional<std::size_t>(inside_bias_reobserve_begin_state_index);
                  consistency_record.inside_bias_delta_roll_applied_rad =
                    inside_bias_update->delta_roll_rad;
                  consistency_record.inside_bias_delta_pitch_applied_rad =
                    inside_bias_update->delta_pitch_rad;
                  consistency_record.inside_bias_delta_baz_applied_mps2 =
                    inside_bias_update->delta_baz_mps2;
                  consistency_record.inside_bias_equivalent_acc_mps2 =
                    inside_bias_update->equivalent_acc_mps2;
                  consistency_record.inside_bias_residual_delta_m =
                    inside_bias_update->residual_delta_m;
                  consistency_record.inside_bias_window_dt_s = inside_bias_update->window_dt_s;
                  consistency_record.inside_bias_anchor_state_index =
                    static_cast<long long>(inside_bias_update->anchor_state_index);
                  consistency_record.inside_bias_observation_count =
                    inside_bias_update->observation_count;
                  consistency_record.local_postfit_residual_u_m = corrected_residual_enu_m.z();
                  consistency_record.prefit_residual_u_after_local_recovery_m =
                    corrected_residual_enu_m.z();
                  consistency_record.vertical_gate_inside = 1.0;
                }
              }
            }
          } else {
            if (sync_result.status == StateMeasSyncStatus::kInterpolated) {
              prefit_pose = InterpolateReferenceState(reference_node_states, record.corrected_time_s).pose;
            } else if (IsSynchronizedStatus(sync_result.status)) {
              const std::size_t prefit_state_index =
                sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i : sync_result.key_index_j;
              prefit_pose = reference_node_states[prefit_state_index].pose;
            }

            const Eigen::Vector3d prefit_residual_enu_m =
              prefit_pose.has_value()
                ? ComputePositionResidualEnu(*prefit_pose, record.measurement_enu_m)
                : Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
            consistency_record.prefit_residual_enu_m = prefit_residual_enu_m;
            if (prefit_pose.has_value()) {
              const double full_prefit_nis = ComputeNis(prefit_residual_enu_m, consistency_base_sigma_m);
              consistency_record.prefit_nis = full_prefit_nis;
              consistency_record.covariance_scale_e = ComputeConsistencyScale(
                config_.gnss_consistency_gate_mode,
                full_prefit_nis,
                gnss_nis_threshold,
                config_.gnss_consistency_relaxed_threshold_ratio,
                config_.gnss_consistency_max_scale_horizontal);
              consistency_record.covariance_scale_n = ComputeConsistencyScale(
                config_.gnss_consistency_gate_mode,
                full_prefit_nis,
                gnss_nis_threshold,
                config_.gnss_consistency_relaxed_threshold_ratio,
                config_.gnss_consistency_max_scale_horizontal);
              consistency_record.covariance_scale_u = ComputeConsistencyScale(
                config_.gnss_consistency_gate_mode,
                full_prefit_nis,
                gnss_nis_threshold,
                config_.gnss_consistency_relaxed_threshold_ratio,
                config_.gnss_consistency_max_scale_up);
              consistency_record.covariance_scale =
                (consistency_record.covariance_scale_e + consistency_record.covariance_scale_n +
                 consistency_record.covariance_scale_u) /
                3.0;
              sigma_m.x() *= consistency_record.covariance_scale_e;
              sigma_m.y() *= consistency_record.covariance_scale_n;
              sigma_m.z() *= consistency_record.covariance_scale_u;
            }
            consistency_record.effective_sigma_u_m = sigma_m.z();
            consistency_record.vertical_direct_position_factor_used = true;
            consistency_record.vertical_sigma_u_used_m = sigma_m.z();
            consistency_record.vertical_gate_inside = std::numeric_limits<double>::quiet_NaN();
          }

          if (iteration.failed) {
            consistency_record.factor_used = false;
            iteration.gnss_factor_records.push_back(record);
            if (collect_gnss_consistency) {
              iteration.gnss_consistency_records.push_back(consistency_record);
            }
            break;
          }

          const bool use_horizontal_only_position_factor =
            use_vertical_rtk_1d_nis_gate && !apply_initial_vertical_anchor;
          const auto noise_model = MakeGnssNoiseModel(config_, sigma_m);
          const Eigen::Vector2d horizontal_sigma_m(sigma_m.x(), sigma_m.y());
          const auto horizontal_noise_model =
            use_horizontal_only_position_factor
              ? MakeHorizontalGnssNoiseModel(config_, horizontal_sigma_m)
              : gtsam::SharedNoiseModel{};
          const auto vertical_inside_kinematic_noise_model =
            use_horizontal_only_position_factor && feedback_anchor_state.has_value() &&
                *feedback_anchor_state < iteration.gate_reference_states.size()
              ? gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(
                  config_.vertical_rtk_feedback_sigma_attitude_rad,
                  config_.vertical_rtk_feedback_sigma_attitude_rad,
                  0.05,
                  std::max(ResolveVerticalAccBiasSigmaMps2(config_), 1e-12)))
              : gtsam::SharedNoiseModel{};

          switch (sync_result.status) {
            case StateMeasSyncStatus::kSynchronizedI:
            case StateMeasSyncStatus::kSynchronizedJ: {
              const std::size_t sync_state_index =
                sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i : sync_result.key_index_j;
              if (use_horizontal_only_position_factor) {
                graph_with_gnss.add(factor::HorizontalPositionFactor(
                  X(sync_state_index),
                  gtsam::Point2(record.measurement_enu_m.x(), record.measurement_enu_m.y()),
                  horizontal_noise_model));
                if (!inside_reference_states.empty() && sync_state_index < inside_reference_states.size()) {
                  const auto &reference_state = inside_reference_states[sync_state_index];
                  graph_with_gnss.add(factor::VerticalInsideKinematicFactor(
                    X(sync_state_index),
                    V(sync_state_index),
                    B(sync_state_index),
                    reference_state.pose,
                    reference_state.velocity,
                    reference_state.bias,
                    vertical_inside_kinematic_noise_model));
                }
              } else {
                graph_with_gnss.add(gtsam::GPSFactor(
                  X(sync_state_index),
                  gtsam::Point3(
                    record.measurement_enu_m.x(),
                    record.measurement_enu_m.y(),
                    record.measurement_enu_m.z()),
                  noise_model));
              }
              ++iteration.run_summary.gnss_factor_count;
              ++iteration.run_summary.gnss_synced_factor_count;
              record.factor_used = true;
              record.synchronized_state_index = sync_state_index;
              record.synchronized_trajectory_row_index = graph_state_to_trajectory_row(sync_state_index);
              if (sync_state_index < trajectory_row_index_by_state.size() &&
                  trajectory_row_index_by_state[sync_state_index].has_value()) {
                auto &row = iteration.trajectory[*trajectory_row_index_by_state[sync_state_index]];
                row.gnss_factor_used = true;
                if (row.gnss_fix_type == GnssFixType::kNoSolution) {
                  row.gnss_fix_type = gnss_sample.fix_type();
                }
              }
              break;
            }
            case StateMeasSyncStatus::kInterpolated: {
              if (!config_.enable_gp_interpolated_gnss || !sync_result.state_j_exists()) {
                ++iteration.run_summary.gnss_dropped_count;
                break;
              }
              gp::GPWNOJInterpolator interpolator = base_interpolator;
              interpolator.Recalculate(
                sync_result.timestamp_j_s - sync_result.timestamp_i_s,
                sync_result.duration_from_state_i_s);
              if (use_horizontal_only_position_factor) {
                graph_with_gnss.add(factor::GPInterpolatedHorizontalPositionFactor(
                  X(sync_result.key_index_i),
                  V(sync_result.key_index_i),
                  W(sync_result.key_index_i),
                  X(sync_result.key_index_j),
                  V(sync_result.key_index_j),
                  W(sync_result.key_index_j),
                  gtsam::Point2(record.measurement_enu_m.x(), record.measurement_enu_m.y()),
                  gtsam::Vector3::Zero(),
                  horizontal_noise_model,
                  interpolator));
                if (!inside_reference_states.empty() &&
                    sync_result.key_index_j < inside_reference_states.size()) {
                  const auto &reference_state = inside_reference_states[sync_result.key_index_j];
                  graph_with_gnss.add(factor::VerticalInsideKinematicFactor(
                    X(sync_result.key_index_j),
                    V(sync_result.key_index_j),
                    B(sync_result.key_index_j),
                    reference_state.pose,
                    reference_state.velocity,
                    reference_state.bias,
                    vertical_inside_kinematic_noise_model));
                }
              } else {
                graph_with_gnss.add(factor::GPInterpolatedGPSFactor(
                  X(sync_result.key_index_i),
                  V(sync_result.key_index_i),
                  W(sync_result.key_index_i),
                  X(sync_result.key_index_j),
                  V(sync_result.key_index_j),
                  W(sync_result.key_index_j),
                  gtsam::Point3(
                    record.measurement_enu_m.x(),
                    record.measurement_enu_m.y(),
                    record.measurement_enu_m.z()),
                  gtsam::Vector3::Zero(),
                  noise_model,
                  interpolator));
              }
              ++iteration.run_summary.gnss_factor_count;
              ++iteration.run_summary.gnss_interpolated_factor_count;
              record.factor_used = true;
              if (sync_result.key_index_i < trajectory_row_index_by_state.size() &&
                  trajectory_row_index_by_state[sync_result.key_index_i].has_value()) {
                auto &row_i = iteration.trajectory[*trajectory_row_index_by_state[sync_result.key_index_i]];
                row_i.gnss_factor_used = true;
                if (row_i.gnss_fix_type == GnssFixType::kNoSolution) {
                  row_i.gnss_fix_type = gnss_sample.fix_type();
                }
              }
              if (sync_result.key_index_j < trajectory_row_index_by_state.size() &&
                  trajectory_row_index_by_state[sync_result.key_index_j].has_value()) {
                auto &row_j = iteration.trajectory[*trajectory_row_index_by_state[sync_result.key_index_j]];
                row_j.gnss_factor_used = true;
                if (row_j.gnss_fix_type == GnssFixType::kNoSolution) {
                  row_j.gnss_fix_type = gnss_sample.fix_type();
                }
              }
              break;
            }
            case StateMeasSyncStatus::kCached:
              ++iteration.run_summary.gnss_cached_count;
              break;
            case StateMeasSyncStatus::kDropped:
            default:
              ++iteration.run_summary.gnss_dropped_count;
              break;
          }

          if (use_vertical_rtk_1d_nis_gate && record.factor_used && feedback_anchor_state.has_value()) {
            if (apply_initial_vertical_anchor) {
              const std::size_t anchor_state_index = *feedback_anchor_state;
              if (anchor_state_index < iteration.gate_reference_states.size()) {
                auto anchored_state = iteration.gate_reference_states[anchor_state_index];
                anchored_state.pose = gtsam::Pose3(
                  anchored_state.pose.rotation(),
                  gtsam::Point3(
                    anchored_state.pose.translation().x(),
                    anchored_state.pose.translation().y(),
                    record.measurement_enu_m.z()));
                iteration.gate_reference_states[anchor_state_index] = anchored_state;
                if (anchor_state_index < inside_reference_states.size()) {
                  inside_reference_states[anchor_state_index] = anchored_state;
                }
                UpsertReferenceStateInitialValues(anchor_state_index, anchored_state, &sequential_initial_values);
                sequential_reference_valid_until_index = anchor_state_index;
              }
              const std::size_t observe_begin = graph_timeline.dynamic_start_index;
              if (observe_begin <= anchor_state_index) {
                nhc_jump_detector.ObserveConfirmedWindow(
                  iteration.gate_reference_states,
                  observe_begin,
                  anchor_state_index);
                iteration_sparse_jump_planner.ObserveConfirmedWindow(
                  iteration.gate_reference_states,
                  vertical_vz_reference_by_state,
                  observe_begin,
                  anchor_state_index);
              }
              confirmed_inside_state_index = anchor_state_index;
              vertical_initial_anchor_applied = true;
            } else if (inside_gate) {
              if (consistency_record.vertical_gate_inside >= 0.5) {
                ++iteration.run_summary.vertical_gate_inside_count;
              } else {
                ++iteration.run_summary.vertical_gate_outside_count;
              }
              const std::size_t nominal_observe_begin =
                confirmed_inside_state_index.has_value() ? *confirmed_inside_state_index + 1U : *feedback_anchor_state;
              std::size_t observe_begin = nominal_observe_begin;
              if (history_reobserve_begin_state_index.has_value()) {
                const std::size_t reobserve_begin =
                  std::min(*history_reobserve_begin_state_index, *feedback_anchor_state);
                nhc_jump_detector.RewindFromStateIndex(reobserve_begin);
                iteration_sparse_jump_planner.RewindFromStateIndex(reobserve_begin);
                if (inside_bias_adapter_reobserve_begin_state_index.has_value() &&
                    *inside_bias_adapter_reobserve_begin_state_index <= *feedback_anchor_state) {
                  inside_bias_adapter.RewindFromStateIndex(
                    std::min(*inside_bias_adapter_reobserve_begin_state_index, *feedback_anchor_state));
                }
                observe_begin = std::min(observe_begin, reobserve_begin);
                history_reobserve_begin_state_index.reset();
                inside_bias_adapter_reobserve_begin_state_index.reset();
              }
              if (observe_begin <= *feedback_anchor_state) {
                nhc_jump_detector.ObserveConfirmedWindow(
                  iteration.gate_reference_states,
                  observe_begin,
                  *feedback_anchor_state);
                iteration_sparse_jump_planner.ObserveConfirmedWindow(
                  iteration.gate_reference_states,
                  vertical_vz_reference_by_state,
                  observe_begin,
                  *feedback_anchor_state);
              }
              confirmed_inside_state_index = *feedback_anchor_state;
            }
          }

          consistency_record.factor_used = record.factor_used;
          iteration.gnss_factor_records.push_back(record);
          if (collect_gnss_consistency) {
            iteration.gnss_consistency_records.push_back(consistency_record);
          }
        }
      }

      gtsam::LevenbergMarquardtOptimizer optimizer(graph_with_gnss, sequential_initial_values, optimizer_params);
      iteration.run_summary.initial_error = optimizer.error();
      iteration.optimized_values = optimizer.optimize();
      iteration.run_summary.final_error = graph_with_gnss.error(iteration.optimized_values);
      PopulateGnssPostfitResiduals(
        iteration.optimized_values,
        base_interpolator,
        trajectory_row_index_by_state,
        iteration.gnss_factor_records,
        collect_gnss_consistency ? &iteration.gnss_consistency_records : nullptr,
        &iteration.trajectory);
      return iteration;
    };

  VerticalGateIterationResult final_iteration_result =
    run_vertical_gate_iteration(gate_seed_reference_states, gate_seed_values);

  run_result.run_summary = final_iteration_result.run_summary;
  run_result.gnss_factor_records = std::move(final_iteration_result.gnss_factor_records);
  run_result.gnss_consistency_records = std::move(final_iteration_result.gnss_consistency_records);
  run_result.vertical_local_recovery_iterations =
    std::move(final_iteration_result.vertical_local_recovery_iterations);
  run_result.trajectory = std::move(final_iteration_result.trajectory);
  if (collect_reference_states) {
    run_result.reference_node_trajectory.clear();
    run_result.reference_node_trajectory.reserve(final_iteration_result.gate_reference_states.size());
    for (const auto &reference_state : final_iteration_result.gate_reference_states) {
      run_result.reference_node_trajectory.push_back(MakeReferenceNodeRow(reference_state));
    }
  }
  const gtsam::Values optimized_values = final_iteration_result.optimized_values;
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
    const gp::GPWNOJInterpolator base_interpolator(
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance)));

    for (std::size_t record_index = 0; record_index < run_result.gnss_factor_records.size(); ++record_index) {
      auto &record = run_result.gnss_factor_records[record_index];
      GnssConsistencyRecord *consistency_record =
        collect_gnss_consistency && record_index < run_result.gnss_consistency_records.size()
          ? &run_result.gnss_consistency_records[record_index]
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
        if (record.synchronized_state_index < trajectory_row_index_by_state.size() &&
            trajectory_row_index_by_state[record.synchronized_state_index].has_value()) {
          auto &row = run_result.trajectory[*trajectory_row_index_by_state[record.synchronized_state_index]];
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
        if (record.state_index_i < trajectory_row_index_by_state.size() &&
            trajectory_row_index_by_state[record.state_index_i].has_value()) {
          auto &row_i = run_result.trajectory[*trajectory_row_index_by_state[record.state_index_i]];
          if (!std::isfinite(row_i.gnss_residual_m)) {
            row_i.gnss_residual_m = record.residual_m;
          }
        }
        if (record.state_index_j < trajectory_row_index_by_state.size() &&
            trajectory_row_index_by_state[record.state_index_j].has_value()) {
          auto &row_j = run_result.trajectory[*trajectory_row_index_by_state[record.state_index_j]];
          if (!std::isfinite(row_j.gnss_residual_m)) {
            row_j.gnss_residual_m = record.residual_m;
          }
        }
      }
    }

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

      if (!final_iteration_result.gate_reference_states.empty()) {
        const ReferenceNodeState reference_state =
          InterpolateReferenceState(final_iteration_result.gate_reference_states, record.corrected_time_s);
        const Eigen::Vector3d reference_ypr = Rot3ToYpr(reference_state.pose.rotation());
        row.reference_available = true;
        row.reference_up_m = reference_state.pose.translation().z();
        row.reference_vz_mps = reference_state.velocity.z();
        row.reference_pitch_rad = reference_ypr.y();
        row.reference_roll_rad = reference_ypr.z();
        row.reference_baz_mps2 = reference_state.bias.accelerometer().z();
        row.delta_up_m = row.optimized_up_m - row.reference_up_m;
        row.delta_vz_mps = row.optimized_vz_mps - row.reference_vz_mps;
        row.delta_pitch_rad = WrapAngleRad(row.optimized_pitch_rad - row.reference_pitch_rad);
        row.delta_roll_rad = WrapAngleRad(row.optimized_roll_rad - row.reference_roll_rad);
        row.delta_baz_mps2 = row.optimized_baz_mps2 - row.reference_baz_mps2;
      }

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

  if (final_iteration_result.failed) {
    throw OfflineRunFailure(final_iteration_result.failure_message, std::move(run_result));
  }

  return run_result;
}

}  // namespace offline_lc_minimal
