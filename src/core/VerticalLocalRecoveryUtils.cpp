#include "offline_lc_minimal/core/VerticalLocalRecoveryUtils.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>

namespace offline_lc_minimal {

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimeEpsilonS = 1e-9;
constexpr double kBodyZSeedTailVelocityMicroFeedbackLimitMps = 0.08;
constexpr double kBodyZSeedTailPositionMicroFeedbackLimitM = 0.02;

Eigen::Vector3d Rot3ToYprForVerticalRecovery(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

double SmoothStep01(const double alpha) {
  const double bounded_alpha = std::clamp(alpha, 0.0, 1.0);
  return bounded_alpha * bounded_alpha * (3.0 - 2.0 * bounded_alpha);
}

}  // namespace

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

std::optional<double> EstimateIntervalVelocityFeedbackDelta(
  const OfflineRunnerConfig &config,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalHoldWindowSpec> &interval_specs,
  const double active_window_end_time_s) {
  if (interval_specs.size() <
      static_cast<std::size_t>(std::max(config.vertical_jump_future_trend_min_fix_count, 1))) {
    return std::nullopt;
  }

  std::vector<double> interval_times_s;
  std::vector<double> interval_residuals_m;
  interval_times_s.reserve(interval_specs.size());
  interval_residuals_m.reserve(interval_specs.size());
  double max_abs_interval_residual_m = 0.0;
  for (const auto &interval_spec : interval_specs) {
    const ReferenceNodeState interval_state =
      ResolveReferenceStateForHoldWindowSpec(reference_states, interval_spec);
    const Eigen::Vector3d residual_enu_m =
      ComputePositionResidualEnu(interval_state.pose, interval_spec.measurement_enu_m);
    const double tau_s = interval_spec.corrected_time_s - active_window_end_time_s;
    if (!std::isfinite(residual_enu_m.z()) || tau_s <= 0.1) {
      continue;
    }
    max_abs_interval_residual_m =
      std::max(max_abs_interval_residual_m, std::abs(residual_enu_m.z()));
    interval_times_s.push_back(tau_s);
    interval_residuals_m.push_back(residual_enu_m.z());
  }
  if (interval_residuals_m.size() <
      static_cast<std::size_t>(std::max(config.vertical_jump_future_trend_min_fix_count, 1))) {
    return std::nullopt;
  }

  const double interval_duration_s =
    interval_times_s.back() - interval_times_s.front();
  const auto interval_trend = EvaluateVerticalFutureTrend(
    reference_states,
    interval_specs,
    config.vertical_jump_future_trend_min_fix_count,
    0.0,
    1.0);
  if (interval_duration_s < config.vertical_interval_feedback_min_duration_s ||
      !interval_trend.valid ||
      !std::isfinite(interval_trend.residual_slope_mps)) {
    return std::nullopt;
  }

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
  const double residual_mad_sigma_m =
    1.4826 * MedianFinite(abs_mad_residuals_m);
  const double residual_noise_m =
    std::max(
      {residual_std_m,
       residual_mad_sigma_m,
       config.vertical_interval_feedback_noise_floor_m});
  const double residual_trend_signal_m =
    std::abs(interval_trend.residual_slope_mps) * interval_duration_s;
  const double residual_trend_snr =
    residual_noise_m > 1e-12 ? residual_trend_signal_m / residual_noise_m
                              : std::numeric_limits<double>::infinity();
  if (residual_trend_snr < config.vertical_interval_feedback_snr_threshold ||
      std::abs(interval_trend.residual_slope_mps) <
        config.vertical_interval_feedback_min_slope_mps ||
      residual_trend_signal_m < config.vertical_interval_feedback_min_drift_m ||
      max_abs_interval_residual_m < config.vertical_interval_feedback_min_residual_m) {
    return std::nullopt;
  }

  return std::clamp(
    -config.vertical_interval_feedback_gain * interval_trend.residual_slope_mps,
    -config.vertical_interval_feedback_max_delta_vz_mps,
    config.vertical_interval_feedback_max_delta_vz_mps);
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
  const Eigen::Vector3d anchor_ypr = Rot3ToYprForVerticalRecovery(anchor_state.pose.rotation());
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
  const double velocity_feedback_delta_mps,
  const double stored_tail_velocity_target_mps,
  const bool allow_initial_velocity_feedback) {
  std::vector<double> targets;
  if (window_candidate.end_state_index >= reference_states.size()) {
    targets.push_back(std::numeric_limits<double>::quiet_NaN());
    return targets;
  }

  const bool velocity_feedback_allowed =
    velocity_feedback_requested_for_window &&
    (velocity_already_corrected || allow_initial_velocity_feedback);
  if (!velocity_feedback_allowed) {
    PushUniqueCandidateValue(
      &targets,
      velocity_already_corrected
        ? (std::isfinite(stored_tail_velocity_target_mps)
             ? stored_tail_velocity_target_mps
             : reference_states[window_candidate.end_state_index].velocity.z())
        : std::numeric_limits<double>::quiet_NaN());
    return targets;
  }

  const double feedback_base_vz_mps =
    std::isfinite(stored_tail_velocity_target_mps)
      ? stored_tail_velocity_target_mps
      : (velocity_already_corrected
           ? reference_states[window_candidate.end_state_index].velocity.z()
           : reference_states[window_candidate.start_state_index - 1U].velocity.z());
  const double previous_vz_mps =
    window_candidate.start_state_index > 0U
      ? reference_states[window_candidate.start_state_index - 1U].velocity.z()
      : reference_states[window_candidate.end_state_index].velocity.z();
  const double bounded_feedback_delta_mps = std::clamp(
    velocity_feedback_delta_mps,
    -kBodyZSeedTailVelocityMicroFeedbackLimitMps,
    kBodyZSeedTailVelocityMicroFeedbackLimitMps);
  const double bounded_feedback_target_mps = std::clamp(
    feedback_base_vz_mps + bounded_feedback_delta_mps,
    previous_vz_mps - kBodyZSeedTailVelocityMicroFeedbackLimitMps,
    previous_vz_mps + kBodyZSeedTailVelocityMicroFeedbackLimitMps);
  PushUniqueCandidateValue(&targets, bounded_feedback_target_mps);
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

std::optional<VerticalVelocityWindowCorrection> BuildVerticalVelocityWindowCorrection(
  const OfflineRunnerConfig &config,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const SparseVerticalJumpWindowCandidate &window_candidate,
  const std::size_t segment_end_state_index,
  const double tail_delta_scale,
  const double forced_tail_delta_mps,
  const double forced_tail_delta_up_m) {
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
    target_tail_vz_mps =
      body_z_seed_candidate
        ? std::clamp(
            forced_tail_delta_mps,
            previous_vz_mps - kBodyZSeedTailVelocityMicroFeedbackLimitMps,
            previous_vz_mps + kBodyZSeedTailVelocityMicroFeedbackLimitMps)
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
      body_z_seed_candidate
        ? continuity_tail_up_m +
            std::clamp(
              forced_tail_delta_up_m,
              -kBodyZSeedTailPositionMicroFeedbackLimitM,
              kBodyZSeedTailPositionMicroFeedbackLimitM)
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
      const double tail_position_offset_m =
        SmoothStep01(alpha) * decoupled_tail_up_offset_m;
      correction.corrected_up_m.push_back(
        integrated_window_up_m[window_offset] + tail_position_offset_m);
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

std::optional<SparseVerticalJumpWindowCandidate> FindCoveringBodyZSeedWindowCandidate(
  const std::vector<BodyZJumpWindowCandidate> &body_z_windows,
  const std::vector<double> &state_timestamps,
  const std::vector<ReferenceNodeState> &reference_states,
  const std::vector<VerticalVzReferenceSample> &vertical_vz_reference,
  const std::size_t dynamic_start_index,
  const std::size_t feedback_anchor_state_index,
  const double corrected_time_s,
  const double sync_upper_bound_s) {
  std::optional<SparseVerticalJumpWindowCandidate> covering_window_candidate;
  const std::unordered_set<std::size_t> no_nhc_support;
  for (const auto &body_z_window : body_z_windows) {
    if (body_z_window.start_state_index < dynamic_start_index + 1U ||
        body_z_window.end_state_index >= reference_states.size()) {
      continue;
    }
    const bool feedback_state_inside_window =
      body_z_window.start_state_index <= feedback_anchor_state_index &&
      feedback_anchor_state_index <= body_z_window.end_state_index;
    const bool gnss_time_inside_window =
      corrected_time_s >= body_z_window.start_time_s - kTimeEpsilonS &&
      corrected_time_s <=
        body_z_window.end_time_s + std::max(sync_upper_bound_s, kTimeEpsilonS);
    if (!feedback_state_inside_window && !gnss_time_inside_window) {
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
    if (!covering_window_candidate.has_value() ||
        window_candidate->start_state_index > covering_window_candidate->start_state_index) {
      covering_window_candidate = *window_candidate;
    }
  }
  return covering_window_candidate;
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

}  // namespace offline_lc_minimal
