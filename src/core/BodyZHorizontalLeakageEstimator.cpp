#include "offline_lc_minimal/core/BodyZHorizontalLeakageEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr int kRobustIterationCount = 8;

struct LeakageSample {
  double v_body_x_mps = 0.0;
  double v_body_y_mps = 0.0;
  double v_body_z_mps = 0.0;
};

bool TimeInsideAnyWindow(
  const double time_s,
  const std::vector<BodyZJumpConstraintWindow> &windows,
  const double guard_s) {
  for (const auto &window : windows) {
    if (time_s + kTimeEpsilonS >= window.start_time_s - guard_s &&
        time_s <= window.end_time_s + guard_s + kTimeEpsilonS) {
      return true;
    }
  }
  return false;
}

bool HasReferenceStates(const BodyZHorizontalLeakageEstimateRequest &request) {
  return request.reference_states != nullptr &&
         request.state_timestamps != nullptr &&
         request.reference_states->size() == request.state_timestamps->size();
}

std::optional<LeakageSample> BuildSample(
  const BodyZHorizontalLeakageEstimateRequest &request,
  const std::size_t state_index,
  const bool use_reference_states) {
  try {
    gtsam::Pose3 pose;
    gtsam::Vector3 velocity;
    if (use_reference_states) {
      const auto &reference_state = (*request.reference_states)[state_index];
      pose = reference_state.pose;
      velocity = reference_state.velocity;
    } else {
      pose = request.initial_values->at<gtsam::Pose3>(symbol::X(state_index));
      velocity = request.initial_values->at<gtsam::Vector3>(symbol::V(state_index));
    }
    if (!velocity.allFinite()) {
      return std::nullopt;
    }
    const factor::BodyFrameAxesNav axes = factor::BodyFrameAxesNavFromPose(pose);
    LeakageSample sample;
    sample.v_body_x_mps = factor::BodyXVelocityMps(axes, velocity);
    sample.v_body_y_mps = factor::BodyYVelocityMps(axes, velocity);
    sample.v_body_z_mps = factor::BodyZRawVelocityMps(axes, velocity);
    if (!std::isfinite(sample.v_body_x_mps) ||
        !std::isfinite(sample.v_body_y_mps) ||
        !std::isfinite(sample.v_body_z_mps)) {
      return std::nullopt;
    }
    return sample;
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

bool SolveWeightedLeastSquares(
  const std::vector<LeakageSample> &samples,
  const std::vector<double> &weights,
  double &leak_x_rad,
  double &leak_y_rad) {
  double a00 = 0.0;
  double a01 = 0.0;
  double a11 = 0.0;
  double b0 = 0.0;
  double b1 = 0.0;
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const double weight = weights[index];
    const double x = samples[index].v_body_x_mps;
    const double y = samples[index].v_body_y_mps;
    const double z = samples[index].v_body_z_mps;
    a00 += weight * x * x;
    a01 += weight * x * y;
    a11 += weight * y * y;
    b0 += weight * x * z;
    b1 += weight * y * z;
  }
  const double determinant = a00 * a11 - a01 * a01;
  if (!std::isfinite(determinant) || std::abs(determinant) < 1.0e-12) {
    return false;
  }
  leak_x_rad = (b0 * a11 - b1 * a01) / determinant;
  leak_y_rad = (a00 * b1 - a01 * b0) / determinant;
  return std::isfinite(leak_x_rad) && std::isfinite(leak_y_rad);
}

void ClampLeakageCoefficients(
  const double max_abs_coeff_rad,
  double &leak_x_rad,
  double &leak_y_rad) {
  leak_x_rad = std::clamp(leak_x_rad, -max_abs_coeff_rad, max_abs_coeff_rad);
  leak_y_rad = std::clamp(leak_y_rad, -max_abs_coeff_rad, max_abs_coeff_rad);
}

std::pair<double, double> ComputeResidualRmsAndMax(
  const std::vector<LeakageSample> &samples,
  const double leak_x_rad,
  const double leak_y_rad) {
  double sum_square = 0.0;
  double max_abs = 0.0;
  for (const auto &sample : samples) {
    const double residual =
      sample.v_body_z_mps -
      leak_x_rad * sample.v_body_x_mps -
      leak_y_rad * sample.v_body_y_mps;
    sum_square += residual * residual;
    max_abs = std::max(max_abs, std::abs(residual));
  }
  const double rms = samples.empty()
    ? std::numeric_limits<double>::quiet_NaN()
    : std::sqrt(sum_square / static_cast<double>(samples.size()));
  return {rms, max_abs};
}

std::vector<double> BuildHuberWeights(
  const std::vector<LeakageSample> &samples,
  const double leak_x_rad,
  const double leak_y_rad,
  const double huber_sigma_mps) {
  std::vector<double> weights(samples.size(), 1.0);
  for (std::size_t index = 0; index < samples.size(); ++index) {
    const double residual =
      samples[index].v_body_z_mps -
      leak_x_rad * samples[index].v_body_x_mps -
      leak_y_rad * samples[index].v_body_y_mps;
    const double abs_residual = std::abs(residual);
    if (abs_residual > huber_sigma_mps) {
      weights[index] = huber_sigma_mps / abs_residual;
    }
  }
  return weights;
}

}  // namespace

BodyZHorizontalLeakageEstimator::BodyZHorizontalLeakageEstimator(
  BodyZHorizontalLeakageEstimateRequest request)
    : request_(std::move(request)) {}

BodyZHorizontalLeakageEstimate BodyZHorizontalLeakageEstimator::Estimate() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.excluded_windows == nullptr || request_.initial_values == nullptr) {
    throw std::runtime_error("BodyZHorizontalLeakageEstimator received an incomplete request");
  }

  BodyZHorizontalLeakageEstimate estimate;
  auto &diagnostic = estimate.diagnostic;
  diagnostic.enabled = request_.config->enable_body_z_nhc_horizontal_leakage_correction;
  diagnostic.min_speed_mps = request_.config->body_z_nhc_horizontal_leakage_min_speed_mps;
  diagnostic.huber_sigma_mps = request_.config->body_z_nhc_horizontal_leakage_huber_sigma_mps;
  diagnostic.max_abs_coeff_rad = request_.config->body_z_nhc_horizontal_leakage_max_abs_coeff_rad;

  if (!request_.config->enable_body_z_nhc_horizontal_leakage_correction) {
    diagnostic.skip_reason = "DISABLED";
    return estimate;
  }
  if (request_.dynamic_start_index >= request_.state_timestamps->size()) {
    diagnostic.skip_reason = "INVALID_DYNAMIC_START";
    return estimate;
  }

  const bool use_reference_states = HasReferenceStates(request_);
  diagnostic.velocity_source = use_reference_states ? "REFERENCE_STATES" : "INITIAL_VALUES";
  std::vector<LeakageSample> samples;
  for (std::size_t state_index = request_.dynamic_start_index;
       state_index < request_.state_timestamps->size();
       ++state_index) {
    ++diagnostic.candidate_sample_count;
    const double time_s = (*request_.state_timestamps)[state_index];
    if (!std::isfinite(time_s) ||
        TimeInsideAnyWindow(
          time_s,
          *request_.excluded_windows,
          request_.config->body_z_nhc_horizontal_leakage_guard_s)) {
      ++diagnostic.skipped_window_count;
      continue;
    }
    const auto sample = BuildSample(request_, state_index, use_reference_states);
    if (!sample.has_value()) {
      ++diagnostic.skipped_invalid_count;
      continue;
    }
    const double planar_body_speed_mps =
      std::hypot(sample->v_body_x_mps, sample->v_body_y_mps);
    if (planar_body_speed_mps < request_.config->body_z_nhc_horizontal_leakage_min_speed_mps) {
      ++diagnostic.skipped_low_speed_count;
      continue;
    }
    samples.push_back(*sample);
  }
  diagnostic.used_sample_count = samples.size();

  if (samples.size() <
      static_cast<std::size_t>(request_.config->body_z_nhc_horizontal_leakage_min_sample_count)) {
    diagnostic.skip_reason = "INSUFFICIENT_SAMPLES";
    return estimate;
  }

  auto raw_metrics = ComputeResidualRmsAndMax(samples, 0.0, 0.0);
  diagnostic.raw_rms_body_z_mps = raw_metrics.first;
  diagnostic.raw_max_abs_body_z_mps = raw_metrics.second;

  double leak_x_rad = 0.0;
  double leak_y_rad = 0.0;
  std::vector<double> weights(samples.size(), 1.0);
  for (int iteration = 0; iteration < kRobustIterationCount; ++iteration) {
    if (!SolveWeightedLeastSquares(samples, weights, leak_x_rad, leak_y_rad)) {
      diagnostic.skip_reason = "SINGULAR_NORMAL_MATRIX";
      return estimate;
    }
    ClampLeakageCoefficients(
      request_.config->body_z_nhc_horizontal_leakage_max_abs_coeff_rad,
      leak_x_rad,
      leak_y_rad);
    weights = BuildHuberWeights(
      samples,
      leak_x_rad,
      leak_y_rad,
      request_.config->body_z_nhc_horizontal_leakage_huber_sigma_mps);
  }

  auto corrected_metrics = ComputeResidualRmsAndMax(samples, leak_x_rad, leak_y_rad);
  diagnostic.corrected_rms_body_z_mps = corrected_metrics.first;
  diagnostic.corrected_max_abs_body_z_mps = corrected_metrics.second;
  diagnostic.leak_x_rad = leak_x_rad;
  diagnostic.leak_y_rad = leak_y_rad;
  diagnostic.estimate_valid = true;
  diagnostic.skip_reason = "ADDED";

  estimate.valid = true;
  estimate.model.leak_x_rad = leak_x_rad;
  estimate.model.leak_y_rad = leak_y_rad;
  return estimate;
}

}  // namespace offline_lc_minimal
