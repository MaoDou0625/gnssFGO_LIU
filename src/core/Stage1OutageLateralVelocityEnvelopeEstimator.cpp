#include "offline_lc_minimal/core/Stage1OutageLateralVelocityEnvelopeEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>

#include "offline_lc_minimal/core/Stage2VelocityReference.h"
#include "offline_lc_minimal/factor/BodyZLeakageCorrectedVelocityFactor.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

double Percentile(std::vector<double> values, const double percentile) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const double clamped_percentile = std::clamp(percentile, 0.0, 1.0);
  const double position = clamped_percentile * static_cast<double>(values.size() - 1U);
  const auto lower_index = static_cast<std::size_t>(std::floor(position));
  const auto upper_index = static_cast<std::size_t>(std::ceil(position));
  if (lower_index == upper_index) {
    return values[lower_index];
  }
  const double alpha = position - static_cast<double>(lower_index);
  return (1.0 - alpha) * values[lower_index] + alpha * values[upper_index];
}

Stage1OutageBodyYEnvelopeRow MakeBaseRow(
  const OfflineRunnerConfig &config,
  const RtkOutageWindowRow &window) {
  Stage1OutageBodyYEnvelopeRow row;
  row.window_index = window.window_index;
  row.outage_start_time_s = window.start_time_s;
  row.outage_end_time_s = window.end_time_s;
  row.pre_window_start_time_s =
    window.start_time_s - config.stage1_outage_body_y_pre_window_s;
  row.pre_window_end_time_s = window.start_time_s;
  row.min_speed_mps = config.stage1_outage_body_y_min_speed_mps;
  row.huber_k = config.stage1_outage_body_y_huber_k;
  return row;
}

}  // namespace

Stage1OutageLateralVelocityEnvelopeEstimator::
  Stage1OutageLateralVelocityEnvelopeEstimator(
    Stage1OutageLateralVelocityEnvelopeEstimateRequest request)
    : request_(std::move(request)) {}

Stage1OutageBodyYEnvelopeReference
Stage1OutageLateralVelocityEnvelopeEstimator::Estimate() const {
  if (request_.config == nullptr || request_.outage_windows == nullptr ||
      request_.trajectory == nullptr) {
    throw std::runtime_error(
      "Stage1OutageLateralVelocityEnvelopeEstimator received an incomplete request");
  }

  Stage1OutageBodyYEnvelopeReference reference;
  if (!request_.config->enable_stage1_outage_body_y_envelope ||
      request_.outage_windows->empty() ||
      request_.trajectory->empty()) {
    return reference;
  }
  reference.reference_states =
    BuildStage2ReferenceStatesFromTrajectory(*request_.trajectory);
  reference.envelopes.reserve(request_.outage_windows->size());

  for (const auto &window : *request_.outage_windows) {
    Stage1OutageBodyYEnvelopeRow row = MakeBaseRow(*request_.config, window);
    if (!std::isfinite(window.start_time_s) ||
        !std::isfinite(window.end_time_s) ||
        window.end_time_s <= window.start_time_s + kTimeEpsilonS) {
      row.skip_reason = "INVALID_OUTAGE_WINDOW";
      reference.envelopes.push_back(row);
      continue;
    }

    std::vector<double> body_y_values;
    for (const auto &trajectory_row : *request_.trajectory) {
      if (!std::isfinite(trajectory_row.time_s) ||
          trajectory_row.time_s + kTimeEpsilonS < row.pre_window_start_time_s ||
          trajectory_row.time_s >= row.pre_window_end_time_s - kTimeEpsilonS) {
        continue;
      }
      ++row.candidate_sample_count;
      const gtsam::Pose3 pose(
        gtsam::Rot3::Ypr(
          trajectory_row.ypr_rad.x(),
          trajectory_row.ypr_rad.y(),
          trajectory_row.ypr_rad.z()),
        gtsam::Point3(
          trajectory_row.enu_position_m.x(),
          trajectory_row.enu_position_m.y(),
          trajectory_row.enu_position_m.z()));
      const auto axes = factor::BodyFrameAxesNavFromPose(pose);
      const gtsam::Vector3 velocity(
        trajectory_row.enu_velocity_mps.x(),
        trajectory_row.enu_velocity_mps.y(),
        trajectory_row.enu_velocity_mps.z());
      if (!velocity.allFinite()) {
        ++row.skipped_invalid_count;
        continue;
      }
      const double body_x_mps = factor::BodyXVelocityMps(axes, velocity);
      const double body_y_mps = factor::BodyYVelocityMps(axes, velocity);
      if (!std::isfinite(body_x_mps) || !std::isfinite(body_y_mps)) {
        ++row.skipped_invalid_count;
        continue;
      }
      if (std::abs(body_x_mps) < request_.config->stage1_outage_body_y_min_speed_mps) {
        ++row.skipped_low_speed_count;
        continue;
      }
      body_y_values.push_back(body_y_mps);
    }
    row.used_sample_count = body_y_values.size();
    if (row.used_sample_count <
        static_cast<std::size_t>(request_.config->stage1_outage_body_y_min_sample_count)) {
      row.skip_reason = "INSUFFICIENT_SAMPLES";
      reference.envelopes.push_back(row);
      continue;
    }

    double sum = 0.0;
    for (const double body_y_mps : body_y_values) {
      sum += body_y_mps;
    }
    row.mean_body_y_mps = sum / static_cast<double>(body_y_values.size());
    double sum_squares = 0.0;
    std::vector<double> abs_centered_values;
    abs_centered_values.reserve(body_y_values.size());
    for (const double body_y_mps : body_y_values) {
      const double centered = body_y_mps - row.mean_body_y_mps;
      sum_squares += centered * centered;
      abs_centered_values.push_back(std::abs(centered));
    }
    row.rmse_body_y_mps =
      std::sqrt(sum_squares / static_cast<double>(body_y_values.size()));
    row.p95_abs_body_y_mps = Percentile(std::move(abs_centered_values), 0.95);
    if (!std::isfinite(row.rmse_body_y_mps) || row.rmse_body_y_mps <= 0.0) {
      row.skip_reason = "INVALID_RMSE";
      reference.envelopes.push_back(row);
      continue;
    }
    row.deadband_mps =
      request_.config->stage1_outage_body_y_deadband_rmse_multiplier *
      row.rmse_body_y_mps;
    row.sigma_mps = std::clamp(
      row.rmse_body_y_mps,
      request_.config->stage1_outage_body_y_min_sigma_mps,
      request_.config->stage1_outage_body_y_max_sigma_mps);
    row.valid = true;
    row.skip_reason = "OK";
    reference.envelopes.push_back(row);
  }

  return reference;
}

}  // namespace offline_lc_minimal
