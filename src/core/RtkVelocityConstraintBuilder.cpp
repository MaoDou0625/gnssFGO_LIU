#include "offline_lc_minimal/core/RtkVelocityConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/RtkHorizontalVelocityFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

struct RtkVelocitySample {
  std::size_t sample_index = 0;
  double raw_time_s = 0.0;
  double time_s = 0.0;
  Eigen::Vector3d position_m = Eigen::Vector3d::Zero();
};

bool IsFiniteVector(const Eigen::Vector3d &value) {
  return value.allFinite();
}

std::vector<RtkVelocitySample> CollectUsableRtkSamples(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::function<bool(const GnssSolutionSample &sample)> &should_use_sample,
  const std::function<double(const GnssSolutionSample &sample)> &corrected_time_s,
  const double start_time_s,
  const double end_time_s) {
  std::vector<RtkVelocitySample> samples;
  samples.reserve(gnss_samples.size());
  for (std::size_t index = 0; index < gnss_samples.size(); ++index) {
    const auto &sample = gnss_samples[index];
    if (sample.fix_type() != GnssFixType::kRtkFix || !sample.has_enu_position ||
        !IsFiniteVector(sample.enu_position_m) || !should_use_sample(sample)) {
      continue;
    }
    const double time_s = corrected_time_s(sample);
    if (!std::isfinite(time_s)) {
      continue;
    }
    if (time_s < start_time_s - kTimeEpsilonS || time_s > end_time_s + kTimeEpsilonS) {
      continue;
    }
    samples.push_back(RtkVelocitySample{index, sample.time_s, time_s, sample.enu_position_m});
  }
  std::sort(samples.begin(), samples.end(), [](const auto &lhs, const auto &rhs) {
    return lhs.time_s < rhs.time_s;
  });
  return samples;
}

std::size_t FindIndexAtOrBefore(const std::vector<RtkVelocitySample> &samples, const double time_s) {
  const auto it = std::upper_bound(
    samples.begin(),
    samples.end(),
    time_s,
    [](const double target_time_s, const RtkVelocitySample &sample) {
      return target_time_s < sample.time_s;
    });
  if (it == samples.begin()) {
    return samples.size();
  }
  return static_cast<std::size_t>(std::distance(samples.begin(), std::prev(it)));
}

std::size_t FindIndexAtOrAfter(const std::vector<RtkVelocitySample> &samples, const double time_s) {
  const auto it = std::lower_bound(
    samples.begin(),
    samples.end(),
    time_s,
    [](const RtkVelocitySample &sample, const double target_time_s) {
      return sample.time_s < target_time_s;
    });
  return static_cast<std::size_t>(std::distance(samples.begin(), it));
}

RtkVelocityDiagnosticRow MakeBaseDiagnostic(const RtkVelocitySample &sample) {
  RtkVelocityDiagnosticRow row;
  row.sample_index = sample.sample_index;
  row.raw_time_s = sample.raw_time_s;
  row.corrected_time_s = sample.time_s;
  row.skip_reason = "UNSET";
  return row;
}

bool PopulateRtkVelocityMeasurement(
  const std::vector<RtkVelocitySample> &samples,
  const std::size_t center_index,
  const double window_s,
  RtkVelocityDiagnosticRow &row) {
  const double half_window_s = 0.5 * window_s;
  const double center_time_s = samples[center_index].time_s;
  const double left_time_s = center_time_s - half_window_s;
  const double right_time_s = center_time_s + half_window_s;
  if (left_time_s < samples.front().time_s || right_time_s > samples.back().time_s) {
    row.skip_reason = "boundary";
    return false;
  }

  const std::size_t left_index = FindIndexAtOrBefore(samples, left_time_s);
  const std::size_t right_index = FindIndexAtOrAfter(samples, right_time_s);
  if (left_index >= samples.size() || right_index >= samples.size() ||
      left_index >= center_index || right_index <= center_index || left_index >= right_index) {
    row.skip_reason = "invalid_window";
    return false;
  }

  const double left_gap_s = left_time_s - samples[left_index].time_s;
  const double right_gap_s = samples[right_index].time_s - right_time_s;
  if (left_gap_s > half_window_s + kTimeEpsilonS ||
      right_gap_s > half_window_s + kTimeEpsilonS) {
    row.skip_reason = "large_gap";
    return false;
  }

  const double dt_s = samples[right_index].time_s - samples[left_index].time_s;
  if (!std::isfinite(dt_s) || dt_s <= 0.0) {
    row.skip_reason = "nonpositive_dt";
    return false;
  }

  const Eigen::Vector3d velocity_mps =
    (samples[right_index].position_m - samples[left_index].position_m) / dt_s;
  if (!velocity_mps.allFinite()) {
    row.skip_reason = "invalid_velocity";
    return false;
  }
  row.window_dt_s = dt_s;
  row.rtk_velocity_mps = velocity_mps;
  return true;
}

std::size_t SynchronizedStateIndex(const StateMeasSyncResult &sync) {
  return sync.status == StateMeasSyncStatus::kSynchronizedI ? sync.key_index_i : sync.key_index_j;
}

void AccumulateOptimizedSummary(
  const std::vector<RtkVelocityDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  double horizontal_residual_sq_sum = 0.0;
  double horizontal_residual_max = 0.0;
  double body_y_sq_sum = 0.0;
  double body_y_max_abs = 0.0;
  std::size_t residual_count = 0;
  std::size_t body_y_count = 0;
  for (const auto &row : diagnostics) {
    if (row.factor_added && std::isfinite(row.horizontal_residual_mps)) {
      horizontal_residual_sq_sum += row.horizontal_residual_mps * row.horizontal_residual_mps;
      horizontal_residual_max = std::max(horizontal_residual_max, row.horizontal_residual_mps);
      ++residual_count;
    }
    if (row.factor_added && std::isfinite(row.rtk_body_y_mps)) {
      body_y_sq_sum += row.rtk_body_y_mps * row.rtk_body_y_mps;
      body_y_max_abs = std::max(body_y_max_abs, std::abs(row.rtk_body_y_mps));
      ++body_y_count;
    }
  }
  if (residual_count > 0U) {
    run_summary.rtk_velocity_horizontal_residual_rms_mps =
      std::sqrt(horizontal_residual_sq_sum / static_cast<double>(residual_count));
    run_summary.rtk_velocity_horizontal_residual_max_mps = horizontal_residual_max;
  }
  if (body_y_count > 0U) {
    run_summary.rtk_velocity_body_y_rms_mps =
      std::sqrt(body_y_sq_sum / static_cast<double>(body_y_count));
    run_summary.rtk_velocity_body_y_max_abs_mps = body_y_max_abs;
  }
}

}  // namespace

RtkVelocityConstraintBuilder::RtkVelocityConstraintBuilder(
  RtkVelocityConstraintBuildRequest request)
    : request_(std::move(request)) {}

void RtkVelocityConstraintBuilder::Validate() const {
  if (request_.config == nullptr || request_.gnss_samples == nullptr ||
      request_.state_timestamps == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr ||
      !request_.should_use_sample || !request_.corrected_time_s ||
      !request_.find_state_for_time_s) {
    throw std::runtime_error("RtkVelocityConstraintBuilder received an incomplete request");
  }
  if (request_.state_timestamps->empty()) {
    throw std::runtime_error("RtkVelocityConstraintBuilder received an empty state timeline");
  }
  if (request_.dynamic_start_index >= request_.state_timestamps->size()) {
    throw std::runtime_error("RtkVelocityConstraintBuilder received an invalid dynamic start index");
  }
}

void RtkVelocityConstraintBuilder::Build() const {
  Validate();
  request_.run_summary->rtk_velocity_constraint_enabled =
    request_.config->enable_rtk_velocity_constraint;
  if (!request_.config->enable_rtk_velocity_constraint) {
    return;
  }

  const double half_velocity_window_s = 0.5 * request_.config->rtk_velocity_window_s;
  const double center_start_time_s = (*request_.state_timestamps)[request_.dynamic_start_index];
  const double center_end_time_s = request_.state_timestamps->back();
  const auto rtk_samples = CollectUsableRtkSamples(
    *request_.gnss_samples,
    request_.should_use_sample,
    request_.corrected_time_s,
    center_start_time_s - half_velocity_window_s,
    center_end_time_s + half_velocity_window_s);
  if (rtk_samples.size() < 3U) {
    return;
  }

  const auto noise = gtsam::noiseModel::Isotropic::Sigma(
    2,
    request_.config->rtk_velocity_horizontal_sigma_mps);
  for (std::size_t center_index = 0; center_index < rtk_samples.size(); ++center_index) {
    RtkVelocityDiagnosticRow row = MakeBaseDiagnostic(rtk_samples[center_index]);
    row.sigma_mps = request_.config->rtk_velocity_horizontal_sigma_mps;
    if (row.corrected_time_s < center_start_time_s - kTimeEpsilonS ||
        row.corrected_time_s > center_end_time_s + kTimeEpsilonS) {
      continue;
    }
    ++request_.run_summary->rtk_velocity_candidate_count;
    if (!PopulateRtkVelocityMeasurement(
          rtk_samples,
          center_index,
          request_.config->rtk_velocity_window_s,
          row)) {
      ++request_.run_summary->rtk_velocity_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const StateMeasSyncResult sync = request_.find_state_for_time_s(row.corrected_time_s);
    row.sync_status = sync.status;
    if (sync.status != StateMeasSyncStatus::kSynchronizedI &&
        sync.status != StateMeasSyncStatus::kSynchronizedJ) {
      row.skip_reason = "unsynchronized";
      ++request_.run_summary->rtk_velocity_skipped_unsynced_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    row.state_index = SynchronizedStateIndex(sync);
    if (row.state_index >= request_.state_timestamps->size()) {
      row.skip_reason = "state_out_of_range";
      ++request_.run_summary->rtk_velocity_skipped_unsynced_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    row.state_time_s = (*request_.state_timestamps)[row.state_index];
    request_.graph->add(factor::RtkHorizontalVelocityFactor(
      symbol::V(row.state_index),
      (gtsam::Vector2() << row.rtk_velocity_mps.x(), row.rtk_velocity_mps.y()).finished(),
      noise));
    row.factor_added = true;
    row.skip_reason = "ADDED";
    ++request_.run_summary->rtk_velocity_factor_count;
    request_.diagnostics->push_back(row);
  }
}

void RtkVelocityConstraintBuilder::PopulateDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<RtkVelocityDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    try {
      const auto velocity = optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index));
      const auto pose = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index));
      row.optimized_velocity_mps = Eigen::Vector3d(velocity.x(), velocity.y(), velocity.z());
      row.velocity_residual_mps = row.optimized_velocity_mps - row.rtk_velocity_mps;
      row.horizontal_residual_mps =
        std::hypot(row.velocity_residual_mps.x(), row.velocity_residual_mps.y());

      const auto rotation_matrix = pose.rotation().matrix();
      const Eigen::Vector3d body_x_axis_nav = rotation_matrix.col(0);
      const Eigen::Vector3d body_y_axis_nav = rotation_matrix.col(1);
      const Eigen::Vector3d body_z_axis_nav = rotation_matrix.col(2);
      row.rtk_body_x_mps = body_x_axis_nav.dot(row.rtk_velocity_mps);
      row.rtk_body_y_mps = body_y_axis_nav.dot(row.rtk_velocity_mps);
      row.rtk_body_z_mps = body_z_axis_nav.dot(row.rtk_velocity_mps);
      row.optimized_body_x_mps = body_x_axis_nav.dot(row.optimized_velocity_mps);
      row.optimized_body_y_mps = body_y_axis_nav.dot(row.optimized_velocity_mps);
      row.optimized_body_z_mps = body_z_axis_nav.dot(row.optimized_velocity_mps);
      row.body_y_residual_mps = row.optimized_body_y_mps - row.rtk_body_y_mps;
    } catch (const std::exception &) {
      row.skip_reason = "optimized_value_missing";
    }
  }
  AccumulateOptimizedSummary(diagnostics, run_summary);
}

}  // namespace offline_lc_minimal
