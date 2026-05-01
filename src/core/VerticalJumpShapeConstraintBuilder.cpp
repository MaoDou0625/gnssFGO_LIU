#include "offline_lc_minimal/core/VerticalJumpShapeConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalPositionRampFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityHeightSlopeFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityRampFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

double Mean(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double Percentile(std::vector<double> values, const double ratio) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
    std::clamp(ratio, 0.0, 1.0) * static_cast<double>(values.size() - 1U));
  return values[index];
}

}  // namespace

VerticalJumpShapeConstraintBuilder::VerticalJumpShapeConstraintBuilder(
  VerticalJumpShapeConstraintBuildRequest request)
    : request_(std::move(request)) {}

void VerticalJumpShapeConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.jump_windows == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr) {
    throw std::runtime_error("VerticalJumpShapeConstraintBuilder received an incomplete request");
  }

  request_.diagnostics->reserve(request_.diagnostics->size() + request_.jump_windows->size());
  for (std::size_t window_index = 0; window_index < request_.jump_windows->size(); ++window_index) {
    const auto &window = (*request_.jump_windows)[window_index];
    VerticalJumpVelocityRampDiagnosticRow row;
    row.window_index = window_index;
    row.start_time_s = window.start_time_s;
    row.end_time_s = window.end_time_s;

    const bool add_velocity_ramp = request_.config->enable_vertical_jump_velocity_ramp_smoothing;
    const bool add_position_ramp = request_.config->enable_vertical_jump_position_ramp_smoothing;
    if (!add_velocity_ramp && !add_position_ramp) {
      row.skip_reason = "DISABLED";
      request_.diagnostics->push_back(row);
      continue;
    }
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s)) {
      row.skip_reason = "INVALID_WINDOW";
      ++request_.run_summary->vertical_jump_velocity_ramp_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const double padded_start_s = window.start_time_s - request_.config->vertical_jump_masked_imu_padding_s;
    const double padded_end_s = window.end_time_s + request_.config->vertical_jump_masked_imu_padding_s;
    const std::vector<std::size_t> state_indices = StateIndicesInWindow(padded_start_s, padded_end_s);
    row.start_state_index = state_indices.empty() ? 0U : state_indices.front();
    row.end_state_index = state_indices.empty() ? 0U : state_indices.back();

    if (state_indices.size() < 3U) {
      row.skip_reason = "INSUFFICIENT_STATES";
      ++request_.run_summary->vertical_jump_velocity_ramp_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const double start_time_s = (*request_.state_timestamps)[state_indices.front()];
    const double end_time_s = (*request_.state_timestamps)[state_indices.back()];
    const double duration_s = end_time_s - start_time_s;
    if (duration_s <= 0.0) {
      row.skip_reason = "INVALID_DURATION";
      ++request_.run_summary->vertical_jump_velocity_ramp_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_velocity_ramp_sigma_mps);
    const auto position_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_position_ramp_sigma_m);
    const auto velocity_height_slope_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_velocity_height_slope_sigma_mps);
    if (add_velocity_ramp) {
      for (std::size_t offset = 1; offset + 1U < state_indices.size(); ++offset) {
        const std::size_t state_index = state_indices[offset];
        request_.graph->add(factor::VerticalVelocityHeightSlopeFactor(
          symbol::X(state_indices.front()),
          symbol::V(state_index),
          symbol::X(state_indices.back()),
          duration_s,
          velocity_height_slope_noise));
        ++row.factor_count;
        ++request_.run_summary->vertical_jump_velocity_height_slope_factor_count;
      }
    }
    for (std::size_t offset = 1; offset + 1U < state_indices.size(); ++offset) {
      const std::size_t state_index = state_indices[offset];
      const double alpha = ((*request_.state_timestamps)[state_index] - start_time_s) / duration_s;
      if (add_velocity_ramp) {
        request_.graph->add(factor::VerticalVelocityRampFactor(
          symbol::V(state_indices.front()),
          symbol::V(state_index),
          symbol::V(state_indices.back()),
          alpha,
          velocity_noise));
        ++row.factor_count;
        ++request_.run_summary->vertical_jump_velocity_ramp_factor_count;
      }
      if (add_position_ramp) {
        request_.graph->add(factor::VerticalPositionRampFactor(
          symbol::X(state_indices.front()),
          symbol::X(state_index),
          symbol::X(state_indices.back()),
          alpha,
          position_noise));
        ++row.factor_count;
        ++request_.run_summary->vertical_jump_position_ramp_factor_count;
      }
    }

    row.skip_reason = "ADDED";
    request_.diagnostics->push_back(row);
  }
}

std::vector<std::size_t> VerticalJumpShapeConstraintBuilder::StateIndicesInWindow(
  const double start_time_s,
  const double end_time_s) const {
  std::vector<std::size_t> indices;
  for (std::size_t index = 0; index < request_.state_timestamps->size(); ++index) {
    const double time_s = (*request_.state_timestamps)[index];
    if (time_s >= start_time_s && time_s <= end_time_s) {
      indices.push_back(index);
    }
  }
  return indices;
}

void PopulateVerticalJumpVelocityRampDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalJumpVelocityRampDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (row.factor_count == 0U || row.end_state_index <= row.start_state_index) {
      continue;
    }

    const double start_vz = optimized_values.at<gtsam::Vector3>(symbol::V(row.start_state_index)).z();
    const double end_vz = optimized_values.at<gtsam::Vector3>(symbol::V(row.end_state_index)).z();
    row.jump_delta_vz_mps = end_vz - start_vz;

    std::vector<double> inside_vz;
    std::vector<double> residuals;
    std::vector<double> inside_up;
    std::vector<double> position_residuals;
    inside_vz.reserve(row.end_state_index - row.start_state_index + 1U);
    residuals.reserve(row.end_state_index - row.start_state_index + 1U);
    inside_up.reserve(row.end_state_index - row.start_state_index + 1U);
    position_residuals.reserve(row.end_state_index - row.start_state_index + 1U);
    for (std::size_t state_index = row.start_state_index; state_index <= row.end_state_index; ++state_index) {
      const double vz = optimized_values.at<gtsam::Vector3>(symbol::V(state_index)).z();
      inside_vz.push_back(vz);
      inside_up.push_back(optimized_values.at<gtsam::Pose3>(symbol::X(state_index)).translation().z());
    }
    const auto [min_it, max_it] = std::minmax_element(inside_vz.begin(), inside_vz.end());
    row.inside_vz_min_mps = *min_it;
    row.inside_vz_max_mps = *max_it;
    row.inside_vz_range_mps = *max_it - *min_it;
    const auto [up_min_it, up_max_it] = std::minmax_element(inside_up.begin(), inside_up.end());
    row.inside_up_min_m = *up_min_it;
    row.inside_up_max_m = *up_max_it;
    row.inside_up_range_m = *up_max_it - *up_min_it;

    if (inside_vz.size() >= 3U) {
      const double start_up = inside_up.front();
      const double end_up = inside_up.back();
      for (std::size_t offset = 1; offset + 1U < inside_vz.size(); ++offset) {
        const double alpha =
          static_cast<double>(offset) / static_cast<double>(inside_vz.size() - 1U);
        const double expected_vz = (1.0 - alpha) * start_vz + alpha * end_vz;
        residuals.push_back(std::abs(inside_vz[offset] - expected_vz));
        const double expected_up = (1.0 - alpha) * start_up + alpha * end_up;
        position_residuals.push_back(std::abs(inside_up[offset] - expected_up));
      }
    }
    row.ramp_residual_max_mps =
      residuals.empty()
        ? std::numeric_limits<double>::quiet_NaN()
        : *std::max_element(residuals.begin(), residuals.end());
    row.ramp_residual_p95_mps = Percentile(residuals, 0.95);
    row.position_ramp_residual_max_m =
      position_residuals.empty()
        ? std::numeric_limits<double>::quiet_NaN()
        : *std::max_element(position_residuals.begin(), position_residuals.end());
    row.position_ramp_residual_p95_m = Percentile(position_residuals, 0.95);

    std::vector<double> pre_values;
    std::vector<double> post_values;
    const std::size_t sample_count = std::min<std::size_t>(5U, row.end_state_index - row.start_state_index + 1U);
    for (std::size_t offset = 0; offset < sample_count; ++offset) {
      pre_values.push_back(optimized_values.at<gtsam::Vector3>(symbol::V(row.start_state_index + offset)).z());
      post_values.push_back(optimized_values.at<gtsam::Vector3>(symbol::V(row.end_state_index - offset)).z());
    }
    row.pre_vz_mean_mps = Mean(pre_values);
    row.post_vz_mean_mps = Mean(post_values);
  }
}

}  // namespace offline_lc_minimal
