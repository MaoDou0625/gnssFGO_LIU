#include "offline_lc_minimal/core/VerticalJumpShapeConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalPositionRampFactor.h"
#include "offline_lc_minimal/factor/VerticalPositionVelocityConsistencyFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityHeightSlopeFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityMeanFactor.h"
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

bool SpansOverlapOrTouch(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s && right_start_s <= left_end_s;
}

struct PositionVelocityConsistencyMetrics {
  double delta_z_m = std::numeric_limits<double>::quiet_NaN();
  double velocity_integral_m = std::numeric_limits<double>::quiet_NaN();
  double mismatch_m = std::numeric_limits<double>::quiet_NaN();
};

std::optional<PositionVelocityConsistencyMetrics> ComputePositionVelocityConsistencyMetrics(
  const gtsam::Values &values,
  const std::vector<double> &state_timestamps,
  const std::size_t state_i,
  const std::size_t state_j) {
  if (state_i >= state_timestamps.size() || state_j >= state_timestamps.size()) {
    return std::nullopt;
  }
  const double dt_s = state_timestamps[state_j] - state_timestamps[state_i];
  if (dt_s <= 0.0) {
    return std::nullopt;
  }
  const double z_i = values.at<gtsam::Pose3>(symbol::X(state_i)).translation().z();
  const double z_j = values.at<gtsam::Pose3>(symbol::X(state_j)).translation().z();
  const double vz_i = values.at<gtsam::Vector3>(symbol::V(state_i)).z();
  const double vz_j = values.at<gtsam::Vector3>(symbol::V(state_j)).z();
  PositionVelocityConsistencyMetrics metrics;
  metrics.delta_z_m = z_j - z_i;
  metrics.velocity_integral_m = 0.5 * dt_s * (vz_i + vz_j);
  metrics.mismatch_m = metrics.delta_z_m - metrics.velocity_integral_m;
  return metrics;
}

std::optional<double> ComputeVelocityMean(
  const gtsam::Values &values,
  const std::size_t start_state_index,
  const std::size_t end_state_index) {
  if (end_state_index < start_state_index) {
    return std::nullopt;
  }
  double sum = 0.0;
  std::size_t count = 0;
  for (std::size_t state_index = start_state_index; state_index <= end_state_index; ++state_index) {
    sum += values.at<gtsam::Vector3>(symbol::V(state_index)).z();
    ++count;
  }
  if (count == 0U) {
    return std::nullopt;
  }
  return sum / static_cast<double>(count);
}

std::optional<double> ComputeMaxVelocityResidualAgainstMean(
  const gtsam::Values &values,
  const std::vector<std::size_t> &state_indices,
  const double mean_vz_mps) {
  if (state_indices.empty() || !std::isfinite(mean_vz_mps)) {
    return std::nullopt;
  }
  double max_residual_mps = 0.0;
  for (const std::size_t state_index : state_indices) {
    const double vz_mps = values.at<gtsam::Vector3>(symbol::V(state_index)).z();
    max_residual_mps = std::max(max_residual_mps, std::abs(vz_mps - mean_vz_mps));
  }
  return max_residual_mps;
}

}  // namespace

VerticalJumpShapeConstraintBuilder::VerticalJumpShapeConstraintBuilder(
  VerticalJumpShapeConstraintBuildRequest request)
    : request_(std::move(request)) {}

void VerticalJumpShapeConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.jump_windows == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr ||
      request_.continuity_diagnostics == nullptr) {
    throw std::runtime_error("VerticalJumpShapeConstraintBuilder received an incomplete request");
  }

  const bool add_velocity_ramp = request_.config->enable_vertical_jump_velocity_ramp_smoothing;
  const bool add_position_ramp = request_.config->enable_vertical_jump_position_ramp_smoothing;
  const bool add_velocity_continuity = request_.config->enable_vertical_jump_velocity_continuity;
  const bool add_velocity_context_mean = request_.config->enable_vertical_jump_velocity_context_mean;
  const bool add_position_velocity_consistency =
    request_.config->enable_vertical_jump_position_velocity_consistency;
  const bool add_velocity_height_slope =
    request_.config->enable_vertical_jump_velocity_height_slope_constraint;
  if (!add_velocity_ramp && !add_position_ramp && !add_velocity_continuity &&
      !add_velocity_context_mean && !add_position_velocity_consistency && !add_velocity_height_slope) {
    return;
  }

  const std::vector<Span> spans = BuildMergedSpans();
  request_.diagnostics->reserve(request_.diagnostics->size() + spans.size());
  request_.continuity_diagnostics->reserve(request_.continuity_diagnostics->size() + spans.size());
  for (const auto &span : spans) {
    VerticalJumpVelocityRampDiagnosticRow row;
    row.window_index = span.window_index;
    row.start_time_s = span.start_time_s;
    row.end_time_s = span.end_time_s;
    row.start_state_index = span.state_indices.empty() ? 0U : span.state_indices.front();
    row.end_state_index = span.state_indices.empty() ? 0U : span.state_indices.back();

    if (span.state_indices.empty()) {
      row.skip_reason = "INSUFFICIENT_STATES";
      if (add_velocity_ramp || add_position_ramp || add_velocity_height_slope) {
        ++request_.run_summary->vertical_jump_velocity_ramp_skipped_count;
      }
      if (add_velocity_continuity || add_position_velocity_consistency) {
        ++request_.run_summary->vertical_jump_continuity_skipped_count;
      }
      if (add_velocity_context_mean) {
        ++request_.run_summary->vertical_jump_velocity_context_skipped_count;
      }
      request_.diagnostics->push_back(row);
      continue;
    }

    const auto velocity_continuity_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_velocity_continuity_sigma_mps);
    const auto boundary_position_velocity_consistency_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_boundary_position_velocity_consistency_sigma_m);
    const auto add_velocity_context_mean_factors =
      [this](const std::vector<std::size_t> &context_state_indices,
             const std::vector<std::size_t> &boundary_state_indices) {
        if (context_state_indices.empty() || boundary_state_indices.empty()) {
          return std::size_t{0};
        }
        const auto context_noise = gtsam::noiseModel::Isotropic::Sigma(
          1,
          request_.config->vertical_jump_velocity_context_mean_sigma_mps);
        std::size_t factor_count = 0;
        for (const std::size_t boundary_state_index : boundary_state_indices) {
          std::vector<gtsam::Key> context_velocity_keys;
          context_velocity_keys.reserve(context_state_indices.size());
          for (const std::size_t context_state_index : context_state_indices) {
            if (boundary_state_index != context_state_index) {
              context_velocity_keys.push_back(symbol::V(context_state_index));
            }
          }
          if (context_velocity_keys.empty()) {
            continue;
          }
          request_.graph->add(factor::VerticalVelocityMeanFactor(
            symbol::V(boundary_state_index),
            std::move(context_velocity_keys),
            context_noise));
          ++factor_count;
        }
        return factor_count;
      };
    const auto add_boundary_position_velocity_factor =
      [this, &boundary_position_velocity_consistency_noise](const std::size_t state_i, const std::size_t state_j) {
        const double dt_s = (*request_.state_timestamps)[state_j] - (*request_.state_timestamps)[state_i];
        if (dt_s <= 0.0) {
          return false;
        }
        request_.graph->add(factor::VerticalPositionVelocityConsistencyFactor(
          symbol::X(state_i),
          symbol::V(state_i),
          symbol::X(state_j),
          symbol::V(state_j),
          dt_s,
          boundary_position_velocity_consistency_noise));
        ++request_.run_summary->vertical_jump_position_velocity_consistency_factor_count;
        return true;
      };

    if (add_velocity_continuity || add_velocity_context_mean || add_position_velocity_consistency) {
      VerticalJumpContinuityDiagnosticRow continuity_row;
      continuity_row.window_index = span.window_index;
      continuity_row.start_state_index = span.state_indices.front();
      continuity_row.end_state_index = span.state_indices.back();
      continuity_row.pre_anchor_state_index = span.pre_anchor_state_index.value_or(0U);
      continuity_row.post_anchor_state_index = span.post_anchor_state_index.value_or(0U);
      continuity_row.pre_context_state_count = span.pre_context_state_indices.size();
      continuity_row.post_context_state_count = span.post_context_state_indices.size();
      if (!span.pre_context_state_indices.empty()) {
        continuity_row.pre_context_start_state_index = span.pre_context_state_indices.front();
        continuity_row.pre_context_end_state_index = span.pre_context_state_indices.back();
      }
      if (!span.post_context_state_indices.empty()) {
        continuity_row.post_context_start_state_index = span.post_context_state_indices.front();
        continuity_row.post_context_end_state_index = span.post_context_state_indices.back();
      }
      continuity_row.start_time_s = span.start_time_s;
      continuity_row.end_time_s = span.end_time_s;
      if (span.pre_anchor_state_index.has_value()) {
        const std::size_t pre_anchor = *span.pre_anchor_state_index;
        if (add_velocity_continuity) {
          request_.graph->add(factor::VerticalVelocityDeltaFactor(
            symbol::V(pre_anchor),
            symbol::V(span.state_indices.front()),
            0.0,
            velocity_continuity_noise));
          ++request_.run_summary->vertical_jump_velocity_continuity_factor_count;
          continuity_row.entry_factor_added = true;
        }
        if (add_position_velocity_consistency) {
          continuity_row.entry_position_velocity_factor_added =
            add_boundary_position_velocity_factor(pre_anchor, span.state_indices.front());
          if (!continuity_row.entry_position_velocity_factor_added) {
            ++request_.run_summary->vertical_jump_continuity_skipped_count;
          }
        }
      } else {
        if (add_velocity_continuity || add_position_velocity_consistency) {
          ++request_.run_summary->vertical_jump_continuity_skipped_count;
        }
      }
      if (span.post_anchor_state_index.has_value()) {
        const std::size_t post_anchor = *span.post_anchor_state_index;
        if (add_velocity_continuity) {
          request_.graph->add(factor::VerticalVelocityDeltaFactor(
            symbol::V(span.state_indices.back()),
            symbol::V(post_anchor),
            0.0,
            velocity_continuity_noise));
          ++request_.run_summary->vertical_jump_velocity_continuity_factor_count;
          continuity_row.exit_factor_added = true;
        }
        if (add_position_velocity_consistency) {
          continuity_row.exit_position_velocity_factor_added =
            add_boundary_position_velocity_factor(span.state_indices.back(), post_anchor);
          if (!continuity_row.exit_position_velocity_factor_added) {
            ++request_.run_summary->vertical_jump_continuity_skipped_count;
          }
        }
      } else {
        if (add_velocity_continuity || add_position_velocity_consistency) {
          ++request_.run_summary->vertical_jump_continuity_skipped_count;
        }
      }
      if (add_velocity_context_mean) {
        std::vector<std::size_t> pre_boundary_state_indices{span.state_indices.front()};
        if (span.pre_anchor_state_index.has_value()) {
          pre_boundary_state_indices.insert(pre_boundary_state_indices.begin(), *span.pre_anchor_state_index);
        }
        std::vector<std::size_t> post_boundary_state_indices{span.state_indices.back()};
        if (span.post_anchor_state_index.has_value()) {
          post_boundary_state_indices.push_back(*span.post_anchor_state_index);
        }
        const std::size_t pre_context_factor_count = add_velocity_context_mean_factors(
          span.pre_context_state_indices,
          pre_boundary_state_indices);
        const std::size_t post_context_factor_count = add_velocity_context_mean_factors(
          span.post_context_state_indices,
          post_boundary_state_indices);
        continuity_row.velocity_context_factor_count =
          pre_context_factor_count + post_context_factor_count;
        request_.run_summary->vertical_jump_velocity_context_factor_count +=
          continuity_row.velocity_context_factor_count;
        if (pre_context_factor_count == 0U) {
          ++request_.run_summary->vertical_jump_velocity_context_skipped_count;
        }
        if (post_context_factor_count == 0U) {
          ++request_.run_summary->vertical_jump_velocity_context_skipped_count;
        }
      }
      const bool any_boundary_factor_added =
        continuity_row.entry_factor_added ||
        continuity_row.exit_factor_added ||
        continuity_row.entry_position_velocity_factor_added ||
        continuity_row.exit_position_velocity_factor_added ||
        continuity_row.velocity_context_factor_count > 0U;
      continuity_row.skip_reason = any_boundary_factor_added ? "ADDED" : "MISSING_ANCHORS_OR_CONTEXT";
      request_.continuity_diagnostics->push_back(continuity_row);
    }

    if (span.state_indices.size() < 2U) {
      row.skip_reason = "INSUFFICIENT_STATES_FOR_SHAPE";
      if (add_velocity_ramp || add_position_ramp || add_velocity_height_slope) {
        ++request_.run_summary->vertical_jump_velocity_ramp_skipped_count;
      }
      request_.diagnostics->push_back(row);
      continue;
    }

    const double start_time_s = (*request_.state_timestamps)[span.state_indices.front()];
    const double end_time_s = (*request_.state_timestamps)[span.state_indices.back()];
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
    const auto position_velocity_consistency_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_position_velocity_consistency_sigma_m);
    const auto velocity_height_slope_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_velocity_height_slope_sigma_mps);

    if (add_velocity_height_slope) {
      for (std::size_t offset = 1; offset + 1U < span.state_indices.size(); ++offset) {
        const std::size_t state_index = span.state_indices[offset];
        request_.graph->add(factor::VerticalVelocityHeightSlopeFactor(
          symbol::X(span.state_indices.front()),
          symbol::V(state_index),
          symbol::X(span.state_indices.back()),
          duration_s,
          velocity_height_slope_noise));
        ++row.factor_count;
        ++request_.run_summary->vertical_jump_velocity_height_slope_factor_count;
      }
    }
    if (add_position_velocity_consistency) {
      for (std::size_t offset = 0; offset + 1U < span.state_indices.size(); ++offset) {
        const std::size_t state_i = span.state_indices[offset];
        const std::size_t state_j = span.state_indices[offset + 1U];
        const double dt_s = (*request_.state_timestamps)[state_j] - (*request_.state_timestamps)[state_i];
        if (dt_s <= 0.0) {
          ++request_.run_summary->vertical_jump_continuity_skipped_count;
          continue;
        }
        request_.graph->add(factor::VerticalPositionVelocityConsistencyFactor(
          symbol::X(state_i),
          symbol::V(state_i),
          symbol::X(state_j),
          symbol::V(state_j),
          dt_s,
          position_velocity_consistency_noise));
        ++row.factor_count;
        ++request_.run_summary->vertical_jump_position_velocity_consistency_factor_count;
      }
    }
    for (std::size_t offset = 1; offset + 1U < span.state_indices.size(); ++offset) {
      const std::size_t state_index = span.state_indices[offset];
      const double alpha = ((*request_.state_timestamps)[state_index] - start_time_s) / duration_s;
      if (add_velocity_ramp) {
        request_.graph->add(factor::VerticalVelocityRampFactor(
          symbol::V(span.state_indices.front()),
          symbol::V(state_index),
          symbol::V(span.state_indices.back()),
          alpha,
          velocity_noise));
        ++row.factor_count;
        ++request_.run_summary->vertical_jump_velocity_ramp_factor_count;
      }
      if (add_position_ramp) {
        request_.graph->add(factor::VerticalPositionRampFactor(
          symbol::X(span.state_indices.front()),
          symbol::X(state_index),
          symbol::X(span.state_indices.back()),
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

std::vector<VerticalJumpShapeConstraintBuilder::Span>
VerticalJumpShapeConstraintBuilder::BuildMergedSpans() const {
  std::vector<Span> spans;
  spans.reserve(request_.jump_windows->size());
  for (std::size_t window_index = 0; window_index < request_.jump_windows->size(); ++window_index) {
    const auto &window = (*request_.jump_windows)[window_index];
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s)) {
      continue;
    }
    Span span;
    span.window_index = window_index;
    span.start_time_s = window.start_time_s - request_.config->vertical_jump_masked_imu_padding_s;
    span.end_time_s = window.end_time_s + request_.config->vertical_jump_masked_imu_padding_s;
    spans.push_back(std::move(span));
  }
  std::sort(spans.begin(), spans.end(), [](const Span &left, const Span &right) {
    return left.start_time_s < right.start_time_s;
  });

  std::vector<Span> merged_spans;
  for (const auto &span : spans) {
    if (merged_spans.empty() ||
        !SpansOverlapOrTouch(
          merged_spans.back().start_time_s,
          merged_spans.back().end_time_s,
          span.start_time_s,
          span.end_time_s)) {
      merged_spans.push_back(span);
      continue;
    }
    merged_spans.back().end_time_s = std::max(merged_spans.back().end_time_s, span.end_time_s);
  }

  const auto is_inside_any_merged_span = [&merged_spans](const double time_s) {
    return std::any_of(merged_spans.begin(), merged_spans.end(), [time_s](const Span &candidate) {
      return time_s + 1e-9 >= candidate.start_time_s && time_s <= candidate.end_time_s + 1e-9;
    });
  };

  for (auto &span : merged_spans) {
    span.state_indices = StateIndicesInWindow(span.start_time_s, span.end_time_s);
    if (!span.state_indices.empty()) {
      const std::size_t first_state_index = span.state_indices.front();
      const std::size_t last_state_index = span.state_indices.back();
      if (first_state_index > 0U) {
        span.pre_anchor_state_index = first_state_index - 1U;
      }
      if (last_state_index + 1U < request_.state_timestamps->size()) {
        span.post_anchor_state_index = last_state_index + 1U;
      }
      const double context_window_s = request_.config->vertical_jump_velocity_context_window_s;
      for (std::size_t state_index = first_state_index; state_index > 0U;) {
        --state_index;
        const double time_s = (*request_.state_timestamps)[state_index];
        if (time_s + 1e-9 < span.start_time_s - context_window_s ||
            is_inside_any_merged_span(time_s)) {
          break;
        }
        if (time_s < span.start_time_s) {
          span.pre_context_state_indices.push_back(state_index);
        }
      }
      std::reverse(span.pre_context_state_indices.begin(), span.pre_context_state_indices.end());
      for (std::size_t state_index = last_state_index + 1U;
           state_index < request_.state_timestamps->size();
           ++state_index) {
        const double time_s = (*request_.state_timestamps)[state_index];
        if (time_s <= span.end_time_s + context_window_s + 1e-9) {
          if (is_inside_any_merged_span(time_s)) {
            break;
          }
          if (time_s > span.end_time_s) {
            span.post_context_state_indices.push_back(state_index);
          }
        } else {
          break;
        }
      }
    }
  }
  return merged_spans;
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

void PopulateVerticalJumpContinuityDiagnostics(
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps,
  std::vector<VerticalJumpContinuityDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (row.end_state_index < row.start_state_index) {
      continue;
    }
    const double first_inside_vz = optimized_values.at<gtsam::Vector3>(symbol::V(row.start_state_index)).z();
    const double last_inside_vz = optimized_values.at<gtsam::Vector3>(symbol::V(row.end_state_index)).z();
    if (row.entry_factor_added) {
      const double pre_vz = optimized_values.at<gtsam::Vector3>(symbol::V(row.pre_anchor_state_index)).z();
      row.entry_delta_vz_mps = first_inside_vz - pre_vz;
      row.entry_residual_mps = row.entry_delta_vz_mps;
    }
    if (row.exit_factor_added) {
      const double post_vz = optimized_values.at<gtsam::Vector3>(symbol::V(row.post_anchor_state_index)).z();
      row.exit_delta_vz_mps = post_vz - last_inside_vz;
      row.exit_residual_mps = row.exit_delta_vz_mps;
    }
    if (row.entry_position_velocity_factor_added) {
      const auto entry_metrics = ComputePositionVelocityConsistencyMetrics(
        optimized_values,
        state_timestamps,
        row.pre_anchor_state_index,
        row.start_state_index);
      if (entry_metrics.has_value()) {
        row.entry_delta_z_m = entry_metrics->delta_z_m;
        row.entry_velocity_integral_m = entry_metrics->velocity_integral_m;
        row.entry_zv_mismatch_m = entry_metrics->mismatch_m;
      }
    }
    if (row.exit_position_velocity_factor_added) {
      const auto exit_metrics = ComputePositionVelocityConsistencyMetrics(
        optimized_values,
        state_timestamps,
        row.end_state_index,
        row.post_anchor_state_index);
      if (exit_metrics.has_value()) {
        row.exit_delta_z_m = exit_metrics->delta_z_m;
        row.exit_velocity_integral_m = exit_metrics->velocity_integral_m;
        row.exit_zv_mismatch_m = exit_metrics->mismatch_m;
      }
    }
    if (row.pre_context_state_count > 0U) {
      const auto mean_vz = ComputeVelocityMean(
        optimized_values,
        row.pre_context_start_state_index,
        row.pre_context_end_state_index);
      if (mean_vz.has_value()) {
        row.pre_context_mean_vz_mps = *mean_vz;
        std::vector<std::size_t> boundary_state_indices{row.start_state_index};
        if (row.pre_anchor_state_index < row.start_state_index) {
          boundary_state_indices.insert(boundary_state_indices.begin(), row.pre_anchor_state_index);
        }
        const auto max_residual = ComputeMaxVelocityResidualAgainstMean(
          optimized_values,
          boundary_state_indices,
          row.pre_context_mean_vz_mps);
        if (max_residual.has_value()) {
          row.max_pre_context_residual_mps = *max_residual;
        }
      }
    }
    if (row.post_context_state_count > 0U) {
      const auto mean_vz = ComputeVelocityMean(
        optimized_values,
        row.post_context_start_state_index,
        row.post_context_end_state_index);
      if (mean_vz.has_value()) {
        row.post_context_mean_vz_mps = *mean_vz;
        std::vector<std::size_t> boundary_state_indices{row.end_state_index};
        if (row.post_anchor_state_index > row.end_state_index) {
          boundary_state_indices.push_back(row.post_anchor_state_index);
        }
        const auto max_residual = ComputeMaxVelocityResidualAgainstMean(
          optimized_values,
          boundary_state_indices,
          row.post_context_mean_vz_mps);
        if (max_residual.has_value()) {
          row.max_post_context_residual_mps = *max_residual;
        }
      }
    }
    std::vector<double> inside_vz;
    inside_vz.reserve(row.end_state_index - row.start_state_index + 1U);
    for (std::size_t state_index = row.start_state_index; state_index <= row.end_state_index; ++state_index) {
      inside_vz.push_back(optimized_values.at<gtsam::Vector3>(symbol::V(state_index)).z());
    }
    if (!inside_vz.empty()) {
      const auto [min_it, max_it] = std::minmax_element(inside_vz.begin(), inside_vz.end());
      row.max_inside_vz_range_mps = *max_it - *min_it;
    }
    const double max_boundary_step_mps = std::max(
      std::isfinite(row.entry_delta_vz_mps) ? std::abs(row.entry_delta_vz_mps) : 0.0,
      std::isfinite(row.exit_delta_vz_mps) ? std::abs(row.exit_delta_vz_mps) : 0.0);
    if (std::isfinite(row.entry_delta_vz_mps) || std::isfinite(row.exit_delta_vz_mps)) {
      row.max_boundary_step_mps = max_boundary_step_mps;
    }
    const double max_boundary_zv_mismatch_m = std::max(
      std::isfinite(row.entry_zv_mismatch_m) ? std::abs(row.entry_zv_mismatch_m) : 0.0,
      std::isfinite(row.exit_zv_mismatch_m) ? std::abs(row.exit_zv_mismatch_m) : 0.0);
    if (std::isfinite(row.entry_zv_mismatch_m) || std::isfinite(row.exit_zv_mismatch_m)) {
      row.max_boundary_zv_mismatch_m = max_boundary_zv_mismatch_m;
    }

    double max_position_velocity_residual_m = 0.0;
    bool has_position_velocity_residual = false;
    for (std::size_t state_index = row.start_state_index; state_index < row.end_state_index; ++state_index) {
      const auto metrics = ComputePositionVelocityConsistencyMetrics(
        optimized_values,
        state_timestamps,
        state_index,
        state_index + 1U);
      if (!metrics.has_value()) {
        continue;
      }
      max_position_velocity_residual_m =
        std::max(max_position_velocity_residual_m, std::abs(metrics->mismatch_m));
      has_position_velocity_residual = true;
    }
    row.max_position_velocity_residual_m =
      has_position_velocity_residual
        ? max_position_velocity_residual_m
        : std::numeric_limits<double>::quiet_NaN();
  }
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
