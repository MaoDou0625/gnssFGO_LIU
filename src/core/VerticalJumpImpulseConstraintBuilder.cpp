#include "offline_lc_minimal/core/VerticalJumpImpulseConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <boost/make_shared.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/factor/VerticalJumpImpulseVelocityFactor.h"
#include "offline_lc_minimal/factor/VerticalMaskedCombinedImuFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

gtsam::Key ImpulseKey(const std::size_t span_index) {
  return gtsam::Symbol('j', span_index);
}

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s && right_start_s <= left_end_s;
}

double SumDetectedDeltaVzInit(
  const std::vector<BodyZSeedJumpWindowRow> &windows,
  const std::vector<std::size_t> &window_indices) {
  double sum = 0.0;
  bool has_value = false;
  for (const std::size_t window_index : window_indices) {
    if (window_index >= windows.size()) {
      continue;
    }
    const double value = windows[window_index].delta_vz_init_mps;
    if (!std::isfinite(value)) {
      continue;
    }
    sum += value;
    has_value = true;
  }
  return has_value ? sum : std::numeric_limits<double>::quiet_NaN();
}

double SumDetectedSignedDeltaVelocity(
  const std::vector<BodyZSeedJumpWindowRow> &windows,
  const std::vector<std::size_t> &window_indices) {
  double sum = 0.0;
  bool has_value = false;
  for (const std::size_t window_index : window_indices) {
    if (window_index >= windows.size()) {
      continue;
    }
    const double value = windows[window_index].signed_delta_velocity_mps;
    if (!std::isfinite(value)) {
      continue;
    }
    sum += value;
    has_value = true;
  }
  return has_value ? sum : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace

VerticalJumpImpulseConstraintBuilder::VerticalJumpImpulseConstraintBuilder(
  VerticalJumpImpulseConstraintBuildRequest request)
    : request_(std::move(request)) {}

void VerticalJumpImpulseConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.jump_windows == nullptr || request_.imu_intervals == nullptr ||
      request_.propagation_records == nullptr || request_.graph == nullptr ||
      request_.initial_values == nullptr || request_.run_summary == nullptr ||
      request_.diagnostics == nullptr) {
    throw std::runtime_error("VerticalJumpImpulseConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_vertical_jump_impulse) {
    return;
  }

  const std::vector<Span> spans = BuildMergedSpans();
  request_.diagnostics->reserve(request_.diagnostics->size() + spans.size());
  const auto impulse_prior_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_jump_impulse_prior_sigma_mps);
  const auto impulse_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_jump_impulse_velocity_sigma_mps);

  for (const auto &span : spans) {
    VerticalJumpImpulseDiagnosticRow row;
    row.span_index = span.span_index;
    row.source_window_index = span.source_window_indices.empty() ? 0U : span.source_window_indices.front();
    row.source_window_count = span.source_window_indices.size();
    row.start_time_s = span.start_time_s;
    row.end_time_s = span.end_time_s;
    row.start_state_index = span.state_indices.empty() ? 0U : span.state_indices.front();
    row.end_state_index = span.state_indices.empty() ? 0U : span.state_indices.back();
    row.pre_anchor_state_index = span.pre_anchor_state_index.value_or(0U);
    row.post_anchor_state_index = span.post_anchor_state_index.value_or(0U);
    row.detected_delta_vz_init_mps = SumDetectedDeltaVzInit(*request_.jump_windows, span.source_window_indices);
    row.detected_signed_delta_velocity_mps =
      SumDetectedSignedDeltaVelocity(*request_.jump_windows, span.source_window_indices);
    row.prior_sigma_mps = request_.config->vertical_jump_impulse_prior_sigma_mps;
    row.velocity_sigma_mps = request_.config->vertical_jump_impulse_velocity_sigma_mps;

    if (!span.pre_anchor_state_index.has_value() || !span.post_anchor_state_index.has_value() ||
        *span.post_anchor_state_index <= *span.pre_anchor_state_index) {
      row.skip_reason = "MISSING_ANCHORS";
      ++request_.run_summary->vertical_jump_impulse_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const std::optional<double> imu_delta_vz_mps =
      SumImuDeltaVz(*span.pre_anchor_state_index, *span.post_anchor_state_index);
    if (!imu_delta_vz_mps.has_value() || !std::isfinite(*imu_delta_vz_mps)) {
      row.skip_reason = "MISSING_IMU_DELTA";
      ++request_.run_summary->vertical_jump_impulse_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    row.imu_delta_vz_mps = *imu_delta_vz_mps;

    for (const auto &interval : *request_.imu_intervals) {
      if (!IntervalsOverlap(interval.start_time_s, interval.end_time_s, span.start_time_s, span.end_time_s)) {
        continue;
      }
      if (interval.graph_factor_index >= request_.graph->size()) {
        continue;
      }
      request_.graph->replace(
        interval.graph_factor_index,
        boost::make_shared<factor::VerticalMaskedCombinedImuFactor>(
          symbol::X(interval.state_index_i),
          symbol::V(interval.state_index_i),
          symbol::X(interval.state_index_j),
          symbol::V(interval.state_index_j),
          symbol::B(interval.state_index_i),
          symbol::B(interval.state_index_j),
          interval.preintegrated_measurements));
      ++row.replaced_imu_factor_count;
      ++request_.run_summary->vertical_jump_impulse_replaced_imu_factor_count;
    }

    const gtsam::Key impulse_key = ImpulseKey(span.span_index);
    if (!request_.initial_values->exists(impulse_key)) {
      request_.initial_values->insert(impulse_key, 0.0);
    }
    request_.graph->add(gtsam::PriorFactor<double>(impulse_key, 0.0, impulse_prior_noise));
    ++request_.run_summary->vertical_jump_impulse_prior_factor_count;
    request_.graph->add(factor::VerticalJumpImpulseVelocityFactor(
      symbol::V(*span.pre_anchor_state_index),
      symbol::V(*span.post_anchor_state_index),
      impulse_key,
      *imu_delta_vz_mps,
      impulse_velocity_noise));
    ++request_.run_summary->vertical_jump_impulse_factor_count;

    row.factor_added = true;
    row.skip_reason = "ADDED";
    request_.diagnostics->push_back(row);
  }
}

std::vector<VerticalJumpImpulseConstraintBuilder::Span>
VerticalJumpImpulseConstraintBuilder::BuildMergedSpans() const {
  std::vector<Span> spans;
  spans.reserve(request_.jump_windows->size());
  for (std::size_t window_index = 0; window_index < request_.jump_windows->size(); ++window_index) {
    const auto &window = (*request_.jump_windows)[window_index];
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s)) {
      continue;
    }
    Span span;
    span.start_time_s = window.start_time_s - request_.config->vertical_jump_masked_imu_padding_s;
    span.end_time_s = window.end_time_s + request_.config->vertical_jump_masked_imu_padding_s;
    span.source_window_indices.push_back(window_index);
    spans.push_back(std::move(span));
  }
  std::sort(spans.begin(), spans.end(), [](const Span &left, const Span &right) {
    return left.start_time_s < right.start_time_s;
  });

  std::vector<Span> merged_spans;
  for (const auto &span : spans) {
    if (merged_spans.empty() ||
        !IntervalsOverlap(
          merged_spans.back().start_time_s,
          merged_spans.back().end_time_s,
          span.start_time_s,
          span.end_time_s)) {
      merged_spans.push_back(span);
      continue;
    }
    merged_spans.back().end_time_s = std::max(merged_spans.back().end_time_s, span.end_time_s);
    merged_spans.back().source_window_indices.insert(
      merged_spans.back().source_window_indices.end(),
      span.source_window_indices.begin(),
      span.source_window_indices.end());
  }

  for (std::size_t span_index = 0; span_index < merged_spans.size(); ++span_index) {
    auto &span = merged_spans[span_index];
    span.span_index = span_index;
    span.state_indices = StateIndicesInWindow(span.start_time_s, span.end_time_s);
    if (span.state_indices.empty()) {
      continue;
    }
    const std::size_t first_state_index = span.state_indices.front();
    const std::size_t last_state_index = span.state_indices.back();
    if (first_state_index > 0U) {
      span.pre_anchor_state_index = first_state_index - 1U;
    }
    if (last_state_index + 1U < request_.state_timestamps->size()) {
      span.post_anchor_state_index = last_state_index + 1U;
    }
  }
  return merged_spans;
}

std::vector<std::size_t> VerticalJumpImpulseConstraintBuilder::StateIndicesInWindow(
  const double start_time_s,
  const double end_time_s) const {
  std::vector<std::size_t> indices;
  for (std::size_t index = 0; index < request_.state_timestamps->size(); ++index) {
    const double time_s = (*request_.state_timestamps)[index];
    if (time_s + kTimeEpsilonS >= start_time_s && time_s <= end_time_s + kTimeEpsilonS) {
      indices.push_back(index);
    }
  }
  return indices;
}

std::optional<double> VerticalJumpImpulseConstraintBuilder::SumImuDeltaVz(
  const std::size_t state_i,
  const std::size_t state_j) const {
  if (state_j <= state_i) {
    return std::nullopt;
  }
  double sum_delta_vz_mps = 0.0;
  std::size_t matched_record_count = 0;
  for (const auto &record : *request_.propagation_records) {
    if (record.state_index_i < state_i || record.state_index_j > state_j) {
      continue;
    }
    if (record.state_index_j != record.state_index_i + 1U) {
      continue;
    }
    if (!std::isfinite(record.target_delta_vz_mps)) {
      return std::nullopt;
    }
    sum_delta_vz_mps += record.target_delta_vz_mps;
    ++matched_record_count;
  }
  if (matched_record_count != state_j - state_i) {
    return std::nullopt;
  }
  return sum_delta_vz_mps;
}

void PopulateVerticalJumpImpulseDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalJumpImpulseDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const double pre_vz_mps =
      optimized_values.at<gtsam::Vector3>(symbol::V(row.pre_anchor_state_index)).z();
    const double post_vz_mps =
      optimized_values.at<gtsam::Vector3>(symbol::V(row.post_anchor_state_index)).z();
    const double estimated_impulse_mps = optimized_values.at<double>(ImpulseKey(row.span_index));
    row.pre_anchor_vz_mps = pre_vz_mps;
    row.post_anchor_vz_mps = post_vz_mps;
    row.optimized_delta_vz_mps = post_vz_mps - pre_vz_mps;
    row.estimated_jump_impulse_mps = estimated_impulse_mps;
    row.corrected_delta_vz_mps = row.imu_delta_vz_mps - estimated_impulse_mps;
    row.residual_mps = row.optimized_delta_vz_mps - row.imu_delta_vz_mps + estimated_impulse_mps;
  }
}

}  // namespace offline_lc_minimal
