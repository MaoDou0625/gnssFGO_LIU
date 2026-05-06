#include "offline_lc_minimal/core/VerticalJumpBiasConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <boost/make_shared.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/factor/VerticalJumpBiasVelocityFactor.h"
#include "offline_lc_minimal/factor/VerticalMaskedCombinedImuFactor.h"
#include "offline_lc_minimal/factor/VerticalPositionVelocityConsistencyFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

gtsam::Key JumpBiasKey(const std::size_t span_index) {
  return gtsam::Symbol('c', span_index);
}

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s && right_start_s <= left_end_s;
}

bool IntervalsHavePositiveOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return std::min(left_end_s, right_end_s) - std::max(left_start_s, right_start_s) > kTimeEpsilonS;
}

double SourceWindowDurationS(
  const std::vector<BodyZSeedJumpWindowRow> &windows,
  const std::vector<std::size_t> &window_indices) {
  double duration_s = 0.0;
  for (const std::size_t window_index : window_indices) {
    if (window_index >= windows.size()) {
      continue;
    }
    const auto &window = windows[window_index];
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s) ||
        window.end_time_s <= window.start_time_s) {
      continue;
    }
    duration_s += window.end_time_s - window.start_time_s;
  }
  return duration_s;
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

VerticalJumpBiasConstraintBuilder::VerticalJumpBiasConstraintBuilder(
  VerticalJumpBiasConstraintBuildRequest request)
    : request_(std::move(request)) {}

void VerticalJumpBiasConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.jump_windows == nullptr || request_.imu_intervals == nullptr ||
      request_.propagation_records == nullptr || request_.graph == nullptr ||
      request_.initial_values == nullptr || request_.run_summary == nullptr ||
      request_.diagnostics == nullptr) {
    throw std::runtime_error("VerticalJumpBiasConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_vertical_jump_bias) {
    return;
  }

  const std::vector<Span> spans = BuildMergedSpans();
  request_.diagnostics->reserve(request_.diagnostics->size() + spans.size());
  const auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_jump_bias_prior_sigma_mps2);
  const auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_jump_bias_velocity_sigma_mps);
  const auto position_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_jump_bias_position_velocity_sigma_m);

  for (const auto &span : spans) {
    VerticalJumpBiasDiagnosticRow row;
    row.span_index = span.span_index;
    row.source_window_index = span.source_window_indices.empty() ? 0U : span.source_window_indices.front();
    row.source_window_count = span.source_window_indices.size();
    row.start_time_s = span.start_time_s;
    row.end_time_s = span.end_time_s;
    row.start_state_index = span.state_indices.empty() ? 0U : span.state_indices.front();
    row.end_state_index = span.state_indices.empty() ? 0U : span.state_indices.back();
    row.source_window_duration_s = SourceWindowDurationS(*request_.jump_windows, span.source_window_indices);
    row.detected_signed_delta_velocity_mps =
      SumDetectedSignedDeltaVelocity(*request_.jump_windows, span.source_window_indices);
    row.prior_sigma_mps2 = request_.config->vertical_jump_bias_prior_sigma_mps2;
    row.velocity_sigma_mps = request_.config->vertical_jump_bias_velocity_sigma_mps;
    row.position_velocity_sigma_m = request_.config->vertical_jump_bias_position_velocity_sigma_m;

    if (row.source_window_duration_s <= 0.0 || !std::isfinite(row.detected_signed_delta_velocity_mps)) {
      row.skip_reason = "MISSING_DETECTED_BIAS";
      ++request_.run_summary->vertical_jump_bias_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    row.detected_bias_mps2 = row.detected_signed_delta_velocity_mps / row.source_window_duration_s;

    const std::vector<MatchedInterval> matched_intervals = FindMatchedIntervals(span);
    if (matched_intervals.empty()) {
      row.skip_reason = "MISSING_IMU_INTERVALS";
      ++request_.run_summary->vertical_jump_bias_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    row.start_state_index = matched_intervals.front().imu_interval->state_index_i;
    row.end_state_index = matched_intervals.back().imu_interval->state_index_j;

    bool invalid_interval = false;
    for (const auto &matched_interval : matched_intervals) {
      const auto &record = *matched_interval.propagation_record;
      const double dt_s = record.end_time_s - record.start_time_s;
      if (dt_s <= 0.0 || !std::isfinite(dt_s) || !std::isfinite(record.target_delta_vz_mps)) {
        invalid_interval = true;
        break;
      }
      row.factor_duration_s += dt_s;
      row.imu_delta_vz_mps += record.target_delta_vz_mps;
    }
    if (invalid_interval) {
      row.skip_reason = "MISSING_IMU_DELTA";
      ++request_.run_summary->vertical_jump_bias_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const gtsam::Key jump_bias_key = JumpBiasKey(span.span_index);
    if (!request_.initial_values->exists(jump_bias_key)) {
      request_.initial_values->insert(jump_bias_key, row.detected_bias_mps2);
    }
    request_.graph->add(gtsam::PriorFactor<double>(jump_bias_key, row.detected_bias_mps2, prior_noise));
    ++request_.run_summary->vertical_jump_bias_prior_factor_count;

    for (const auto &matched_interval : matched_intervals) {
      const auto &interval = *matched_interval.imu_interval;
      const auto &record = *matched_interval.propagation_record;
      const double dt_s = record.end_time_s - record.start_time_s;

      if (interval.graph_factor_index < request_.graph->size()) {
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
        ++request_.run_summary->vertical_jump_bias_replaced_imu_factor_count;
      }

      request_.graph->add(factor::VerticalJumpBiasVelocityFactor(
        symbol::V(record.state_index_i),
        symbol::V(record.state_index_j),
        jump_bias_key,
        record.target_delta_vz_mps,
        dt_s,
        velocity_noise));
      ++row.velocity_factor_count;
      ++request_.run_summary->vertical_jump_bias_velocity_factor_count;

      request_.graph->add(factor::VerticalPositionVelocityConsistencyFactor(
        symbol::X(record.state_index_i),
        symbol::V(record.state_index_i),
        symbol::X(record.state_index_j),
        symbol::V(record.state_index_j),
        dt_s,
        position_velocity_noise));
      ++row.position_velocity_factor_count;
      ++request_.run_summary->vertical_jump_bias_position_velocity_factor_count;
    }

    row.factor_added = true;
    row.skip_reason = "ADDED";
    request_.diagnostics->push_back(row);
  }
}

std::vector<VerticalJumpBiasConstraintBuilder::Span>
VerticalJumpBiasConstraintBuilder::BuildMergedSpans() const {
  std::vector<Span> spans;
  spans.reserve(request_.jump_windows->size());
  for (std::size_t window_index = 0; window_index < request_.jump_windows->size(); ++window_index) {
    const auto &window = (*request_.jump_windows)[window_index];
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s)) {
      continue;
    }
    Span span;
    span.start_time_s = window.start_time_s - request_.config->vertical_jump_bias_padding_s;
    span.end_time_s = window.end_time_s + request_.config->vertical_jump_bias_padding_s;
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
  }
  return merged_spans;
}

std::vector<std::size_t> VerticalJumpBiasConstraintBuilder::StateIndicesInWindow(
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

std::vector<VerticalJumpBiasConstraintBuilder::MatchedInterval>
VerticalJumpBiasConstraintBuilder::FindMatchedIntervals(const Span &span) const {
  std::vector<MatchedInterval> matched_intervals;
  for (const auto &interval : *request_.imu_intervals) {
    if (!IntervalsHavePositiveOverlap(
          interval.start_time_s,
          interval.end_time_s,
          span.start_time_s,
          span.end_time_s)) {
      continue;
    }
    const auto *record = FindPropagationRecord(interval.state_index_i, interval.state_index_j);
    if (record == nullptr) {
      return {};
    }
    matched_intervals.push_back(MatchedInterval{&interval, record});
  }
  return matched_intervals;
}

const VerticalVelocityDeltaPropagationRecord *VerticalJumpBiasConstraintBuilder::FindPropagationRecord(
  const std::size_t state_i,
  const std::size_t state_j) const {
  for (const auto &record : *request_.propagation_records) {
    if (record.state_index_i == state_i && record.state_index_j == state_j) {
      return &record;
    }
  }
  return nullptr;
}

void PopulateVerticalJumpBiasDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalJumpBiasDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const double start_vz_mps =
      optimized_values.at<gtsam::Vector3>(symbol::V(row.start_state_index)).z();
    const double end_vz_mps =
      optimized_values.at<gtsam::Vector3>(symbol::V(row.end_state_index)).z();
    row.optimized_delta_vz_mps = end_vz_mps - start_vz_mps;
    row.estimated_bias_mps2 = optimized_values.at<double>(JumpBiasKey(row.span_index));
    row.corrected_delta_vz_mps = row.imu_delta_vz_mps - row.estimated_bias_mps2 * row.factor_duration_s;
    row.residual_mps = row.optimized_delta_vz_mps - row.corrected_delta_vz_mps;
  }
}

}  // namespace offline_lc_minimal
