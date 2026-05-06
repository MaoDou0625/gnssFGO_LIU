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

double PositiveOverlapDurationS(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return std::max(0.0, std::min(left_end_s, right_end_s) - std::max(left_start_s, right_start_s));
}

double VelocitySigmaMps(
  const OfflineRunnerConfig &config,
  const VerticalJumpBiasSegmentEstimate &segment,
  const double dt_s,
  double *highfreq_inflation_mps) {
  double inflation_mps = 0.0;
  if (std::isfinite(segment.highfreq_rms_mps2) &&
      std::isfinite(dt_s) &&
      dt_s > 0.0 &&
      config.vertical_jump_bias_highfreq_sigma_scale > 0.0) {
    inflation_mps =
      config.vertical_jump_bias_highfreq_sigma_scale *
      segment.highfreq_rms_mps2 *
      std::sqrt(dt_s);
    inflation_mps = std::min(inflation_mps, config.vertical_jump_bias_highfreq_sigma_max_mps);
  }
  if (highfreq_inflation_mps != nullptr) {
    *highfreq_inflation_mps = inflation_mps;
  }
  const double base_sigma_mps = config.vertical_jump_bias_velocity_sigma_mps;
  return std::sqrt(base_sigma_mps * base_sigma_mps + inflation_mps * inflation_mps);
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
  const std::vector<VerticalJumpBiasSpanInput> segmenter_inputs = BuildSegmenterInputs(spans);
  const std::vector<VerticalJumpBiasSegmentEstimate> segments = EstimateVerticalJumpBiasSegments(
    VerticalJumpBiasSegmenterRequest{
      request_.config,
      request_.jump_windows,
      request_.body_z_diagnostics,
      &segmenter_inputs});
  request_.diagnostics->reserve(request_.diagnostics->size() + segments.size());
  const auto position_velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_jump_bias_position_velocity_sigma_m);

  for (const auto &segment : segments) {
    VerticalJumpBiasDiagnosticRow row;
    row.span_index = segment.span_index;
    row.segment_index = segment.segment_index;
    row.segment_count = segment.segment_count;
    row.bias_key_index = segment.bias_key_index;
    row.source_window_index = segment.source_window_index;
    row.source_window_count = segment.source_window_count;
    row.start_time_s = segment.start_time_s;
    row.end_time_s = segment.end_time_s;
    row.source_window_duration_s = segment.source_window_duration_s;
    row.detected_signed_delta_velocity_mps = segment.detected_signed_delta_velocity_mps;
    row.detected_bias_mps2 = segment.detected_bias_mps2;
    row.used_segmented_estimate = segment.used_segmented_estimate;
    row.highfreq_rms_mps2 = segment.highfreq_rms_mps2;
    row.highfreq_p95_abs_mps2 = segment.highfreq_p95_abs_mps2;
    row.prior_sigma_mps2 = request_.config->vertical_jump_bias_prior_sigma_mps2;
    row.base_velocity_sigma_mps = request_.config->vertical_jump_bias_velocity_sigma_mps;
    row.velocity_sigma_mps = row.base_velocity_sigma_mps;
    row.position_velocity_sigma_m = request_.config->vertical_jump_bias_position_velocity_sigma_m;
    const std::vector<std::size_t> segment_state_indices =
      StateIndicesInWindow(row.start_time_s, row.end_time_s);
    if (!segment_state_indices.empty()) {
      row.start_state_index = segment_state_indices.front();
      row.end_state_index = segment_state_indices.back();
    } else {
      bool found_interval_bounds = false;
      for (const auto &interval : *request_.imu_intervals) {
        if (PositiveOverlapDurationS(
              interval.start_time_s,
              interval.end_time_s,
              row.start_time_s,
              row.end_time_s) <= kTimeEpsilonS) {
          continue;
        }
        if (!found_interval_bounds) {
          row.start_state_index = interval.state_index_i;
          found_interval_bounds = true;
        }
        row.end_state_index = interval.state_index_j;
      }
    }

    if (row.source_window_duration_s <= 0.0 || !std::isfinite(row.detected_bias_mps2)) {
      row.skip_reason = "MISSING_DETECTED_BIAS";
      ++request_.run_summary->vertical_jump_bias_skipped_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    const std::vector<MatchedInterval> matched_intervals = FindMatchedIntervals(segment, segments);
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

    const gtsam::Key jump_bias_key = JumpBiasKey(segment.bias_key_index);
    const auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->vertical_jump_bias_prior_sigma_mps2);
    if (!request_.initial_values->exists(jump_bias_key)) {
      request_.initial_values->insert(jump_bias_key, row.detected_bias_mps2);
    }
    request_.graph->add(gtsam::PriorFactor<double>(jump_bias_key, row.detected_bias_mps2, prior_noise));
    ++request_.run_summary->vertical_jump_bias_prior_factor_count;
    ++request_.run_summary->vertical_jump_bias_segment_count;

    for (const auto &matched_interval : matched_intervals) {
      const auto &interval = *matched_interval.imu_interval;
      const auto &record = *matched_interval.propagation_record;
      const double dt_s = record.end_time_s - record.start_time_s;
      double highfreq_inflation_mps = 0.0;
      const double velocity_sigma_mps = VelocitySigmaMps(
        *request_.config,
        segment,
        dt_s,
        &highfreq_inflation_mps);
      const auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(1, velocity_sigma_mps);
      row.highfreq_sigma_inflation_mps =
        std::max(row.highfreq_sigma_inflation_mps, highfreq_inflation_mps);
      row.velocity_sigma_mps = std::max(row.velocity_sigma_mps, velocity_sigma_mps);
      if (highfreq_inflation_mps > kTimeEpsilonS) {
        ++request_.run_summary->vertical_jump_bias_highfreq_inflated_factor_count;
      }

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

std::vector<VerticalJumpBiasSpanInput> VerticalJumpBiasConstraintBuilder::BuildSegmenterInputs(
  const std::vector<Span> &spans) const {
  std::vector<VerticalJumpBiasSpanInput> inputs;
  inputs.reserve(spans.size());
  for (const auto &span : spans) {
    VerticalJumpBiasSpanInput input;
    input.span_index = span.span_index;
    input.start_time_s = span.start_time_s;
    input.end_time_s = span.end_time_s;
    input.source_window_indices = span.source_window_indices;
    inputs.push_back(std::move(input));
  }
  return inputs;
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
VerticalJumpBiasConstraintBuilder::FindMatchedIntervals(
  const VerticalJumpBiasSegmentEstimate &segment,
  const std::vector<VerticalJumpBiasSegmentEstimate> &all_segments) const {
  std::vector<MatchedInterval> matched_intervals;
  for (const auto &interval : *request_.imu_intervals) {
    const double overlap_s = PositiveOverlapDurationS(
      interval.start_time_s,
      interval.end_time_s,
      segment.start_time_s,
      segment.end_time_s);
    if (overlap_s <= kTimeEpsilonS) {
      continue;
    }
    bool assigned_to_another_segment = false;
    for (const auto &other_segment : all_segments) {
      if (other_segment.bias_key_index == segment.bias_key_index ||
          other_segment.span_index != segment.span_index) {
        continue;
      }
      const double other_overlap_s = PositiveOverlapDurationS(
        interval.start_time_s,
        interval.end_time_s,
        other_segment.start_time_s,
        other_segment.end_time_s);
      if (other_overlap_s > overlap_s + kTimeEpsilonS ||
          (std::abs(other_overlap_s - overlap_s) <= kTimeEpsilonS &&
           other_segment.bias_key_index < segment.bias_key_index)) {
        assigned_to_another_segment = true;
        break;
      }
    }
    if (assigned_to_another_segment) {
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
    row.estimated_bias_mps2 = optimized_values.at<double>(JumpBiasKey(row.bias_key_index));
    row.corrected_delta_vz_mps = row.imu_delta_vz_mps - row.estimated_bias_mps2 * row.factor_duration_s;
    row.residual_mps = row.optimized_delta_vz_mps - row.corrected_delta_vz_mps;
  }
}

}  // namespace offline_lc_minimal
