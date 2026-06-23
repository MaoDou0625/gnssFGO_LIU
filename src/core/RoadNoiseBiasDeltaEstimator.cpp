#include "offline_lc_minimal/core/RoadNoiseBiasDeltaEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kMadToSigmaScale = 1.4826;

bool IsFiniteSegment(const BodyZBiasReestimateSegmentRow &segment) {
  return std::isfinite(segment.start_time_s) &&
         std::isfinite(segment.end_time_s) &&
         segment.end_time_s > segment.start_time_s + kTimeEpsilonS;
}

bool IsFiniteInterval(const VerticalVelocityDeltaPropagationRecord &record) {
  return std::isfinite(record.start_time_s) &&
         std::isfinite(record.end_time_s) &&
         record.end_time_s > record.start_time_s + kTimeEpsilonS &&
         std::isfinite(record.target_delta_vz_mps);
}

bool ContainsRecordMidpoint(
  const BodyZBiasReestimateSegmentRow &segment,
  const VerticalVelocityDeltaPropagationRecord &record) {
  if (!IsFiniteSegment(segment) || !IsFiniteInterval(record)) {
    return false;
  }
  const double midpoint_time_s = 0.5 * (record.start_time_s + record.end_time_s);
  return segment.start_time_s <= midpoint_time_s + kTimeEpsilonS &&
         midpoint_time_s <= segment.end_time_s + kTimeEpsilonS;
}

double Median(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle_index = values.size() / 2U;
  if (values.size() % 2U == 1U) {
    return values[middle_index];
  }
  return 0.5 * (values[middle_index - 1U] + values[middle_index]);
}

double Mean(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum = 0.0;
  for (const double value : values) {
    sum += value;
  }
  return sum / static_cast<double>(values.size());
}

double TrimmedMean(std::vector<double> values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  if (values.size() < 10U) {
    return Mean(values);
  }
  std::sort(values.begin(), values.end());
  const std::size_t trim_count = values.size() / 10U;
  const auto begin = values.begin() + static_cast<std::ptrdiff_t>(trim_count);
  const auto end = values.end() - static_cast<std::ptrdiff_t>(trim_count);
  return Mean(std::vector<double>(begin, end));
}

std::vector<double> EquivalentAccelerations(
  const BodyZBiasReestimateSegmentRow &segment,
  const std::vector<VerticalVelocityDeltaPropagationRecord> &propagation_records) {
  std::vector<double> values;
  values.reserve(propagation_records.size());
  for (const auto &record : propagation_records) {
    if (!ContainsRecordMidpoint(segment, record)) {
      continue;
    }
    const double dt_s = record.end_time_s - record.start_time_s;
    const double equivalent_acc_mps2 = record.target_delta_vz_mps / dt_s;
    if (!std::isfinite(equivalent_acc_mps2)) {
      continue;
    }
    values.push_back(equivalent_acc_mps2);
  }
  return values;
}

std::vector<double> RejectOutliersByMad(
  const std::vector<double> &values,
  const double mad_scale) {
  if (values.empty()) {
    return {};
  }
  const double median = Median(values);
  if (!std::isfinite(median)) {
    return {};
  }

  std::vector<double> deviations;
  deviations.reserve(values.size());
  for (const double value : values) {
    deviations.push_back(std::abs(value - median));
  }
  const double mad = Median(std::move(deviations));
  if (!std::isfinite(mad)) {
    return {};
  }

  const double threshold =
    std::max(kTimeEpsilonS, std::max(0.0, mad_scale) * kMadToSigmaScale * mad);
  std::vector<double> filtered;
  filtered.reserve(values.size());
  for (const double value : values) {
    if (std::abs(value - median) <= threshold) {
      filtered.push_back(value);
    }
  }
  return filtered;
}

double ClampBiasDelta(const double bias_delta_mps2, const double max_abs_mps2) {
  if (!std::isfinite(bias_delta_mps2)) {
    return 0.0;
  }
  if (!std::isfinite(max_abs_mps2) || max_abs_mps2 <= 0.0) {
    return bias_delta_mps2;
  }
  return std::clamp(bias_delta_mps2, -max_abs_mps2, max_abs_mps2);
}

}  // namespace

RoadNoiseBiasDeltaEstimate EstimateRoadNoiseBiasDelta(
  const BodyZBiasReestimateSegmentRow &segment,
  const std::vector<VerticalVelocityDeltaPropagationRecord> &propagation_records,
  const RoadNoiseBiasDeltaEstimateOptions &options) {
  RoadNoiseBiasDeltaEstimate result;
  if (!IsFiniteSegment(segment) || propagation_records.empty()) {
    return result;
  }

  const std::vector<double> candidate_values =
    EquivalentAccelerations(segment, propagation_records);
  result.candidate_record_count = candidate_values.size();
  if (candidate_values.size() < options.min_record_count) {
    return result;
  }

  const std::vector<double> filtered_values =
    RejectOutliersByMad(candidate_values, options.mad_scale);
  result.used_record_count = filtered_values.size();
  if (filtered_values.size() < options.min_record_count) {
    return result;
  }

  result.bias_delta_mps2 = ClampBiasDelta(
    TrimmedMean(filtered_values),
    options.max_abs_bias_delta_mps2);
  result.estimated = std::isfinite(result.bias_delta_mps2);
  if (!result.estimated) {
    result.bias_delta_mps2 = 0.0;
  }
  return result;
}

}  // namespace offline_lc_minimal
