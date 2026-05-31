#include "offline_lc_minimal/core/Stage3VerticalReferenceSmoother.h"

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace offline_lc_minimal {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool CanSmoothRow(const TrajectoryRow &row) {
  return std::isfinite(row.time_s) && std::isfinite(row.enu_position_m.z());
}

double LowpassAlpha(const double dt_s, const double tau_s) {
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return 0.0;
  }
  return 1.0 - std::exp(-dt_s / tau_s);
}

void FilterSegment(
  const OfflineRunnerConfig &config,
  const std::vector<std::size_t> &indices,
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &input_up_m,
  std::vector<double> &lowpass_up_m) {
  if (indices.empty()) {
    return;
  }

  const double tau_s = 1.0 / (kTwoPi * config.stage3_vertical_reference_lowpass_cutoff_hz);
  std::vector<double> forward(indices.size(), 0.0);
  forward.front() = input_up_m[indices.front()];

  for (std::size_t i = 1; i < indices.size(); ++i) {
    const auto &prev_row = trajectory[indices[i - 1U]];
    const auto &row = trajectory[indices[i]];
    const double alpha = LowpassAlpha(row.time_s - prev_row.time_s, tau_s);
    forward[i] = forward[i - 1U] + alpha * (input_up_m[indices[i]] - forward[i - 1U]);
  }

  std::vector<double> zero_phase = forward;
  for (std::size_t reverse = indices.size() - 1U; reverse > 0U; --reverse) {
    const std::size_t i = reverse - 1U;
    const auto &row = trajectory[indices[i]];
    const auto &next_row = trajectory[indices[i + 1U]];
    const double alpha = LowpassAlpha(next_row.time_s - row.time_s, tau_s);
    zero_phase[i] = zero_phase[i + 1U] + alpha * (forward[i] - zero_phase[i + 1U]);
  }

  for (std::size_t i = 0; i < indices.size(); ++i) {
    lowpass_up_m[indices[i]] = zero_phase[i];
  }
}

std::vector<double> BuildLowpassProfile(
  const OfflineRunnerConfig &config,
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &input_up_m,
  const std::size_t first_filter_index,
  const std::size_t one_past_last_filter_index) {
  std::vector<double> lowpass_up_m(
    trajectory.size(),
    std::numeric_limits<double>::quiet_NaN());
  std::vector<std::size_t> segment;
  segment.reserve(trajectory.size());
  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    if (index < first_filter_index || index >= one_past_last_filter_index) {
      if (CanSmoothRow(trajectory[index]) && std::isfinite(input_up_m[index])) {
        lowpass_up_m[index] = input_up_m[index];
      }
      continue;
    }
    if (!CanSmoothRow(trajectory[index]) || !std::isfinite(input_up_m[index])) {
      FilterSegment(config, segment, trajectory, input_up_m, lowpass_up_m);
      segment.clear();
      continue;
    }
    if (!segment.empty()) {
      const auto &prev_row = trajectory[segment.back()];
      const auto &row = trajectory[index];
      if (row.time_s <= prev_row.time_s) {
        FilterSegment(config, segment, trajectory, input_up_m, lowpass_up_m);
        segment.clear();
      }
    }
    segment.push_back(index);
  }
  FilterSegment(config, segment, trajectory, input_up_m, lowpass_up_m);
  return lowpass_up_m;
}

std::vector<double> BuildCumulativeStation(
  const std::vector<TrajectoryRow> &trajectory) {
  std::vector<double> station_m(
    trajectory.size(),
    std::numeric_limits<double>::quiet_NaN());
  if (trajectory.empty()) {
    return station_m;
  }
  station_m.front() = 0.0;
  for (std::size_t index = 1; index < trajectory.size(); ++index) {
    if (!trajectory[index - 1U].enu_position_m.allFinite() ||
        !trajectory[index].enu_position_m.allFinite() ||
        !std::isfinite(station_m[index - 1U])) {
      station_m[index] = station_m[index - 1U];
      continue;
    }
    const double step_m = std::hypot(
      trajectory[index].enu_position_m.x() - trajectory[index - 1U].enu_position_m.x(),
      trajectory[index].enu_position_m.y() - trajectory[index - 1U].enu_position_m.y());
    station_m[index] = station_m[index - 1U] + (std::isfinite(step_m) ? step_m : 0.0);
  }
  return station_m;
}

double NearestFiniteValue(
  const std::vector<double> &values,
  const std::size_t start_index,
  const int direction) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::ptrdiff_t index = static_cast<std::ptrdiff_t>(start_index);
  while (index >= 0 && index < static_cast<std::ptrdiff_t>(values.size())) {
    const double value = values[static_cast<std::size_t>(index)];
    if (std::isfinite(value)) {
      return value;
    }
    index += direction;
  }
  return std::numeric_limits<double>::quiet_NaN();
}

double StartBoundaryValue(
  const std::vector<double> &input_up_m,
  const std::size_t first_filter_index) {
  if (first_filter_index > 0U) {
    const double value =
      NearestFiniteValue(input_up_m, first_filter_index - 1U, -1);
    if (std::isfinite(value)) {
      return value;
    }
  }
  return NearestFiniteValue(input_up_m, first_filter_index, 1);
}

double EndBoundaryValue(
  const std::vector<double> &input_up_m,
  const std::size_t last_filter_index) {
  if (last_filter_index + 1U < input_up_m.size()) {
    const double value =
      NearestFiniteValue(input_up_m, last_filter_index + 1U, 1);
    if (std::isfinite(value)) {
      return value;
    }
  }
  return NearestFiniteValue(input_up_m, last_filter_index, -1);
}

std::array<double, 4U> CubicBasis(const double u) {
  const double one_minus_u = 1.0 - u;
  const double u2 = u * u;
  const double u3 = u2 * u;
  return {
    one_minus_u * one_minus_u * one_minus_u / 6.0,
    (3.0 * u3 - 6.0 * u2 + 4.0) / 6.0,
    (-3.0 * u3 + 3.0 * u2 + 3.0 * u + 1.0) / 6.0,
    u3 / 6.0};
}

std::array<double, 4U> CubicBasisDerivative(const double u, const double knot_spacing_m) {
  const double u2 = u * u;
  return {
    -0.5 * (1.0 - u) * (1.0 - u) / knot_spacing_m,
    (1.5 * u2 - 2.0 * u) / knot_spacing_m,
    (-1.5 * u2 + u + 0.5) / knot_spacing_m,
    0.5 * u2 / knot_spacing_m};
}

struct BasisRow {
  std::size_t first_control_index = 0U;
  std::array<double, 4U> weights{};
};

BasisRow BuildBasisRow(
  const double local_station_m,
  const double knot_spacing_m,
  const std::size_t control_count) {
  const double t = std::max(0.0, local_station_m / knot_spacing_m);
  std::size_t span = static_cast<std::size_t>(std::floor(t));
  if (span + 3U >= control_count) {
    span = control_count - 4U;
  }
  const double u = std::clamp(t - static_cast<double>(span), 0.0, 1.0);
  return BasisRow{span, CubicBasis(u)};
}

BasisRow BuildDerivativeBasisRow(
  const double local_station_m,
  const double knot_spacing_m,
  const std::size_t control_count) {
  const double t = std::max(0.0, local_station_m / knot_spacing_m);
  std::size_t span = static_cast<std::size_t>(std::floor(t));
  if (span + 3U >= control_count) {
    span = control_count - 4U;
  }
  const double u = std::clamp(t - static_cast<double>(span), 0.0, 1.0);
  return BasisRow{span, CubicBasisDerivative(u, knot_spacing_m)};
}

void AddWeightedBasisTerm(
  const BasisRow &basis,
  const double weight,
  const double target,
  std::vector<Eigen::Triplet<double>> &normal_terms,
  Eigen::VectorXd &rhs) {
  if (weight <= 0.0 || !std::isfinite(weight) || !std::isfinite(target)) {
    return;
  }
  for (std::size_t a = 0; a < 4U; ++a) {
    const std::size_t row = basis.first_control_index + a;
    const double wa = weight * basis.weights[a];
    rhs(static_cast<Eigen::Index>(row)) += wa * target;
    for (std::size_t b = 0; b < 4U; ++b) {
      const std::size_t col = basis.first_control_index + b;
      normal_terms.emplace_back(
        static_cast<Eigen::Index>(row),
        static_cast<Eigen::Index>(col),
        wa * basis.weights[b]);
    }
  }
}

double EvaluateBasis(
  const BasisRow &basis,
  const Eigen::VectorXd &coefficients) {
  double value = 0.0;
  for (std::size_t i = 0; i < 4U; ++i) {
    value += basis.weights[i] *
             coefficients(static_cast<Eigen::Index>(basis.first_control_index + i));
  }
  return value;
}

void AddSecondDifferencePenalty(
  const double lambda,
  const std::size_t control_count,
  std::vector<Eigen::Triplet<double>> &normal_terms) {
  if (lambda <= 0.0 || !std::isfinite(lambda) || control_count < 3U) {
    return;
  }
  for (std::size_t i = 0; i + 2U < control_count; ++i) {
    const std::array<Eigen::Index, 3U> columns{
      static_cast<Eigen::Index>(i),
      static_cast<Eigen::Index>(i + 1U),
      static_cast<Eigen::Index>(i + 2U)};
    const std::array<double, 3U> weights{1.0, -2.0, 1.0};
    for (std::size_t a = 0; a < columns.size(); ++a) {
      for (std::size_t b = 0; b < columns.size(); ++b) {
        normal_terms.emplace_back(
          columns[a],
          columns[b],
          lambda * weights[a] * weights[b]);
      }
    }
  }
}

void AddEndpointSlopePenalty(
  const std::vector<double> &station_m,
  const std::vector<std::size_t> &indices,
  const double knot_spacing_m,
  const double slope_weight,
  const std::size_t control_count,
  std::vector<Eigen::Triplet<double>> &normal_terms) {
  if (slope_weight <= 0.0 || !std::isfinite(slope_weight) || indices.empty()) {
    return;
  }
  const double start_station_m = station_m[indices.front()];
  const double end_station_m = station_m[indices.back()];
  for (const double station : {start_station_m, end_station_m}) {
    const BasisRow derivative =
      BuildDerivativeBasisRow(station - start_station_m, knot_spacing_m, control_count);
    for (std::size_t a = 0; a < 4U; ++a) {
      const std::size_t row = derivative.first_control_index + a;
      const double wa = slope_weight * derivative.weights[a];
      for (std::size_t b = 0; b < 4U; ++b) {
        const std::size_t col = derivative.first_control_index + b;
        normal_terms.emplace_back(
          static_cast<Eigen::Index>(row),
          static_cast<Eigen::Index>(col),
          wa * derivative.weights[b]);
      }
    }
  }
}

void FitSplineSegment(
  const OfflineRunnerConfig &config,
  const std::vector<double> &station_m,
  const std::vector<double> &input_up_m,
  const std::vector<std::size_t> &indices,
  std::vector<double> &output_up_m) {
  if (indices.size() < 4U) {
    for (const std::size_t index : indices) {
      output_up_m[index] = input_up_m[index];
    }
    return;
  }
  const double start_station_m = station_m[indices.front()];
  const double end_station_m = station_m[indices.back()];
  const double station_range_m = end_station_m - start_station_m;
  if (!std::isfinite(station_range_m) || station_range_m <= 1.0e-6) {
    for (const std::size_t index : indices) {
      output_up_m[index] = input_up_m[index];
    }
    return;
  }

  const double knot_spacing_m =
    std::max(1.0e-6, config.stage3_vertical_reference_spline_knot_spacing_m);
  const std::size_t control_count =
    static_cast<std::size_t>(std::ceil(station_range_m / knot_spacing_m)) + 4U;
  std::vector<Eigen::Triplet<double>> normal_terms;
  normal_terms.reserve(indices.size() * 16U + control_count * 10U + 64U);
  Eigen::VectorXd rhs =
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(control_count));

  for (const std::size_t index : indices) {
    const BasisRow basis =
      BuildBasisRow(station_m[index] - start_station_m, knot_spacing_m, control_count);
    AddWeightedBasisTerm(basis, 1.0, input_up_m[index], normal_terms, rhs);
  }

  AddSecondDifferencePenalty(
    config.stage3_vertical_reference_spline_smooth_lambda,
    control_count,
    normal_terms);

  const double start_target = StartBoundaryValue(input_up_m, indices.front());
  const double end_target = EndBoundaryValue(input_up_m, indices.back());
  AddWeightedBasisTerm(
    BuildBasisRow(0.0, knot_spacing_m, control_count),
    config.stage3_vertical_reference_spline_anchor_weight,
    start_target,
    normal_terms,
    rhs);
  AddWeightedBasisTerm(
    BuildBasisRow(station_range_m, knot_spacing_m, control_count),
    config.stage3_vertical_reference_spline_anchor_weight,
    end_target,
    normal_terms,
    rhs);
  AddEndpointSlopePenalty(
    station_m,
    indices,
    knot_spacing_m,
    config.stage3_vertical_reference_spline_slope_weight,
    control_count,
    normal_terms);

  for (std::size_t index = 0; index < control_count; ++index) {
    normal_terms.emplace_back(
      static_cast<Eigen::Index>(index),
      static_cast<Eigen::Index>(index),
      1.0e-12);
  }
  Eigen::SparseMatrix<double> normal(
    static_cast<Eigen::Index>(control_count),
    static_cast<Eigen::Index>(control_count));
  normal.setFromTriplets(normal_terms.begin(), normal_terms.end());
  Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
  solver.compute(normal);
  if (solver.info() != Eigen::Success) {
    throw std::runtime_error("Stage3 spline baseline sparse factorization failed");
  }
  const Eigen::VectorXd coefficients =
    solver.solve(rhs);
  if (solver.info() != Eigen::Success || !coefficients.allFinite()) {
    throw std::runtime_error("Stage3 spline baseline solve produced non-finite coefficients");
  }
  for (const std::size_t index : indices) {
    const BasisRow basis =
      BuildBasisRow(station_m[index] - start_station_m, knot_spacing_m, control_count);
    output_up_m[index] = EvaluateBasis(basis, coefficients);
  }
}

std::vector<double> BuildSplineBaselineProfile(
  const OfflineRunnerConfig &config,
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &input_up_m,
  const std::size_t first_filter_index,
  const std::size_t one_past_last_filter_index) {
  std::vector<double> output_up_m(
    trajectory.size(),
    std::numeric_limits<double>::quiet_NaN());
  const std::vector<double> station_m = BuildCumulativeStation(trajectory);
  std::vector<std::size_t> segment;
  segment.reserve(trajectory.size());

  for (std::size_t index = 0; index < trajectory.size(); ++index) {
    if (index < first_filter_index || index >= one_past_last_filter_index) {
      if (CanSmoothRow(trajectory[index]) && std::isfinite(input_up_m[index])) {
        output_up_m[index] = input_up_m[index];
      }
      continue;
    }
    if (!CanSmoothRow(trajectory[index]) ||
        !std::isfinite(input_up_m[index]) ||
        !std::isfinite(station_m[index])) {
      FitSplineSegment(
        config,
        station_m,
        input_up_m,
        segment,
        output_up_m);
      segment.clear();
      continue;
    }
    if (!segment.empty() && station_m[index] + 1.0e-9 < station_m[segment.back()]) {
      FitSplineSegment(
        config,
        station_m,
        input_up_m,
        segment,
        output_up_m);
      segment.clear();
    }
    segment.push_back(index);
  }
  FitSplineSegment(
    config,
    station_m,
    input_up_m,
    segment,
    output_up_m);
  return output_up_m;
}

}  // namespace

std::vector<double> BuildStage3VerticalReferenceSmoothedProfile(
  const OfflineRunnerConfig &config,
  const std::vector<TrajectoryRow> &trajectory,
  const std::vector<double> &input_up_m,
  const std::size_t first_filter_index,
  const std::size_t one_past_last_filter_index) {
  switch (config.stage3_vertical_reference_smoothing_method) {
    case Stage3VerticalReferenceSmoothingMethod::kSplineBaseline:
      return BuildSplineBaselineProfile(
        config,
        trajectory,
        input_up_m,
        first_filter_index,
        one_past_last_filter_index);
    case Stage3VerticalReferenceSmoothingMethod::kLowpass:
    default:
      return BuildLowpassProfile(
        config,
        trajectory,
        input_up_m,
        first_filter_index,
        one_past_last_filter_index);
  }
}

}  // namespace offline_lc_minimal
