#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalVelocityDeltaFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-6;

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s && right_start_s <= left_end_s;
}

VerticalVelocityDeltaDiagnosticRow MakeDiagnosticRow(
  const VerticalVelocityDeltaPropagationRecord &record,
  const double sigma_mps,
  const double target_delta_vz_mps) {
  VerticalVelocityDeltaDiagnosticRow row;
  row.state_index_i = record.state_index_i;
  row.state_index_j = record.state_index_j;
  row.start_time_s = record.start_time_s;
  row.end_time_s = record.end_time_s;
  row.dt_s = record.end_time_s - record.start_time_s;
  row.raw_target_delta_vz_mps = record.target_delta_vz_mps;
  row.target_delta_vz_mps = target_delta_vz_mps;
  row.target_clamped =
    std::isfinite(row.raw_target_delta_vz_mps) &&
    std::isfinite(row.target_delta_vz_mps) &&
    std::abs(row.raw_target_delta_vz_mps - row.target_delta_vz_mps) > 1.0e-12;
  row.sigma_mps = sigma_mps;
  return row;
}

}  // namespace

VerticalMotionConstraintBuilder::VerticalMotionConstraintBuilder(VerticalMotionConstraintBuildRequest request)
    : request_(std::move(request)) {}

void VerticalMotionConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.propagation_records == nullptr ||
      request_.jump_windows == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr) {
    throw std::runtime_error("VerticalMotionConstraintBuilder received an incomplete request");
  }

  request_.diagnostics->reserve(request_.diagnostics->size() + request_.propagation_records->size());
  for (const auto &record : *request_.propagation_records) {
    const double dt_s = record.end_time_s - record.start_time_s;
    VerticalVelocityDeltaDiagnosticRow row =
      MakeDiagnosticRow(record, SigmaMps(dt_s), TargetDeltaVzMps(record, dt_s));

    if (!request_.config->enable_vertical_velocity_delta_constraint) {
      row.skip_reason = "DISABLED";
      ++request_.run_summary->vertical_velocity_delta_skipped_disabled_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (record.state_index_j < request_.dynamic_start_index) {
      row.skip_reason = "STATIC_INTERIOR";
      ++request_.run_summary->vertical_velocity_delta_skipped_static_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (dt_s <= 0.0 || !std::isfinite(dt_s) || !std::isfinite(record.target_delta_vz_mps)) {
      row.skip_reason = "INVALID_INTERVAL";
      ++request_.run_summary->vertical_velocity_delta_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (request_.gnss_support_end_time_s.has_value() &&
        record.end_time_s > *request_.gnss_support_end_time_s + kTimeEpsilonS) {
      row.skip_reason = "OUTSIDE_GNSS_SUPPORT";
      ++request_.run_summary->vertical_velocity_delta_skipped_gnss_support_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (OverlapsJumpPadding(record.start_time_s, record.end_time_s)) {
      row.in_jump_padding = true;
      row.skip_reason = "JUMP_PADDING";
      ++request_.run_summary->vertical_velocity_delta_skipped_jump_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    row.factor_added = true;
    row.skip_reason = "ADDED";
    ++request_.run_summary->vertical_velocity_delta_factor_count;
    if (row.target_clamped) {
      ++request_.run_summary->vertical_velocity_delta_target_clamped_count;
    }
    request_.graph->add(factor::VerticalVelocityDeltaFactor(
      symbol::V(record.state_index_i),
      symbol::V(record.state_index_j),
      row.target_delta_vz_mps,
      gtsam::noiseModel::Isotropic::Sigma(1, row.sigma_mps)));
    request_.diagnostics->push_back(row);
  }
}

bool VerticalMotionConstraintBuilder::OverlapsJumpPadding(
  const double start_time_s,
  const double end_time_s) const {
  const double padding_s = request_.config->vertical_velocity_delta_jump_padding_s;
  for (const auto &window : *request_.jump_windows) {
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s)) {
      continue;
    }
    const double padded_start_s = window.start_time_s - padding_s;
    const double padded_end_s = window.end_time_s + padding_s;
    if (IntervalsOverlap(start_time_s, end_time_s, padded_start_s, padded_end_s)) {
      return true;
    }
  }
  return false;
}

double VerticalMotionConstraintBuilder::SigmaMps(const double dt_s) const {
  return std::max(
    request_.config->vertical_velocity_delta_min_sigma_mps,
    request_.config->vertical_velocity_delta_acc_sigma_mps2 * std::max(dt_s, 0.0));
}

double VerticalMotionConstraintBuilder::TargetDeltaVzMps(
  const VerticalVelocityDeltaPropagationRecord &record,
  const double dt_s) const {
  if (dt_s <= 0.0 || !std::isfinite(dt_s) || !std::isfinite(record.target_delta_vz_mps)) {
    return record.target_delta_vz_mps;
  }
  const double limit_mps =
    request_.config->vertical_velocity_delta_target_acc_limit_mps2 * dt_s;
  return std::clamp(record.target_delta_vz_mps, -limit_mps, limit_mps);
}

void PopulateVerticalVelocityDeltaDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalVelocityDeltaDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const auto velocity_i = optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index_i));
    const auto velocity_j = optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index_j));
    row.optimized_delta_vz_mps = velocity_j.z() - velocity_i.z();
    row.residual_mps = row.optimized_delta_vz_mps - row.target_delta_vz_mps;
  }
}

}  // namespace offline_lc_minimal
