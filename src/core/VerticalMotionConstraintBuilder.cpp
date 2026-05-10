#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"
#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaBiasFactor.h"
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
  const VerticalVelocityDeltaSigmaResult &sigma,
  const double target_delta_vz_mps,
  const int outer_pass,
  const VerticalMotionAdaptiveReweightingDiagnosticRow *stability_entry) {
  VerticalVelocityDeltaDiagnosticRow row;
  row.state_index_i = record.state_index_i;
  row.state_index_j = record.state_index_j;
  row.outer_pass = outer_pass;
  row.start_time_s = record.start_time_s;
  row.end_time_s = record.end_time_s;
  row.dt_s = record.end_time_s - record.start_time_s;
  row.raw_target_delta_vz_mps = record.target_delta_vz_mps;
  row.target_delta_vz_mps = target_delta_vz_mps;
  row.target_clamped =
    std::isfinite(row.raw_target_delta_vz_mps) &&
    std::isfinite(row.target_delta_vz_mps) &&
    std::abs(row.raw_target_delta_vz_mps - row.target_delta_vz_mps) > 1.0e-12;
  row.sigma_mps = sigma.sigma_mps;
  row.sigma_model = sigma.model;
  row.legacy_sigma_mps = sigma.legacy_sigma_mps;
  row.bias_sigma_mps = sigma.bias_sigma_mps;
  row.attitude_sigma_mps = sigma.attitude_sigma_mps;
  row.sigma_floor_mps = sigma.sigma_floor_mps;
  row.sigma_ceiling_mps = sigma.sigma_ceiling_mps;
  row.reference_ba_z_ug = Mps2ToMicroG(record.reference_ba_z_mps2);
  if (stability_entry != nullptr) {
    row.adaptive_motion_score = stability_entry->motion_score;
    row.adaptive_sigma_mps = sigma.sigma_mps;
    row.adaptive_sigma_ratio =
      stability_entry->dvz_sigma_before_mps > 0.0
        ? sigma.sigma_mps / stability_entry->dvz_sigma_before_mps
        : std::numeric_limits<double>::quiet_NaN();
    row.local_horizontal_speed_rms_mps = stability_entry->horizontal_speed_rms_mps;
    row.local_vz_rms_mps = stability_entry->vz_rms_mps;
    row.local_vz_range_mps = stability_entry->vz_range_mps;
    row.local_target_acc_rms_mps2 = stability_entry->target_vertical_acc_rms_mps2;
  }
  return row;
}

bool ApplyClampedTargetSigmaFallback(VerticalVelocityDeltaDiagnosticRow &row) {
  if (!row.target_clamped ||
      row.sigma_model == "legacy" ||
      row.sigma_model == "legacy_clamped_target") {
    return false;
  }
  row.sigma_model = "legacy_clamped_target";
  row.sigma_mps = row.legacy_sigma_mps;
  if (std::isfinite(row.adaptive_motion_score)) {
    row.adaptive_sigma_mps = row.sigma_mps;
    row.adaptive_sigma_ratio =
      row.legacy_sigma_mps > 0.0
        ? row.sigma_mps / row.legacy_sigma_mps
        : std::numeric_limits<double>::quiet_NaN();
  }
  return true;
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

  VerticalVelocityDeltaSigmaModel sigma_model(*request_.config);
  request_.run_summary->vertical_velocity_delta_bias_consistent_sigma_enabled =
    request_.config->enable_vertical_velocity_delta_bias_consistent_sigma;
  request_.run_summary->vertical_velocity_delta_bias_aware_target_enabled =
    request_.config->enable_vertical_velocity_delta_bias_aware_target;
  double added_sigma_sum_mps = 0.0;
  double added_sigma_max_mps = -std::numeric_limits<double>::infinity();
  const std::vector<BodyZJumpConstraintWindow> jump_constraint_windows =
    BuildBodyZJumpConstraintWindows(
      *request_.jump_windows,
      VerticalVelocityDeltaJumpConstraintWindowOptions(*request_.config));

  request_.diagnostics->reserve(request_.diagnostics->size() + request_.propagation_records->size());
  for (const auto &record : *request_.propagation_records) {
    const double dt_s = record.end_time_s - record.start_time_s;
    const auto *stability_entry = FindStabilityProfileEntry(
      request_.stability_profile,
      record.state_index_i,
      record.state_index_j);
    const VerticalVelocityDeltaSigmaResult sigma = sigma_model.Compute(dt_s, stability_entry);
    VerticalVelocityDeltaDiagnosticRow row =
      MakeDiagnosticRow(record, sigma, TargetDeltaVzMps(record, dt_s), request_.outer_pass, stability_entry);
    const bool used_clamped_target_fallback = ApplyClampedTargetSigmaFallback(row);

    if (!request_.config->enable_vertical_velocity_delta_constraint) {
      row.skip_reason = "DISABLED";
      ++request_.run_summary->vertical_velocity_delta_skipped_disabled_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    const bool static_interior = record.state_index_j < request_.dynamic_start_index;
    if (static_interior && !request_.config->enable_vertical_velocity_delta_initial_static_constraint) {
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
    if (OverlapsJumpPadding(record.start_time_s, record.end_time_s, jump_constraint_windows)) {
      row.in_jump_padding = true;
      row.skip_reason = "JUMP_PADDING";
      ++request_.run_summary->vertical_velocity_delta_skipped_jump_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    row.factor_added = true;
    row.bias_aware_factor = request_.config->enable_vertical_velocity_delta_bias_aware_target;
    row.skip_reason = "ADDED";
    ++request_.run_summary->vertical_velocity_delta_factor_count;
    if (static_interior) {
      ++request_.run_summary->vertical_velocity_delta_static_factor_count;
    }
    if (row.bias_aware_factor) {
      ++request_.run_summary->vertical_velocity_delta_bias_aware_factor_count;
    }
    added_sigma_sum_mps += row.sigma_mps;
    added_sigma_max_mps = std::max(added_sigma_max_mps, row.sigma_mps);
    if (!used_clamped_target_fallback && sigma.clamped_floor) {
      ++request_.run_summary->vertical_velocity_delta_sigma_clamped_floor_count;
    }
    if (!used_clamped_target_fallback && sigma.clamped_ceiling) {
      ++request_.run_summary->vertical_velocity_delta_sigma_clamped_ceiling_count;
    }
    if (row.target_clamped) {
      ++request_.run_summary->vertical_velocity_delta_target_clamped_count;
    }
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, row.sigma_mps);
    if (row.bias_aware_factor) {
      request_.graph->add(factor::VerticalVelocityDeltaBiasFactor(
        symbol::V(record.state_index_i),
        symbol::V(record.state_index_j),
        symbol::B(record.state_index_i),
        row.target_delta_vz_mps,
        record.reference_ba_z_mps2,
        dt_s,
        noise));
    } else {
      request_.graph->add(factor::VerticalVelocityDeltaFactor(
        symbol::V(record.state_index_i),
        symbol::V(record.state_index_j),
        row.target_delta_vz_mps,
        noise));
    }
    request_.diagnostics->push_back(row);
  }

  if (request_.run_summary->vertical_velocity_delta_factor_count > 0U) {
    request_.run_summary->vertical_velocity_delta_sigma_mean_mps =
      added_sigma_sum_mps / static_cast<double>(request_.run_summary->vertical_velocity_delta_factor_count);
    request_.run_summary->vertical_velocity_delta_sigma_max_mps = added_sigma_max_mps;
  }
}

bool VerticalMotionConstraintBuilder::OverlapsJumpPadding(
  const double start_time_s,
  const double end_time_s,
  const std::vector<BodyZJumpConstraintWindow> &jump_constraint_windows) const {
  for (const auto &window : jump_constraint_windows) {
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      return true;
    }
  }
  return false;
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
    if (row.bias_aware_factor) {
      const auto bias_i = optimized_values.at<gtsam::imuBias::ConstantBias>(symbol::B(row.state_index_i));
      const double optimized_ba_z_mps2 = bias_i.accelerometer().z();
      const double reference_ba_z_mps2 = MicroGToMps2(row.reference_ba_z_ug);
      row.optimized_ba_z_ug = Mps2ToMicroG(optimized_ba_z_mps2);
      row.bias_delta_ug = Mps2ToMicroG(optimized_ba_z_mps2 - reference_ba_z_mps2);
      row.bias_delta_velocity_correction_mps =
        (optimized_ba_z_mps2 - reference_ba_z_mps2) * row.dt_s;
      row.residual_mps =
        row.optimized_delta_vz_mps - row.target_delta_vz_mps +
        row.bias_delta_velocity_correction_mps;
    } else {
      row.residual_mps = row.optimized_delta_vz_mps - row.target_delta_vz_mps;
    }
  }
}

}  // namespace offline_lc_minimal
