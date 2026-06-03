#include "offline_lc_minimal/core/VerticalMotionStabilityEstimator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>

#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/core/VerticalVelocityDeltaSigmaModel.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s && right_start_s <= left_end_s;
}

double Clamp01(const double value) {
  return std::clamp(value, 0.0, 1.0);
}

double Rms(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double sum_sq = 0.0;
  for (const double value : values) {
    sum_sq += value * value;
  }
  return std::sqrt(sum_sq / static_cast<double>(values.size()));
}

double Range(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const auto [min_it, max_it] = std::minmax_element(values.begin(), values.end());
  return *max_it - *min_it;
}

std::string StabilityClass(const double motion_score, const bool in_jump_padding) {
  if (in_jump_padding) {
    return "JUMP_PADDING";
  }
  if (motion_score < 0.25) {
    return "LOW_MOTION";
  }
  if (motion_score < 0.75) {
    return "MIXED";
  }
  return "DYNAMIC";
}

}  // namespace

VerticalMotionStabilityEstimator::VerticalMotionStabilityEstimator(
  VerticalMotionStabilityEstimateRequest request)
    : request_(std::move(request)) {
  if (request_.config != nullptr && request_.jump_windows != nullptr) {
    jump_constraint_windows_ = BuildBodyZJumpConstraintWindows(
      *request_.jump_windows,
      VerticalVelocityDeltaJumpConstraintWindowOptions(*request_.config));
  }
}

VerticalMotionStabilityProfile VerticalMotionStabilityEstimator::Estimate() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.propagation_records == nullptr || request_.optimized_values == nullptr) {
    throw std::runtime_error("VerticalMotionStabilityEstimator received an incomplete request");
  }

  VerticalVelocityDeltaSigmaModel sigma_model(*request_.config);
  VerticalMotionStabilityProfile profile;
  profile.reserve(request_.propagation_records->size());
  const double half_window_s = 0.5 * request_.config->vertical_motion_adaptive_stability_window_s;

  for (const auto &record : *request_.propagation_records) {
    VerticalMotionAdaptiveReweightingDiagnosticRow row;
    row.outer_pass = request_.outer_pass;
    row.state_index_i = record.state_index_i;
    row.state_index_j = record.state_index_j;
    row.start_time_s = record.start_time_s;
    row.end_time_s = record.end_time_s;
    row.dt_s = record.end_time_s - record.start_time_s;
    row.in_jump_padding = OverlapsJumpPadding(record.start_time_s, record.end_time_s);

    if (row.dt_s <= 0.0 || !std::isfinite(row.dt_s)) {
      row.motion_score = 1.0;
      row.stability_class = "INVALID";
      row.skip_reason = "INVALID_INTERVAL";
      profile.push_back(row);
      continue;
    }

    const double center_time_s = 0.5 * (record.start_time_s + record.end_time_s);
    const double window_start_s = center_time_s - half_window_s;
    const double window_end_s = center_time_s + half_window_s;
    std::vector<double> horizontal_speeds;
    std::vector<double> vz_values;
    std::vector<double> target_acc_values;

    for (const auto &candidate : *request_.propagation_records) {
      const double candidate_center_s = 0.5 * (candidate.start_time_s + candidate.end_time_s);
      if (candidate_center_s < window_start_s || candidate_center_s > window_end_s) {
        continue;
      }
      const double candidate_dt_s = candidate.end_time_s - candidate.start_time_s;
      if (candidate_dt_s > 0.0 && std::isfinite(candidate.target_delta_vz_mps)) {
        target_acc_values.push_back(candidate.target_delta_vz_mps / candidate_dt_s);
      }
      if (request_.optimized_values->exists(symbol::V(candidate.state_index_i))) {
        const auto velocity_i =
          request_.optimized_values->at<gtsam::Vector3>(symbol::V(candidate.state_index_i));
        horizontal_speeds.push_back(std::hypot(velocity_i.x(), velocity_i.y()));
        vz_values.push_back(velocity_i.z());
      }
      if (request_.optimized_values->exists(symbol::V(candidate.state_index_j))) {
        const auto velocity_j =
          request_.optimized_values->at<gtsam::Vector3>(symbol::V(candidate.state_index_j));
        horizontal_speeds.push_back(std::hypot(velocity_j.x(), velocity_j.y()));
        vz_values.push_back(velocity_j.z());
      }
    }

    row.horizontal_speed_rms_mps = Rms(horizontal_speeds);
    row.vz_rms_mps = Rms(vz_values);
    row.vz_range_mps = Range(vz_values);
    row.target_vertical_acc_rms_mps2 = Rms(target_acc_values);

    if (row.in_jump_padding) {
      row.motion_score = 1.0;
      row.skip_reason = "JUMP_PADDING";
    } else if (!std::isfinite(row.horizontal_speed_rms_mps) ||
               !std::isfinite(row.vz_rms_mps) ||
               !std::isfinite(row.target_vertical_acc_rms_mps2)) {
      row.motion_score = 1.0;
      row.skip_reason = "INSUFFICIENT_METRICS";
    } else {
      const double speed_score =
        Clamp01(row.horizontal_speed_rms_mps /
                request_.config->vertical_motion_adaptive_static_horizontal_speed_rms_mps);
      const double vz_score =
        Clamp01(row.vz_rms_mps / request_.config->vertical_motion_adaptive_static_vz_rms_mps);
      const double target_acc_score =
        Clamp01(row.target_vertical_acc_rms_mps2 /
                request_.config->vertical_motion_adaptive_static_target_acc_rms_mps2);
      row.motion_score = std::max({speed_score, vz_score, target_acc_score});
      row.skip_reason = "ADDED";
    }
    row.stability_class = StabilityClass(row.motion_score, row.in_jump_padding);

    const auto base_sigma = sigma_model.Compute(row.dt_s);
    const auto adaptive_sigma = sigma_model.Compute(row.dt_s, &row);
    row.dvz_sigma_before_mps = base_sigma.sigma_mps;
    row.dvz_sigma_after_mps = adaptive_sigma.sigma_mps;

    row.baz_gm_sigma_before_ug = Mps2ToMicroG(
      VerticalAccelBiasGmConstraintBuilder::DrivingSigmaMps2(
        *request_.config,
        false,
        nullptr));
    row.baz_gm_sigma_after_ug = Mps2ToMicroG(
      VerticalAccelBiasGmConstraintBuilder::DrivingSigmaMps2(
        *request_.config,
        false,
        &row));
    profile.push_back(row);
  }
  return profile;
}

bool VerticalMotionStabilityEstimator::OverlapsJumpPadding(
  const double start_time_s,
  const double end_time_s) const {
  for (const auto &window : jump_constraint_windows_) {
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      return true;
    }
  }
  return false;
}

void AccumulateAdaptiveReweightingSummary(
  const VerticalMotionStabilityProfile &profile,
  const std::vector<VerticalVelocityDeltaDiagnosticRow> &vertical_velocity_delta_diagnostics,
  const double dynamic_start_time_s,
  RunSummary &run_summary) {
  const double first20_end_s = dynamic_start_time_s + 20.0;
  std::vector<double> first20_dvz_sigmas;
  std::vector<double> first20_baz_sigmas_ug;
  for (const auto &entry : profile) {
    if (entry.start_time_s < dynamic_start_time_s || entry.end_time_s > first20_end_s) {
      continue;
    }
    if (entry.motion_score < 0.25 && !entry.in_jump_padding) {
      ++run_summary.vertical_motion_adaptive_first20_static_like_interval_count;
    }
    if (std::isfinite(entry.dvz_sigma_after_mps)) {
      first20_dvz_sigmas.push_back(entry.dvz_sigma_after_mps);
    }
    if (std::isfinite(entry.baz_gm_sigma_after_ug)) {
      first20_baz_sigmas_ug.push_back(entry.baz_gm_sigma_after_ug);
    }
  }

  auto assign_mean_max = [](const std::vector<double> &values, double &mean, double &max_value) {
    if (values.empty()) {
      return;
    }
    mean = std::accumulate(values.begin(), values.end(), 0.0) /
           static_cast<double>(values.size());
    max_value = *std::max_element(values.begin(), values.end());
  };
  assign_mean_max(
    first20_dvz_sigmas,
    run_summary.vertical_motion_adaptive_first20_dvz_sigma_mean_mps,
    run_summary.vertical_motion_adaptive_first20_dvz_sigma_max_mps);
  assign_mean_max(
    first20_baz_sigmas_ug,
    run_summary.vertical_motion_adaptive_first20_baz_gm_sigma_mean_ug,
    run_summary.vertical_motion_adaptive_first20_baz_gm_sigma_max_ug);

  double sum_delta = 0.0;
  bool has_delta = false;
  for (const auto &row : vertical_velocity_delta_diagnostics) {
    if (!row.factor_added ||
        row.start_time_s < dynamic_start_time_s ||
        row.end_time_s > first20_end_s ||
        !std::isfinite(row.optimized_delta_vz_mps) ||
        !std::isfinite(row.target_delta_vz_mps)) {
      continue;
    }
    sum_delta += row.optimized_delta_vz_mps - row.target_delta_vz_mps;
    has_delta = true;
  }
  if (has_delta) {
    run_summary.vertical_motion_adaptive_first20_sum_optimized_minus_target_dvz_mps = sum_delta;
  }
}

}  // namespace offline_lc_minimal
