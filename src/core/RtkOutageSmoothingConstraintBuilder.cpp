#include "offline_lc_minimal/core/RtkOutageSmoothingConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/core/BodyZJumpConstraintWindowPlanner.h"
#include "offline_lc_minimal/factor/VerticalPositionRampFactor.h"
#include "offline_lc_minimal/factor/VerticalVelocityDeltaFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

bool ContainsInterval(
  const double outer_start_s,
  const double outer_end_s,
  const double inner_start_s,
  const double inner_end_s) {
  return inner_start_s + kTimeEpsilonS >= outer_start_s &&
         inner_end_s <= outer_end_s + kTimeEpsilonS;
}

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

}  // namespace

RtkOutageSmoothingConstraintBuilder::RtkOutageSmoothingConstraintBuilder(
  RtkOutageSmoothingConstraintBuildRequest request)
    : request_(std::move(request)) {}

void RtkOutageSmoothingConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.body_z_jump_windows == nullptr || request_.propagation_records == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.outage_windows == nullptr) {
    throw std::runtime_error("RtkOutageSmoothingConstraintBuilder received an incomplete request");
  }
  request_.run_summary->rtk_outage_smoothing_enabled =
    request_.config->enable_rtk_outage_smoothing;
  if (!request_.config->enable_rtk_outage_smoothing) {
    return;
  }

  const auto position_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->rtk_outage_position_ramp_sigma_m);
  const auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->rtk_outage_velocity_delta_sigma_mps);
  const std::size_t stride =
    static_cast<std::size_t>(std::max(1, request_.config->rtk_outage_position_ramp_stride));

  request_.run_summary->rtk_outage_window_count = request_.outage_windows->size();
  for (auto &window : *request_.outage_windows) {
    if (window.body_z_jump_overlap_count > 0U) {
      ++request_.run_summary->rtk_outage_window_with_body_z_jump_count;
    }
    if (window.skip_reason != "PLANNED") {
      continue;
    }
    if (window.post_anchor_state_index <= window.pre_anchor_state_index + 1U ||
        window.post_anchor_state_index >= request_.state_timestamps->size()) {
      window.skip_reason = "INVALID_ANCHORS";
      continue;
    }
    const double duration_s = window.end_time_s - window.start_time_s;
    if (!std::isfinite(duration_s) || duration_s <= 0.0) {
      window.skip_reason = "INVALID_DURATION";
      continue;
    }

    for (std::size_t state_index = window.pre_anchor_state_index + 1U;
         state_index < window.post_anchor_state_index;
         state_index += stride) {
      const double alpha =
        ((*request_.state_timestamps)[state_index] - window.start_time_s) / duration_s;
      if (!std::isfinite(alpha) || alpha <= 0.0 || alpha >= 1.0) {
        continue;
      }
      request_.graph->add(factor::VerticalPositionRampFactor(
        symbol::X(window.pre_anchor_state_index),
        symbol::X(state_index),
        symbol::X(window.post_anchor_state_index),
        alpha,
        position_noise));
      ++window.position_ramp_factor_count;
      ++request_.run_summary->rtk_outage_position_ramp_factor_count;
    }

    for (const auto &record : *request_.propagation_records) {
      if (!ContainsInterval(
            window.start_time_s,
            window.end_time_s,
            record.start_time_s,
            record.end_time_s)) {
        continue;
      }
      const double dt_s = record.end_time_s - record.start_time_s;
      if (dt_s <= 0.0 || !std::isfinite(dt_s) ||
          !std::isfinite(record.target_delta_vz_mps)) {
        continue;
      }
      if (OverlapsBodyZJump(record.start_time_s, record.end_time_s)) {
        ++window.velocity_delta_skipped_body_z_jump_count;
        ++request_.run_summary->rtk_outage_velocity_delta_skipped_body_z_jump_count;
        continue;
      }
      request_.graph->add(factor::VerticalVelocityDeltaFactor(
        symbol::V(record.state_index_i),
        symbol::V(record.state_index_j),
        ClampedTargetDeltaVzMps(record, dt_s),
        velocity_noise));
      ++window.velocity_delta_factor_count;
      ++request_.run_summary->rtk_outage_velocity_delta_factor_count;
    }

    window.factor_added =
      window.position_ramp_factor_count > 0U ||
      window.velocity_delta_factor_count > 0U;
    window.skip_reason = window.factor_added ? "ADDED" : "NO_FACTORS";
  }
}

bool RtkOutageSmoothingConstraintBuilder::OverlapsBodyZJump(
  const double start_time_s,
  const double end_time_s) const {
  const std::vector<BodyZJumpConstraintWindow> jump_windows =
    BuildBodyZJumpConstraintWindows(
      *request_.body_z_jump_windows,
      VerticalVelocityDeltaJumpConstraintWindowOptions(*request_.config));
  for (const auto &window : jump_windows) {
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
      return true;
    }
  }
  return false;
}

double RtkOutageSmoothingConstraintBuilder::ClampedTargetDeltaVzMps(
  const VerticalVelocityDeltaPropagationRecord &record,
  const double dt_s) const {
  if (dt_s <= 0.0 || !std::isfinite(dt_s) || !std::isfinite(record.target_delta_vz_mps)) {
    return record.target_delta_vz_mps;
  }
  const double limit_mps =
    request_.config->rtk_outage_velocity_delta_target_acc_limit_mps2 * dt_s;
  return std::clamp(record.target_delta_vz_mps, -limit_mps, limit_mps);
}

}  // namespace offline_lc_minimal
