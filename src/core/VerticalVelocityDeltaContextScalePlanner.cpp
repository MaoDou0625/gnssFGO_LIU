#include "offline_lc_minimal/core/VerticalVelocityDeltaContextScalePlanner.h"
#include "offline_lc_minimal/core/BodyZBiasReestimateSourcePolicy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace offline_lc_minimal {
namespace {

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s && right_start_s <= left_end_s;
}

bool IsFiniteInterval(const double start_time_s, const double end_time_s) {
  return std::isfinite(start_time_s) &&
         std::isfinite(end_time_s) &&
         end_time_s >= start_time_s;
}

bool IsRoughBiasSource(const BodyZBiasReestimateSegmentRow &segment) {
  return segment.source_type == "BODY_Z_BIAS" ||
         segment.source_type == "ROUGH_ROAD" ||
         IsRoadHighNoiseBiasReestimateSource(segment);
}

std::string JoinContextLabels(const std::vector<std::string> &labels) {
  if (labels.empty()) {
    return "NORMAL";
  }
  std::string joined = labels.front();
  for (std::size_t index = 1; index < labels.size(); ++index) {
    joined += "+";
    joined += labels[index];
  }
  return joined;
}

}  // namespace

VerticalVelocityDeltaContextScalePlanner::VerticalVelocityDeltaContextScalePlanner(
  VerticalVelocityDeltaContextScaleRequest request)
    : request_(std::move(request)) {
  if (request_.config == nullptr) {
    throw std::runtime_error(
      "VerticalVelocityDeltaContextScalePlanner received a null config");
  }
}

VerticalVelocityDeltaContextScaleDecision
VerticalVelocityDeltaContextScalePlanner::Evaluate(
  const double start_time_s,
  const double end_time_s) const {
  VerticalVelocityDeltaContextScaleDecision decision;
  decision.output_sigma_scale =
    request_.config->vertical_velocity_delta_sigma_scale;
  decision.overlaps_jump = OverlapsJump(start_time_s, end_time_s);
  decision.overlaps_rtk_outage = OverlapsRtkOutage(start_time_s, end_time_s);
  decision.overlaps_rough_bias = OverlapsRoughBias(start_time_s, end_time_s);
  decision.overlaps_road_high_noise_bias =
    OverlapsRoadHighNoiseBias(start_time_s, end_time_s);
  if (!request_.config->enable_vertical_velocity_delta_context_sigma_scale) {
    decision.context = "GLOBAL";
    return decision;
  }

  decision.output_sigma_scale =
    request_.config->vertical_velocity_delta_context_normal_sigma_scale;

  std::vector<std::string> labels;
  if (decision.overlaps_jump) {
    labels.push_back("JUMP");
    decision.output_sigma_scale = std::max(
      decision.output_sigma_scale,
      request_.config->vertical_velocity_delta_context_jump_sigma_scale);
  }
  if (decision.overlaps_rtk_outage) {
    labels.push_back("RTK_OUTAGE");
    decision.output_sigma_scale = std::max(
      decision.output_sigma_scale,
      request_.config->vertical_velocity_delta_context_outage_sigma_scale);
  }
  if (decision.overlaps_rough_bias) {
    labels.push_back("ROUGH_BIAS");
    decision.output_sigma_scale = std::max(
      decision.output_sigma_scale,
      request_.config->vertical_velocity_delta_context_rough_sigma_scale);
  }
  decision.context = JoinContextLabels(labels);
  return decision;
}

bool VerticalVelocityDeltaContextScalePlanner::OverlapsJump(
  const double start_time_s,
  const double end_time_s) const {
  if (request_.jump_constraint_windows == nullptr ||
      !IsFiniteInterval(start_time_s, end_time_s)) {
    return false;
  }
  for (const auto &window : *request_.jump_constraint_windows) {
    const double padded_start_time_s =
      window.start_time_s -
      request_.config->vertical_velocity_delta_context_jump_extra_padding_s;
    const double padded_end_time_s =
      window.end_time_s +
      request_.config->vertical_velocity_delta_context_jump_extra_padding_s;
    if (IsFiniteInterval(padded_start_time_s, padded_end_time_s) &&
        IntervalsOverlap(start_time_s, end_time_s, padded_start_time_s, padded_end_time_s)) {
      return true;
    }
  }
  return false;
}

bool VerticalVelocityDeltaContextScalePlanner::OverlapsRtkOutage(
  const double start_time_s,
  const double end_time_s) const {
  if (!IsFiniteInterval(start_time_s, end_time_s)) {
    return false;
  }
  if (request_.rtk_outage_windows != nullptr) {
    for (const auto &window : *request_.rtk_outage_windows) {
      if (IsFiniteInterval(window.start_time_s, window.end_time_s) &&
          IntervalsOverlap(start_time_s, end_time_s, window.start_time_s, window.end_time_s)) {
        return true;
      }
    }
  }
  if (request_.bias_reestimate_segments != nullptr) {
    for (const auto &segment : *request_.bias_reestimate_segments) {
      if (segment.source_type != "RTK_OUTAGE") {
        continue;
      }
      if (IsFiniteInterval(segment.start_time_s, segment.end_time_s) &&
          IntervalsOverlap(start_time_s, end_time_s, segment.start_time_s, segment.end_time_s)) {
        return true;
      }
    }
  }
  return false;
}

bool VerticalVelocityDeltaContextScalePlanner::OverlapsRoughBias(
  const double start_time_s,
  const double end_time_s) const {
  if (request_.bias_reestimate_segments == nullptr ||
      !IsFiniteInterval(start_time_s, end_time_s)) {
    return false;
  }
  for (const auto &segment : *request_.bias_reestimate_segments) {
    if (!IsRoughBiasSource(segment)) {
      continue;
    }
    if (IsFiniteInterval(segment.start_time_s, segment.end_time_s) &&
        IntervalsOverlap(start_time_s, end_time_s, segment.start_time_s, segment.end_time_s)) {
      return true;
    }
  }
  return false;
}

bool VerticalVelocityDeltaContextScalePlanner::OverlapsRoadHighNoiseBias(
  const double start_time_s,
  const double end_time_s) const {
  if (request_.bias_reestimate_segments == nullptr ||
      !IsFiniteInterval(start_time_s, end_time_s)) {
    return false;
  }
  for (const auto &segment : *request_.bias_reestimate_segments) {
    if (!IsRoadHighNoiseBiasReestimateSource(segment)) {
      continue;
    }
    if (IsFiniteInterval(segment.start_time_s, segment.end_time_s) &&
        IntervalsOverlap(start_time_s, end_time_s, segment.start_time_s, segment.end_time_s)) {
      return true;
    }
  }
  return false;
}

}  // namespace offline_lc_minimal
