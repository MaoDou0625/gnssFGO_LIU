#include "offline_lc_minimal/core/RtkOutageBiasContinuityPolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "offline_lc_minimal/common/Units.h"

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED" || window.skip_reason == "ADDED";
}

bool Overlaps(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return std::isfinite(left_start_s) && std::isfinite(left_end_s) &&
         std::isfinite(right_start_s) && std::isfinite(right_end_s) &&
         left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

std::vector<std::string> BoundaryRoles() {
  return {"OUTAGE_START", "OUTAGE_END"};
}

}  // namespace

RtkOutageBiasContinuityPolicy::RtkOutageBiasContinuityPolicy(
  RtkOutageBiasContinuityPolicyRequest request)
    : request_(std::move(request)) {}

std::vector<RtkOutageBiasContinuityPolicyRow>
RtkOutageBiasContinuityPolicy::Build() const {
  if (request_.config == nullptr || request_.outage_windows == nullptr ||
      request_.bias_reestimate_segments == nullptr) {
    throw std::runtime_error(
      "RtkOutageBiasContinuityPolicy received an incomplete request");
  }

  std::vector<RtkOutageBiasContinuityPolicyRow> rows;
  rows.reserve(request_.outage_windows->size() * 2U);
  const double threshold_ug = Mps2ToMicroG(
    request_.config->rtk_outage_baz_continuity_break_delta_threshold_mps2);

  for (const auto &window : *request_.outage_windows) {
    if (!IsPlannedOutage(window)) {
      continue;
    }

    bool overlaps_reestimate = false;
    bool exceeds_threshold = false;
    double max_abs_delta_ug = 0.0;
    bool has_delta = false;
    for (const auto &segment : *request_.bias_reestimate_segments) {
      const bool same_outage =
        segment.source_outage_window_index >= 0 &&
        static_cast<std::size_t>(segment.source_outage_window_index) ==
          window.window_index;
      const bool time_overlap = Overlaps(
        segment.start_time_s,
        segment.end_time_s,
        window.start_time_s,
        window.end_time_s);
      if (!same_outage && !time_overlap) {
        continue;
      }
      overlaps_reestimate = true;
      if (std::isfinite(segment.detected_bias_delta_mps2)) {
        const double delta_ug = std::abs(Mps2ToMicroG(segment.detected_bias_delta_mps2));
        max_abs_delta_ug = std::max(max_abs_delta_ug, delta_ug);
        has_delta = true;
        if (delta_ug >
            Mps2ToMicroG(request_.config->rtk_outage_baz_continuity_break_delta_threshold_mps2)) {
          exceeds_threshold = true;
        }
      }
    }

    for (const auto &boundary_role : BoundaryRoles()) {
      RtkOutageBiasContinuityPolicyRow row;
      row.window_index = window.window_index;
      row.boundary_role = boundary_role;
      row.overlaps_reestimate_segment = overlaps_reestimate;
      row.threshold_ug = threshold_ug;
      row.max_abs_detected_delta_ug =
        has_delta ? max_abs_delta_ug : std::numeric_limits<double>::quiet_NaN();
      row.ba_z_continuity_allowed = true;
      row.reset_reason = exceeds_threshold
        ? "delta_threshold_exceeded_continuity_preserved"
        : "NONE";
      rows.push_back(row);
    }
  }
  return rows;
}

bool AllowsRtkOutageBazContinuity(
  const std::vector<RtkOutageBiasContinuityPolicyRow> &policy_rows,
  const std::size_t window_index,
  const std::string &boundary_role) {
  const auto it = std::find_if(
    policy_rows.begin(),
    policy_rows.end(),
    [&](const RtkOutageBiasContinuityPolicyRow &row) {
      return row.window_index == window_index && row.boundary_role == boundary_role;
    });
  return it == policy_rows.end() ? true : it->ba_z_continuity_allowed;
}

}  // namespace offline_lc_minimal
