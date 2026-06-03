#include "offline_lc_minimal/core/RtkOutageInitialValueSmoother.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED";
}

gtsam::Pose3 WithZ(const gtsam::Pose3 &pose, const double up_m) {
  const gtsam::Point3 translation = pose.translation();
  return gtsam::Pose3(
    pose.rotation(),
    gtsam::Point3(translation.x(), translation.y(), up_m));
}

gtsam::Pose3 ShiftZ(const gtsam::Pose3 &pose, const double offset_m) {
  const gtsam::Point3 translation = pose.translation();
  return gtsam::Pose3(
    pose.rotation(),
    gtsam::Point3(translation.x(), translation.y(), translation.z() + offset_m));
}

}  // namespace

RtkOutageInitialValueSmoother::RtkOutageInitialValueSmoother(
  RtkOutageInitialValueSmoothRequest request)
    : request_(std::move(request)) {}

void RtkOutageInitialValueSmoother::Apply() const {
  if (request_.state_timestamps == nullptr || request_.gnss_samples == nullptr ||
      request_.propagation_records == nullptr || request_.initial_values == nullptr ||
      request_.outage_windows == nullptr) {
    throw std::runtime_error("RtkOutageInitialValueSmoother received an incomplete request");
  }

  for (auto &window : *request_.outage_windows) {
    if (!IsPlannedOutage(window) ||
        window.pre_sample_index >= request_.gnss_samples->size() ||
        window.post_sample_index >= request_.gnss_samples->size() ||
        window.post_anchor_state_index >= request_.state_timestamps->size() ||
        window.post_anchor_state_index <= window.pre_anchor_state_index) {
      continue;
    }
    const GnssSolutionSample &pre_sample = (*request_.gnss_samples)[window.pre_sample_index];
    const GnssSolutionSample &post_sample = (*request_.gnss_samples)[window.post_sample_index];
    if (!pre_sample.has_enu_position || !post_sample.has_enu_position) {
      continue;
    }
    const double pre_up_m = pre_sample.enu_position_m.z();
    const double post_up_m = post_sample.enu_position_m.z();
    const double duration_s = window.end_time_s - window.start_time_s;
    if (!std::isfinite(pre_up_m) || !std::isfinite(post_up_m) ||
        !std::isfinite(duration_s) || duration_s <= 0.0) {
      continue;
    }
    const gtsam::Key post_pose_key = symbol::X(window.post_anchor_state_index);
    if (!request_.initial_values->exists(post_pose_key)) {
      continue;
    }
    const double original_post_up_m =
      request_.initial_values->at<gtsam::Pose3>(post_pose_key).translation().z();
    const double post_offset_m = post_up_m - original_post_up_m;

    const std::vector<double> smoothed_vz_mps =
      BuildVelocityProfile(window, pre_up_m, post_up_m);
    std::size_t smoothed_count = 0;
    for (std::size_t state_index = window.pre_anchor_state_index;
         state_index <= window.post_anchor_state_index;
         ++state_index) {
      const double alpha =
        ((*request_.state_timestamps)[state_index] - window.start_time_s) / duration_s;
      if (!std::isfinite(alpha)) {
        continue;
      }
      const double bounded_alpha = std::clamp(alpha, 0.0, 1.0);
      const double up_m = (1.0 - bounded_alpha) * pre_up_m + bounded_alpha * post_up_m;
      const gtsam::Key pose_key = symbol::X(state_index);
      if (request_.initial_values->exists(pose_key)) {
        const auto pose = request_.initial_values->at<gtsam::Pose3>(pose_key);
        request_.initial_values->update(pose_key, WithZ(pose, up_m));
        ++smoothed_count;
      }
      const std::size_t velocity_offset = state_index - window.pre_anchor_state_index;
      const gtsam::Key velocity_key = symbol::V(state_index);
      if (velocity_offset < smoothed_vz_mps.size() &&
          request_.initial_values->exists(velocity_key)) {
        auto velocity = request_.initial_values->at<gtsam::Vector3>(velocity_key);
        velocity.z() = smoothed_vz_mps[velocity_offset];
        request_.initial_values->update(velocity_key, velocity);
      }
    }

    if (std::isfinite(post_offset_m) && std::abs(post_offset_m) > kTimeEpsilonS) {
      for (std::size_t state_index = window.post_anchor_state_index + 1U;
           state_index < request_.state_timestamps->size();
           ++state_index) {
        const gtsam::Key pose_key = symbol::X(state_index);
        if (!request_.initial_values->exists(pose_key)) {
          continue;
        }
        const auto pose = request_.initial_values->at<gtsam::Pose3>(pose_key);
        request_.initial_values->update(pose_key, ShiftZ(pose, post_offset_m));
      }
    }

    window.initial_value_smoothing_applied = smoothed_count > 0U;
    window.initial_value_smoothed_state_count = smoothed_count;
    window.pre_anchor_up_m = pre_up_m;
    window.post_anchor_up_m = post_up_m;
    window.post_anchor_up_offset_m = post_offset_m;
  }
}

std::vector<double> RtkOutageInitialValueSmoother::BuildVelocityProfile(
  const RtkOutageWindowRow &window,
  const double pre_up_m,
  const double post_up_m) const {
  const std::size_t state_count =
    window.post_anchor_state_index - window.pre_anchor_state_index + 1U;
  std::vector<double> cumulative_dvz(state_count, 0.0);
  std::vector<double> interval_dt(state_count > 0U ? state_count - 1U : 0U, 0.0);
  for (const auto &record : *request_.propagation_records) {
    if (record.state_index_i < window.pre_anchor_state_index ||
        record.state_index_j > window.post_anchor_state_index ||
        record.state_index_j != record.state_index_i + 1U ||
        !std::isfinite(record.target_delta_vz_mps)) {
      continue;
    }
    const std::size_t offset = record.state_index_i - window.pre_anchor_state_index;
    if (offset + 1U >= state_count) {
      continue;
    }
    const double dt_s = record.end_time_s - record.start_time_s;
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      continue;
    }
    interval_dt[offset] = dt_s;
    cumulative_dvz[offset + 1U] = record.target_delta_vz_mps;
  }
  for (std::size_t index = 1U; index < cumulative_dvz.size(); ++index) {
    cumulative_dvz[index] += cumulative_dvz[index - 1U];
  }

  double total_dt_s = 0.0;
  double cumulative_integral_m = 0.0;
  for (std::size_t index = 0U; index < interval_dt.size(); ++index) {
    double dt_s = interval_dt[index];
    if (dt_s <= 0.0) {
      dt_s = (*request_.state_timestamps)[window.pre_anchor_state_index + index + 1U] -
             (*request_.state_timestamps)[window.pre_anchor_state_index + index];
    }
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
      continue;
    }
    total_dt_s += dt_s;
    cumulative_integral_m +=
      0.5 * dt_s * (cumulative_dvz[index] + cumulative_dvz[index + 1U]);
  }
  if (!std::isfinite(total_dt_s) || total_dt_s <= 0.0) {
    return std::vector<double>(state_count, 0.0);
  }

  const double base_vz_mps =
    ((post_up_m - pre_up_m) - cumulative_integral_m) / total_dt_s;
  std::vector<double> velocities;
  velocities.reserve(cumulative_dvz.size());
  for (const double delta_vz_mps : cumulative_dvz) {
    velocities.push_back(base_vz_mps + delta_vz_mps);
  }
  return velocities;
}

}  // namespace offline_lc_minimal
