#include "offline_lc_minimal/core/RtkOutageRecoveryConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/AttitudeReferenceFactor.h"
#include "offline_lc_minimal/factor/VelocityDeltaFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED" || window.skip_reason == "ADDED";
}

bool ContainsInterval(
  const double outer_start_s,
  const double outer_end_s,
  const double inner_start_s,
  const double inner_end_s) {
  return inner_start_s + kTimeEpsilonS >= outer_start_s &&
         inner_end_s <= outer_end_s + kTimeEpsilonS;
}

std::size_t FirstStateIndexAtOrAfter(
  const std::vector<double> &timestamps,
  const double time_s) {
  const auto it = std::lower_bound(timestamps.begin(), timestamps.end(), time_s);
  if (it == timestamps.end()) {
    return timestamps.size() - 1U;
  }
  return static_cast<std::size_t>(it - timestamps.begin());
}

std::size_t LastStateIndexAtOrBefore(
  const std::vector<double> &timestamps,
  const double time_s) {
  const auto it = std::upper_bound(timestamps.begin(), timestamps.end(), time_s);
  if (it == timestamps.begin()) {
    return 0U;
  }
  return static_cast<std::size_t>((it - timestamps.begin()) - 1);
}

std::pair<std::size_t, std::size_t> AttitudeHoldSpan(
  const RtkOutageWindowRow &window,
  const std::vector<double> &timestamps,
  const double guard_duration_s) {
  const std::size_t guarded_start = FirstStateIndexAtOrAfter(
    timestamps,
    window.start_time_s - guard_duration_s);
  const std::size_t guarded_end = LastStateIndexAtOrBefore(
    timestamps,
    window.end_time_s + guard_duration_s);
  return {
    std::min(window.pre_anchor_state_index, guarded_start),
    std::max(window.post_anchor_state_index, guarded_end)};
}

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

RtkOutageAttitudeHoldDiagnosticRow MakeRelativeRotationDiagnostic(
  const std::size_t window_index,
  const std::size_t state_index_i,
  const std::size_t state_index_j,
  const double time_i_s,
  const double time_j_s,
  const gtsam::Rot3 &reference_rotation_i,
  const gtsam::Rot3 &reference_rotation_j,
  const std::string &reference_source,
  const double sigma_rad) {
  RtkOutageAttitudeHoldDiagnosticRow row;
  row.window_index = window_index;
  row.constraint_type = "relative_rotation";
  row.state_index_i = state_index_i;
  row.state_index_j = state_index_j;
  row.time_i_s = time_i_s;
  row.time_j_s = time_j_s;
  row.factor_added = true;
  row.skip_reason = "ADDED";
  row.reference_source = reference_source;
  row.sigma_rad = sigma_rad;
  row.reference_rotation_i = reference_rotation_i;
  row.reference_rotation_j = reference_rotation_j;
  row.reference_ypr_i_rad = Rot3ToYpr(reference_rotation_i);
  row.reference_ypr_j_rad = Rot3ToYpr(reference_rotation_j);
  row.reference_relative_rotvec_rad =
    gtsam::Rot3::Logmap(reference_rotation_i.between(reference_rotation_j));
  row.reference_relative_angle_rad = row.reference_relative_rotvec_rad.norm();
  row.reference_delta_yaw_rad =
    factor::RelativeYawRad(reference_rotation_i, reference_rotation_j);
  return row;
}

RtkOutageVelocityDelta3dDiagnosticRow MakeVelocityDeltaDiagnostic(
  const std::size_t window_index,
  const VelocityDeltaPropagationRecord &record,
  const double sigma_mps) {
  RtkOutageVelocityDelta3dDiagnosticRow row;
  row.window_index = window_index;
  row.state_index_i = record.state_index_i;
  row.state_index_j = record.state_index_j;
  row.time_i_s = record.start_time_s;
  row.time_j_s = record.end_time_s;
  row.factor_added = true;
  row.skip_reason = "ADDED";
  row.sigma_mps = sigma_mps;
  row.target_delta_v_mps = record.target_delta_v_mps;
  return row;
}

void AccumulateAttitudeSummary(
  const std::vector<RtkOutageAttitudeHoldDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  double absolute_max = 0.0;
  bool has_absolute = false;
  double relative_max = 0.0;
  bool has_relative = false;
  for (const auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    if (row.constraint_type == "absolute") {
      for (int index = 0; index < 3; ++index) {
        if (std::isfinite(row.residual_rad(index))) {
          absolute_max = std::max(absolute_max, std::abs(row.residual_rad(index)));
          has_absolute = true;
        }
      }
    } else if (row.constraint_type == "relative_yaw" &&
               std::isfinite(row.residual_yaw_rad)) {
      relative_max = std::max(relative_max, std::abs(row.residual_yaw_rad));
      has_relative = true;
    } else if (row.constraint_type == "relative_rotation" &&
               std::isfinite(row.residual_norm_rad)) {
      relative_max = std::max(relative_max, row.residual_norm_rad);
      has_relative = true;
    }
  }
  if (has_absolute) {
    run_summary.rtk_outage_attitude_hold_max_abs_residual_rad = absolute_max;
  }
  if (has_relative) {
    run_summary.rtk_outage_relative_attitude_max_abs_residual_rad = relative_max;
  }
}

void AccumulateVelocitySummary(
  const std::vector<RtkOutageVelocityDelta3dDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  double residual_sq_sum = 0.0;
  std::size_t residual_count = 0;
  for (const auto &row : diagnostics) {
    if (!row.factor_added || !std::isfinite(row.residual_norm_mps)) {
      continue;
    }
    residual_sq_sum += row.residual_norm_mps * row.residual_norm_mps;
    ++residual_count;
  }
  if (residual_count > 0U) {
    run_summary.rtk_outage_velocity_delta_3d_rms_mps =
      std::sqrt(residual_sq_sum / static_cast<double>(residual_count));
  }
}

}  // namespace

RtkOutageRecoveryConstraintBuilder::RtkOutageRecoveryConstraintBuilder(
  RtkOutageRecoveryConstraintBuildRequest request)
    : request_(std::move(request)) {}

bool RtkOutageRecoveryConstraintBuilder::HasCompleteAttitudeRequest() const {
  return request_.config != nullptr && request_.state_timestamps != nullptr &&
         request_.outage_windows != nullptr && request_.reference_states != nullptr &&
         request_.graph != nullptr && request_.run_summary != nullptr &&
         request_.attitude_diagnostics != nullptr;
}

bool RtkOutageRecoveryConstraintBuilder::HasCompleteVelocityRequest() const {
  return request_.config != nullptr && request_.outage_windows != nullptr &&
         request_.velocity_delta_records != nullptr && request_.graph != nullptr &&
         request_.run_summary != nullptr && request_.velocity_diagnostics != nullptr;
}

void RtkOutageRecoveryConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.outage_windows == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr) {
    throw std::runtime_error("RtkOutageRecoveryConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_rtk_outage_smoothing ||
      request_.outage_windows->empty()) {
    return;
  }

  if (request_.config->enable_rtk_outage_attitude_hold) {
    if (!HasCompleteAttitudeRequest()) {
      throw std::runtime_error("RTK outage attitude hold received an incomplete request");
    }
    if (request_.reference_states->size() != request_.state_timestamps->size()) {
      throw std::runtime_error("RTK outage attitude hold requires one reference state per graph state");
    }
    const auto relative_rotation_noise = gtsam::noiseModel::Isotropic::Sigma(
      3,
      request_.config->rtk_outage_relative_attitude_sigma_rad);
    for (const auto &window : *request_.outage_windows) {
      if (!IsPlannedOutage(window) ||
          window.post_anchor_state_index >= request_.state_timestamps->size() ||
          window.pre_anchor_state_index > window.post_anchor_state_index) {
        continue;
      }
      const auto [hold_start_index, hold_end_index] = AttitudeHoldSpan(
        window,
        *request_.state_timestamps,
        request_.config->rtk_outage_attitude_guard_duration_s);
      for (std::size_t state_index_j = hold_start_index + 1U;
           state_index_j <= hold_end_index;
           ++state_index_j) {
        const std::size_t state_index_i = state_index_j - 1U;
        const auto &reference_state_i = (*request_.reference_states)[state_index_i];
        const auto &reference_state_j = (*request_.reference_states)[state_index_j];
        request_.graph->add(factor::RelativeRotationReferenceFactor(
          symbol::X(state_index_i),
          symbol::X(state_index_j),
          reference_state_i.pose.rotation(),
          reference_state_j.pose.rotation(),
          relative_rotation_noise));
        ++request_.run_summary->rtk_outage_relative_attitude_factor_count;
        request_.attitude_diagnostics->push_back(MakeRelativeRotationDiagnostic(
          window.window_index,
          state_index_i,
          state_index_j,
          (*request_.state_timestamps)[state_index_i],
          (*request_.state_timestamps)[state_index_j],
          reference_state_i.pose.rotation(),
          reference_state_j.pose.rotation(),
          request_.attitude_reference_source,
          request_.config->rtk_outage_relative_attitude_sigma_rad));
      }
    }
  }

  if (request_.config->enable_rtk_outage_velocity_delta_3d) {
    if (!HasCompleteVelocityRequest()) {
      throw std::runtime_error("RTK outage 3D velocity delta received an incomplete request");
    }
    const auto velocity_noise = gtsam::noiseModel::Isotropic::Sigma(
      3,
      request_.config->rtk_outage_velocity_delta_3d_sigma_mps);
    for (const auto &window : *request_.outage_windows) {
      if (!IsPlannedOutage(window)) {
        continue;
      }
      for (const auto &record : *request_.velocity_delta_records) {
        if (!ContainsInterval(
              window.start_time_s,
              window.end_time_s,
              record.start_time_s,
              record.end_time_s)) {
          continue;
        }
        if (!record.target_delta_v_mps.allFinite()) {
          continue;
        }
        request_.graph->add(factor::VelocityDeltaFactor(
          symbol::V(record.state_index_i),
          symbol::V(record.state_index_j),
          record.target_delta_v_mps,
          velocity_noise));
        ++request_.run_summary->rtk_outage_velocity_delta_3d_factor_count;
        request_.velocity_diagnostics->push_back(MakeVelocityDeltaDiagnostic(
          window.window_index,
          record,
          request_.config->rtk_outage_velocity_delta_3d_sigma_mps));
      }
    }
  }
}

void PopulateRtkOutageRecoveryDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<RtkOutageAttitudeHoldDiagnosticRow> &attitude_diagnostics,
  std::vector<RtkOutageVelocityDelta3dDiagnosticRow> &velocity_diagnostics,
  RunSummary &run_summary) {
  for (auto &row : attitude_diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    if (row.constraint_type == "absolute") {
      const auto optimized_pose = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_i));
      row.optimized_ypr_i_rad = Rot3ToYpr(optimized_pose.rotation());
      row.optimized_ypr_j_rad = row.optimized_ypr_i_rad;
      row.residual_rad =
        gtsam::Rot3::Logmap(row.reference_rotation_i.between(optimized_pose.rotation()));
      row.residual_norm_rad = row.residual_rad.norm();
    } else if (row.constraint_type == "relative_yaw") {
      const auto optimized_pose_i = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_i));
      const auto optimized_pose_j = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_j));
      row.optimized_ypr_i_rad = Rot3ToYpr(optimized_pose_i.rotation());
      row.optimized_ypr_j_rad = Rot3ToYpr(optimized_pose_j.rotation());
      const gtsam::Rot3 optimized_delta_rotation =
        optimized_pose_i.rotation().between(optimized_pose_j.rotation());
      row.optimized_relative_rotvec_rad =
        gtsam::Rot3::Logmap(optimized_delta_rotation);
      row.optimized_relative_angle_rad = row.optimized_relative_rotvec_rad.norm();
      row.optimized_delta_yaw_rad =
        factor::RelativeYawRad(optimized_pose_i.rotation(), optimized_pose_j.rotation());
      row.residual_yaw_rad =
        factor::NormalizeAngleRad(row.optimized_delta_yaw_rad - row.reference_delta_yaw_rad);
      row.residual_norm_rad = std::abs(row.residual_yaw_rad);
    } else if (row.constraint_type == "relative_rotation") {
      const auto optimized_pose_i = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_i));
      const auto optimized_pose_j = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_j));
      row.optimized_ypr_i_rad = Rot3ToYpr(optimized_pose_i.rotation());
      row.optimized_ypr_j_rad = Rot3ToYpr(optimized_pose_j.rotation());
      const gtsam::Rot3 reference_delta_rotation =
        row.reference_rotation_i.between(row.reference_rotation_j);
      const gtsam::Rot3 optimized_delta_rotation =
        optimized_pose_i.rotation().between(optimized_pose_j.rotation());
      row.optimized_relative_rotvec_rad =
        gtsam::Rot3::Logmap(optimized_delta_rotation);
      row.optimized_relative_angle_rad = row.optimized_relative_rotvec_rad.norm();
      row.residual_rad =
        gtsam::Rot3::Logmap(reference_delta_rotation.between(optimized_delta_rotation));
      row.residual_norm_rad = row.residual_rad.norm();
      row.optimized_delta_yaw_rad =
        factor::RelativeYawRad(optimized_pose_i.rotation(), optimized_pose_j.rotation());
      row.residual_yaw_rad =
        factor::NormalizeAngleRad(row.optimized_delta_yaw_rad - row.reference_delta_yaw_rad);
    }
  }
  for (auto &row : velocity_diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const gtsam::Vector3 velocity_i =
      optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index_i));
    const gtsam::Vector3 velocity_j =
      optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index_j));
    row.optimized_delta_v_mps = velocity_j - velocity_i;
    row.residual_mps = row.optimized_delta_v_mps - row.target_delta_v_mps;
    row.residual_norm_mps = row.residual_mps.norm();
  }
  AccumulateAttitudeSummary(attitude_diagnostics, run_summary);
  AccumulateVelocitySummary(velocity_diagnostics, run_summary);
}

}  // namespace offline_lc_minimal
