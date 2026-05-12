#include "offline_lc_minimal/core/AttitudeReferenceConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/AttitudeReferenceFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

AttitudeReferenceDiagnosticRow MakeDiagnosticRow(
  const std::size_t state_index,
  const double time_s,
  const gtsam::Rot3 &reference_rotation) {
  AttitudeReferenceDiagnosticRow row;
  row.state_index = state_index;
  row.time_s = time_s;
  row.factor_added = true;
  row.skip_reason = "ADDED";
  row.reference_ypr_rad = Rot3ToYpr(reference_rotation);
  return row;
}

RelativeYawReferenceDiagnosticRow MakeRelativeYawDiagnosticRow(
  const std::size_t edge_index,
  const std::size_t state_index_i,
  const std::size_t state_index_j,
  const double time_i_s,
  const double time_j_s,
  const gtsam::Rot3 &reference_rotation_i,
  const gtsam::Rot3 &reference_rotation_j,
  const double sigma_rad) {
  RelativeYawReferenceDiagnosticRow row;
  row.edge_index = edge_index;
  row.state_index_i = state_index_i;
  row.state_index_j = state_index_j;
  row.time_i_s = time_i_s;
  row.time_j_s = time_j_s;
  row.factor_added = true;
  row.skip_reason = "ADDED";
  row.sigma_rad = sigma_rad;
  row.reference_delta_yaw_rad =
    factor::RelativeYawRad(reference_rotation_i, reference_rotation_j);
  return row;
}

}  // namespace

AttitudeReferenceConstraintBuilder::AttitudeReferenceConstraintBuilder(
  AttitudeReferenceConstraintBuildRequest request)
    : request_(std::move(request)) {}

void AttitudeReferenceConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.reference_states == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr ||
      request_.relative_yaw_diagnostics == nullptr) {
    throw std::runtime_error("AttitudeReferenceConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_attitude_reference_constraint) {
    return;
  }
  if (request_.reference_states->empty()) {
    throw std::runtime_error(
      "attitude reference constraint requires successful seed reference optimization");
  }
  if (request_.reference_states->size() != request_.state_timestamps->size()) {
    throw std::runtime_error("attitude reference state count does not match graph state count");
  }
  if (request_.dynamic_start_index >= request_.state_timestamps->size()) {
    return;
  }

  const auto roll_pitch_noise =
    gtsam::noiseModel::Isotropic::Sigma(2, request_.config->attitude_reference_sigma_rad);
  const auto relative_yaw_noise =
    gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->attitude_reference_relative_yaw_sigma_rad);
  const std::size_t dynamic_state_count =
    request_.state_timestamps->size() - request_.dynamic_start_index;
  request_.diagnostics->reserve(request_.diagnostics->size() + dynamic_state_count);
  request_.relative_yaw_diagnostics->reserve(
    request_.relative_yaw_diagnostics->size() +
    (dynamic_state_count > 0U ? dynamic_state_count - 1U : 0U));

  for (std::size_t state_index = request_.dynamic_start_index;
       state_index < request_.state_timestamps->size();
       ++state_index) {
    const auto &reference_state = (*request_.reference_states)[state_index];
    request_.graph->add(factor::RollPitchReferenceFactor(
      symbol::X(state_index),
      reference_state.pose.rotation(),
      roll_pitch_noise));
    ++request_.run_summary->attitude_reference_factor_count;
    request_.diagnostics->push_back(MakeDiagnosticRow(
      state_index,
      (*request_.state_timestamps)[state_index],
      reference_state.pose.rotation()));
  }

  for (std::size_t state_index_j = request_.dynamic_start_index + 1U;
       state_index_j < request_.state_timestamps->size();
       ++state_index_j) {
    const std::size_t state_index_i = state_index_j - 1U;
    const auto &reference_state_i = (*request_.reference_states)[state_index_i];
    const auto &reference_state_j = (*request_.reference_states)[state_index_j];
    request_.graph->add(factor::RelativeYawReferenceFactor(
      symbol::X(state_index_i),
      symbol::X(state_index_j),
      reference_state_i.pose.rotation(),
      reference_state_j.pose.rotation(),
      relative_yaw_noise));
    ++request_.run_summary->attitude_reference_factor_count;
    request_.relative_yaw_diagnostics->push_back(MakeRelativeYawDiagnosticRow(
      request_.relative_yaw_diagnostics->size(),
      state_index_i,
      state_index_j,
      (*request_.state_timestamps)[state_index_i],
      (*request_.state_timestamps)[state_index_j],
      reference_state_i.pose.rotation(),
      reference_state_j.pose.rotation(),
      request_.config->attitude_reference_relative_yaw_sigma_rad));
  }
}

void PopulateAttitudeReferenceDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<AttitudeReferenceDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const auto optimized_pose = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index));
    row.optimized_ypr_rad = Rot3ToYpr(optimized_pose.rotation());
    row.residual_x_rad = std::numeric_limits<double>::quiet_NaN();
    row.residual_y_rad =
      factor::NormalizeAngleRad(row.optimized_ypr_rad.y() - row.reference_ypr_rad.y());
    row.residual_z_rad =
      factor::NormalizeAngleRad(row.optimized_ypr_rad.z() - row.reference_ypr_rad.z());
    row.residual_norm_rad = std::hypot(row.residual_y_rad, row.residual_z_rad);
  }
}

void PopulateRelativeYawReferenceDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<RelativeYawReferenceDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const auto optimized_pose_i = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_i));
    const auto optimized_pose_j = optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_j));
    row.optimized_delta_yaw_rad =
      factor::RelativeYawRad(optimized_pose_i.rotation(), optimized_pose_j.rotation());
    row.residual_yaw_rad =
      factor::NormalizeAngleRad(row.optimized_delta_yaw_rad - row.reference_delta_yaw_rad);
  }
}

}  // namespace offline_lc_minimal
