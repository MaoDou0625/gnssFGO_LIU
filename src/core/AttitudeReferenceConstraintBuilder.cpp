#include "offline_lc_minimal/core/AttitudeReferenceConstraintBuilder.h"

#include <algorithm>
#include <cmath>
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

}  // namespace

AttitudeReferenceConstraintBuilder::AttitudeReferenceConstraintBuilder(
  AttitudeReferenceConstraintBuildRequest request)
    : request_(std::move(request)) {}

void AttitudeReferenceConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.reference_states == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr) {
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

  const auto noise =
    gtsam::noiseModel::Isotropic::Sigma(3, request_.config->attitude_reference_sigma_rad);
  request_.diagnostics->reserve(
    request_.diagnostics->size() + request_.state_timestamps->size() - request_.dynamic_start_index);
  for (std::size_t state_index = request_.dynamic_start_index;
       state_index < request_.state_timestamps->size();
       ++state_index) {
    const auto &reference_state = (*request_.reference_states)[state_index];
    request_.graph->add(factor::AttitudeReferenceFactor(
      symbol::X(state_index),
      reference_state.pose.rotation(),
      noise));
    ++request_.run_summary->attitude_reference_factor_count;
    request_.diagnostics->push_back(MakeDiagnosticRow(
      state_index,
      (*request_.state_timestamps)[state_index],
      reference_state.pose.rotation()));
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
    const gtsam::Rot3 reference_rotation =
      gtsam::Rot3::Ypr(row.reference_ypr_rad.x(), row.reference_ypr_rad.y(), row.reference_ypr_rad.z());
    const gtsam::Vector3 residual = gtsam::Rot3::Logmap(reference_rotation.between(optimized_pose.rotation()));
    row.residual_x_rad = residual.x();
    row.residual_y_rad = residual.y();
    row.residual_z_rad = residual.z();
    row.residual_norm_rad = residual.norm();
  }
}

}  // namespace offline_lc_minimal
