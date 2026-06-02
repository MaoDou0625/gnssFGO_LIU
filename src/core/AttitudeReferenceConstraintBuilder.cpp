#include "offline_lc_minimal/core/AttitudeReferenceConstraintBuilder.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>

#include "offline_lc_minimal/factor/AttitudeReferenceFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

}  // namespace

AttitudeReferenceConstraintBuilder::AttitudeReferenceConstraintBuilder(
  AttitudeReferenceConstraintBuildRequest request)
    : request_(std::move(request)) {}

void AttitudeReferenceConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr) {
    throw std::runtime_error("AttitudeReferenceConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_attitude_reference_constraint) {
    return;
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
