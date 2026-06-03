#pragma once

#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalPositionVelocityConsistencyFactor final
    : public gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3> {
 public:
  VerticalPositionVelocityConsistencyFactor(
    gtsam::Key pose_i_key,
    gtsam::Key velocity_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key velocity_j_key,
    double dt_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor4<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3, gtsam::Vector3>(
          model,
          pose_i_key,
          velocity_i_key,
          pose_j_key,
          velocity_j_key),
        dt_s_(dt_s) {
    if (dt_s_ <= 0.0) {
      throw std::invalid_argument("VerticalPositionVelocityConsistencyFactor requires positive dt");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalPositionVelocityConsistencyFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &velocity_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &velocity_j,
    boost::optional<gtsam::Matrix &> h_pose_i = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_i = boost::none,
    boost::optional<gtsam::Matrix &> h_pose_j = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_j = boost::none) const override {
    const auto error_function =
      [this](
        const gtsam::Pose3 &left_pose,
        const gtsam::Vector3 &left_velocity,
        const gtsam::Pose3 &right_pose,
        const gtsam::Vector3 &right_velocity) {
        return Evaluate(left_pose, left_velocity, right_pose, right_velocity);
      };
    if (h_pose_i) {
      *h_pose_i = gtsam::numericalDerivative41<
        gtsam::Vector1,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::Pose3,
        gtsam::Vector3>(error_function, pose_i, velocity_i, pose_j, velocity_j);
    }
    if (h_velocity_i) {
      *h_velocity_i = (gtsam::Matrix(1, 3) << 0.0, 0.0, -0.5 * dt_s_).finished();
    }
    if (h_pose_j) {
      *h_pose_j = gtsam::numericalDerivative43<
        gtsam::Vector1,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::Pose3,
        gtsam::Vector3>(error_function, pose_i, velocity_i, pose_j, velocity_j);
    }
    if (h_velocity_j) {
      *h_velocity_j = (gtsam::Matrix(1, 3) << 0.0, 0.0, -0.5 * dt_s_).finished();
    }
    return Evaluate(pose_i, velocity_i, pose_j, velocity_j);
  }

 private:
  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &velocity_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &velocity_j) const {
    const double delta_z = pose_j.translation().z() - pose_i.translation().z();
    const double trapezoid_delta_z = 0.5 * dt_s_ * (velocity_i.z() + velocity_j.z());
    return gtsam::Vector1(delta_z - trapezoid_delta_z);
  }

  double dt_s_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
