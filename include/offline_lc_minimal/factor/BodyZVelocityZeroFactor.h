#pragma once

#include <functional>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

[[nodiscard]] inline double BodyZVelocityMps(
  const gtsam::Pose3 &pose,
  const gtsam::Vector3 &nav_velocity) {
  return pose.rotation().unrotate(nav_velocity).z();
}

[[nodiscard]] inline gtsam::Matrix BodyZVelocityPoseJacobian(
  const gtsam::Pose3 &pose,
  const gtsam::Vector3 &nav_velocity) {
  const std::function<gtsam::Vector1(const gtsam::Pose3 &)> error_function =
    [&nav_velocity](const gtsam::Pose3 &current_pose) {
      return gtsam::Vector1(BodyZVelocityMps(current_pose, nav_velocity));
    };
  return gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
    error_function,
    pose);
}

[[nodiscard]] inline gtsam::Matrix BodyZVelocityVelocityJacobian(const gtsam::Pose3 &pose) {
  return pose.rotation().matrix().transpose().row(2);
}

class BodyZVelocityZeroFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
 public:
  BodyZVelocityZeroFactor(
    gtsam::Key pose_key,
    gtsam::Key velocity_key,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(
          model,
          pose_key,
          velocity_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new BodyZVelocityZeroFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    const gtsam::Vector3 &nav_velocity,
    boost::optional<gtsam::Matrix &> h_pose = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override {
    if (h_pose) {
      *h_pose = BodyZVelocityPoseJacobian(pose, nav_velocity);
    }
    if (h_velocity) {
      *h_velocity = BodyZVelocityVelocityJacobian(pose);
    }
    return gtsam::Vector1(BodyZVelocityMps(pose, nav_velocity));
  }
};

}  // namespace offline_lc_minimal::factor
