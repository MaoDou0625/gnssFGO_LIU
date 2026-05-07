#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

[[nodiscard]] inline gtsam::Vector3 NormalizeBodyZAxisNav(const gtsam::Vector3 &body_z_axis_nav) {
  const double norm = body_z_axis_nav.norm();
  if (!body_z_axis_nav.allFinite() || !std::isfinite(norm) || norm <= 0.0) {
    throw std::invalid_argument("Body-Z NHC fixed axis must be finite and non-zero");
  }
  return body_z_axis_nav / norm;
}

[[nodiscard]] inline gtsam::Vector3 BodyZAxisNavFromPose(const gtsam::Pose3 &pose) {
  return NormalizeBodyZAxisNav(pose.rotation().matrix().col(2));
}

[[nodiscard]] inline double BodyZVelocityMps(
  const gtsam::Vector3 &body_z_axis_nav,
  const gtsam::Vector3 &nav_velocity) {
  return body_z_axis_nav.dot(nav_velocity);
}

[[nodiscard]] inline gtsam::Matrix BodyZVelocityVelocityJacobian(
  const gtsam::Vector3 &body_z_axis_nav) {
  gtsam::Matrix jacobian(1, 3);
  jacobian.row(0) = body_z_axis_nav.transpose();
  return jacobian;
}

class BodyZVelocityZeroFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  BodyZVelocityZeroFactor(
    gtsam::Key velocity_key,
    const gtsam::Vector3 &body_z_axis_nav,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(
          model,
          velocity_key),
        body_z_axis_nav_(NormalizeBodyZAxisNav(body_z_axis_nav)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new BodyZVelocityZeroFactor(*this)));
  }

  [[nodiscard]] const gtsam::Vector3 &bodyZAxisNav() const {
    return body_z_axis_nav_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &nav_velocity,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override {
    if (h_velocity) {
      *h_velocity = BodyZVelocityVelocityJacobian(body_z_axis_nav_);
    }
    return gtsam::Vector1(BodyZVelocityMps(body_z_axis_nav_, nav_velocity));
  }

 private:
  gtsam::Vector3 body_z_axis_nav_;
};

}  // namespace offline_lc_minimal::factor
