#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "offline_lc_minimal/factor/BodyZVelocityZeroFactor.h"

namespace offline_lc_minimal::factor {

struct BodyZHorizontalLeakageModel {
  double leak_x_rad = 0.0;
  double leak_y_rad = 0.0;
};

[[nodiscard]] inline double BodyXVelocityMps(
  const BodyFrameAxesNav &axes,
  const gtsam::Vector3 &nav_velocity) {
  return axes.body_x_axis_nav.dot(nav_velocity);
}

[[nodiscard]] inline double BodyYVelocityMps(
  const BodyFrameAxesNav &axes,
  const gtsam::Vector3 &nav_velocity) {
  return axes.body_y_axis_nav.dot(nav_velocity);
}

[[nodiscard]] inline double BodyZRawVelocityMps(
  const BodyFrameAxesNav &axes,
  const gtsam::Vector3 &nav_velocity) {
  return axes.body_z_axis_nav.dot(nav_velocity);
}

[[nodiscard]] inline double BodyZLeakageCorrectionMps(
  const BodyFrameAxesNav &axes,
  const BodyZHorizontalLeakageModel &leakage,
  const gtsam::Vector3 &nav_velocity) {
  return leakage.leak_x_rad * BodyXVelocityMps(axes, nav_velocity) +
         leakage.leak_y_rad * BodyYVelocityMps(axes, nav_velocity);
}

[[nodiscard]] inline double BodyZLeakageCorrectedVelocityMps(
  const BodyFrameAxesNav &axes,
  const BodyZHorizontalLeakageModel &leakage,
  const gtsam::Vector3 &nav_velocity) {
  return BodyZRawVelocityMps(axes, nav_velocity) -
         BodyZLeakageCorrectionMps(axes, leakage, nav_velocity);
}

[[nodiscard]] inline gtsam::Matrix BodyZLeakageCorrectedVelocityJacobian(
  const BodyFrameAxesNav &axes,
  const BodyZHorizontalLeakageModel &leakage) {
  gtsam::Matrix jacobian(1, 3);
  jacobian.row(0) =
    axes.body_z_axis_nav.transpose() -
    leakage.leak_x_rad * axes.body_x_axis_nav.transpose() -
    leakage.leak_y_rad * axes.body_y_axis_nav.transpose();
  return jacobian;
}

[[nodiscard]] inline BodyFrameAxesNav NormalizeBodyFrameAxesNav(const BodyFrameAxesNav &axes) {
  BodyFrameAxesNav normalized;
  normalized.body_x_axis_nav = NormalizeBodyZAxisNav(axes.body_x_axis_nav);
  normalized.body_y_axis_nav = NormalizeBodyZAxisNav(axes.body_y_axis_nav);
  normalized.body_z_axis_nav = NormalizeBodyZAxisNav(axes.body_z_axis_nav);
  return normalized;
}

[[nodiscard]] inline BodyZHorizontalLeakageModel ValidateBodyZHorizontalLeakageModel(
  const BodyZHorizontalLeakageModel &leakage) {
  if (!std::isfinite(leakage.leak_x_rad) || !std::isfinite(leakage.leak_y_rad)) {
    throw std::invalid_argument("Body-Z horizontal leakage model must be finite");
  }
  return leakage;
}

class BodyZLeakageCorrectedVelocityZeroFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  BodyZLeakageCorrectedVelocityZeroFactor(
    gtsam::Key velocity_key,
    const BodyFrameAxesNav &body_axes_nav,
    const BodyZHorizontalLeakageModel &leakage,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(
          model,
          velocity_key),
        body_axes_nav_(NormalizeBodyFrameAxesNav(body_axes_nav)),
        leakage_(ValidateBodyZHorizontalLeakageModel(leakage)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new BodyZLeakageCorrectedVelocityZeroFactor(*this)));
  }

  [[nodiscard]] const BodyFrameAxesNav &bodyAxesNav() const {
    return body_axes_nav_;
  }

  [[nodiscard]] const BodyZHorizontalLeakageModel &leakage() const {
    return leakage_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &nav_velocity,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override {
    if (h_velocity) {
      *h_velocity = BodyZLeakageCorrectedVelocityJacobian(body_axes_nav_, leakage_);
    }
    return gtsam::Vector1(
      BodyZLeakageCorrectedVelocityMps(body_axes_nav_, leakage_, nav_velocity));
  }

 private:
  BodyFrameAxesNav body_axes_nav_;
  BodyZHorizontalLeakageModel leakage_;
};

}  // namespace offline_lc_minimal::factor
