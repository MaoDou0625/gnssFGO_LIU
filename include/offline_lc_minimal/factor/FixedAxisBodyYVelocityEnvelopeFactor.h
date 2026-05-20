#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

[[nodiscard]] inline gtsam::Vector3 NormalizeBodyYAxisNav(
  const gtsam::Vector3 &body_y_axis_nav) {
  if (!body_y_axis_nav.allFinite()) {
    throw std::invalid_argument("body-y axis must be finite");
  }
  const double norm = body_y_axis_nav.norm();
  if (norm <= 1.0e-12) {
    throw std::invalid_argument("body-y axis must be non-zero");
  }
  return body_y_axis_nav / norm;
}

[[nodiscard]] inline double FixedAxisBodyYEnvelopeResidualMps(
  const gtsam::Vector3 &body_y_axis_nav,
  const gtsam::Vector3 &nav_velocity,
  const double mean_body_y_mps,
  const double deadband_mps) {
  if (!nav_velocity.allFinite() ||
      !std::isfinite(mean_body_y_mps) ||
      !std::isfinite(deadband_mps) ||
      deadband_mps < 0.0) {
    throw std::invalid_argument("body-y envelope residual input must be finite");
  }
  const double centered_body_y_mps =
    body_y_axis_nav.dot(nav_velocity) - mean_body_y_mps;
  const double abs_centered_body_y_mps = std::abs(centered_body_y_mps);
  if (abs_centered_body_y_mps <= deadband_mps) {
    return 0.0;
  }
  return centered_body_y_mps > 0.0
    ? centered_body_y_mps - deadband_mps
    : centered_body_y_mps + deadband_mps;
}

class FixedAxisBodyYVelocityEnvelopeFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  FixedAxisBodyYVelocityEnvelopeFactor(
    gtsam::Key velocity_key,
    const gtsam::Vector3 &body_y_axis_nav,
    double mean_body_y_mps,
    double deadband_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, velocity_key),
        body_y_axis_nav_(NormalizeBodyYAxisNav(body_y_axis_nav)),
        mean_body_y_mps_(mean_body_y_mps),
        deadband_mps_(deadband_mps) {
    if (!std::isfinite(mean_body_y_mps_) ||
        !std::isfinite(deadband_mps_) ||
        deadband_mps_ < 0.0) {
      throw std::invalid_argument("body-y envelope parameters must be finite");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new FixedAxisBodyYVelocityEnvelopeFactor(*this)));
  }

  [[nodiscard]] const gtsam::Vector3 &bodyYAxisNav() const {
    return body_y_axis_nav_;
  }

  [[nodiscard]] double meanBodyYMps() const {
    return mean_body_y_mps_;
  }

  [[nodiscard]] double deadbandMps() const {
    return deadband_mps_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &nav_velocity,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override {
    const double centered_body_y_mps =
      body_y_axis_nav_.dot(nav_velocity) - mean_body_y_mps_;
    const bool outside_deadband =
      std::abs(centered_body_y_mps) > deadband_mps_;
    if (h_velocity) {
      *h_velocity = gtsam::Matrix::Zero(1, 3);
      if (outside_deadband) {
        (*h_velocity).row(0) = body_y_axis_nav_.transpose();
      }
    }
    return gtsam::Vector1(
      outside_deadband
        ? FixedAxisBodyYEnvelopeResidualMps(
            body_y_axis_nav_,
            nav_velocity,
            mean_body_y_mps_,
            deadband_mps_)
        : 0.0);
  }

 private:
  gtsam::Vector3 body_y_axis_nav_ = gtsam::Vector3::UnitY();
  double mean_body_y_mps_ = 0.0;
  double deadband_mps_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
