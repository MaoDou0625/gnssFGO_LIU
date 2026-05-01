#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalVelocityRampFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3> {
 public:
  VerticalVelocityRampFactor(
    gtsam::Key start_velocity_key,
    gtsam::Key interior_velocity_key,
    gtsam::Key end_velocity_key,
    double alpha,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::Vector3>(
          model,
          start_velocity_key,
          interior_velocity_key,
          end_velocity_key),
        alpha_(alpha) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalVelocityRampFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &start_velocity,
    const gtsam::Vector3 &interior_velocity,
    const gtsam::Vector3 &end_velocity,
    boost::optional<gtsam::Matrix &> h_start = boost::none,
    boost::optional<gtsam::Matrix &> h_interior = boost::none,
    boost::optional<gtsam::Matrix &> h_end = boost::none) const override {
    if (h_start) {
      *h_start = (gtsam::Matrix(1, 3) << 0.0, 0.0, -(1.0 - alpha_)).finished();
    }
    if (h_interior) {
      *h_interior = (gtsam::Matrix(1, 3) << 0.0, 0.0, 1.0).finished();
    }
    if (h_end) {
      *h_end = (gtsam::Matrix(1, 3) << 0.0, 0.0, -alpha_).finished();
    }
    const double expected_vz =
      (1.0 - alpha_) * start_velocity.z() + alpha_ * end_velocity.z();
    return gtsam::Vector1(interior_velocity.z() - expected_vz);
  }

 private:
  double alpha_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
