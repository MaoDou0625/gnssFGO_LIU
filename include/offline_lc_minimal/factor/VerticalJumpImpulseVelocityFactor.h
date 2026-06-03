#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalJumpImpulseVelocityFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, double> {
 public:
  VerticalJumpImpulseVelocityFactor(
    gtsam::Key velocity_i_key,
    gtsam::Key velocity_j_key,
    gtsam::Key impulse_key,
    double imu_delta_vz_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, double>(
          model,
          velocity_i_key,
          velocity_j_key,
          impulse_key),
        imu_delta_vz_mps_(imu_delta_vz_mps) {
    if (!std::isfinite(imu_delta_vz_mps_)) {
      throw std::invalid_argument("VerticalJumpImpulseVelocityFactor requires finite IMU delta-vz");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalJumpImpulseVelocityFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity_i,
    const gtsam::Vector3 &velocity_j,
    const double &jump_impulse_mps,
    boost::optional<gtsam::Matrix &> h_velocity_i = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_j = boost::none,
    boost::optional<gtsam::Matrix &> h_impulse = boost::none) const override {
    if (h_velocity_i) {
      *h_velocity_i = (gtsam::Matrix(1, 3) << 0.0, 0.0, -1.0).finished();
    }
    if (h_velocity_j) {
      *h_velocity_j = (gtsam::Matrix(1, 3) << 0.0, 0.0, 1.0).finished();
    }
    if (h_impulse) {
      *h_impulse = gtsam::Matrix11::Identity();
    }
    return gtsam::Vector1((velocity_j.z() - velocity_i.z()) - imu_delta_vz_mps_ + jump_impulse_mps);
  }

 private:
  double imu_delta_vz_mps_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
