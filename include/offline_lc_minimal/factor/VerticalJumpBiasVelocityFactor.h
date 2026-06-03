#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalJumpBiasVelocityFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, double> {
 public:
  VerticalJumpBiasVelocityFactor(
    gtsam::Key velocity_i_key,
    gtsam::Key velocity_j_key,
    gtsam::Key jump_bias_key,
    double imu_delta_vz_mps,
    double dt_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, double>(
          model,
          velocity_i_key,
          velocity_j_key,
          jump_bias_key),
        imu_delta_vz_mps_(imu_delta_vz_mps),
        dt_s_(dt_s) {
    if (!std::isfinite(imu_delta_vz_mps_)) {
      throw std::invalid_argument("VerticalJumpBiasVelocityFactor requires finite IMU delta-vz");
    }
    if (!std::isfinite(dt_s_) || dt_s_ <= 0.0) {
      throw std::invalid_argument("VerticalJumpBiasVelocityFactor requires positive dt");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalJumpBiasVelocityFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity_i,
    const gtsam::Vector3 &velocity_j,
    const double &jump_bias_mps2,
    boost::optional<gtsam::Matrix &> h_velocity_i = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_j = boost::none,
    boost::optional<gtsam::Matrix &> h_jump_bias = boost::none) const override {
    if (h_velocity_i) {
      *h_velocity_i = (gtsam::Matrix(1, 3) << 0.0, 0.0, -1.0).finished();
    }
    if (h_velocity_j) {
      *h_velocity_j = (gtsam::Matrix(1, 3) << 0.0, 0.0, 1.0).finished();
    }
    if (h_jump_bias) {
      *h_jump_bias = (gtsam::Matrix(1, 1) << dt_s_).finished();
    }
    return gtsam::Vector1((velocity_j.z() - velocity_i.z()) - imu_delta_vz_mps_ + jump_bias_mps2 * dt_s_);
  }

 private:
  double imu_delta_vz_mps_ = 0.0;
  double dt_s_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
