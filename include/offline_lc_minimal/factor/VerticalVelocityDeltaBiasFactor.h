#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalVelocityDeltaBiasFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::imuBias::ConstantBias> {
 public:
  VerticalVelocityDeltaBiasFactor(
    gtsam::Key velocity_i_key,
    gtsam::Key velocity_j_key,
    gtsam::Key bias_i_key,
    double target_delta_vz_mps,
    double reference_ba_z_mps2,
    double dt_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor3<gtsam::Vector3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
          model,
          velocity_i_key,
          velocity_j_key,
          bias_i_key),
        target_delta_vz_mps_(target_delta_vz_mps),
        reference_ba_z_mps2_(reference_ba_z_mps2),
        dt_s_(dt_s) {
    if (!std::isfinite(target_delta_vz_mps_) || !std::isfinite(reference_ba_z_mps2_)) {
      throw std::invalid_argument("VerticalVelocityDeltaBiasFactor requires finite inputs");
    }
    if (!std::isfinite(dt_s_) || dt_s_ <= 0.0) {
      throw std::invalid_argument("VerticalVelocityDeltaBiasFactor requires positive dt");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalVelocityDeltaBiasFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity_i,
    const gtsam::Vector3 &velocity_j,
    const gtsam::imuBias::ConstantBias &bias_i,
    boost::optional<gtsam::Matrix &> h_velocity_i = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_j = boost::none,
    boost::optional<gtsam::Matrix &> h_bias_i = boost::none) const override {
    if (h_velocity_i) {
      *h_velocity_i = (gtsam::Matrix(1, 3) << 0.0, 0.0, -1.0).finished();
    }
    if (h_velocity_j) {
      *h_velocity_j = (gtsam::Matrix(1, 3) << 0.0, 0.0, 1.0).finished();
    }
    if (h_bias_i) {
      h_bias_i->setZero(1, 6);
      (*h_bias_i)(0, 2) = dt_s_;
    }
    const double bias_correction_mps =
      (bias_i.accelerometer().z() - reference_ba_z_mps2_) * dt_s_;
    return gtsam::Vector1(
      (velocity_j.z() - velocity_i.z()) - target_delta_vz_mps_ + bias_correction_mps);
  }

 private:
  double target_delta_vz_mps_ = 0.0;
  double reference_ba_z_mps2_ = 0.0;
  double dt_s_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
