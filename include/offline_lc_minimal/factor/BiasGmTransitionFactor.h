#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class BiasGmTransitionFactor
    : public gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::imuBias::ConstantBias> {
 public:
  BiasGmTransitionFactor(
    gtsam::Key bias_i_key,
    gtsam::Key bias_j_key,
    gtsam::Matrix3 phi_acc,
    gtsam::Matrix3 phi_gyro,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::imuBias::ConstantBias>(
        noise_model,
        bias_i_key,
        bias_j_key),
        phi_acc_(std::move(phi_acc)),
        phi_gyro_(std::move(phi_gyro)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new BiasGmTransitionFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::imuBias::ConstantBias &bias_i,
    const gtsam::imuBias::ConstantBias &bias_j,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    if (h1) {
      gtsam::Matrix66 jacobian = gtsam::Matrix66::Zero();
      jacobian.block<3, 3>(0, 0) = -phi_acc_;
      jacobian.block<3, 3>(3, 3) = -phi_gyro_;
      *h1 = jacobian;
    }
    if (h2) {
      *h2 = gtsam::Matrix66::Identity();
    }

    gtsam::Vector6 residual;
    residual.segment<3>(0) = bias_j.accelerometer() - phi_acc_ * bias_i.accelerometer();
    residual.segment<3>(3) = bias_j.gyroscope() - phi_gyro_ * bias_i.gyroscope();
    return residual;
  }

 private:
  gtsam::Matrix3 phi_acc_;
  gtsam::Matrix3 phi_gyro_;
};

}  // namespace offline_lc_minimal::factor
