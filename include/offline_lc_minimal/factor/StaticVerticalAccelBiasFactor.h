#pragma once

#include <boost/pointer_cast.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticVerticalAccelBiasFactor
    : public gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::Vector3> {
 public:
  StaticVerticalAccelBiasFactor(
    gtsam::Key bias_key,
    gtsam::Key global_acc_bias_key,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::Vector3>(
          noise_model,
          bias_key,
          global_acc_bias_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticVerticalAccelBiasFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::imuBias::ConstantBias &bias,
    const gtsam::Vector3 &global_acc_bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    if (h1) {
      gtsam::Matrix16 jacobian = gtsam::Matrix16::Zero();
      jacobian(0, 2) = 1.0;
      *h1 = jacobian;
    }
    if (h2) {
      gtsam::Matrix13 jacobian = gtsam::Matrix13::Zero();
      jacobian(0, 2) = -1.0;
      *h2 = jacobian;
    }

    return (gtsam::Vector1() << bias.accelerometer().z() - global_acc_bias.z()).finished();
  }
};

}  // namespace offline_lc_minimal::factor
