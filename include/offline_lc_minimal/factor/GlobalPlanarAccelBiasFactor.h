#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class GlobalPlanarAccelBiasFactor : public gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::Vector3> {
 public:
  GlobalPlanarAccelBiasFactor(
    gtsam::Key bias_key,
    gtsam::Key global_acc_bias_key,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::Vector3>(
        noise_model,
        bias_key,
        global_acc_bias_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GlobalPlanarAccelBiasFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::imuBias::ConstantBias &bias,
    const gtsam::Vector3 &global_acc_bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    if (h1) {
      gtsam::Matrix26 jacobian = gtsam::Matrix26::Zero();
      jacobian(0, 0) = 1.0;
      jacobian(1, 1) = 1.0;
      *h1 = jacobian;
    }
    if (h2) {
      gtsam::Matrix23 jacobian = gtsam::Matrix23::Zero();
      jacobian(0, 0) = -1.0;
      jacobian(1, 1) = -1.0;
      *h2 = jacobian;
    }
    return (gtsam::Vector2() << bias.accelerometer().x() - global_acc_bias.x(),
            bias.accelerometer().y() - global_acc_bias.y())
      .finished();
  }
};

}  // namespace offline_lc_minimal::factor
