#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class GlobalAccelBiasFactor : public gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::Vector3> {
 public:
  GlobalAccelBiasFactor(
    gtsam::Key bias_key,
    gtsam::Key global_acc_bias_key,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::imuBias::ConstantBias, gtsam::Vector3>(
        noise_model,
        bias_key,
        global_acc_bias_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GlobalAccelBiasFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::imuBias::ConstantBias &bias,
    const gtsam::Vector3 &global_acc_bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    if (h1) {
      gtsam::Matrix36 jacobian = gtsam::Matrix36::Zero();
      jacobian.block<3, 3>(0, 0) = gtsam::I_3x3;
      *h1 = jacobian;
    }
    if (h2) {
      *h2 = -gtsam::I_3x3;
    }
    return bias.accelerometer() - global_acc_bias;
  }
};

}  // namespace offline_lc_minimal::factor
