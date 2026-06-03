#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalAccelBiasGmTransitionFactor
    : public gtsam::NoiseModelFactor3<gtsam::imuBias::ConstantBias, gtsam::imuBias::ConstantBias, gtsam::Vector3> {
 public:
  VerticalAccelBiasGmTransitionFactor(
    gtsam::Key bias_i_key,
    gtsam::Key bias_j_key,
    gtsam::Key global_acc_bias_key,
    const double phi,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor3<gtsam::imuBias::ConstantBias, gtsam::imuBias::ConstantBias, gtsam::Vector3>(
        noise_model,
        bias_i_key,
        bias_j_key,
        global_acc_bias_key),
        phi_(phi) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalAccelBiasGmTransitionFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::imuBias::ConstantBias &bias_i,
    const gtsam::imuBias::ConstantBias &bias_j,
    const gtsam::Vector3 &global_acc_bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none,
    boost::optional<gtsam::Matrix &> h3 = boost::none) const override {
    if (h1) {
      gtsam::Matrix16 jacobian = gtsam::Matrix16::Zero();
      jacobian(0, 2) = -phi_;
      *h1 = jacobian;
    }
    if (h2) {
      gtsam::Matrix16 jacobian = gtsam::Matrix16::Zero();
      jacobian(0, 2) = 1.0;
      *h2 = jacobian;
    }
    if (h3) {
      gtsam::Matrix13 jacobian = gtsam::Matrix13::Zero();
      jacobian(0, 2) = -(1.0 - phi_);
      *h3 = jacobian;
    }

    const double mean_baz = global_acc_bias.z();
    const double predicted_baz = mean_baz + phi_ * (bias_i.accelerometer().z() - mean_baz);
    return (gtsam::Vector1() << bias_j.accelerometer().z() - predicted_baz).finished();
  }

 private:
  double phi_ = 1.0;
};

}  // namespace offline_lc_minimal::factor
