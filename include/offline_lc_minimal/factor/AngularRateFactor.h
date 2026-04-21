#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class AngularRateFactor : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::imuBias::ConstantBias> {
 public:
  AngularRateFactor(
    gtsam::Key omega_key,
    gtsam::Key bias_key,
    gtsam::Vector3 measured_angular_rate,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::imuBias::ConstantBias>(noise_model, omega_key, bias_key),
        measured_angular_rate_(std::move(measured_angular_rate)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new AngularRateFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &omega,
    const gtsam::imuBias::ConstantBias &bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    gtsam::Matrix36 bias_jacobian;
    const gtsam::Vector3 corrected = bias.correctGyroscope(measured_angular_rate_, h2 ? &bias_jacobian : nullptr);
    if (h1) {
      *h1 = gtsam::I_3x3;
    }
    if (h2) {
      *h2 = -bias_jacobian;
    }
    return omega - corrected;
  }

 private:
  gtsam::Vector3 measured_angular_rate_;
};

}  // namespace offline_lc_minimal::factor
