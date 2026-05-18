#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalAccelBiasPriorFactor final
    : public gtsam::NoiseModelFactor1<gtsam::imuBias::ConstantBias> {
 public:
  VerticalAccelBiasPriorFactor(
    gtsam::Key bias_key,
    double target_ba_z_mps2,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::imuBias::ConstantBias>(model, bias_key),
        target_ba_z_mps2_(target_ba_z_mps2) {
    if (!std::isfinite(target_ba_z_mps2_)) {
      throw std::invalid_argument("VerticalAccelBiasPriorFactor requires finite target ba_z");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalAccelBiasPriorFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::imuBias::ConstantBias &bias,
    boost::optional<gtsam::Matrix &> h_bias = boost::none) const override {
    if (h_bias) {
      h_bias->setZero(1, 6);
      (*h_bias)(0, 2) = 1.0;
    }
    return gtsam::Vector1(bias.accelerometer().z() - target_ba_z_mps2_);
  }

 private:
  double target_ba_z_mps2_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
