#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalAccelBiasLocalContinuityFactor final
    : public gtsam::NoiseModelFactor2<
        gtsam::imuBias::ConstantBias,
        gtsam::imuBias::ConstantBias> {
 public:
  VerticalAccelBiasLocalContinuityFactor(
    gtsam::Key bias_i_key,
    gtsam::Key bias_j_key,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<
          gtsam::imuBias::ConstantBias,
          gtsam::imuBias::ConstantBias>(
          model,
          bias_i_key,
          bias_j_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new VerticalAccelBiasLocalContinuityFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::imuBias::ConstantBias &bias_i,
    const gtsam::imuBias::ConstantBias &bias_j,
    boost::optional<gtsam::Matrix &> h_bias_i = boost::none,
    boost::optional<gtsam::Matrix &> h_bias_j = boost::none) const override {
    if (h_bias_i) {
      h_bias_i->setZero(1, 6);
      (*h_bias_i)(0, 2) = -1.0;
    }
    if (h_bias_j) {
      h_bias_j->setZero(1, 6);
      (*h_bias_j)(0, 2) = 1.0;
    }
    return gtsam::Vector1(
      bias_j.accelerometer().z() - bias_i.accelerometer().z());
  }
};

}  // namespace offline_lc_minimal::factor
