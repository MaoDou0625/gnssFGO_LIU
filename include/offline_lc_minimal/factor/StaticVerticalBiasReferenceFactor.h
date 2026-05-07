#pragma once

#include <boost/pointer_cast.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticVerticalBiasReferenceFactor final : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  StaticVerticalBiasReferenceFactor(
    gtsam::Key global_acc_bias_key,
    const double static_baz_ref_mps2,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(noise_model, global_acc_bias_key),
        static_baz_ref_mps2_(static_baz_ref_mps2) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticVerticalBiasReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &global_acc_bias,
    boost::optional<gtsam::Matrix &> h = boost::none) const override {
    if (h) {
      gtsam::Matrix13 jacobian = gtsam::Matrix13::Zero();
      jacobian(0, 2) = 1.0;
      *h = jacobian;
    }
    return (gtsam::Vector1() << global_acc_bias.z() - static_baz_ref_mps2_).finished();
  }

 private:
  double static_baz_ref_mps2_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
