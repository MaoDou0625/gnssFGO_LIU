#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VelocityDeltaFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  VelocityDeltaFactor(
    gtsam::Key velocity_i_key,
    gtsam::Key velocity_j_key,
    const gtsam::Vector3 &target_delta_v_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(
          model,
          velocity_i_key,
          velocity_j_key),
        target_delta_v_mps_(target_delta_v_mps) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VelocityDeltaFactor(*this)));
  }

  [[nodiscard]] const gtsam::Vector3 &targetDeltaVMps() const {
    return target_delta_v_mps_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity_i,
    const gtsam::Vector3 &velocity_j,
    boost::optional<gtsam::Matrix &> h_i = boost::none,
    boost::optional<gtsam::Matrix &> h_j = boost::none) const override {
    if (h_i) {
      *h_i = -gtsam::I_3x3;
    }
    if (h_j) {
      *h_j = gtsam::I_3x3;
    }
    return (velocity_j - velocity_i) - target_delta_v_mps_;
  }

 private:
  gtsam::Vector3 target_delta_v_mps_ = gtsam::Vector3::Zero();
};

}  // namespace offline_lc_minimal::factor
