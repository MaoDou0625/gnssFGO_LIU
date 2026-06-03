#pragma once

#include <boost/pointer_cast.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalVelocityPriorFactor final : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  VerticalVelocityPriorFactor(
    gtsam::Key velocity_key,
    const double target_vz_mps,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(noise_model, velocity_key),
        target_vz_mps_(target_vz_mps) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalVelocityPriorFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity,
    boost::optional<gtsam::Matrix &> h = boost::none) const override {
    if (h) {
      gtsam::Matrix13 jacobian = gtsam::Matrix13::Zero();
      jacobian(0, 2) = 1.0;
      *h = jacobian;
    }
    return gtsam::Vector1(velocity.z() - target_vz_mps_);
  }

 private:
  double target_vz_mps_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
