#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalVelocityDeltaFactor final : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  VerticalVelocityDeltaFactor(
    gtsam::Key velocity_i_key,
    gtsam::Key velocity_j_key,
    double target_delta_vz_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(model, velocity_i_key, velocity_j_key),
        target_delta_vz_mps_(target_delta_vz_mps) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalVelocityDeltaFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity_i,
    const gtsam::Vector3 &velocity_j,
    boost::optional<gtsam::Matrix &> h_i = boost::none,
    boost::optional<gtsam::Matrix &> h_j = boost::none) const override {
    if (h_i) {
      *h_i = (gtsam::Matrix(1, 3) << 0.0, 0.0, -1.0).finished();
    }
    if (h_j) {
      *h_j = (gtsam::Matrix(1, 3) << 0.0, 0.0, 1.0).finished();
    }
    return gtsam::Vector1((velocity_j.z() - velocity_i.z()) - target_delta_vz_mps_);
  }

 private:
  double target_delta_vz_mps_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
