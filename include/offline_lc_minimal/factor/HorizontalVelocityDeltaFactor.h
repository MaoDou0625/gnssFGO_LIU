#pragma once

#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class HorizontalVelocityDeltaFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3> {
 public:
  HorizontalVelocityDeltaFactor(
    gtsam::Key velocity_i_key,
    gtsam::Key velocity_j_key,
    const gtsam::Vector2 &target_delta_v_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Vector3, gtsam::Vector3>(
          model,
          velocity_i_key,
          velocity_j_key),
        target_delta_v_mps_(target_delta_v_mps) {
    if (!target_delta_v_mps_.allFinite()) {
      throw std::invalid_argument("HorizontalVelocityDeltaFactor requires finite target");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new HorizontalVelocityDeltaFactor(*this)));
  }

  [[nodiscard]] const gtsam::Vector2 &targetDeltaVMps() const {
    return target_delta_v_mps_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity_i,
    const gtsam::Vector3 &velocity_j,
    boost::optional<gtsam::Matrix &> h_i = boost::none,
    boost::optional<gtsam::Matrix &> h_j = boost::none) const override {
    if (h_i) {
      *h_i = (gtsam::Matrix(2, 3) <<
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0).finished();
    }
    if (h_j) {
      *h_j = (gtsam::Matrix(2, 3) <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0).finished();
    }
    return (gtsam::Vector2() <<
      velocity_j.x() - velocity_i.x() - target_delta_v_mps_.x(),
      velocity_j.y() - velocity_i.y() - target_delta_v_mps_.y()).finished();
  }

 private:
  gtsam::Vector2 target_delta_v_mps_ = gtsam::Vector2::Zero();
};

}  // namespace offline_lc_minimal::factor
