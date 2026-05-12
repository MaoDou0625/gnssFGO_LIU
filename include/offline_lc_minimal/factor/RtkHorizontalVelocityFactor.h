#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class RtkHorizontalVelocityFactor final
    : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  RtkHorizontalVelocityFactor(
    gtsam::Key velocity_key,
    const gtsam::Vector2 &rtk_horizontal_velocity_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, velocity_key),
        rtk_horizontal_velocity_mps_(rtk_horizontal_velocity_mps) {
    if (!rtk_horizontal_velocity_mps_.allFinite()) {
      throw std::invalid_argument("RTK horizontal velocity measurement must be finite");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new RtkHorizontalVelocityFactor(*this)));
  }

  [[nodiscard]] const gtsam::Vector2 &rtkHorizontalVelocityMps() const {
    return rtk_horizontal_velocity_mps_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &nav_velocity,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override {
    if (h_velocity) {
      *h_velocity = (gtsam::Matrix(2, 3) <<
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0).finished();
    }
    return (gtsam::Vector2() <<
      nav_velocity.x() - rtk_horizontal_velocity_mps_.x(),
      nav_velocity.y() - rtk_horizontal_velocity_mps_.y()).finished();
  }

 private:
  gtsam::Vector2 rtk_horizontal_velocity_mps_ = gtsam::Vector2::Zero();
};

}  // namespace offline_lc_minimal::factor
