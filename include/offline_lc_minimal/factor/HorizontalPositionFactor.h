#pragma once

#include <boost/bind/bind.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class HorizontalPositionFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  HorizontalPositionFactor(
    gtsam::Key pose_key,
    const gtsam::Point2 &measured_position,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        measured_position_(measured_position) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new HorizontalPositionFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h = boost::none) const override {
    if (h) {
      using boost::placeholders::_1;
      *h = gtsam::numericalDerivative11<gtsam::Vector2, gtsam::Pose3>(
        boost::bind(&HorizontalPositionFactor::EvaluateErrorNoJacobians, this, _1),
        pose,
        1e-5);
    }

    return EvaluateErrorNoJacobians(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector2 EvaluateErrorNoJacobians(const gtsam::Pose3 &pose) const {
    const auto translation = pose.translation();
    return gtsam::Vector2(
      translation.x() - measured_position_.x(),
      translation.y() - measured_position_.y());
  }
  gtsam::Point2 measured_position_;
};

}  // namespace offline_lc_minimal::factor
