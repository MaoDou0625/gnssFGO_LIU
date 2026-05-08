#pragma once

#include <boost/pointer_cast.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticVerticalPositionHoldFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  StaticVerticalPositionHoldFactor(
    gtsam::Key reference_pose_key,
    gtsam::Key pose_key,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
          noise_model,
          reference_pose_key,
          pose_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticVerticalPositionHoldFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &reference_pose,
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    if (h1) {
      gtsam::Matrix16 jacobian = gtsam::Matrix16::Zero();
      jacobian(0, 5) = -1.0;
      *h1 = jacobian;
    }
    if (h2) {
      gtsam::Matrix16 jacobian = gtsam::Matrix16::Zero();
      jacobian(0, 5) = 1.0;
      *h2 = jacobian;
    }

    return (gtsam::Vector1() << pose.translation().z() - reference_pose.translation().z()).finished();
  }
};

}  // namespace offline_lc_minimal::factor
