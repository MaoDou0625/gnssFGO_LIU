#pragma once

#include <boost/pointer_cast.hpp>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticPositionHoldFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  StaticPositionHoldFactor(
    gtsam::Key reference_pose_key,
    gtsam::Key pose_key,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
          noise_model,
          reference_pose_key,
          pose_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticPositionHoldFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &reference_pose,
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    if (h1) {
      gtsam::Matrix36 jacobian = gtsam::Matrix36::Zero();
      jacobian.block<3, 3>(0, 3) = -reference_pose.rotation().matrix();
      *h1 = jacobian;
    }
    if (h2) {
      gtsam::Matrix36 jacobian = gtsam::Matrix36::Zero();
      jacobian.block<3, 3>(0, 3) = pose.rotation().matrix();
      *h2 = jacobian;
    }

    return (gtsam::Vector3() << pose.translation().x() - reference_pose.translation().x(),
            pose.translation().y() - reference_pose.translation().y(),
            pose.translation().z() - reference_pose.translation().z())
      .finished();
  }
};

}  // namespace offline_lc_minimal::factor
