#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class AttitudeHoldFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  AttitudeHoldFactor(
    gtsam::Key pose_key,
    const gtsam::Rot3 &reference_rotation,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        reference_rotation_(reference_rotation) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new AttitudeHoldFactor(*this)));
  }

  [[nodiscard]] const gtsam::Rot3 &referenceRotation() const {
    return reference_rotation_;
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h_pose = boost::none) const override {
    const auto error_function = [this](const gtsam::Pose3 &candidate_pose) {
      return gtsam::Rot3::Logmap(
        reference_rotation_.between(candidate_pose.rotation()));
    };
    if (h_pose) {
      *h_pose = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
        error_function,
        pose);
    }
    return error_function(pose);
  }

 private:
  gtsam::Rot3 reference_rotation_;
};

}  // namespace offline_lc_minimal::factor
