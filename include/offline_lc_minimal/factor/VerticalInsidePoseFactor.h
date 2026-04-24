#pragma once

#include <boost/bind/bind.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalInsidePoseFactor : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  VerticalInsidePoseFactor(
    gtsam::Key pose_key,
    const gtsam::Pose3 &reference_pose,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(noise_model, pose_key),
        reference_pose_(reference_pose) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalInsidePoseFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h = boost::none) const override {
    if (h) {
      using boost::placeholders::_1;
      *h = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(
        boost::bind(&VerticalInsidePoseFactor::EvaluateErrorNoJacobians, this, _1),
        pose,
        1e-6);
    }
    return EvaluateErrorNoJacobians(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector3 EvaluateErrorNoJacobians(const gtsam::Pose3 &pose) const {
    const gtsam::Vector3 reference_rpy = reference_pose_.rotation().rpy();
    const gtsam::Vector3 pose_rpy = pose.rotation().rpy();
    return gtsam::Vector3(
      WrapAngle(pose_rpy.x() - reference_rpy.x()),
      WrapAngle(pose_rpy.y() - reference_rpy.y()),
      pose.translation().z() - reference_pose_.translation().z());
  }

  [[nodiscard]] static double WrapAngle(const double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
  }

  gtsam::Pose3 reference_pose_;
};

}  // namespace offline_lc_minimal::factor
