#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class AttitudeReferenceFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  AttitudeReferenceFactor(
    gtsam::Key pose_key,
    const gtsam::Rot3 &reference_rotation,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        reference_rotation_(reference_rotation) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new AttitudeReferenceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h_pose = boost::none) const override {
    const auto error_function = [this](const gtsam::Pose3 &candidate_pose) {
      return Evaluate(candidate_pose);
    };
    if (h_pose) {
      *h_pose = gtsam::numericalDerivative11<gtsam::Vector3, gtsam::Pose3>(error_function, pose);
    }
    return Evaluate(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector3 Evaluate(const gtsam::Pose3 &pose) const {
    return gtsam::Rot3::Logmap(reference_rotation_.between(pose.rotation()));
  }

  gtsam::Rot3 reference_rotation_;
};

}  // namespace offline_lc_minimal::factor
