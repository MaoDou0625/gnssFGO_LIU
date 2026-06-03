#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticAttitudeDriftFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  StaticAttitudeDriftFactor(
    gtsam::Key pose_i_key,
    gtsam::Key pose_j_key,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(noise_model, pose_i_key, pose_j_key) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticAttitudeDriftFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    const std::function<gtsam::Vector3(const gtsam::Pose3 &, const gtsam::Pose3 &)> error_function =
      [](const gtsam::Pose3 &candidate_pose_i, const gtsam::Pose3 &candidate_pose_j) {
        const gtsam::Rot3 relative_rotation = candidate_pose_i.rotation().between(candidate_pose_j.rotation());
        return gtsam::Rot3::Logmap(relative_rotation);
      };
    if (h1) {
      *h1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        pose_i,
        pose_j);
    }
    if (h2) {
      *h2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        pose_i,
        pose_j);
    }
    return error_function(pose_i, pose_j);
  }
};

}  // namespace offline_lc_minimal::factor
