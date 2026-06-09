#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class Stage2VerticalIncrementHoldFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 public:
  Stage2VerticalIncrementHoldFactor(
    gtsam::Key pose_i_key,
    gtsam::Key pose_j_key,
    double reference_delta_z_m,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
          model,
          pose_i_key,
          pose_j_key),
        reference_delta_z_m_(reference_delta_z_m) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new Stage2VerticalIncrementHoldFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j,
    boost::optional<gtsam::Matrix &> h_pose_i = boost::none,
    boost::optional<gtsam::Matrix &> h_pose_j = boost::none) const override {
    const auto error_function =
      [this](const gtsam::Pose3 &left_pose, const gtsam::Pose3 &right_pose) {
        return Evaluate(left_pose, right_pose);
      };
    if (h_pose_i) {
      *h_pose_i =
        gtsam::numericalDerivative21<gtsam::Vector1, gtsam::Pose3, gtsam::Pose3>(
          error_function,
          pose_i,
          pose_j);
    }
    if (h_pose_j) {
      *h_pose_j =
        gtsam::numericalDerivative22<gtsam::Vector1, gtsam::Pose3, gtsam::Pose3>(
          error_function,
          pose_i,
          pose_j);
    }
    return Evaluate(pose_i, pose_j);
  }

 private:
  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Pose3 &pose_j) const {
    const double optimized_delta_z_m =
      pose_j.translation().z() - pose_i.translation().z();
    return gtsam::Vector1(optimized_delta_z_m - reference_delta_z_m_);
  }

  double reference_delta_z_m_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
