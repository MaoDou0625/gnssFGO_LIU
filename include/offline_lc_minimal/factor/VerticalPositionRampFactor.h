#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalPositionRampFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3> {
 public:
  VerticalPositionRampFactor(
    gtsam::Key start_pose_key,
    gtsam::Key interior_pose_key,
    gtsam::Key end_pose_key,
    double alpha,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(
          model,
          start_pose_key,
          interior_pose_key,
          end_pose_key),
        alpha_(alpha) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalPositionRampFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &start_pose,
    const gtsam::Pose3 &interior_pose,
    const gtsam::Pose3 &end_pose,
    boost::optional<gtsam::Matrix &> h_start = boost::none,
    boost::optional<gtsam::Matrix &> h_interior = boost::none,
    boost::optional<gtsam::Matrix &> h_end = boost::none) const override {
    const auto error_function =
      [this](
        const gtsam::Pose3 &start,
        const gtsam::Pose3 &interior,
        const gtsam::Pose3 &end) {
        return Evaluate(start, interior, end);
      };
    if (h_start) {
      *h_start = gtsam::numericalDerivative31<gtsam::Vector1, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        start_pose,
        interior_pose,
        end_pose);
    }
    if (h_interior) {
      *h_interior = gtsam::numericalDerivative32<gtsam::Vector1, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        start_pose,
        interior_pose,
        end_pose);
    }
    if (h_end) {
      *h_end = gtsam::numericalDerivative33<gtsam::Vector1, gtsam::Pose3, gtsam::Pose3, gtsam::Pose3>(
        error_function,
        start_pose,
        interior_pose,
        end_pose);
    }
    return Evaluate(start_pose, interior_pose, end_pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &start_pose,
    const gtsam::Pose3 &interior_pose,
    const gtsam::Pose3 &end_pose) const {
    const double expected_z =
      (1.0 - alpha_) * start_pose.translation().z() + alpha_ * end_pose.translation().z();
    return gtsam::Vector1(interior_pose.translation().z() - expected_z);
  }

  double alpha_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
