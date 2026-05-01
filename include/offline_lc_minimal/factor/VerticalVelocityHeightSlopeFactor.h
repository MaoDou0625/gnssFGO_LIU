#pragma once

#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalVelocityHeightSlopeFactor final
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3> {
 public:
  VerticalVelocityHeightSlopeFactor(
    gtsam::Key start_pose_key,
    gtsam::Key velocity_key,
    gtsam::Key end_pose_key,
    double duration_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::Pose3>(
          model,
          start_pose_key,
          velocity_key,
          end_pose_key),
        duration_s_(duration_s) {
    if (duration_s_ <= 0.0) {
      throw std::invalid_argument("VerticalVelocityHeightSlopeFactor requires positive duration");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalVelocityHeightSlopeFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &start_pose,
    const gtsam::Vector3 &velocity,
    const gtsam::Pose3 &end_pose,
    boost::optional<gtsam::Matrix &> h_start = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none,
    boost::optional<gtsam::Matrix &> h_end = boost::none) const override {
    const auto error_function =
      [this](
        const gtsam::Pose3 &start,
        const gtsam::Vector3 &current_velocity,
        const gtsam::Pose3 &end) {
        return Evaluate(start, current_velocity, end);
      };
    if (h_start) {
      *h_start = gtsam::numericalDerivative31<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3, gtsam::Pose3>(
        error_function,
        start_pose,
        velocity,
        end_pose);
    }
    if (h_velocity) {
      *h_velocity = (gtsam::Matrix(1, 3) << 0.0, 0.0, 1.0).finished();
    }
    if (h_end) {
      *h_end = gtsam::numericalDerivative33<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3, gtsam::Pose3>(
        error_function,
        start_pose,
        velocity,
        end_pose);
    }
    return Evaluate(start_pose, velocity, end_pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &start_pose,
    const gtsam::Vector3 &velocity,
    const gtsam::Pose3 &end_pose) const {
    const double expected_vz =
      (end_pose.translation().z() - start_pose.translation().z()) / duration_s_;
    return gtsam::Vector1(velocity.z() - expected_vz);
  }

  double duration_s_ = 1.0;
};

}  // namespace offline_lc_minimal::factor
