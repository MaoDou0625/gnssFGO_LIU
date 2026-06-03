#pragma once

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class HorizontalPositionHoldFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  HorizontalPositionHoldFactor(
    gtsam::Key pose_key,
    const gtsam::Vector2 &reference_east_north_m,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        reference_east_north_m_(reference_east_north_m) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new HorizontalPositionHoldFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    boost::optional<gtsam::Matrix &> h_pose = boost::none) const override {
    const auto error_function = [this](const gtsam::Pose3 &candidate_pose) {
      return (gtsam::Vector2()
        << candidate_pose.translation().x() - reference_east_north_m_.x(),
           candidate_pose.translation().y() - reference_east_north_m_.y())
        .finished();
    };
    if (h_pose) {
      *h_pose = gtsam::numericalDerivative11<gtsam::Vector, gtsam::Pose3>(
        error_function,
        pose);
    }
    return error_function(pose);
  }

 private:
  gtsam::Vector2 reference_east_north_m_;
};

class HorizontalVelocityHoldFactor final : public gtsam::NoiseModelFactor1<gtsam::Vector3> {
 public:
  HorizontalVelocityHoldFactor(
    gtsam::Key velocity_key,
    const gtsam::Vector2 &reference_velocity_east_north_mps,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor1<gtsam::Vector3>(model, velocity_key),
        reference_velocity_east_north_mps_(reference_velocity_east_north_mps) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new HorizontalVelocityHoldFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Vector3 &velocity,
    boost::optional<gtsam::Matrix &> h_velocity = boost::none) const override {
    if (h_velocity) {
      *h_velocity = (gtsam::Matrix(2, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0).finished();
    }
    return (gtsam::Vector2()
      << velocity.x() - reference_velocity_east_north_mps_.x(),
         velocity.y() - reference_velocity_east_north_mps_.y())
      .finished();
  }

 private:
  gtsam::Vector2 reference_velocity_east_north_mps_;
};

}  // namespace offline_lc_minimal::factor
