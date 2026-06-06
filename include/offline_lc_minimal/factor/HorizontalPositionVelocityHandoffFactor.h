#pragma once

#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class HorizontalPositionVelocityHandoffFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
 public:
  HorizontalPositionVelocityHandoffFactor(
    gtsam::Key pose_i_key,
    gtsam::Key velocity_i_key,
    const gtsam::Vector2 &reference_horizontal_position_j_m,
    const gtsam::Vector2 &reference_horizontal_velocity_j_mps,
    double dt_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(
          model,
          pose_i_key,
          velocity_i_key),
        reference_horizontal_position_j_m_(reference_horizontal_position_j_m),
        reference_horizontal_velocity_j_mps_(reference_horizontal_velocity_j_mps),
        dt_s_(dt_s) {
    if (!reference_horizontal_position_j_m_.allFinite() ||
        !reference_horizontal_velocity_j_mps_.allFinite()) {
      throw std::invalid_argument(
        "HorizontalPositionVelocityHandoffFactor requires finite reference state");
    }
    if (!(dt_s_ > 0.0)) {
      throw std::invalid_argument(
        "HorizontalPositionVelocityHandoffFactor requires positive dt");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new HorizontalPositionVelocityHandoffFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &velocity_i,
    boost::optional<gtsam::Matrix &> h_pose_i = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_i = boost::none) const override {
    const auto error_function =
      [this](const gtsam::Pose3 &pose, const gtsam::Vector3 &velocity) {
        return Evaluate(pose, velocity);
      };
    if (h_pose_i) {
      *h_pose_i = gtsam::numericalDerivative21<gtsam::Vector2, gtsam::Pose3, gtsam::Vector3>(
        error_function,
        pose_i,
        velocity_i);
    }
    if (h_velocity_i) {
      *h_velocity_i = (gtsam::Matrix(2, 3) <<
        -0.5 * dt_s_, 0.0, 0.0,
        0.0, -0.5 * dt_s_, 0.0).finished();
    }
    return Evaluate(pose_i, velocity_i);
  }

 private:
  [[nodiscard]] gtsam::Vector2 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &velocity_i) const {
    const gtsam::Point3 translation_i = pose_i.translation();
    const gtsam::Vector2 horizontal_position_i(
      translation_i.x(),
      translation_i.y());
    const gtsam::Vector2 horizontal_velocity_i(
      velocity_i.x(),
      velocity_i.y());
    const gtsam::Vector2 reference_delta_m =
      reference_horizontal_position_j_m_ - horizontal_position_i;
    const gtsam::Vector2 velocity_integrated_delta_m =
      0.5 * dt_s_ * (horizontal_velocity_i + reference_horizontal_velocity_j_mps_);
    return reference_delta_m - velocity_integrated_delta_m;
  }

  gtsam::Vector2 reference_horizontal_position_j_m_ = gtsam::Vector2::Zero();
  gtsam::Vector2 reference_horizontal_velocity_j_mps_ = gtsam::Vector2::Zero();
  double dt_s_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
