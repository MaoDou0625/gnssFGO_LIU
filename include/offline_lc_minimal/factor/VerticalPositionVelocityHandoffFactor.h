#pragma once

#include <cmath>
#include <stdexcept>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalPositionVelocityHandoffFactor final
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3> {
 public:
  VerticalPositionVelocityHandoffFactor(
    gtsam::Key pose_i_key,
    gtsam::Key velocity_i_key,
    double reference_up_j_m,
    double reference_vz_j_mps,
    double reference_time_minus_state_time_s,
    const gtsam::SharedNoiseModel &model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Vector3>(
          model,
          pose_i_key,
          velocity_i_key),
        reference_up_j_m_(reference_up_j_m),
        reference_vz_j_mps_(reference_vz_j_mps),
        reference_time_minus_state_time_s_(reference_time_minus_state_time_s) {
    if (!std::isfinite(reference_up_j_m_) ||
        !std::isfinite(reference_vz_j_mps_)) {
      throw std::invalid_argument(
        "VerticalPositionVelocityHandoffFactor requires finite reference state");
    }
    if (!std::isfinite(reference_time_minus_state_time_s_) ||
        std::abs(reference_time_minus_state_time_s_) <= 1.0e-12) {
      throw std::invalid_argument(
        "VerticalPositionVelocityHandoffFactor requires nonzero signed dt");
    }
  }

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(
        new VerticalPositionVelocityHandoffFactor(*this)));
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
      *h_pose_i = gtsam::numericalDerivative21<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3>(
        error_function,
        pose_i,
        velocity_i);
    }
    if (h_velocity_i) {
      *h_velocity_i = (gtsam::Matrix(1, 3) <<
        0.0, 0.0, -0.5 * reference_time_minus_state_time_s_).finished();
    }
    return Evaluate(pose_i, velocity_i);
  }

 private:
  [[nodiscard]] gtsam::Vector1 Evaluate(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &velocity_i) const {
    const double up_i_m = pose_i.translation().z();
    const double vz_i_mps = velocity_i.z();
    const double reference_delta_m = reference_up_j_m_ - up_i_m;
    const double velocity_integrated_delta_m =
      0.5 * reference_time_minus_state_time_s_ *
      (vz_i_mps + reference_vz_j_mps_);
    return (gtsam::Vector1() << reference_delta_m - velocity_integrated_delta_m).finished();
  }

  double reference_up_j_m_ = 0.0;
  double reference_vz_j_mps_ = 0.0;
  double reference_time_minus_state_time_s_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
