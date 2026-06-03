#pragma once

#include <memory>
#include <utility>

#include <boost/bind/bind.hpp>
#include <boost/optional.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal::factor {

class GPInterpolatedGPSFactor
    : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
 public:
  GPInterpolatedGPSFactor(
    gtsam::Key pose_key_i,
    gtsam::Key vel_key_i,
    gtsam::Key omega_key_i,
    gtsam::Key pose_key_j,
    gtsam::Key vel_key_j,
    gtsam::Key omega_key_j,
    gtsam::Point3 measured_position,
    gtsam::Vector3 lever_arm_body,
    const gtsam::SharedNoiseModel &noise_model,
    gp::GPWNOJInterpolator interpolator)
      : gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
          noise_model, pose_key_i, vel_key_i, omega_key_i, pose_key_j, vel_key_j, omega_key_j),
        measured_position_(std::move(measured_position)),
        lever_arm_body_(std::move(lever_arm_body)),
        interpolator_(std::move(interpolator)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GPInterpolatedGPSFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none,
    boost::optional<gtsam::Matrix &> h3 = boost::none,
    boost::optional<gtsam::Matrix &> h4 = boost::none,
    boost::optional<gtsam::Matrix &> h5 = boost::none,
    boost::optional<gtsam::Matrix &> h6 = boost::none) const override {
    using boost::placeholders::_1;
    using boost::placeholders::_2;
    using boost::placeholders::_3;
    using boost::placeholders::_4;
    using boost::placeholders::_5;
    using boost::placeholders::_6;

    if (h1) {
      *h1 = gtsam::numericalDerivative61<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                         gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
        boost::bind(&GPInterpolatedGPSFactor::EvaluateErrorNoJacobians, this, _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (h2) {
      *h2 = gtsam::numericalDerivative62<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                         gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
        boost::bind(&GPInterpolatedGPSFactor::EvaluateErrorNoJacobians, this, _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (h3) {
      *h3 = gtsam::numericalDerivative63<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                         gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
        boost::bind(&GPInterpolatedGPSFactor::EvaluateErrorNoJacobians, this, _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (h4) {
      *h4 = gtsam::numericalDerivative64<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                         gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
        boost::bind(&GPInterpolatedGPSFactor::EvaluateErrorNoJacobians, this, _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (h5) {
      *h5 = gtsam::numericalDerivative65<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                         gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
        boost::bind(&GPInterpolatedGPSFactor::EvaluateErrorNoJacobians, this, _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (h6) {
      *h6 = gtsam::numericalDerivative66<gtsam::Vector3, gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                         gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
        boost::bind(&GPInterpolatedGPSFactor::EvaluateErrorNoJacobians, this, _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }

    return EvaluateErrorNoJacobians(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  }

 private:
  [[nodiscard]] gtsam::Vector3 EvaluateErrorNoJacobians(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j) const {
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    return interpolated_pose.translation() + interpolated_pose.rotation().rotate(lever_arm_body_) - measured_position_;
  }

  gtsam::Point3 measured_position_;
  gtsam::Vector3 lever_arm_body_;
  gp::GPWNOJInterpolator interpolator_;
};

}  // namespace offline_lc_minimal::factor
