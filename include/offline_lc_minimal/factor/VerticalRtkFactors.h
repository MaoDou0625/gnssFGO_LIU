#pragma once

#include <boost/bind/bind.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/base/Vector.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal::factor {

class VerticalPositionFactor final : public gtsam::NoiseModelFactor1<gtsam::Pose3> {
 public:
  VerticalPositionFactor(gtsam::Key pose_key, double measured_up,
                         const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor1<gtsam::Pose3>(model, pose_key),
        measured_up_(measured_up) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalPositionFactor(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose,
                              boost::optional<gtsam::Matrix&> H = boost::none) const override {
    if (H) {
      using boost::placeholders::_1;
      *H = gtsam::numericalDerivative11<gtsam::Vector1, gtsam::Pose3>(
        boost::bind(&VerticalPositionFactor::EvaluateErrorNoJacobians, this, _1),
        pose,
        1e-5);
    }
    return EvaluateErrorNoJacobians(pose);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(const gtsam::Pose3& pose) const {
    return gtsam::Vector1(pose.translation().z() - measured_up_);
  }

  double measured_up_ = 0.0;
};

class GPInterpolatedVerticalPositionFactor final
    : public gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                      gtsam::Pose3, gtsam::Vector3, gtsam::Vector3> {
 public:
  GPInterpolatedVerticalPositionFactor(gtsam::Key pose_i_key, gtsam::Key vel_i_key,
                                       gtsam::Key omega_i_key, gtsam::Key pose_j_key,
                                       gtsam::Key vel_j_key, gtsam::Key omega_j_key,
                                       double measured_up,
                                       const offline_lc_minimal::gp::GPWNOJInterpolator& interpolator,
                                       const gtsam::SharedNoiseModel& model)
      : gtsam::NoiseModelFactor6<gtsam::Pose3, gtsam::Vector3, gtsam::Vector3,
                                 gtsam::Pose3, gtsam::Vector3, gtsam::Vector3>(
            model, pose_i_key, vel_i_key, omega_i_key, pose_j_key, vel_j_key, omega_j_key),
        measured_up_(measured_up),
        interpolator_(interpolator) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new GPInterpolatedVerticalPositionFactor(*this)));
  }

  gtsam::Vector evaluateError(const gtsam::Pose3& pose_i, const gtsam::Vector3& vel_i,
                              const gtsam::Vector3& omega_i, const gtsam::Pose3& pose_j,
                              const gtsam::Vector3& vel_j, const gtsam::Vector3& omega_j,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none,
                              boost::optional<gtsam::Matrix&> H3 = boost::none,
                              boost::optional<gtsam::Matrix&> H4 = boost::none,
                              boost::optional<gtsam::Matrix&> H5 = boost::none,
                              boost::optional<gtsam::Matrix&> H6 = boost::none) const override {
    using boost::placeholders::_1;
    using boost::placeholders::_2;
    using boost::placeholders::_3;
    using boost::placeholders::_4;
    using boost::placeholders::_5;
    using boost::placeholders::_6;

    if (H1) {
      *H1 = gtsam::numericalDerivative61<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H2) {
      *H2 = gtsam::numericalDerivative62<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H3) {
      *H3 = gtsam::numericalDerivative63<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H4) {
      *H4 = gtsam::numericalDerivative64<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H5) {
      *H5 = gtsam::numericalDerivative65<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }
    if (H6) {
      *H6 = gtsam::numericalDerivative66<gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3, gtsam::Pose3, gtsam::Vector3,
                                         gtsam::Vector3>(
        boost::bind(&GPInterpolatedVerticalPositionFactor::EvaluateErrorNoJacobians, this,
                    _1, _2, _3, _4, _5, _6),
        pose_i, vel_i, omega_i, pose_j, vel_j, omega_j, 1e-5);
    }

    return EvaluateErrorNoJacobians(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
  }

 private:
  [[nodiscard]] gtsam::Vector1 EvaluateErrorNoJacobians(
      const gtsam::Pose3& pose_i, const gtsam::Vector3& vel_i,
      const gtsam::Vector3& omega_i, const gtsam::Pose3& pose_j,
      const gtsam::Vector3& vel_j, const gtsam::Vector3& omega_j) const {
    const gtsam::Pose3 interpolated_pose =
      interpolator_.InterpolatePose(pose_i, vel_i, omega_i, pose_j, vel_j, omega_j);
    return gtsam::Vector1(interpolated_pose.translation().z() - measured_up_);
  }

  double measured_up_ = 0.0;
  offline_lc_minimal::gp::GPWNOJInterpolator interpolator_;
};

}  // namespace offline_lc_minimal::factor
