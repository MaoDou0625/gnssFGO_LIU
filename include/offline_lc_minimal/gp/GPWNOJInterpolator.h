#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/gp/GPUtils.h"
#include "offline_lc_minimal/gp/Pose3Utils.h"

namespace offline_lc_minimal::gp {

class GPWNOJInterpolator {
 public:
  explicit GPWNOJInterpolator(
    const gtsam::SharedNoiseModel &qc_model,
    double delta_t = 0.0,
    double tau = 0.0)
      : qc_(GetQc(qc_model)) {
    Recalculate(delta_t, tau);
  }

  void Recalculate(double delta_t, double tau) {
    delta_t_ = delta_t;
    tau_ = tau;
    lambda_ = CalcLambda3<6>(qc_, delta_t_, tau_);
    psi_ = CalcPsi3<6>(qc_, delta_t_, tau_);
  }

  [[nodiscard]] double tau() const { return tau_; }

  [[nodiscard]] gtsam::Pose3 InterpolatePose(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::Vector3 &omega_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::Vector3 &omega_j) const {
    const gtsam::Vector6 relative_pose = gtsam::Pose3::Logmap(pose_i.inverse().compose(pose_j));
    const gtsam::Vector6 body_velocity_i = ConvertVwWbToVbWb(vel_i, omega_i, pose_i);
    const gtsam::Vector6 body_velocity_j = ConvertVwWbToVbWb(vel_j, omega_j, pose_j);
    const gtsam::Matrix6 right_jacobian_inv = RightJacobianPose3Inv(relative_pose);
    const gtsam::Matrix6 skew_velocity_j = CurlyHat(right_jacobian_inv * body_velocity_j);

    const Vector_18 state_i =
      (Vector_18() << gtsam::Vector6::Zero(), body_velocity_i, gtsam::Vector6::Zero()).finished();
    const Vector_18 state_j =
      (Vector_18() << relative_pose, right_jacobian_inv * body_velocity_j,
        -0.5 * skew_velocity_j * body_velocity_j).finished();

    const gtsam::Vector6 xi_i_tau =
      lambda_.block<6, 18>(0, 0) * state_i + psi_.block<6, 18>(0, 0) * state_j;
    return pose_i.compose(gtsam::Pose3::Expmap(xi_i_tau));
  }

 private:
  gtsam::Matrix6 qc_ = gtsam::Matrix6::Identity();
  Matrix_18 lambda_ = Matrix_18::Identity();
  Matrix_18 psi_ = Matrix_18::Identity();
  double delta_t_ = 0.0;
  double tau_ = 0.0;
};

}  // namespace offline_lc_minimal::gp
