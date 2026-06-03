#include "offline_lc_minimal/gp/Pose3Utils.h"

#include <cmath>
#include <limits>

namespace offline_lc_minimal::gp {

using gtsam::Matrix;
using gtsam::Matrix3;
using gtsam::Matrix6;
using gtsam::OptionalJacobian;
using gtsam::Pose3;
using gtsam::Vector3;
using gtsam::Vector6;

gtsam::Vector6 ConvertVwWbToVbWb(
  const gtsam::Vector3 &velocity_world,
  const gtsam::Vector3 &omega_body,
  const gtsam::Pose3 &pose,
  gtsam::OptionalJacobian<6, 3> h_velocity_world,
  gtsam::OptionalJacobian<6, 3> h_omega_body,
  gtsam::OptionalJacobian<6, 6> h_pose) {
  gtsam::Vector6 velocity_body_and_omega_body;
  if (h_velocity_world || h_omega_body || h_pose) {
    gtsam::Matrix3 h_pose_velocity;
    gtsam::Matrix3 h_rotation_velocity;
    velocity_body_and_omega_body =
      (gtsam::Vector6() << omega_body, pose.rotation().unrotate(velocity_world, &h_pose_velocity, &h_rotation_velocity))
        .finished();
    if (h_velocity_world) {
      *h_velocity_world = (gtsam::Matrix63() << gtsam::Z_3x3, h_rotation_velocity).finished();
    }
    if (h_omega_body) {
      *h_omega_body = (gtsam::Matrix63() << gtsam::I_3x3, gtsam::Z_3x3).finished();
    }
    if (h_pose) {
      *h_pose = (gtsam::Matrix66() << gtsam::Z_3x3, gtsam::Z_3x3, h_pose_velocity, gtsam::Z_3x3).finished();
    }
  } else {
    velocity_body_and_omega_body =
      (gtsam::Vector6() << omega_body, pose.rotation().unrotate(velocity_world)).finished();
  }
  return velocity_body_and_omega_body;
}

gtsam::Matrix6 CurlyHat(const gtsam::Vector6 &vector) {
  const gtsam::Vector3 &omega = vector.head(3);
  const gtsam::Vector3 &rho = vector.tail(3);
  const gtsam::Matrix3 omega_skew = gtsam::skewSymmetric(omega);
  const gtsam::Matrix3 rho_skew = gtsam::skewSymmetric(rho);
  return (gtsam::Matrix6() << omega_skew, gtsam::Matrix3::Zero(), rho_skew, omega_skew).finished();
}

namespace {

Matrix3 LeftJacobianPose3Q(const Vector6 &xi) {
  const Vector3 omega = xi.head(3);
  const Vector3 rho = xi.tail(3);
  const double theta = omega.norm();
  const Matrix3 x = gtsam::skewSymmetric(omega);
  const Matrix3 y = gtsam::skewSymmetric(rho);

  const Matrix3 xy = x * y;
  const Matrix3 yx = y * x;
  const Matrix3 xyx = x * yx;
  if (std::fabs(theta) > 1e-5) {
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    const double theta4 = theta3 * theta;
    const double theta5 = theta4 * theta;

    return 0.5 * y + (theta - sin_theta) / theta3 * (xy + yx + xyx) -
           (1.0 - 0.5 * theta2 - cos_theta) / theta4 * (x * xy + yx * x - 3.0 * xyx) -
           0.5 * ((1.0 - 0.5 * theta2 - cos_theta) / theta4 -
                  3.0 * (theta - sin_theta - theta3 / 6.0) / theta5) *
             (xyx * x + x * xyx);
  }

  return 0.5 * y + 1.0 / 6.0 * (xy + yx + xyx) -
         1.0 / 24.0 * (x * xy + yx * x - 3.0 * xyx) -
         0.5 * (1.0 / 24.0 + 3.0 / 120.0) * (xyx * x + x * xyx);
}

Matrix3 RightJacobianPose3Q(const Vector6 &xi) {
  const Vector3 omega = xi.head(3);
  const Vector3 rho = xi.tail(3);
  const double theta = omega.norm();
  const Matrix3 x = gtsam::skewSymmetric(omega);
  const Matrix3 y = gtsam::skewSymmetric(rho);

  const Matrix3 xy = x * y;
  const Matrix3 yx = y * x;
  const Matrix3 xyx = x * yx;
  if (std::fabs(theta) > 1e-5) {
    const double sin_theta = std::sin(theta);
    const double cos_theta = std::cos(theta);
    const double theta2 = theta * theta;
    const double theta3 = theta2 * theta;
    const double theta4 = theta3 * theta;
    const double theta5 = theta4 * theta;

    return -0.5 * y + (theta - sin_theta) / theta3 * (xy + yx - xyx) +
           (1.0 - 0.5 * theta2 - cos_theta) / theta4 * (x * xy + yx * x - 3.0 * xyx) -
           0.5 * ((1.0 - 0.5 * theta2 - cos_theta) / theta4 -
                  3.0 * (theta - sin_theta - theta3 / 6.0) / theta5) *
             (xyx * x + x * xyx);
  }

  return -0.5 * y + 1.0 / 6.0 * (xy + yx - xyx) +
         1.0 / 24.0 * (x * xy + yx * x - 3.0 * xyx) -
         0.5 * (1.0 / 24.0 + 3.0 / 120.0) * (xyx * x + x * xyx);
}

Matrix3 LeftJacobianRot3(const Vector3 &omega) {
  const double theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<double>::epsilon()) {
    return Matrix::Identity(3, 3);
  }
  const double theta = std::sqrt(theta2);
  const Vector3 direction = omega / theta;
  const double sin_theta = std::sin(theta);
  const Matrix3 a = gtsam::skewSymmetric(omega) / theta;

  return sin_theta / theta * Matrix::Identity(3, 3) +
         (1.0 - sin_theta / theta) * (direction * direction.transpose()) +
         (1.0 - std::cos(theta)) / theta * a;
}

Matrix3 RightJacobianRot3Inv(const Vector3 &omega) {
  const double theta2 = omega.dot(omega);
  if (theta2 <= std::numeric_limits<double>::epsilon()) {
    return Matrix::Identity(3, 3);
  }
  const double theta = std::sqrt(theta2);
  const Matrix3 x = gtsam::skewSymmetric(omega);
  return Matrix::Identity(3, 3) + 0.5 * x +
         (1.0 / theta2 - (1.0 + std::cos(theta)) / (2.0 * theta * std::sin(theta))) * x * x;
}

}  // namespace

gtsam::Matrix6 LeftJacobianPose3(const gtsam::Vector6 &xi) {
  const Vector3 omega = xi.head(3);
  const Matrix3 q = LeftJacobianPose3Q(xi);
  const Matrix3 j = LeftJacobianRot3(omega);
  return (Matrix6() << j, Matrix::Zero(3, 3), q, j).finished();
}

gtsam::Matrix6 JacobianMethodNumericalDiff(
  boost::function<gtsam::Matrix6(const gtsam::Vector6 &)> function,
  const gtsam::Vector6 &xi,
  const gtsam::Vector6 &x,
  double dxi) {
  Matrix6 diff = Matrix6::Zero();
  for (long index = 0; index < 6; ++index) {
    Vector6 xi_plus = xi;
    Vector6 xi_minus = xi;
    xi_plus(index) += dxi;
    xi_minus(index) -= dxi;
    const Matrix6 jacobian_plus = function(xi_plus);
    const Matrix6 jacobian_minus = function(xi_minus);
    diff.block<6, 1>(0, index) = (jacobian_plus - jacobian_minus) / (2.0 * dxi) * x;
  }
  return diff;
}

gtsam::Matrix6 RightJacobianPose3Inv(const gtsam::Vector6 &xi) {
  const Vector3 omega = xi.head<3>();
  const Matrix3 j_rotation = RightJacobianRot3Inv(omega);
  const Matrix3 q = RightJacobianPose3Q(xi);
  const Matrix3 q2 = -j_rotation * q * j_rotation;
  Matrix6 jacobian;
  jacobian << j_rotation, Matrix::Zero(3, 3), q2, j_rotation;
  return jacobian;
}

}  // namespace offline_lc_minimal::gp
