#pragma once

#include <boost/function.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>

namespace offline_lc_minimal::gp {

using Vector_18 = Eigen::Matrix<double, 18, 1>;
using Matrix_18 = Eigen::Matrix<double, 18, 18>;
using Matrix_18_6 = Eigen::Matrix<double, 18, 6>;
using Matrix_12_6 = Eigen::Matrix<double, 12, 6>;

gtsam::Vector6 ConvertVwWbToVbWb(
  const gtsam::Vector3 &velocity_world,
  const gtsam::Vector3 &omega_body,
  const gtsam::Pose3 &pose,
  gtsam::OptionalJacobian<6, 3> h_velocity_world = boost::none,
  gtsam::OptionalJacobian<6, 3> h_omega_body = boost::none,
  gtsam::OptionalJacobian<6, 6> h_pose = boost::none);

gtsam::Matrix6 CurlyHat(const gtsam::Vector6 &vector);
gtsam::Matrix6 LeftJacobianPose3(const gtsam::Vector6 &xi);
gtsam::Matrix6 RightJacobianPose3Inv(const gtsam::Vector6 &xi);
gtsam::Matrix6 JacobianMethodNumericalDiff(
  boost::function<gtsam::Matrix6(const gtsam::Vector6 &)> function,
  const gtsam::Vector6 &xi,
  const gtsam::Vector6 &x,
  double dxi = 1e-6);

}  // namespace offline_lc_minimal::gp
