#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticVerticalSpecificForceFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::imuBias::ConstantBias> {
 public:
  StaticVerticalSpecificForceFactor(
    gtsam::Key pose_key,
    gtsam::Key bias_key,
    double measured_acc_z_mps2,
    const Eigen::Vector3d &gravity_enu,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::imuBias::ConstantBias>(noise_model, pose_key, bias_key),
        measured_acc_z_mps2_(measured_acc_z_mps2),
        gravity_enu_(gravity_enu) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticVerticalSpecificForceFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    const gtsam::imuBias::ConstantBias &bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    gtsam::Vector3 predicted_acc_body;
    gtsam::Matrix33 h_unrotate = gtsam::Matrix33::Zero();
    if (h1) {
      predicted_acc_body = pose.rotation().unrotate(gravity_enu_, h_unrotate);
    } else {
      predicted_acc_body = pose.rotation().unrotate(gravity_enu_);
    }

    Eigen::Matrix<double, 1, 1> residual;
    residual[0] = predicted_acc_body.z() + bias.accelerometer().z() - measured_acc_z_mps2_;

    if (h1) {
      Eigen::Matrix<double, 1, 6> jacobian = Eigen::Matrix<double, 1, 6>::Zero();
      jacobian.block<1, 3>(0, 0) = h_unrotate.row(2);
      *h1 = jacobian;
    }
    if (h2) {
      Eigen::Matrix<double, 1, 6> jacobian = Eigen::Matrix<double, 1, 6>::Zero();
      jacobian(0, 2) = 1.0;
      *h2 = jacobian;
    }
    return residual;
  }

 private:
  double measured_acc_z_mps2_ = 0.0;
  gtsam::Vector3 gravity_enu_ = gtsam::Vector3::Zero();
};

}  // namespace offline_lc_minimal::factor
