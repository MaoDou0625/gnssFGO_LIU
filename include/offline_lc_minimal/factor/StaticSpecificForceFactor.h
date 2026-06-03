#pragma once

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticSpecificForceFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::imuBias::ConstantBias> {
 public:
  StaticSpecificForceFactor(
    gtsam::Key pose_key,
    gtsam::Key bias_key,
    const Eigen::Vector3d &measured_acc_mps2,
    const Eigen::Vector3d &gravity_enu,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::imuBias::ConstantBias>(noise_model, pose_key, bias_key),
        measured_acc_mps2_(measured_acc_mps2),
        gravity_enu_(gravity_enu) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticSpecificForceFactor(*this)));
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

    if (h1) {
      gtsam::Matrix36 jacobian = gtsam::Matrix36::Zero();
      jacobian.block<3, 3>(0, 0) = h_unrotate;
      *h1 = jacobian;
    }
    if (h2) {
      gtsam::Matrix36 jacobian = gtsam::Matrix36::Zero();
      jacobian.block<3, 3>(0, 0) = gtsam::I_3x3;
      *h2 = jacobian;
    }
    return predicted_acc_body + bias.accelerometer() - measured_acc_mps2_;
  }

 private:
  gtsam::Vector3 measured_acc_mps2_;
  gtsam::Vector3 gravity_enu_;
};

}  // namespace offline_lc_minimal::factor
