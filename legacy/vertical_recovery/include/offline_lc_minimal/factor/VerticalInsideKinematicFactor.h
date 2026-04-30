#pragma once

#include <boost/bind/bind.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalInsideKinematicFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias> {
 public:
  VerticalInsideKinematicFactor(
    gtsam::Key pose_key,
    gtsam::Key velocity_key,
    gtsam::Key bias_key,
    const gtsam::Pose3 &reference_pose,
    const gtsam::Vector3 &reference_velocity,
    const gtsam::imuBias::ConstantBias &reference_bias,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
          noise_model,
          pose_key,
          velocity_key,
          bias_key),
        reference_pose_(reference_pose),
        reference_velocity_(reference_velocity),
        reference_bias_(reference_bias) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalInsideKinematicFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    const gtsam::Vector3 &velocity,
    const gtsam::imuBias::ConstantBias &bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none,
    boost::optional<gtsam::Matrix &> h3 = boost::none) const override {
    if (h1) {
      using boost::placeholders::_1;
      using boost::placeholders::_2;
      using boost::placeholders::_3;
      *h1 = gtsam::numericalDerivative31<gtsam::Vector4, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
        boost::bind(&VerticalInsideKinematicFactor::EvaluateErrorNoJacobians, this, _1, _2, _3),
        pose,
        velocity,
        bias,
        1e-6);
    }
    if (h2) {
      using boost::placeholders::_1;
      using boost::placeholders::_2;
      using boost::placeholders::_3;
      *h2 =
        gtsam::numericalDerivative32<gtsam::Vector4, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
          boost::bind(&VerticalInsideKinematicFactor::EvaluateErrorNoJacobians, this, _1, _2, _3),
          pose,
          velocity,
          bias,
          1e-6);
    }
    if (h3) {
      using boost::placeholders::_1;
      using boost::placeholders::_2;
      using boost::placeholders::_3;
      *h3 =
        gtsam::numericalDerivative33<gtsam::Vector4, gtsam::Pose3, gtsam::Vector3, gtsam::imuBias::ConstantBias>(
          boost::bind(&VerticalInsideKinematicFactor::EvaluateErrorNoJacobians, this, _1, _2, _3),
          pose,
          velocity,
          bias,
          1e-6);
    }
    return EvaluateErrorNoJacobians(pose, velocity, bias);
  }

 private:
  [[nodiscard]] gtsam::Vector4 EvaluateErrorNoJacobians(
    const gtsam::Pose3 &pose,
    const gtsam::Vector3 &velocity,
    const gtsam::imuBias::ConstantBias &bias) const {
    const gtsam::Vector3 reference_rpy = reference_pose_.rotation().rpy();
    const gtsam::Vector3 pose_rpy = pose.rotation().rpy();
    return gtsam::Vector4(
      WrapAngle(pose_rpy.y() - reference_rpy.y()),
      WrapAngle(pose_rpy.x() - reference_rpy.x()),
      velocity.z() - reference_velocity_.z(),
      bias.accelerometer().z() - reference_bias_.accelerometer().z());
  }

  [[nodiscard]] static double WrapAngle(const double angle_rad) {
    return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
  }

  gtsam::Pose3 reference_pose_;
  gtsam::Vector3 reference_velocity_ = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias reference_bias_;
};

}  // namespace offline_lc_minimal::factor
