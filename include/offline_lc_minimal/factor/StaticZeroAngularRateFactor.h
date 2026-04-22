#pragma once

#include <functional>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class StaticZeroAngularRateFactor : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::imuBias::ConstantBias> {
 public:
  StaticZeroAngularRateFactor(
    gtsam::Key pose_key,
    gtsam::Key bias_key,
    gtsam::Vector3 measured_angular_rate,
    gtsam::Vector3 earth_rate_enu,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::imuBias::ConstantBias>(noise_model, pose_key, bias_key),
        measured_angular_rate_(std::move(measured_angular_rate)),
        earth_rate_enu_(std::move(earth_rate_enu)) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new StaticZeroAngularRateFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    const gtsam::imuBias::ConstantBias &bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none) const override {
    const auto error_function =
      [this](const gtsam::Pose3 &candidate_pose, const gtsam::imuBias::ConstantBias &candidate_bias) {
        return this->ComputeError(candidate_pose, candidate_bias);
      };
    if (h1) {
      *h1 = gtsam::numericalDerivative21<gtsam::Vector3, gtsam::Pose3, gtsam::imuBias::ConstantBias>(
        error_function,
        pose,
        bias);
    }
    if (h2) {
      *h2 = gtsam::numericalDerivative22<gtsam::Vector3, gtsam::Pose3, gtsam::imuBias::ConstantBias>(
        error_function,
        pose,
        bias);
    }
    return ComputeError(pose, bias);
  }

 private:
  [[nodiscard]] gtsam::Vector3 ComputeError(
    const gtsam::Pose3 &pose,
    const gtsam::imuBias::ConstantBias &bias) const {
    const gtsam::Vector3 corrected_angular_rate = bias.correctGyroscope(measured_angular_rate_);
    const gtsam::Vector3 expected_angular_rate = pose.rotation().matrix().transpose() * earth_rate_enu_;
    return corrected_angular_rate - expected_angular_rate;
  }

  gtsam::Vector3 measured_angular_rate_;
  gtsam::Vector3 earth_rate_enu_;
};

}  // namespace offline_lc_minimal::factor
