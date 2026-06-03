#pragma once

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "offline_lc_minimal/common/Types.h"
#include "offline_lc_minimal/core/ErrorStateModel.h"

namespace offline_lc_minimal::factor {

class ErrorStateCorrectionFactor
    : public gtsam::NoiseModelFactor5<
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        ErrorStateVector,
        ErrorStateVector> {
 public:
  ErrorStateCorrectionFactor(
    gtsam::Key pose_key,
    gtsam::Key velocity_key,
    gtsam::Key bias_key,
    gtsam::Key error_left_key,
    gtsam::Key error_right_key,
    ReferenceNodeState reference_state,
    double interpolation_alpha,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor5<
          gtsam::Pose3,
          gtsam::Vector3,
          gtsam::imuBias::ConstantBias,
          ErrorStateVector,
          ErrorStateVector>(noise_model, pose_key, velocity_key, bias_key, error_left_key, error_right_key),
        reference_state_(std::move(reference_state)),
        interpolation_alpha_(interpolation_alpha) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new ErrorStateCorrectionFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose,
    const gtsam::Vector3 &velocity,
    const gtsam::imuBias::ConstantBias &bias,
    const ErrorStateVector &error_left,
    const ErrorStateVector &error_right,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none,
    boost::optional<gtsam::Matrix &> h3 = boost::none,
    boost::optional<gtsam::Matrix &> h4 = boost::none,
    boost::optional<gtsam::Matrix &> h5 = boost::none) const override {
    const auto error_function =
      [this](
        const gtsam::Pose3 &candidate_pose,
        const gtsam::Vector3 &candidate_velocity,
        const gtsam::imuBias::ConstantBias &candidate_bias,
        const ErrorStateVector &candidate_error_left,
        const ErrorStateVector &candidate_error_right) {
        return this->ComputeError(
          candidate_pose,
          candidate_velocity,
          candidate_bias,
          candidate_error_left,
          candidate_error_right);
      };

    if (h1) {
      *h1 = gtsam::numericalDerivative51<
        gtsam::Vector,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        ErrorStateVector,
        ErrorStateVector>(error_function, pose, velocity, bias, error_left, error_right);
    }
    if (h2) {
      *h2 = gtsam::numericalDerivative52<
        gtsam::Vector,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        ErrorStateVector,
        ErrorStateVector>(error_function, pose, velocity, bias, error_left, error_right);
    }
    if (h3) {
      *h3 = gtsam::numericalDerivative53<
        gtsam::Vector,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        ErrorStateVector,
        ErrorStateVector>(error_function, pose, velocity, bias, error_left, error_right);
    }
    if (h4) {
      *h4 = gtsam::numericalDerivative54<
        gtsam::Vector,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        ErrorStateVector,
        ErrorStateVector>(error_function, pose, velocity, bias, error_left, error_right);
    }
    if (h5) {
      *h5 = gtsam::numericalDerivative55<
        gtsam::Vector,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        ErrorStateVector,
        ErrorStateVector>(error_function, pose, velocity, bias, error_left, error_right);
    }

    return ComputeError(pose, velocity, bias, error_left, error_right);
  }

 private:
  [[nodiscard]] gtsam::Vector ComputeError(
    const gtsam::Pose3 &pose,
    const gtsam::Vector3 &velocity,
    const gtsam::imuBias::ConstantBias &bias,
    const ErrorStateVector &error_left,
    const ErrorStateVector &error_right) const {
    const ErrorStateVector interpolated_error =
      offline_lc_minimal::InterpolateErrorState(error_left, error_right, interpolation_alpha_);
    const Eigen::Vector3d dtheta = interpolated_error.segment<3>(0);
    const Eigen::Vector3d dv = interpolated_error.segment<3>(3);
    const Eigen::Vector3d dp = interpolated_error.segment<3>(6);
    const Eigen::Vector3d dbg = interpolated_error.segment<3>(9);
    const Eigen::Vector3d dba = interpolated_error.segment<3>(12);

    const gtsam::Rot3 corrected_rotation =
      gtsam::Rot3::Expmap(gtsam::Vector3(dtheta.x(), dtheta.y(), dtheta.z()))
        .compose(reference_state_.pose.rotation());
    const gtsam::Point3 corrected_translation(
      reference_state_.pose.translation().x() + dp.x(),
      reference_state_.pose.translation().y() + dp.y(),
      reference_state_.pose.translation().z() + dp.z());
    const gtsam::Pose3 corrected_pose(corrected_rotation, corrected_translation);
    const gtsam::Vector3 corrected_velocity =
      reference_state_.velocity + gtsam::Vector3(dv.x(), dv.y(), dv.z());
    const gtsam::imuBias::ConstantBias corrected_bias(
      reference_state_.bias.accelerometer() + dba,
      reference_state_.bias.gyroscope() + dbg);

    gtsam::Vector residual(15);
    residual.segment<3>(0) = gtsam::Rot3::Logmap(corrected_pose.rotation().between(pose.rotation()));
    residual.segment<3>(3) = velocity - corrected_velocity;
    residual.segment<3>(6) =
      gtsam::Vector3(
        pose.translation().x() - corrected_pose.translation().x(),
        pose.translation().y() - corrected_pose.translation().y(),
        pose.translation().z() - corrected_pose.translation().z());
    residual.segment<3>(9) = bias.gyroscope() - corrected_bias.gyroscope();
    residual.segment<3>(12) = bias.accelerometer() - corrected_bias.accelerometer();
    return residual;
  }

  ReferenceNodeState reference_state_;
  double interpolation_alpha_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
