#pragma once

#include <algorithm>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class VerticalRtkPreintegrationFeedbackFactor
    : public gtsam::NoiseModelFactor {
 public:
  VerticalRtkPreintegrationFeedbackFactor(
    gtsam::Key pose_i_key,
    gtsam::Key vel_i_key,
    gtsam::Key bias_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key vel_j_key,
    gtsam::Key bias_j_key,
    gtsam::Key global_acc_bias_key,
    const gtsam::PreintegratedImuMeasurements &preintegrated_measurements,
    const double phi_vertical_acc,
    const double delta_time_s,
    const double vertical_rtk_residual_m,
    const double feedback_gain_scale,
    const double vertical_rtk_feedback_bias_gain,
    const double vertical_rtk_feedback_attitude_gain,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor(
        noise_model,
        gtsam::KeyVector{pose_i_key, vel_i_key, bias_i_key, pose_j_key, vel_j_key, bias_j_key, global_acc_bias_key}),
        imu_factor_(pose_i_key, vel_i_key, pose_j_key, vel_j_key, bias_i_key, preintegrated_measurements),
        phi_vertical_acc_(phi_vertical_acc),
        delta_time_s_(std::max(delta_time_s, 1e-6)),
        target_baz_mps2_(
          -feedback_gain_scale * vertical_rtk_feedback_bias_gain * vertical_rtk_residual_m /
          (delta_time_s_ * delta_time_s_)),
        attitude_scale_(feedback_gain_scale * vertical_rtk_feedback_attitude_gain) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new VerticalRtkPreintegrationFeedbackFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector unwhitenedError(
    const gtsam::Values &values,
    boost::optional<std::vector<gtsam::Matrix> &> jacobians = boost::none) const override {
    boost::optional<gtsam::Matrix &> h1 = boost::none;
    boost::optional<gtsam::Matrix &> h2 = boost::none;
    boost::optional<gtsam::Matrix &> h3 = boost::none;
    boost::optional<gtsam::Matrix &> h4 = boost::none;
    boost::optional<gtsam::Matrix &> h5 = boost::none;
    boost::optional<gtsam::Matrix &> h6 = boost::none;
    boost::optional<gtsam::Matrix &> h7 = boost::none;
    if (jacobians) {
      jacobians->resize(7);
      h1 = (*jacobians)[0];
      h2 = (*jacobians)[1];
      h3 = (*jacobians)[2];
      h4 = (*jacobians)[3];
      h5 = (*jacobians)[4];
      h6 = (*jacobians)[5];
      h7 = (*jacobians)[6];
    }
    return evaluateError(
      values.at<gtsam::Pose3>(this->keys()[0]),
      values.at<gtsam::Vector3>(this->keys()[1]),
      values.at<gtsam::imuBias::ConstantBias>(this->keys()[2]),
      values.at<gtsam::Pose3>(this->keys()[3]),
      values.at<gtsam::Vector3>(this->keys()[4]),
      values.at<gtsam::imuBias::ConstantBias>(this->keys()[5]),
      values.at<gtsam::Vector3>(this->keys()[6]),
      h1,
      h2,
      h3,
      h4,
      h5,
      h6,
      h7);
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::imuBias::ConstantBias &bias_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::imuBias::ConstantBias &bias_j,
    const gtsam::Vector3 &global_acc_bias,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none,
    boost::optional<gtsam::Matrix &> h3 = boost::none,
    boost::optional<gtsam::Matrix &> h4 = boost::none,
    boost::optional<gtsam::Matrix &> h5 = boost::none,
    boost::optional<gtsam::Matrix &> h6 = boost::none,
    boost::optional<gtsam::Matrix &> h7 = boost::none) const {
    gtsam::Matrix h_pose_i_imu;
    gtsam::Matrix h_vel_i_imu;
    gtsam::Matrix h_pose_j_imu;
    gtsam::Matrix h_vel_j_imu;
    gtsam::Matrix h_bias_i_imu;
    const gtsam::Vector imu_error = imu_factor_.evaluateError(
      pose_i,
      vel_i,
      pose_j,
      vel_j,
      bias_i,
      h1 ? boost::optional<gtsam::Matrix &>(h_pose_i_imu) : boost::none,
      h2 ? boost::optional<gtsam::Matrix &>(h_vel_i_imu) : boost::none,
      h4 ? boost::optional<gtsam::Matrix &>(h_pose_j_imu) : boost::none,
      h5 ? boost::optional<gtsam::Matrix &>(h_vel_j_imu) : boost::none,
      h3 ? boost::optional<gtsam::Matrix &>(h_bias_i_imu) : boost::none);

    const gtsam::Vector3 dtheta_rad = imu_error.segment<3>(0);
    const double mean_baz = global_acc_bias.z();
    const double predicted_baz = mean_baz + phi_vertical_acc_ * (bias_i.accelerometer().z() - mean_baz);

    gtsam::Vector3 residual = gtsam::Vector3::Zero();
    residual.x() = attitude_scale_ * dtheta_rad.x();
    residual.y() = attitude_scale_ * dtheta_rad.y();
    residual.z() = bias_j.accelerometer().z() - predicted_baz - target_baz_mps2_;

    if (h1) {
      *h1 = ComposeAttitudeJacobian(h_pose_i_imu);
    }
    if (h2) {
      *h2 = ComposeAttitudeJacobian(h_vel_i_imu);
    }
    if (h3) {
      gtsam::Matrix jacobian = gtsam::Matrix::Zero(3, 6);
      if (h_bias_i_imu.rows() == 9 && h_bias_i_imu.cols() == 6) {
        jacobian.block(0, 0, 2, 6) = attitude_scale_ * h_bias_i_imu.block(0, 0, 2, 6);
      }
      jacobian(2, 2) = -phi_vertical_acc_;
      *h3 = jacobian;
    }
    if (h4) {
      *h4 = ComposeAttitudeJacobian(h_pose_j_imu);
    }
    if (h5) {
      *h5 = ComposeAttitudeJacobian(h_vel_j_imu);
    }
    if (h6) {
      gtsam::Matrix jacobian = gtsam::Matrix::Zero(3, 6);
      jacobian(2, 2) = 1.0;
      *h6 = jacobian;
    }
    if (h7) {
      gtsam::Matrix jacobian = gtsam::Matrix::Zero(3, 3);
      jacobian(2, 2) = -(1.0 - phi_vertical_acc_);
      *h7 = jacobian;
    }

    return residual;
  }

 private:
  [[nodiscard]] gtsam::Matrix ComposeAttitudeJacobian(const gtsam::Matrix &imu_jacobian) const {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(3, imu_jacobian.cols());
    if (imu_jacobian.rows() != 9) {
      return jacobian;
    }
    jacobian.block(0, 0, 2, imu_jacobian.cols()) = attitude_scale_ * imu_jacobian.block(0, 0, 2, imu_jacobian.cols());
    return jacobian;
  }

  gtsam::ImuFactor imu_factor_;
  double phi_vertical_acc_ = 1.0;
  double delta_time_s_ = 1e-6;
  double target_baz_mps2_ = 0.0;
  double attitude_scale_ = 0.0;
};

}  // namespace offline_lc_minimal::factor
