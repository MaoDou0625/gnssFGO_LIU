#pragma once

#include <algorithm>
#include <cmath>

#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class SegmentBiasFeedbackFactor
    : public gtsam::NoiseModelFactor6<
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias> {
 public:
  SegmentBiasFeedbackFactor(
    gtsam::Key pose_i_key,
    gtsam::Key vel_i_key,
    gtsam::Key bias_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key vel_j_key,
    gtsam::Key bias_j_key,
    const gtsam::PreintegratedImuMeasurements &preintegrated_measurements,
    const gtsam::Matrix3 &phi_acc,
    const gtsam::Matrix3 &phi_gyro,
    const double delta_time_s,
    const double attitude_gain,
    const double velocity_gain,
    const double position_gain,
    const gtsam::SharedNoiseModel &noise_model)
      : gtsam::NoiseModelFactor6<
          gtsam::Pose3,
          gtsam::Vector3,
          gtsam::imuBias::ConstantBias,
          gtsam::Pose3,
          gtsam::Vector3,
          gtsam::imuBias::ConstantBias>(noise_model, pose_i_key, vel_i_key, bias_i_key, pose_j_key, vel_j_key, bias_j_key),
        imu_factor_(pose_i_key, vel_i_key, pose_j_key, vel_j_key, bias_i_key, preintegrated_measurements),
        phi_acc_(phi_acc),
        phi_gyro_(phi_gyro),
        delta_time_s_(std::max(delta_time_s, 1e-6)),
        attitude_gain_(attitude_gain),
        velocity_gain_(velocity_gain),
        position_gain_(position_gain) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new SegmentBiasFeedbackFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &vel_i,
    const gtsam::imuBias::ConstantBias &bias_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &vel_j,
    const gtsam::imuBias::ConstantBias &bias_j,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none,
    boost::optional<gtsam::Matrix &> h3 = boost::none,
    boost::optional<gtsam::Matrix &> h4 = boost::none,
    boost::optional<gtsam::Matrix &> h5 = boost::none,
    boost::optional<gtsam::Matrix &> h6 = boost::none) const override {
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
      (h3 || h6) ? boost::optional<gtsam::Matrix &>(h_bias_i_imu) : boost::none);

    const gtsam::Vector3 dtheta_rad = imu_error.segment<3>(0);
    const gtsam::Vector3 dp_m = imu_error.segment<3>(3);
    const gtsam::Vector3 dv_mps = imu_error.segment<3>(6);

    const double inv_dt = 1.0 / delta_time_s_;
    const double inv_dt_sq = inv_dt * inv_dt;
    const gtsam::Vector3 target_dbg = -attitude_gain_ * inv_dt * dtheta_rad;
    const gtsam::Vector3 target_dba =
      -(velocity_gain_ * inv_dt * dv_mps + position_gain_ * (2.0 * inv_dt_sq) * dp_m);

    gtsam::Vector6 residual = gtsam::Vector6::Zero();
    residual.segment<3>(0) = bias_j.accelerometer() - phi_acc_ * bias_i.accelerometer() - target_dba;
    residual.segment<3>(3) = bias_j.gyroscope() - phi_gyro_ * bias_i.gyroscope() - target_dbg;

    if (h1) {
      *h1 = ComposePoseJacobian(h_pose_i_imu);
    }
    if (h2) {
      *h2 = ComposeVelocityJacobian(h_vel_i_imu);
    }
    if (h3) {
      gtsam::Matrix66 jacobian = gtsam::Matrix66::Zero();
      jacobian.block<3, 3>(0, 0) = -phi_acc_;
      jacobian.block<3, 3>(3, 3) = -phi_gyro_;
      if (h_bias_i_imu.rows() == 9 && h_bias_i_imu.cols() == 6) {
        jacobian.block<3, 6>(0, 0) +=
          velocity_gain_ * inv_dt * h_bias_i_imu.block<3, 6>(6, 0) +
          position_gain_ * (2.0 * inv_dt_sq) * h_bias_i_imu.block<3, 6>(3, 0);
        jacobian.block<3, 6>(3, 0) += attitude_gain_ * inv_dt * h_bias_i_imu.block<3, 6>(0, 0);
      }
      *h3 = jacobian;
    }
    if (h4) {
      *h4 = ComposePoseJacobian(h_pose_j_imu);
    }
    if (h5) {
      *h5 = ComposeVelocityJacobian(h_vel_j_imu);
    }
    if (h6) {
      *h6 = gtsam::Matrix66::Identity();
    }

    return residual;
  }

 private:
  [[nodiscard]] gtsam::Matrix ComposePoseJacobian(const gtsam::Matrix &imu_pose_jacobian) const {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(6, imu_pose_jacobian.cols());
    if (imu_pose_jacobian.rows() != 9) {
      return jacobian;
    }
    const double inv_dt = 1.0 / delta_time_s_;
    const double inv_dt_sq = inv_dt * inv_dt;
    jacobian.block(0, 0, 3, imu_pose_jacobian.cols()) =
      velocity_gain_ * inv_dt * imu_pose_jacobian.block(6, 0, 3, imu_pose_jacobian.cols()) +
      position_gain_ * (2.0 * inv_dt_sq) * imu_pose_jacobian.block(3, 0, 3, imu_pose_jacobian.cols());
    jacobian.block(3, 0, 3, imu_pose_jacobian.cols()) =
      attitude_gain_ * inv_dt * imu_pose_jacobian.block(0, 0, 3, imu_pose_jacobian.cols());
    return jacobian;
  }

  [[nodiscard]] gtsam::Matrix ComposeVelocityJacobian(const gtsam::Matrix &imu_velocity_jacobian) const {
    gtsam::Matrix jacobian = gtsam::Matrix::Zero(6, imu_velocity_jacobian.cols());
    if (imu_velocity_jacobian.rows() != 9) {
      return jacobian;
    }
    const double inv_dt = 1.0 / delta_time_s_;
    const double inv_dt_sq = inv_dt * inv_dt;
    jacobian.block(0, 0, 3, imu_velocity_jacobian.cols()) =
      velocity_gain_ * inv_dt * imu_velocity_jacobian.block(6, 0, 3, imu_velocity_jacobian.cols()) +
      position_gain_ * (2.0 * inv_dt_sq) * imu_velocity_jacobian.block(3, 0, 3, imu_velocity_jacobian.cols());
    jacobian.block(3, 0, 3, imu_velocity_jacobian.cols()) =
      attitude_gain_ * inv_dt * imu_velocity_jacobian.block(0, 0, 3, imu_velocity_jacobian.cols());
    return jacobian;
  }

  gtsam::ImuFactor imu_factor_;
  gtsam::Matrix3 phi_acc_;
  gtsam::Matrix3 phi_gyro_;
  double delta_time_s_;
  double attitude_gain_;
  double velocity_gain_;
  double position_gain_;
};

}  // namespace offline_lc_minimal::factor
