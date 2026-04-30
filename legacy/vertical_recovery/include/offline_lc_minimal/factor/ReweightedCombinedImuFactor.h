#pragma once

#include <algorithm>
#include <cmath>
#include <utility>

#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class ReweightedCombinedImuFactor
    : public gtsam::NoiseModelFactor6<
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        gtsam::imuBias::ConstantBias> {
 public:
  ReweightedCombinedImuFactor(
    const gtsam::Key pose_i_key,
    const gtsam::Key velocity_i_key,
    const gtsam::Key pose_j_key,
    const gtsam::Key velocity_j_key,
    const gtsam::Key bias_i_key,
    const gtsam::Key bias_j_key,
    const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements,
    const double attitude_sigma_rad,
    const gtsam::Vector3 &specific_force_sigma_mps2_by_axis)
      : gtsam::NoiseModelFactor6<
          gtsam::Pose3,
          gtsam::Vector3,
          gtsam::Pose3,
          gtsam::Vector3,
          gtsam::imuBias::ConstantBias,
          gtsam::imuBias::ConstantBias>(
          BuildNoiseModel(
            preintegrated_measurements,
            attitude_sigma_rad,
            specific_force_sigma_mps2_by_axis),
          pose_i_key,
          velocity_i_key,
          pose_j_key,
          velocity_j_key,
          bias_i_key,
          bias_j_key),
        combined_imu_factor_(
          pose_i_key,
          velocity_i_key,
          pose_j_key,
          velocity_j_key,
          bias_i_key,
          bias_j_key,
          preintegrated_measurements) {}

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new ReweightedCombinedImuFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &velocity_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &velocity_j,
    const gtsam::imuBias::ConstantBias &bias_i,
    const gtsam::imuBias::ConstantBias &bias_j,
    boost::optional<gtsam::Matrix &> h1 = boost::none,
    boost::optional<gtsam::Matrix &> h2 = boost::none,
    boost::optional<gtsam::Matrix &> h3 = boost::none,
    boost::optional<gtsam::Matrix &> h4 = boost::none,
    boost::optional<gtsam::Matrix &> h5 = boost::none,
    boost::optional<gtsam::Matrix &> h6 = boost::none) const override {
    return combined_imu_factor_.evaluateError(
      pose_i,
      velocity_i,
      pose_j,
      velocity_j,
      bias_i,
      bias_j,
      h1,
      h2,
      h3,
      h4,
      h5,
      h6);
  }

  [[nodiscard]] const gtsam::PreintegratedCombinedMeasurements &preintegratedMeasurements() const {
    return combined_imu_factor_.preintegratedMeasurements();
  }

 private:
  [[nodiscard]] static gtsam::SharedNoiseModel BuildNoiseModel(
    const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements,
    const double attitude_sigma_rad,
    const gtsam::Vector3 &specific_force_sigma_mps2_by_axis) {
    (void)specific_force_sigma_mps2_by_axis;
    gtsam::Matrix covariance = preintegrated_measurements.preintMeasCov();
    covariance = 0.5 * (covariance + covariance.transpose());

    gtsam::Matrix scaling = gtsam::Matrix::Identity(15, 15);
    const auto tighten_axis = [&](const int row, const double target_sigma) {
      if (target_sigma <= 0.0) {
        return;
      }
      const double variance = std::max(covariance(row, row), 1e-12);
      const double sigma = std::sqrt(variance);
      scaling(row, row) = std::min(1.0, target_sigma / sigma);
    };

    for (int axis = 0; axis < 3; ++axis) {
      tighten_axis(axis, attitude_sigma_rad);
    }

    for (int axis = 0; axis < 15; ++axis) {
      const double variance = std::max(covariance(axis, axis), 1e-12);
      const double sigma = std::sqrt(variance);
      if (!std::isfinite(sigma)) {
        scaling(axis, axis) = 1.0;
      }
    }

    covariance = scaling * covariance * scaling.transpose();
    covariance += gtsam::Matrix::Identity(15, 15) * 1e-12;
    return gtsam::noiseModel::Gaussian::Covariance(covariance);
  }

  gtsam::CombinedImuFactor combined_imu_factor_;
};

inline gtsam::NonlinearFactor::shared_ptr MakeReweightedCombinedImuFactor(
  const gtsam::Key pose_i_key,
  const gtsam::Key velocity_i_key,
  const gtsam::Key pose_j_key,
  const gtsam::Key velocity_j_key,
  const gtsam::Key bias_i_key,
  const gtsam::Key bias_j_key,
  const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements,
  const double attitude_sigma_rad,
  const gtsam::Vector3 &specific_force_sigma_mps2_by_axis) {
  return gtsam::NonlinearFactor::shared_ptr(new ReweightedCombinedImuFactor(
    pose_i_key,
    velocity_i_key,
    pose_j_key,
    velocity_j_key,
    bias_i_key,
    bias_j_key,
    preintegrated_measurements,
    attitude_sigma_rad,
    specific_force_sigma_mps2_by_axis));
}

}  // namespace offline_lc_minimal::factor
