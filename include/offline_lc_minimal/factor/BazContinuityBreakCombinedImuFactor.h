#pragma once

#include <array>

#include <boost/pointer_cast.hpp>
#include <gtsam/base/Matrix.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace offline_lc_minimal::factor {

class BazContinuityBreakCombinedImuFactor final
    : public gtsam::NoiseModelFactor6<
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        gtsam::imuBias::ConstantBias> {
 public:
  static constexpr std::size_t kKeptErrorDim = 14U;

  BazContinuityBreakCombinedImuFactor(
    gtsam::Key pose_i_key,
    gtsam::Key velocity_i_key,
    gtsam::Key pose_j_key,
    gtsam::Key velocity_j_key,
    gtsam::Key bias_i_key,
    gtsam::Key bias_j_key,
    const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements);

  [[nodiscard]] gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
      gtsam::NonlinearFactor::shared_ptr(new BazContinuityBreakCombinedImuFactor(*this)));
  }

  [[nodiscard]] gtsam::Vector evaluateError(
    const gtsam::Pose3 &pose_i,
    const gtsam::Vector3 &velocity_i,
    const gtsam::Pose3 &pose_j,
    const gtsam::Vector3 &velocity_j,
    const gtsam::imuBias::ConstantBias &bias_i,
    const gtsam::imuBias::ConstantBias &bias_j,
    boost::optional<gtsam::Matrix &> h_pose_i = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_i = boost::none,
    boost::optional<gtsam::Matrix &> h_pose_j = boost::none,
    boost::optional<gtsam::Matrix &> h_velocity_j = boost::none,
    boost::optional<gtsam::Matrix &> h_bias_i = boost::none,
    boost::optional<gtsam::Matrix &> h_bias_j = boost::none) const override;

  [[nodiscard]] const gtsam::PreintegratedCombinedMeasurements &preintegratedMeasurements() const {
    return preintegrated_measurements_;
  }

 private:
  [[nodiscard]] static gtsam::SharedNoiseModel BuildNoiseModel(
    const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements);

  [[nodiscard]] static gtsam::Vector SelectErrorRows(
    const gtsam::Vector9 &preintegration_error,
    const gtsam::Vector6 &bias_error);

  static void AssignSelectedJacobians(
    const gtsam::Matrix &preintegration_jacobian,
    const gtsam::Matrix *bias_jacobian,
    gtsam::Matrix *output);

  gtsam::PreintegratedCombinedMeasurements preintegrated_measurements_;
};

}  // namespace offline_lc_minimal::factor
