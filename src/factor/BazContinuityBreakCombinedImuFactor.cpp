#include "offline_lc_minimal/factor/BazContinuityBreakCombinedImuFactor.h"

#include <array>

#include <gtsam/linear/NoiseModel.h>

namespace offline_lc_minimal::factor {
namespace {

constexpr std::array<int, BazContinuityBreakCombinedImuFactor::kKeptErrorDim> kKeptErrorRows{
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13, 14};

constexpr double kCovarianceDiagonalFloor = 1.0e-12;

gtsam::Matrix SelectCovarianceRows(const gtsam::Matrix &full_covariance) {
  gtsam::Matrix selected_covariance(
    BazContinuityBreakCombinedImuFactor::kKeptErrorDim,
    BazContinuityBreakCombinedImuFactor::kKeptErrorDim);
  for (std::size_t row = 0; row < kKeptErrorRows.size(); ++row) {
    for (std::size_t col = 0; col < kKeptErrorRows.size(); ++col) {
      selected_covariance(row, col) = full_covariance(kKeptErrorRows[row], kKeptErrorRows[col]);
    }
    if (selected_covariance(row, row) < kCovarianceDiagonalFloor) {
      selected_covariance(row, row) = kCovarianceDiagonalFloor;
    }
  }
  return selected_covariance;
}

}  // namespace

BazContinuityBreakCombinedImuFactor::BazContinuityBreakCombinedImuFactor(
  const gtsam::Key pose_i_key,
  const gtsam::Key velocity_i_key,
  const gtsam::Key pose_j_key,
  const gtsam::Key velocity_j_key,
  const gtsam::Key bias_i_key,
  const gtsam::Key bias_j_key,
  const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements)
    : gtsam::NoiseModelFactor6<
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::Pose3,
        gtsam::Vector3,
        gtsam::imuBias::ConstantBias,
        gtsam::imuBias::ConstantBias>(
        BuildNoiseModel(preintegrated_measurements),
        pose_i_key,
        velocity_i_key,
        pose_j_key,
        velocity_j_key,
        bias_i_key,
        bias_j_key),
      preintegrated_measurements_(preintegrated_measurements) {}

gtsam::SharedNoiseModel BazContinuityBreakCombinedImuFactor::BuildNoiseModel(
  const gtsam::PreintegratedCombinedMeasurements &preintegrated_measurements) {
  return gtsam::noiseModel::Gaussian::Covariance(
    SelectCovarianceRows(preintegrated_measurements.preintMeasCov()));
}

gtsam::Vector BazContinuityBreakCombinedImuFactor::SelectErrorRows(
  const gtsam::Vector9 &preintegration_error,
  const gtsam::Vector6 &bias_error) {
  gtsam::Vector full_error(15);
  full_error.head<9>() = preintegration_error;
  full_error.tail<6>() = bias_error;

  gtsam::Vector selected_error(kKeptErrorDim);
  for (std::size_t row = 0; row < kKeptErrorRows.size(); ++row) {
    selected_error(static_cast<Eigen::Index>(row)) = full_error(kKeptErrorRows[row]);
  }
  return selected_error;
}

void BazContinuityBreakCombinedImuFactor::AssignSelectedJacobians(
  const gtsam::Matrix &preintegration_jacobian,
  const gtsam::Matrix *bias_jacobian,
  gtsam::Matrix *output) {
  if (output == nullptr) {
    return;
  }
  const Eigen::Index cols =
    bias_jacobian != nullptr ? bias_jacobian->cols() : preintegration_jacobian.cols();
  output->setZero(kKeptErrorDim, cols);

  for (std::size_t row = 0; row < kKeptErrorRows.size(); ++row) {
    const int source_row = kKeptErrorRows[row];
    if (source_row < 9) {
      output->row(static_cast<Eigen::Index>(row)) = preintegration_jacobian.row(source_row);
    } else if (bias_jacobian != nullptr) {
      output->row(static_cast<Eigen::Index>(row)) = bias_jacobian->row(source_row - 9);
    }
  }
}

gtsam::Vector BazContinuityBreakCombinedImuFactor::evaluateError(
  const gtsam::Pose3 &pose_i,
  const gtsam::Vector3 &velocity_i,
  const gtsam::Pose3 &pose_j,
  const gtsam::Vector3 &velocity_j,
  const gtsam::imuBias::ConstantBias &bias_i,
  const gtsam::imuBias::ConstantBias &bias_j,
  boost::optional<gtsam::Matrix &> h_pose_i,
  boost::optional<gtsam::Matrix &> h_velocity_i,
  boost::optional<gtsam::Matrix &> h_pose_j,
  boost::optional<gtsam::Matrix &> h_velocity_j,
  boost::optional<gtsam::Matrix &> h_bias_i,
  boost::optional<gtsam::Matrix &> h_bias_j) const {
  gtsam::Matrix h_pre_pose_i;
  gtsam::Matrix h_pre_velocity_i;
  gtsam::Matrix h_pre_pose_j;
  gtsam::Matrix h_pre_velocity_j;
  gtsam::Matrix h_pre_bias_i;
  const bool need_h_pose_i = static_cast<bool>(h_pose_i);
  const bool need_h_velocity_i = static_cast<bool>(h_velocity_i);
  const bool need_h_pose_j = static_cast<bool>(h_pose_j);
  const bool need_h_velocity_j = static_cast<bool>(h_velocity_j);
  const bool need_h_bias_i = static_cast<bool>(h_bias_i);

  const gtsam::Vector9 preintegration_error =
    preintegrated_measurements_.computeErrorAndJacobians(
      pose_i,
      velocity_i,
      pose_j,
      velocity_j,
      bias_i,
      need_h_pose_i ? boost::optional<gtsam::Matrix &>(h_pre_pose_i) : boost::none,
      need_h_velocity_i ? boost::optional<gtsam::Matrix &>(h_pre_velocity_i) : boost::none,
      need_h_pose_j ? boost::optional<gtsam::Matrix &>(h_pre_pose_j) : boost::none,
      need_h_velocity_j ? boost::optional<gtsam::Matrix &>(h_pre_velocity_j) : boost::none,
      need_h_bias_i ? boost::optional<gtsam::Matrix &>(h_pre_bias_i) : boost::none);

  const gtsam::Vector6 bias_error = bias_i.vector() - bias_j.vector();

  if (h_pose_i) {
    AssignSelectedJacobians(h_pre_pose_i, nullptr, &(*h_pose_i));
  }
  if (h_velocity_i) {
    AssignSelectedJacobians(h_pre_velocity_i, nullptr, &(*h_velocity_i));
  }
  if (h_pose_j) {
    AssignSelectedJacobians(h_pre_pose_j, nullptr, &(*h_pose_j));
  }
  if (h_velocity_j) {
    AssignSelectedJacobians(h_pre_velocity_j, nullptr, &(*h_velocity_j));
  }
  if (h_bias_i) {
    const gtsam::Matrix h_bias_error_i = gtsam::Matrix66::Identity();
    AssignSelectedJacobians(h_pre_bias_i, &h_bias_error_i, &(*h_bias_i));
  }
  if (h_bias_j) {
    const gtsam::Matrix h_pre_bias_j = gtsam::Matrix::Zero(9, 6);
    const gtsam::Matrix h_bias_error_j = -gtsam::Matrix66::Identity();
    AssignSelectedJacobians(h_pre_bias_j, &h_bias_error_j, &(*h_bias_j));
  }

  return SelectErrorRows(preintegration_error, bias_error);
}

}  // namespace offline_lc_minimal::factor
