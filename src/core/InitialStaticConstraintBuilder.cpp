#include "offline_lc_minimal/core/InitialStaticConstraintBuilder.h"

#include <stdexcept>

#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/factor/StaticZeroAngularRateFactor.h"
#include "offline_lc_minimal/factor/StaticSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalSpecificForceFactor.h"

namespace offline_lc_minimal {

InitialStaticConstraintData InitialStaticConstraintBuilder::Collect(
  const std::vector<ImuSample> &imu_samples,
  const double alignment_start_time_s,
  const double alignment_end_time_s,
  const OfflineRunnerConfig &config) {
  InitialStaticConstraintData constraint_data;
  if (!config.enable_initial_static_zupt_zaru &&
      !config.enable_initial_static_zero_specific_force &&
      !config.enable_initial_static_vertical_specific_force) {
    return constraint_data;
  }

  const double duration_s = alignment_end_time_s - alignment_start_time_s;
  if (duration_s <= 0.0) {
    throw std::runtime_error("initial static constraints require alignment_end_time_s > alignment_start_time_s");
  }

  constraint_data.window_summary = StaticImuAlignment::CollectWindow(
    imu_samples,
    alignment_start_time_s,
    duration_s,
    config,
    false);
  if (constraint_data.window_summary.sample_count == 0U) {
    throw std::runtime_error("initial static constraints found no stationary IMU samples in the alignment window");
  }

  constraint_data.valid = true;
  return constraint_data;
}

void InitialStaticConstraintBuilder::AddFactors(
  const InitialStaticConstraintData &constraint_data,
  const Eigen::Vector3d &earth_rate_enu,
  const OfflineRunnerConfig &config,
  gtsam::NonlinearFactorGraph &graph,
  const gtsam::Key pose_key,
  const gtsam::Key velocity_key,
  const gtsam::Key bias_key) {
  if (!constraint_data.valid) {
    return;
  }

  if (config.enable_initial_static_zupt_zaru) {
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      velocity_key,
      gtsam::Vector3::Zero(),
      gtsam::noiseModel::Isotropic::Sigma(3, config.initial_static_zupt_velocity_sigma_mps)));
  }
  if (config.enable_initial_static_zero_specific_force) {
    graph.add(factor::StaticSpecificForceFactor(
      pose_key,
      bias_key,
      constraint_data.window_summary.mean_acc_mps2,
      Eigen::Vector3d(0.0, 0.0, config.gravity_mps2),
      gtsam::noiseModel::Isotropic::Sigma(3, config.initial_static_specific_force_sigma_mps2)));
  }
  if (config.enable_initial_static_vertical_specific_force) {
    graph.add(factor::StaticVerticalSpecificForceFactor(
      pose_key,
      bias_key,
      constraint_data.window_summary.mean_acc_mps2.z(),
      Eigen::Vector3d(0.0, 0.0, config.gravity_mps2),
      gtsam::noiseModel::Isotropic::Sigma(1, config.initial_static_vertical_specific_force_sigma_mps2)));
  }
  if (config.enable_initial_static_zupt_zaru && !config.enable_initial_static_subgraph) {
    graph.add(factor::StaticZeroAngularRateFactor(
      pose_key,
      bias_key,
      constraint_data.window_summary.mean_gyro_radps,
      earth_rate_enu,
      gtsam::noiseModel::Isotropic::Sigma(3, config.initial_static_zaru_sigma_radps)));
  }
}

}  // namespace offline_lc_minimal
