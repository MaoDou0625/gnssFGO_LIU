#include "offline_lc_minimal/core/OfflineBatchRunner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/TrajectoryInitializer.h"

namespace offline_lc_minimal {

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

struct ImuWindowIntegration {
  gtsam::PreintegratedCombinedMeasurements preintegrated_measurements;
  std::size_t imu_segments = 0;

  explicit ImuWindowIntegration(
    const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &params,
    const gtsam::imuBias::ConstantBias &bias)
      : preintegrated_measurements(params, bias) {}
};

ImuWindowIntegration IntegrateImuWindow(
  const std::vector<ImuSample> &imu_samples,
  const double start_time_s,
  const double end_time_s,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &params,
  const gtsam::imuBias::ConstantBias &bias) {
  if (imu_samples.empty()) {
    throw std::runtime_error("cannot integrate IMU window without IMU samples");
  }
  if (end_time_s <= start_time_s) {
    throw std::runtime_error("invalid IMU integration window");
  }

  ImuWindowIntegration window(params, bias);

  const auto lower_it = std::lower_bound(
    imu_samples.begin(),
    imu_samples.end(),
    start_time_s,
    [](const ImuSample &sample, const double timestamp_s) { return sample.time_s < timestamp_s; });

  std::size_t begin_index = static_cast<std::size_t>(std::distance(imu_samples.begin(), lower_it));
  std::size_t hold_index = 0U;
  if (begin_index > 0U) {
    hold_index = begin_index - 1U;
  } else {
    hold_index = 0U;
  }

  ImuSample held_sample = imu_samples[hold_index];
  double last_time_s = start_time_s;

  for (std::size_t index = begin_index; index < imu_samples.size(); ++index) {
    const auto &current_sample = imu_samples[index];
    if (current_sample.time_s > end_time_s) {
      break;
    }

    const double dt_s = current_sample.time_s - last_time_s;
    if (dt_s > 0.0) {
      window.preintegrated_measurements.integrateMeasurement(
        held_sample.accel_mps2,
        held_sample.gyro_radps,
        dt_s);
      ++window.imu_segments;
    }
    held_sample = current_sample;
    last_time_s = current_sample.time_s;
  }

  if (end_time_s > last_time_s) {
    window.preintegrated_measurements.integrateMeasurement(
      held_sample.accel_mps2,
      held_sample.gyro_radps,
      end_time_s - last_time_s);
    ++window.imu_segments;
  }

  if (window.imu_segments == 0U) {
    throw std::runtime_error("no IMU data available for state interval");
  }

  return window;
}

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

}  // namespace

OfflineBatchRunner::OfflineBatchRunner(OfflineRunnerConfig config)
    : config_(std::move(config)) {}

std::size_t OfflineBatchRunner::FindOriginIndex(const std::vector<GnssSolutionSample> &gnss_samples) const {
  const auto iterator = std::find_if(gnss_samples.begin(), gnss_samples.end(), [](const auto &sample) {
    return sample.has_valid_position();
  });
  if (iterator == gnss_samples.end()) {
    throw std::runtime_error("failed to find a valid GNSS origin sample");
  }
  return static_cast<std::size_t>(std::distance(gnss_samples.begin(), iterator));
}

void OfflineBatchRunner::PopulateEnuPositions(
  std::vector<GnssSolutionSample> &gnss_samples,
  const GeoReference &geo_reference) const {
  for (auto &sample : gnss_samples) {
    if (!sample.has_valid_position()) {
      continue;
    }
    sample.enu_position_m = geo_reference.Forward(sample.lat_rad, sample.lon_rad, sample.h_m);
    sample.has_enu_position = true;
  }
}

bool OfflineBatchRunner::ShouldUseGnssFactor(const GnssSolutionSample &sample, RunSummary &run_summary) const {
  if (!sample.has_valid_position() || !sample.has_enu_position) {
    return false;
  }
  if (config_.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config_.required_best_sol_status_code) {
    ++run_summary.dropped_bad_status_count;
    return false;
  }
  if (config_.drop_no_solution && sample.fix_type() == GnssFixType::kNoSolution) {
    ++run_summary.dropped_no_solution_count;
    return false;
  }
  if (config_.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    ++run_summary.dropped_nonfinite_sigma_count;
    return false;
  }
  return true;
}

double OfflineBatchRunner::GnssFixScale(const GnssFixType fix_type) const {
  switch (fix_type) {
    case GnssFixType::kRtkFix:
      return config_.rtkfix_scale;
    case GnssFixType::kRtkFloat:
      return config_.rtkfloat_scale;
    case GnssFixType::kSingle:
      return config_.single_scale;
    case GnssFixType::kNoSolution:
    default:
      return std::numeric_limits<double>::infinity();
  }
}

Eigen::Vector3d OfflineBatchRunner::ClampGnssSigma(const GnssSolutionSample &sample) const {
  Eigen::Vector3d sigma(sample.sigma_lon_m, sample.sigma_lat_m, sample.sigma_h_m);
  for (int dimension = 0; dimension < sigma.size(); ++dimension) {
    if (!std::isfinite(sigma[dimension])) {
      sigma[dimension] = config_.position_sigma_ceiling_m;
    }
    sigma[dimension] = std::clamp(sigma[dimension], config_.position_sigma_floor_m, config_.position_sigma_ceiling_m);
  }
  sigma *= GnssFixScale(sample.fix_type());
  return sigma;
}

OfflineRunResult OfflineBatchRunner::Run(DataSet dataset) const {
  if (dataset.imu_samples.empty()) {
    throw std::runtime_error("offline runner received an empty IMU data set");
  }
  if (dataset.gnss_samples.empty()) {
    throw std::runtime_error("offline runner received an empty GNSS data set");
  }

  OfflineRunResult run_result;
  run_result.data_summary = dataset.summary;
  run_result.run_summary.gnss_enabled = config_.enable_gnss;

  const std::size_t origin_index = FindOriginIndex(dataset.gnss_samples);
  const auto &origin_sample = dataset.gnss_samples[origin_index];
  GeoReference geo_reference(origin_sample.lat_rad, origin_sample.lon_rad, origin_sample.h_m);
  PopulateEnuPositions(dataset.gnss_samples, geo_reference);

  run_result.run_summary.origin_lat_rad = geo_reference.origin_lat_rad();
  run_result.run_summary.origin_lon_rad = geo_reference.origin_lon_rad();
  run_result.run_summary.origin_h_m = geo_reference.origin_h_m();

  const InitialPoseEstimate initial_pose =
    TrajectoryInitializer::Estimate(dataset.imu_samples, dataset.gnss_samples, origin_index, config_);
  run_result.run_summary.yaw_source = initial_pose.yaw_source;

  const auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(config_.gravity_mps2);
  imu_params->accelerometerCovariance =
    std::pow(config_.imu_sigma_acc, 2.0) * gtsam::I_3x3;
  imu_params->gyroscopeCovariance =
    std::pow(config_.imu_sigma_gyro, 2.0) * gtsam::I_3x3;
  imu_params->integrationCovariance =
    std::pow(config_.integration_sigma, 2.0) * gtsam::I_3x3;
  imu_params->biasAccCovariance =
    std::pow(config_.bias_acc_sigma, 2.0) * gtsam::I_3x3;
  imu_params->biasOmegaCovariance =
    std::pow(config_.bias_gyro_sigma, 2.0) * gtsam::I_3x3;
  imu_params->setOmegaCoriolis(geo_reference.EarthRateEnu());
  imu_params->setUse2ndOrderCoriolis(true);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const gtsam::Pose3 initial_pose_world(
    initial_pose.orientation,
    gtsam::Point3(origin_sample.enu_position_m.x(), origin_sample.enu_position_m.y(), origin_sample.enu_position_m.z()));
  const gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias initial_bias;

  const gtsam::Vector6 pose_sigmas =
    (gtsam::Vector6() << config_.initial_roll_pitch_sigma_rad,
      config_.initial_roll_pitch_sigma_rad,
      config_.initial_yaw_sigma_rad,
      config_.initial_position_sigma_m,
      config_.initial_position_sigma_m,
      config_.initial_position_sigma_m).finished();
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(X(0), initial_pose_world, gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas)));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
    V(0),
    initial_velocity,
    gtsam::noiseModel::Isotropic::Sigma(3, config_.initial_velocity_sigma_mps)));
  graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
    B(0),
    initial_bias,
    gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << config_.bias_acc_prior_sigma,
        config_.bias_acc_prior_sigma,
        config_.bias_acc_prior_sigma,
        config_.bias_gyro_prior_sigma,
        config_.bias_gyro_prior_sigma,
        config_.bias_gyro_prior_sigma).finished())));

  initial_values.insert(X(0), initial_pose_world);
  initial_values.insert(V(0), initial_velocity);
  initial_values.insert(B(0), initial_bias);

  gtsam::NavState previous_nav_state(initial_pose_world, initial_velocity);
  gtsam::imuBias::ConstantBias previous_bias = initial_bias;
  double previous_time_s = origin_sample.time_s;
  std::size_t state_index = 0;

  {
    TrajectoryRow row;
    row.time_s = origin_sample.time_s;
    row.enu_position_m = origin_sample.enu_position_m;
    row.enu_velocity_mps = Eigen::Vector3d::Zero();
    row.ypr_rad = Eigen::Vector3d(initial_pose.yaw_rad, initial_pose.pitch_rad, initial_pose.roll_rad);
    row.bias_acc = initial_bias.accelerometer();
    row.bias_gyro = initial_bias.gyroscope();
    run_result.trajectory.push_back(row);
  }

  for (std::size_t sample_index = origin_index + 1; sample_index < dataset.gnss_samples.size(); ++sample_index) {
    const auto &gnss_sample = dataset.gnss_samples[sample_index];
    if (gnss_sample.time_s <= previous_time_s) {
      continue;
    }

    const auto imu_window =
      IntegrateImuWindow(dataset.imu_samples, previous_time_s, gnss_sample.time_s, imu_params, previous_bias);
    ++state_index;

    graph.add(gtsam::CombinedImuFactor(
      X(state_index - 1),
      V(state_index - 1),
      X(state_index),
      V(state_index),
      B(state_index - 1),
      B(state_index),
      imu_window.preintegrated_measurements));

    const gtsam::NavState predicted_state = imu_window.preintegrated_measurements.predict(previous_nav_state, previous_bias);
    initial_values.insert(X(state_index), predicted_state.pose());
    initial_values.insert(V(state_index), predicted_state.v());
    initial_values.insert(B(state_index), previous_bias);

    const double elapsed_s = gnss_sample.time_s - origin_sample.time_s;
    bool gnss_factor_used = false;
    if (config_.enable_gnss && ShouldUseGnssFactor(gnss_sample, run_result.run_summary)) {
      Eigen::Vector3d sigma_m = ClampGnssSigma(gnss_sample);
      if (config_.early_gnss_relaxation_duration_s > 0.0 &&
          elapsed_s < config_.early_gnss_relaxation_duration_s) {
        const double alpha = 1.0 - (elapsed_s / config_.early_gnss_relaxation_duration_s);
        sigma_m *= 1.0 + alpha * (config_.early_gnss_relaxation_scale - 1.0);
      }

      graph.add(gtsam::GPSFactor(
        X(state_index),
        gtsam::Point3(gnss_sample.enu_position_m.x(), gnss_sample.enu_position_m.y(), gnss_sample.enu_position_m.z()),
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sigma_m.x(), sigma_m.y(), sigma_m.z()))));
      ++run_result.run_summary.gnss_factor_count;
      gnss_factor_used = true;
    }

    TrajectoryRow row;
    row.time_s = gnss_sample.time_s;
    row.enu_position_m = Eigen::Vector3d(predicted_state.position().x(), predicted_state.position().y(), predicted_state.position().z());
    row.enu_velocity_mps = Eigen::Vector3d(predicted_state.v().x(), predicted_state.v().y(), predicted_state.v().z());
    row.ypr_rad = Rot3ToYpr(predicted_state.pose().rotation());
    row.bias_acc = previous_bias.accelerometer();
    row.bias_gyro = previous_bias.gyroscope();
    row.gnss_factor_used = gnss_factor_used;
    row.gnss_fix_type = gnss_sample.fix_type();
    if (gnss_sample.has_enu_position) {
      row.gnss_residual_m = (row.enu_position_m - gnss_sample.enu_position_m).norm();
    }
    run_result.trajectory.push_back(row);

    previous_nav_state = predicted_state;
    previous_time_s = gnss_sample.time_s;
  }

  run_result.run_summary.state_count = state_index + 1U;
  if (state_index == 0U) {
    throw std::runtime_error("runner could not build more than one state");
  }

  gtsam::LevenbergMarquardtParams optimizer_params;
  optimizer_params.maxIterations = config_.lm_max_iterations;
  optimizer_params.lambdaInitial = config_.lm_lambda_initial;
  optimizer_params.setVerbosity(config_.verbose ? "ERROR" : "SILENT");
  optimizer_params.setVerbosityLM(config_.verbose ? "TRYLAMBDA" : "SILENT");

  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, optimizer_params);
  run_result.run_summary.initial_error = optimizer.error();
  const gtsam::Values optimized_values = optimizer.optimize();
  run_result.run_summary.final_error = graph.error(optimized_values);

  for (std::size_t index = 0; index < run_result.trajectory.size(); ++index) {
    const auto pose = optimized_values.at<gtsam::Pose3>(X(index));
    const auto velocity = optimized_values.at<gtsam::Vector3>(V(index));
    const auto bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(index));

    auto &row = run_result.trajectory[index];
    row.enu_position_m = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
    row.enu_velocity_mps = Eigen::Vector3d(velocity.x(), velocity.y(), velocity.z());
    row.ypr_rad = Rot3ToYpr(pose.rotation());
    row.bias_acc = bias.accelerometer();
    row.bias_gyro = bias.gyroscope();

    const std::size_t gnss_sample_index = origin_index + index;
    if (gnss_sample_index < dataset.gnss_samples.size()) {
      const auto &gnss_sample = dataset.gnss_samples[gnss_sample_index];
      row.gnss_fix_type = gnss_sample.fix_type();
      if (gnss_sample.has_enu_position) {
        row.gnss_residual_m = (row.enu_position_m - gnss_sample.enu_position_m).norm();
      }
    }
  }

  return run_result;
}

}  // namespace offline_lc_minimal
