#include "offline_lc_minimal/core/StaticVerticalBiasCalibrator.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/GraphTimelineBuilder.h"
#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/core/StaticImuAlignment.h"
#include "offline_lc_minimal/factor/GlobalPlanarAccelBiasFactor.h"
#include "offline_lc_minimal/factor/StaticAttitudeDriftFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalAccelBiasFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticZeroAngularRateFactor.h"
#include "offline_lc_minimal/factor/VerticalAccelBiasGmTransitionFactor.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::X;

constexpr double kDisabledAccBiasPriorSigmaMps2 = 1e6;
constexpr double kDisabledGyroBiasPriorSigmaRadps = 1e6;
constexpr double kTimeEpsilonS = 1e-9;

double ComputeBiasDecay(const double dt_s, const double tau_s) {
  return std::exp(-std::max(dt_s, 0.0) / std::max(tau_s, 1e-9));
}

double ResolveVerticalAccBiasSigmaMps2(const OfflineRunnerConfig &config) {
  if (config.enable_static_vertical_bias_carryover &&
      config.static_vertical_bias_carryover_tighten_gm) {
    return config.static_vertical_bias_carryover_vertical_gm_sigma_mps2;
  }
  return config.vertical_acc_bias_sigma_mps2 > 0.0 ? config.vertical_acc_bias_sigma_mps2 : config.bias_acc_sigma;
}

double ComputeVerticalAccBiasProcessVariance(const double dt_s, const OfflineRunnerConfig &config) {
  const double bounded_dt_s = std::max(dt_s, 1e-6);
  return std::pow(ResolveVerticalAccBiasSigmaMps2(config), 2.0) *
         config.vertical_acc_bias_process_noise_scale *
         std::max(1.0 - std::exp(-2.0 * bounded_dt_s / std::max(config.vertical_acc_bias_tau_s, 1e-9)), 1e-9);
}

boost::shared_ptr<gtsam::PreintegrationCombinedParams> MakeImuParams(
  const Eigen::Vector3d &earth_rate_enu,
  const OfflineRunnerConfig &config) {
  auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(config.gravity_mps2);
  imu_params->accelerometerCovariance = std::pow(config.imu_sigma_acc, 2.0) * gtsam::I_3x3;
  imu_params->gyroscopeCovariance = std::pow(config.imu_sigma_gyro, 2.0) * gtsam::I_3x3;
  imu_params->integrationCovariance = std::pow(config.integration_sigma, 2.0) * gtsam::I_3x3;
  imu_params->biasAccCovariance = std::pow(config.bias_acc_sigma, 2.0) * gtsam::I_3x3;
  if (config.enable_vertical_acc_bias_gm_process) {
    imu_params->biasAccCovariance(2, 2) *= 1e6;
  }
  imu_params->biasOmegaCovariance = std::pow(config.bias_gyro_sigma, 2.0) * gtsam::I_3x3;
  imu_params->setOmegaCoriolis(earth_rate_enu);
  imu_params->setUse2ndOrderCoriolis(true);
  return imu_params;
}

void AddStaticStateFactors(
  const StaticImuWindowSummary &summary,
  const OfflineRunnerConfig &config,
  const Eigen::Vector3d &earth_rate_enu,
  const gtsam::Key global_acc_bias_key,
  const std::size_t state_index,
  const double zaru_sigma_radps,
  const double vertical_specific_force_sigma_mps2,
  gtsam::NonlinearFactorGraph &graph) {
  if (config.enable_initial_static_zupt_zaru) {
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      V(state_index),
      gtsam::Vector3::Zero(),
      gtsam::noiseModel::Isotropic::Sigma(3, config.initial_static_zupt_velocity_sigma_mps)));
    graph.add(factor::StaticZeroAngularRateFactor(
      X(state_index),
      B(state_index),
      summary.mean_gyro_radps,
      earth_rate_enu,
      gtsam::noiseModel::Isotropic::Sigma(3, zaru_sigma_radps)));
  }
  if (config.enable_initial_static_vertical_specific_force) {
    graph.add(factor::StaticVerticalSpecificForceFactor(
      X(state_index),
      B(state_index),
      summary.mean_acc_mps2.z(),
      Eigen::Vector3d(0.0, 0.0, config.gravity_mps2),
      gtsam::noiseModel::Isotropic::Sigma(1, vertical_specific_force_sigma_mps2)));
  }
  if (config.enable_initial_static_vertical_bias_soft_prior) {
    graph.add(factor::StaticVerticalAccelBiasFactor(
      B(state_index),
      global_acc_bias_key,
      gtsam::noiseModel::Isotropic::Sigma(1, config.initial_static_vertical_bias_sigma_mps2)));
  }
}

StaticImuWindowSummary SummarizeImuWindow(
  const ImuWindowIntegration &imu_window,
  const StaticImuWindowSummary &fallback_summary) {
  if (imu_window.imu_segments == 0U) {
    return fallback_summary;
  }
  StaticImuWindowSummary summary;
  summary.mean_acc_mps2 = imu_window.mean_acc_mps2;
  summary.mean_gyro_radps = imu_window.mean_gyro_radps;
  summary.sample_count = imu_window.imu_segments;
  summary.used_stationary_filter = false;
  return summary;
}

double ScaleWindowSigma(
  const double base_sigma,
  const StaticImuWindowSummary &static_window,
  const ImuWindowIntegration &imu_window) {
  if (imu_window.imu_segments == 0U || static_window.sample_count == 0U) {
    return base_sigma;
  }
  return base_sigma * std::sqrt(
    static_cast<double>(static_window.sample_count) /
    static_cast<double>(imu_window.imu_segments));
}

}  // namespace

StaticVerticalBiasCalibrationResult StaticVerticalBiasCalibrator::Calibrate(
  const StaticVerticalBiasCalibrationRequest &request) {
  if (request.imu_samples == nullptr || request.imu_samples->empty()) {
    throw std::runtime_error("static vertical bias carryover calibration requires IMU samples");
  }
  if (request.alignment_end_time_s <= request.alignment_start_time_s + kTimeEpsilonS) {
    throw std::runtime_error("static vertical bias carryover calibration requires a positive static window");
  }

  const OfflineRunnerConfig &config = request.config;
  const StaticImuWindowSummary static_window = StaticImuAlignment::CollectWindow(
    *request.imu_samples,
    request.alignment_start_time_s,
    request.alignment_end_time_s - request.alignment_start_time_s,
    config,
    false);
  if (static_window.sample_count == 0U) {
    throw std::runtime_error("static vertical bias carryover calibration found no stationary IMU samples");
  }

  InitialPoseEstimate initial_pose;
  if (!StaticImuAlignment::TryEstimateDualVectorInitialization(
        static_window,
        request.earth_rate_enu,
        config,
        initial_pose)) {
    throw std::runtime_error("static vertical bias carryover calibration failed static IMU alignment");
  }

  const std::vector<double> state_timestamps_s = BuildStateTimestamps(
    request.alignment_start_time_s,
    request.alignment_end_time_s,
    config.initial_static_state_frequency_hz);
  const gtsam::Key global_acc_bias_key = gtsam::Symbol('a', 0);
  const gtsam::Pose3 initial_pose_world(initial_pose.orientation, gtsam::Point3(0.0, 0.0, 0.0));
  const gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias initial_bias = initial_pose.imu_bias;
  const auto imu_params = MakeImuParams(request.earth_rate_enu, config);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const gtsam::Vector6 pose_sigmas =
    (gtsam::Vector6() << config.initial_static_attitude_drift_sigma_rad,
      config.initial_static_attitude_drift_sigma_rad,
      config.initial_yaw_sigma_rad,
      config.initial_position_sigma_m,
      config.initial_position_sigma_m,
      config.initial_position_sigma_m).finished();
  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    X(0),
    initial_pose_world,
    gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas)));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
    V(0),
    initial_velocity,
    gtsam::noiseModel::Isotropic::Sigma(3, config.initial_velocity_sigma_mps)));
  graph.add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(
    B(0),
    initial_bias,
    gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector6() << kDisabledAccBiasPriorSigmaMps2,
        kDisabledAccBiasPriorSigmaMps2,
        kDisabledAccBiasPriorSigmaMps2,
        kDisabledGyroBiasPriorSigmaRadps,
        kDisabledGyroBiasPriorSigmaRadps,
        kDisabledGyroBiasPriorSigmaRadps).finished())));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
    global_acc_bias_key,
    initial_bias.accelerometer(),
    gtsam::noiseModel::Isotropic::Sigma(3, config.bias_acc_prior_sigma)));
  graph.add(factor::GlobalPlanarAccelBiasFactor(
    B(0),
    global_acc_bias_key,
    gtsam::noiseModel::Isotropic::Sigma(2, config.global_acc_bias_tie_sigma_xy_mps2)));
  AddStaticStateFactors(
    static_window,
    config,
    request.earth_rate_enu,
    global_acc_bias_key,
    0U,
    config.initial_static_zaru_sigma_radps,
    config.initial_static_vertical_specific_force_sigma_mps2,
    graph);

  initial_values.insert(X(0), initial_pose_world);
  initial_values.insert(V(0), initial_velocity);
  initial_values.insert(B(0), initial_bias);
  initial_values.insert(global_acc_bias_key, initial_bias.accelerometer());

  gtsam::NavState previous_nav_state(initial_pose_world, initial_velocity);
  gtsam::imuBias::ConstantBias previous_bias = initial_bias;
  double previous_time_s = state_timestamps_s.front();
  for (std::size_t state_index = 1; state_index < state_timestamps_s.size(); ++state_index) {
    const double current_time_s = state_timestamps_s[state_index];
    const auto imu_window =
      IntegrateImuWindow(*request.imu_samples, previous_time_s, current_time_s, imu_params, previous_bias);
    graph.add(gtsam::CombinedImuFactor(
      X(state_index - 1U),
      V(state_index - 1U),
      X(state_index),
      V(state_index),
      B(state_index - 1U),
      B(state_index),
      imu_window.preintegrated_measurements));

    const gtsam::NavState predicted_state =
      imu_window.preintegrated_measurements.predict(previous_nav_state, previous_bias);
    initial_values.insert(X(state_index), predicted_state.pose());
    initial_values.insert(V(state_index), predicted_state.v());
    initial_values.insert(B(state_index), previous_bias);
    graph.add(factor::GlobalPlanarAccelBiasFactor(
      B(state_index),
      global_acc_bias_key,
      gtsam::noiseModel::Isotropic::Sigma(2, config.global_acc_bias_tie_sigma_xy_mps2)));
    const double delta_time_s = current_time_s - previous_time_s;
    const double phi_vertical_acc = ComputeBiasDecay(delta_time_s, config.vertical_acc_bias_tau_s);
    const double vertical_acc_variance = ComputeVerticalAccBiasProcessVariance(delta_time_s, config);
    graph.add(factor::VerticalAccelBiasGmTransitionFactor(
      B(state_index - 1U),
      B(state_index),
      global_acc_bias_key,
      phi_vertical_acc,
      gtsam::noiseModel::Isotropic::Sigma(1, std::sqrt(std::max(vertical_acc_variance, 1e-12)))));
    graph.add(factor::StaticAttitudeDriftFactor(
      X(state_index - 1U),
      X(state_index),
      gtsam::noiseModel::Isotropic::Sigma(3, config.initial_static_attitude_drift_sigma_rad)));
    const double static_zaru_sigma = ScaleWindowSigma(
      config.initial_static_zaru_sigma_radps,
      static_window,
      imu_window);
    const double static_vertical_specific_force_sigma = ScaleWindowSigma(
      config.initial_static_vertical_specific_force_sigma_mps2,
      static_window,
      imu_window);
    AddStaticStateFactors(
      SummarizeImuWindow(imu_window, static_window),
      config,
      request.earth_rate_enu,
      global_acc_bias_key,
      state_index,
      static_zaru_sigma,
      static_vertical_specific_force_sigma,
      graph);

    previous_nav_state = predicted_state;
    previous_time_s = current_time_s;
  }

  gtsam::LevenbergMarquardtParams optimizer_params;
  optimizer_params.setlambdaInitial(config.lm_lambda_initial);
  optimizer_params.setMaxIterations(config.lm_max_iterations);
  optimizer_params.setVerbosityLM(config.verbose ? "SUMMARY" : "SILENT");
  gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial_values, optimizer_params);
  const gtsam::Values optimized_values = optimizer.optimize();
  const gtsam::Vector3 optimized_global_acc_bias = optimized_values.at<gtsam::Vector3>(global_acc_bias_key);

  StaticVerticalBiasCalibrationResult result;
  result.static_window_start_s = request.alignment_start_time_s;
  result.static_window_end_s = request.alignment_end_time_s;
  result.static_sample_count = static_window.sample_count;
  result.static_state_count = state_timestamps_s.size();
  result.initial_baz_mps2 = initial_bias.accelerometer().z();
  result.static_baz_ref_mps2 = optimized_global_acc_bias.z();
  result.initial_error = graph.error(initial_values);
  result.final_error = graph.error(optimized_values);
  return result;
}

StaticVerticalBiasCarryoverDiagnosticRow BuildStaticVerticalBiasCarryoverDiagnostic(
  const StaticVerticalBiasCalibrationResult &calibration,
  const gtsam::Values &optimized_values,
  const std::vector<double> &state_timestamps_s,
  const std::size_t dynamic_start_index,
  const gtsam::Key global_acc_bias_key) {
  StaticVerticalBiasCarryoverDiagnosticRow row;
  row.static_window_start_s = calibration.static_window_start_s;
  row.static_window_end_s = calibration.static_window_end_s;
  row.static_sample_count = calibration.static_sample_count;
  row.static_state_count = calibration.static_state_count;
  row.initial_baz_mps2 = calibration.initial_baz_mps2;
  row.static_baz_ref_mps2 = calibration.static_baz_ref_mps2;
  row.calibration_initial_error = calibration.initial_error;
  row.calibration_final_error = calibration.final_error;

  const gtsam::Vector3 optimized_global_acc_bias = optimized_values.at<gtsam::Vector3>(global_acc_bias_key);
  row.optimized_global_baz_mps2 = optimized_global_acc_bias.z();
  row.optimized_global_baz_delta_mps2 = row.optimized_global_baz_mps2 - calibration.static_baz_ref_mps2;
  row.dynamic_first20_added_factor_count = 0;

  if (dynamic_start_index >= state_timestamps_s.size()) {
    return row;
  }
  const double dynamic_start_time_s = state_timestamps_s[dynamic_start_index];
  const double dynamic_end_time_s = dynamic_start_time_s + 20.0;
  double abs_delta_sum = 0.0;
  double max_abs_delta = 0.0;
  std::size_t count = 0;
  double first_up = std::numeric_limits<double>::quiet_NaN();
  double last_up = std::numeric_limits<double>::quiet_NaN();
  double min_up = std::numeric_limits<double>::infinity();
  double max_up = -std::numeric_limits<double>::infinity();
  double first_vz = std::numeric_limits<double>::quiet_NaN();
  double last_vz = std::numeric_limits<double>::quiet_NaN();
  double min_vz = std::numeric_limits<double>::infinity();
  double max_vz = -std::numeric_limits<double>::infinity();

  for (std::size_t state_index = dynamic_start_index; state_index < state_timestamps_s.size(); ++state_index) {
    const double time_s = state_timestamps_s[state_index];
    if (time_s > dynamic_end_time_s + kTimeEpsilonS) {
      break;
    }
    const auto bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index));
    const auto pose = optimized_values.at<gtsam::Pose3>(X(state_index));
    const auto velocity = optimized_values.at<gtsam::Vector3>(V(state_index));
    const double abs_delta = std::abs(bias.accelerometer().z() - calibration.static_baz_ref_mps2);
    abs_delta_sum += abs_delta;
    max_abs_delta = std::max(max_abs_delta, abs_delta);
    const double up = pose.translation().z();
    const double vz = velocity.z();
    if (count == 0U) {
      first_up = up;
      first_vz = vz;
    }
    last_up = up;
    last_vz = vz;
    min_up = std::min(min_up, up);
    max_up = std::max(max_up, up);
    min_vz = std::min(min_vz, vz);
    max_vz = std::max(max_vz, vz);
    ++count;
  }

  row.dynamic_first20_state_count = count;
  if (count > 0U) {
    row.dynamic_first20_mean_abs_baz_delta_mps2 = abs_delta_sum / static_cast<double>(count);
    row.dynamic_first20_max_abs_baz_delta_mps2 = max_abs_delta;
    row.dynamic_first20_up_delta_m = last_up - first_up;
    row.dynamic_first20_up_range_m = max_up - min_up;
    row.dynamic_first20_vz_delta_mps = last_vz - first_vz;
    row.dynamic_first20_vz_range_mps = max_vz - min_vz;
  }
  return row;
}

}  // namespace offline_lc_minimal
