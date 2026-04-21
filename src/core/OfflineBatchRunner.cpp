#include "offline_lc_minimal/core/OfflineBatchRunner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/core/ImuRateAvpReconstructor.h"
#include "offline_lc_minimal/core/TrajectoryInitializer.h"
#include "offline_lc_minimal/factor/AngularRateFactor.h"
#include "offline_lc_minimal/factor/GPInterpolatedGPSFactor.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {

namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimeEpsilonS = 1e-9;
constexpr double kAngularRateSigmaRadps = 0.1;
constexpr double kInterpolatorQcVariance = 10000.0;

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

gtsam::SharedNoiseModel MakeGnssNoiseModel(const OfflineRunnerConfig &config, const Eigen::Vector3d &sigma_m) {
  const gtsam::Vector3 variances(
    sigma_m.x() * sigma_m.x(),
    sigma_m.y() * sigma_m.y(),
    sigma_m.z() * sigma_m.z());
  const auto gaussian_model = gtsam::noiseModel::Diagonal::Variances(variances);

  switch (config.gnss_position_noise_model) {
    case GnssNoiseModel::kGaussian:
      return gaussian_model;
    case GnssNoiseModel::kCauchy:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Cauchy::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kHuber:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kDcs:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::DCS::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kTukey:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Tukey::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kGemanMcClure:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::GemanMcClure::Create(config.gnss_position_robust_param),
        gaussian_model);
    case GnssNoiseModel::kWelsch:
      return gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Welsch::Create(config.gnss_position_robust_param),
        gaussian_model);
    default:
      return gaussian_model;
  }
}

std::vector<double> BuildStateTimestamps(
  const double start_time_s,
  const double end_time_s,
  const double state_frequency_hz) {
  if (state_frequency_hz <= 0.0) {
    throw std::runtime_error("state_frequency_hz must be positive");
  }
  if (end_time_s <= start_time_s) {
    throw std::runtime_error("state timeline end must be after start");
  }

  const double dt_s = 1.0 / state_frequency_hz;
  std::vector<double> timestamps;
  timestamps.push_back(start_time_s);

  for (double next_time_s = start_time_s + dt_s; next_time_s < end_time_s - kTimeEpsilonS; next_time_s += dt_s) {
    timestamps.push_back(next_time_s);
  }

  if (timestamps.back() < end_time_s - kTimeEpsilonS) {
    timestamps.push_back(end_time_s);
  } else {
    timestamps.back() = end_time_s;
  }

  if (timestamps.size() < 2U) {
    throw std::runtime_error("state timeline must contain at least two states");
  }
  return timestamps;
}

StateMeasSyncResult FindStateForMeasurement(
  const std::map<std::size_t, double> &state_timestamp_map,
  const double corrected_timestamp_s,
  const OfflineRunnerConfig &config) {
  StateMeasSyncResult result;
  if (state_timestamp_map.empty()) {
    result.status = StateMeasSyncStatus::kDropped;
    return result;
  }

  if (state_timestamp_map.begin()->second > corrected_timestamp_s &&
      (corrected_timestamp_s - state_timestamp_map.begin()->second) < config.state_meas_sync_lower_bound_s) {
    result.status = StateMeasSyncStatus::kDropped;
    return result;
  }

  if (corrected_timestamp_s < state_timestamp_map.begin()->second) {
    result.status = StateMeasSyncStatus::kDropped;
    return result;
  }

  const auto last_state_it = std::prev(state_timestamp_map.end());
  if ((corrected_timestamp_s - last_state_it->second) > config.state_meas_sync_upper_bound_s) {
    result.status = StateMeasSyncStatus::kCached;
    return result;
  }

  const auto next_state_it = std::find_if(
    state_timestamp_map.begin(),
    state_timestamp_map.end(),
    [corrected_timestamp_s](const auto &entry) { return entry.second >= corrected_timestamp_s; });

  const bool has_next = next_state_it != state_timestamp_map.end();
  const bool has_prev = has_next ? next_state_it != state_timestamp_map.begin() : !state_timestamp_map.empty();
  auto prev_state_it = has_next ? next_state_it : last_state_it;
  if (has_next && has_prev) {
    --prev_state_it;
  }

  struct SyncCandidate {
    bool valid{false};
    double abs_dt_s{std::numeric_limits<double>::infinity()};
    bool synchronized_to_i{false};
    std::size_t key_index_i{0U};
    std::size_t key_index_j{0U};
    double timestamp_i_s{0.0};
    double timestamp_j_s{0.0};
    double duration_from_state_i_s{0.0};
  };

  const auto make_sync_candidate =
    [&](const std::map<std::size_t, double>::const_iterator state_it, const bool synchronized_to_i) {
      SyncCandidate candidate;
      if (state_it == state_timestamp_map.end()) {
        return candidate;
      }

      const double state_meas_dt_s = corrected_timestamp_s - state_it->second;
      if (state_meas_dt_s < config.state_meas_sync_lower_bound_s ||
          state_meas_dt_s > config.state_meas_sync_upper_bound_s) {
        return candidate;
      }

      candidate.valid = true;
      candidate.abs_dt_s = std::abs(state_meas_dt_s);
      candidate.synchronized_to_i = synchronized_to_i;

      if (synchronized_to_i) {
        candidate.key_index_i = state_it->first;
        candidate.timestamp_i_s = state_it->second;
        candidate.duration_from_state_i_s = std::max(0.0, corrected_timestamp_s - candidate.timestamp_i_s);

        if (std::next(state_it) != state_timestamp_map.end()) {
          candidate.key_index_j = std::next(state_it)->first;
          candidate.timestamp_j_s = std::next(state_it)->second;
        } else {
          candidate.key_index_j = candidate.key_index_i;
          candidate.timestamp_j_s = candidate.timestamp_i_s;
        }
      } else {
        if (state_it == state_timestamp_map.begin()) {
          candidate.valid = false;
          return candidate;
        }

        const auto previous_state_it = std::prev(state_it);
        candidate.key_index_j = state_it->first;
        candidate.timestamp_j_s = state_it->second;
        candidate.key_index_i = previous_state_it->first;
        candidate.timestamp_i_s = previous_state_it->second;
        candidate.duration_from_state_i_s = std::max(0.0, corrected_timestamp_s - candidate.timestamp_i_s);
      }

      return candidate;
    };

  SyncCandidate best_candidate;
  if (has_prev) {
    best_candidate = make_sync_candidate(prev_state_it, true);
  } else if (has_next) {
    best_candidate = make_sync_candidate(next_state_it, true);
  }
  if (has_next) {
    const SyncCandidate next_candidate = make_sync_candidate(next_state_it, false);
    const bool is_nearly_tied =
      std::abs(next_candidate.abs_dt_s - best_candidate.abs_dt_s) <= std::numeric_limits<double>::epsilon();
    if (next_candidate.valid &&
        (!best_candidate.valid ||
         next_candidate.abs_dt_s < best_candidate.abs_dt_s ||
         (is_nearly_tied && !best_candidate.synchronized_to_i))) {
      best_candidate = next_candidate;
    }
  }

  if (best_candidate.valid) {
    result.found_i = true;
    result.key_index_i = best_candidate.key_index_i;
    result.key_index_j = best_candidate.key_index_j;
    result.timestamp_i_s = best_candidate.timestamp_i_s;
    result.timestamp_j_s = best_candidate.timestamp_j_s;
    result.duration_from_state_i_s = best_candidate.duration_from_state_i_s;
    result.status = best_candidate.synchronized_to_i ? StateMeasSyncStatus::kSynchronizedI
                                                     : StateMeasSyncStatus::kSynchronizedJ;
    return result;
  }

  if (has_prev && has_next) {
    result.key_index_i = prev_state_it->first;
    result.timestamp_i_s = prev_state_it->second;
    result.duration_from_state_i_s = corrected_timestamp_s - result.timestamp_i_s;
    result.key_index_j = next_state_it->first;
    result.timestamp_j_s = next_state_it->second;
    result.found_i = true;
    result.status = StateMeasSyncStatus::kInterpolated;
    return result;
  }

  result.status = StateMeasSyncStatus::kDropped;
  return result;
}

double ComputeResidualNorm(
  const gtsam::Pose3 &pose,
  const Eigen::Vector3d &measurement_enu_m) {
  const Eigen::Vector3d position(
    pose.translation().x(),
    pose.translation().y(),
    pose.translation().z());
  return (position - measurement_enu_m).norm();
}

}  // namespace

OfflineBatchRunner::OfflineBatchRunner(OfflineRunnerConfig config)
    : config_(std::move(config)) {}

double OfflineBatchRunner::CorrectedGnssTime(const GnssSolutionSample &sample) const {
  return sample.time_s - config_.gnss_time_offset_s;
}

bool OfflineBatchRunner::IsWithinImuCoverage(const std::vector<ImuSample> &imu_samples, const double time_s) const {
  if (imu_samples.empty()) {
    return false;
  }
  return time_s >= imu_samples.front().time_s - kTimeEpsilonS &&
         time_s <= imu_samples.back().time_s + kTimeEpsilonS;
}

bool OfflineBatchRunner::CanUseGnssSampleForInitialization(
  const GnssSolutionSample &sample,
  const std::vector<ImuSample> &imu_samples) const {
  if (!sample.has_valid_position()) {
    return false;
  }
  if (config_.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config_.required_best_sol_status_code) {
    return false;
  }
  if (config_.drop_no_solution && sample.fix_type() == GnssFixType::kNoSolution) {
    return false;
  }
  if (config_.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return IsWithinImuCoverage(imu_samples, CorrectedGnssTime(sample));
}

std::size_t OfflineBatchRunner::FindOriginIndex(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<ImuSample> &imu_samples) const {
  for (std::size_t index = 0; index < gnss_samples.size(); ++index) {
    if (CanUseGnssSampleForInitialization(gnss_samples[index], imu_samples)) {
      return index;
    }
  }
  throw std::runtime_error("failed to find a GNSS origin sample that also passes quality and IMU coverage checks");
}

std::vector<std::size_t> OfflineBatchRunner::CollectInitializationCandidateIndices(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<ImuSample> &imu_samples) const {
  std::vector<std::size_t> indices;
  indices.reserve(gnss_samples.size());
  for (std::size_t index = 0; index < gnss_samples.size(); ++index) {
    if (CanUseGnssSampleForInitialization(gnss_samples[index], imu_samples)) {
      indices.push_back(index);
    }
  }
  return indices;
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
    ++run_summary.gnss_dropped_count;
    return false;
  }
  if (config_.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config_.required_best_sol_status_code) {
    ++run_summary.dropped_bad_status_count;
    ++run_summary.gnss_dropped_count;
    return false;
  }
  if (config_.drop_no_solution && sample.fix_type() == GnssFixType::kNoSolution) {
    ++run_summary.dropped_no_solution_count;
    ++run_summary.gnss_dropped_count;
    return false;
  }
  if (config_.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    ++run_summary.dropped_nonfinite_sigma_count;
    ++run_summary.gnss_dropped_count;
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
      return std::max(config_.single_scale * 10.0, config_.single_scale);
  }
}

Eigen::Vector3d OfflineBatchRunner::ClampGnssSigma(const GnssSolutionSample &sample) const {
  Eigen::Vector3d sigma(sample.sigma_lon_m, sample.sigma_lat_m, sample.sigma_h_m);
  for (int dimension = 0; dimension < sigma.size(); ++dimension) {
    if (!std::isfinite(sigma[dimension])) {
      sigma[dimension] = config_.position_sigma_ceiling_m;
    }
    sigma[dimension] =
      std::clamp(sigma[dimension], config_.position_sigma_floor_m, config_.position_sigma_ceiling_m);
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

  const std::size_t origin_index = FindOriginIndex(dataset.gnss_samples, dataset.imu_samples);
  const auto &origin_sample = dataset.gnss_samples[origin_index];
  GeoReference geo_reference(origin_sample.lat_rad, origin_sample.lon_rad, origin_sample.h_m);
  PopulateEnuPositions(dataset.gnss_samples, geo_reference);

  run_result.run_summary.origin_lat_rad = geo_reference.origin_lat_rad();
  run_result.run_summary.origin_lon_rad = geo_reference.origin_lon_rad();
  run_result.run_summary.origin_h_m = geo_reference.origin_h_m();

  const std::vector<std::size_t> initialization_candidate_indices =
    CollectInitializationCandidateIndices(dataset.gnss_samples, dataset.imu_samples);
  const double start_time_s = CorrectedGnssTime(origin_sample);
  const InitialPoseEstimate initial_pose =
    TrajectoryInitializer::Estimate(
      dataset.imu_samples,
      dataset.gnss_samples,
      origin_index,
      start_time_s,
      geo_reference.EarthRateEnu(),
      initialization_candidate_indices,
      config_);
  run_result.run_summary.yaw_source = initial_pose.yaw_source;

  double end_time_s = std::numeric_limits<double>::lowest();
  for (const auto &sample : dataset.gnss_samples) {
    if (!sample.has_valid_position()) {
      continue;
    }
    const double corrected_time_s = CorrectedGnssTime(sample);
    if (IsWithinImuCoverage(dataset.imu_samples, corrected_time_s)) {
      end_time_s = std::max(end_time_s, corrected_time_s);
    }
  }
  if (!std::isfinite(end_time_s) || end_time_s <= start_time_s) {
    throw std::runtime_error("failed to find a valid offline processing time range");
  }

  const std::vector<double> state_timestamps = BuildStateTimestamps(start_time_s, end_time_s, config_.state_frequency_hz);
  std::map<std::size_t, double> state_timestamp_map;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    state_timestamp_map.emplace(index, state_timestamps[index]);
  }

  const auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(config_.gravity_mps2);
  imu_params->accelerometerCovariance = std::pow(config_.imu_sigma_acc, 2.0) * gtsam::I_3x3;
  imu_params->gyroscopeCovariance = std::pow(config_.imu_sigma_gyro, 2.0) * gtsam::I_3x3;
  imu_params->integrationCovariance = std::pow(config_.integration_sigma, 2.0) * gtsam::I_3x3;
  imu_params->biasAccCovariance = std::pow(config_.bias_acc_sigma, 2.0) * gtsam::I_3x3;
  imu_params->biasOmegaCovariance = std::pow(config_.bias_gyro_sigma, 2.0) * gtsam::I_3x3;
  imu_params->setOmegaCoriolis(geo_reference.EarthRateEnu());
  imu_params->setUse2ndOrderCoriolis(true);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const gtsam::Pose3 initial_pose_world(
    initial_pose.orientation,
    gtsam::Point3(
      origin_sample.enu_position_m.x(),
      origin_sample.enu_position_m.y(),
      origin_sample.enu_position_m.z()));
  const gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias initial_bias = initial_pose.imu_bias;
  const std::size_t initial_imu_index = FindNearestImuIndex(dataset.imu_samples, start_time_s);
  const gtsam::Vector3 initial_omega =
    initial_bias.correctGyroscope(dataset.imu_samples[initial_imu_index].gyro_radps);

  const gtsam::Vector6 pose_sigmas =
    (gtsam::Vector6() << config_.initial_roll_pitch_sigma_rad,
      config_.initial_roll_pitch_sigma_rad,
      config_.initial_yaw_sigma_rad,
      config_.initial_position_sigma_m,
      config_.initial_position_sigma_m,
      config_.initial_position_sigma_m).finished();

  graph.add(gtsam::PriorFactor<gtsam::Pose3>(
    X(0),
    initial_pose_world,
    gtsam::noiseModel::Diagonal::Sigmas(pose_sigmas)));
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
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
    W(0),
    initial_omega,
    gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));

  initial_values.insert(X(0), initial_pose_world);
  initial_values.insert(V(0), initial_velocity);
  initial_values.insert(B(0), initial_bias);
  initial_values.insert(W(0), initial_omega);

  gtsam::NavState previous_nav_state(initial_pose_world, initial_velocity);
  gtsam::imuBias::ConstantBias previous_bias = initial_bias;
  double previous_time_s = state_timestamps.front();

  run_result.trajectory.reserve(state_timestamps.size());
  {
    TrajectoryRow row;
    row.time_s = state_timestamps.front();
    row.enu_position_m = origin_sample.enu_position_m;
    row.enu_velocity_mps = Eigen::Vector3d::Zero();
    row.ypr_rad = Eigen::Vector3d(initial_pose.yaw_rad, initial_pose.pitch_rad, initial_pose.roll_rad);
    row.omega_radps = Eigen::Vector3d(initial_omega.x(), initial_omega.y(), initial_omega.z());
    row.bias_acc = initial_bias.accelerometer();
    row.bias_gyro = initial_bias.gyroscope();
    run_result.trajectory.push_back(row);
  }

  for (std::size_t state_index = 1; state_index < state_timestamps.size(); ++state_index) {
    const double current_time_s = state_timestamps[state_index];
    const auto imu_window =
      IntegrateImuWindow(dataset.imu_samples, previous_time_s, current_time_s, imu_params, previous_bias);

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
    initial_values.insert(W(state_index), imu_window.end_gyro_radps);

    graph.add(factor::AngularRateFactor(
      W(state_index),
      B(state_index),
      imu_window.end_gyro_radps,
      gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));

    TrajectoryRow row;
    row.time_s = current_time_s;
    row.enu_position_m =
      Eigen::Vector3d(predicted_state.position().x(), predicted_state.position().y(), predicted_state.position().z());
    row.enu_velocity_mps = Eigen::Vector3d(predicted_state.v().x(), predicted_state.v().y(), predicted_state.v().z());
    row.ypr_rad = Rot3ToYpr(predicted_state.pose().rotation());
    row.omega_radps = Eigen::Vector3d(
      imu_window.end_gyro_radps.x(),
      imu_window.end_gyro_radps.y(),
      imu_window.end_gyro_radps.z());
    row.bias_acc = previous_bias.accelerometer();
    row.bias_gyro = previous_bias.gyroscope();
    run_result.trajectory.push_back(row);

    previous_nav_state = predicted_state;
    previous_time_s = current_time_s;
  }

  run_result.run_summary.state_count = state_timestamps.size();

  if (config_.enable_gnss) {
    const gp::GPWNOJInterpolator base_interpolator(
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance)));

    for (std::size_t sample_index = 0; sample_index < dataset.gnss_samples.size(); ++sample_index) {
      if (sample_index == origin_index) {
        continue;
      }
      const auto &gnss_sample = dataset.gnss_samples[sample_index];
      GnssFactorRecord record;
      record.sample_index = sample_index;
      record.raw_time_s = gnss_sample.time_s;
      record.corrected_time_s = CorrectedGnssTime(gnss_sample);
      record.gnss_fix_type = gnss_sample.fix_type();
      if (gnss_sample.has_enu_position) {
        record.measurement_enu_m = gnss_sample.enu_position_m;
      }

      if (!ShouldUseGnssFactor(gnss_sample, run_result.run_summary)) {
        record.sync_status = StateMeasSyncStatus::kDropped;
        run_result.gnss_factor_records.push_back(record);
        continue;
      }
      if (!IsWithinImuCoverage(dataset.imu_samples, record.corrected_time_s)) {
        ++run_result.run_summary.dropped_out_of_imu_coverage_count;
        ++run_result.run_summary.gnss_dropped_count;
        record.sync_status = StateMeasSyncStatus::kDropped;
        run_result.gnss_factor_records.push_back(record);
        continue;
      }

      const StateMeasSyncResult sync_result =
        FindStateForMeasurement(state_timestamp_map, record.corrected_time_s, config_);
      record.sync_status = sync_result.status;
      record.state_index_i = sync_result.key_index_i;
      record.state_index_j = sync_result.key_index_j;
      record.state_time_i_s = sync_result.timestamp_i_s;
      record.state_time_j_s = sync_result.timestamp_j_s;
      record.duration_from_state_i_s = sync_result.duration_from_state_i_s;

      const double elapsed_s = record.corrected_time_s - state_timestamps.front();
      Eigen::Vector3d sigma_m = ClampGnssSigma(gnss_sample);
      if (config_.early_gnss_relaxation_duration_s > 0.0 &&
          elapsed_s < config_.early_gnss_relaxation_duration_s) {
        const double alpha = 1.0 - (elapsed_s / config_.early_gnss_relaxation_duration_s);
        sigma_m *= 1.0 + alpha * (config_.early_gnss_relaxation_scale - 1.0);
      }
      const auto noise_model = MakeGnssNoiseModel(config_, sigma_m);

      switch (sync_result.status) {
        case StateMeasSyncStatus::kSynchronizedI:
        case StateMeasSyncStatus::kSynchronizedJ: {
          const std::size_t sync_state_index =
            sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i : sync_result.key_index_j;
          graph.add(gtsam::GPSFactor(
            X(sync_state_index),
            gtsam::Point3(
              gnss_sample.enu_position_m.x(),
              gnss_sample.enu_position_m.y(),
              gnss_sample.enu_position_m.z()),
            noise_model));
          ++run_result.run_summary.gnss_factor_count;
          ++run_result.run_summary.gnss_synced_factor_count;
          record.factor_used = true;
          record.synchronized_state_index = sync_state_index;
          run_result.trajectory[sync_state_index].gnss_factor_used = true;
          if (run_result.trajectory[sync_state_index].gnss_fix_type == GnssFixType::kNoSolution) {
            run_result.trajectory[sync_state_index].gnss_fix_type = gnss_sample.fix_type();
          }
          break;
        }
        case StateMeasSyncStatus::kInterpolated: {
          if (!config_.enable_gp_interpolated_gnss || !sync_result.state_j_exists()) {
            ++run_result.run_summary.gnss_dropped_count;
            break;
          }
          gp::GPWNOJInterpolator interpolator = base_interpolator;
          interpolator.Recalculate(
            sync_result.timestamp_j_s - sync_result.timestamp_i_s,
            sync_result.duration_from_state_i_s);
          graph.add(factor::GPInterpolatedGPSFactor(
            X(sync_result.key_index_i),
            V(sync_result.key_index_i),
            W(sync_result.key_index_i),
            X(sync_result.key_index_j),
            V(sync_result.key_index_j),
            W(sync_result.key_index_j),
            gtsam::Point3(
              gnss_sample.enu_position_m.x(),
              gnss_sample.enu_position_m.y(),
              gnss_sample.enu_position_m.z()),
            gtsam::Vector3::Zero(),
            noise_model,
            interpolator));
          ++run_result.run_summary.gnss_factor_count;
          ++run_result.run_summary.gnss_interpolated_factor_count;
          record.factor_used = true;
          run_result.trajectory[sync_result.key_index_i].gnss_factor_used = true;
          run_result.trajectory[sync_result.key_index_j].gnss_factor_used = true;
          if (run_result.trajectory[sync_result.key_index_i].gnss_fix_type == GnssFixType::kNoSolution) {
            run_result.trajectory[sync_result.key_index_i].gnss_fix_type = gnss_sample.fix_type();
          }
          if (run_result.trajectory[sync_result.key_index_j].gnss_fix_type == GnssFixType::kNoSolution) {
            run_result.trajectory[sync_result.key_index_j].gnss_fix_type = gnss_sample.fix_type();
          }
          break;
        }
        case StateMeasSyncStatus::kCached:
          ++run_result.run_summary.gnss_cached_count;
          break;
        case StateMeasSyncStatus::kDropped:
        default:
          ++run_result.run_summary.gnss_dropped_count;
          break;
      }

      run_result.gnss_factor_records.push_back(record);
    }
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
    const auto omega = optimized_values.at<gtsam::Vector3>(W(index));

    auto &row = run_result.trajectory[index];
    row.enu_position_m = Eigen::Vector3d(pose.translation().x(), pose.translation().y(), pose.translation().z());
    row.enu_velocity_mps = Eigen::Vector3d(velocity.x(), velocity.y(), velocity.z());
    row.ypr_rad = Rot3ToYpr(pose.rotation());
    row.omega_radps = Eigen::Vector3d(omega.x(), omega.y(), omega.z());
    row.bias_acc = bias.accelerometer();
    row.bias_gyro = bias.gyroscope();
  }

  if (config_.write_imu_rate_avp) {
    std::vector<OptimizedNodeState> optimized_node_states;
    optimized_node_states.reserve(state_timestamps.size());
    for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
      OptimizedNodeState node_state;
      node_state.time_s = state_timestamps[index];
      node_state.pose = optimized_values.at<gtsam::Pose3>(X(index));
      node_state.velocity = optimized_values.at<gtsam::Vector3>(V(index));
      node_state.bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(index));
      optimized_node_states.push_back(node_state);
    }

    ImuRateAvpReconstructionResult imu_rate_result =
      ImuRateAvpReconstructor::Reconstruct(
        dataset.imu_samples,
        optimized_node_states,
        imu_params,
        config_.verbose);
    run_result.imu_rate_avp = std::move(imu_rate_result.rows);
    run_result.imu_rate_interval_diagnostics = std::move(imu_rate_result.diagnostics);
    run_result.run_summary.imu_rate_avp_count = run_result.imu_rate_avp.size();
    run_result.run_summary.imu_rate_interval_count = state_timestamps.size() - 1U;
    run_result.run_summary.imu_rate_skipped_interval_count =
      std::count_if(
        run_result.imu_rate_interval_diagnostics.begin(),
        run_result.imu_rate_interval_diagnostics.end(),
        [](const ImuRateIntervalDiagnostic &diagnostic) { return !diagnostic.used_interval; });
  }

  if (config_.enable_gnss) {
    const gp::GPWNOJInterpolator base_interpolator(
      gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance)));

    for (auto &record : run_result.gnss_factor_records) {
      if (!record.factor_used) {
        continue;
      }

      if (record.sync_status == StateMeasSyncStatus::kSynchronizedI ||
          record.sync_status == StateMeasSyncStatus::kSynchronizedJ) {
        const auto pose = optimized_values.at<gtsam::Pose3>(X(record.synchronized_state_index));
        record.residual_m = ComputeResidualNorm(pose, record.measurement_enu_m);
        auto &row = run_result.trajectory[record.synchronized_state_index];
        if (!std::isfinite(row.gnss_residual_m)) {
          row.gnss_residual_m = record.residual_m;
        }
      } else if (record.sync_status == StateMeasSyncStatus::kInterpolated) {
        gp::GPWNOJInterpolator interpolator = base_interpolator;
        interpolator.Recalculate(
          record.state_time_j_s - record.state_time_i_s,
          record.duration_from_state_i_s);
        const gtsam::Pose3 interpolated_pose = interpolator.InterpolatePose(
          optimized_values.at<gtsam::Pose3>(X(record.state_index_i)),
          optimized_values.at<gtsam::Vector3>(V(record.state_index_i)),
          optimized_values.at<gtsam::Vector3>(W(record.state_index_i)),
          optimized_values.at<gtsam::Pose3>(X(record.state_index_j)),
          optimized_values.at<gtsam::Vector3>(V(record.state_index_j)),
          optimized_values.at<gtsam::Vector3>(W(record.state_index_j)));
        record.residual_m = ComputeResidualNorm(interpolated_pose, record.measurement_enu_m);
        auto &row_i = run_result.trajectory[record.state_index_i];
        auto &row_j = run_result.trajectory[record.state_index_j];
        if (!std::isfinite(row_i.gnss_residual_m)) {
          row_i.gnss_residual_m = record.residual_m;
        }
        if (!std::isfinite(row_j.gnss_residual_m)) {
          row_j.gnss_residual_m = record.residual_m;
        }
      }
    }
  }

  return run_result;
}

}  // namespace offline_lc_minimal
