#include "offline_lc_minimal/core/OfflineBatchRunner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <iterator>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <utility>
#include <vector>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/AttitudeReferenceConstraintBuilder.h"
#include "offline_lc_minimal/core/BodyZBiasReestimateConstraintBuilder.h"
#include "offline_lc_minimal/core/BodyZBiasReestimatePlanner.h"
#include "offline_lc_minimal/core/BodyZWindowPipeline.h"
#include "offline_lc_minimal/core/BodyZNHCConstraintBuilder.h"
#include "offline_lc_minimal/core/GnssFactorBuilder.h"
#include "offline_lc_minimal/core/GraphTimelineBuilder.h"
#include "offline_lc_minimal/core/ImuIntegrationUtils.h"
#include "offline_lc_minimal/core/InitialStaticBiasConstraintBuilder.h"
#include "offline_lc_minimal/core/InitialStaticConstraintBuilder.h"
#include "offline_lc_minimal/core/InitialDynamicStaticConstraintBuilder.h"
#include "offline_lc_minimal/core/InitialDynamicStaticDetector.h"
#include "offline_lc_minimal/core/InitialStaticPositionConstraintBuilder.h"
#include "offline_lc_minimal/core/InitialStaticRtkHeightConstraintBuilder.h"
#include "offline_lc_minimal/core/ImuRateAvpReconstructor.h"
#include "offline_lc_minimal/core/LateStaticDetector.h"
#include "offline_lc_minimal/core/LateStaticVerticalConstraintBuilder.h"
#include "offline_lc_minimal/core/RunDiagnosticsBuilder.h"
#include "offline_lc_minimal/core/RtkOutageInitialValueSmoother.h"
#include "offline_lc_minimal/core/RtkOutageBazReestimatePlanner.h"
#include "offline_lc_minimal/core/RtkOutageBoundaryConstraintBuilder.h"
#include "offline_lc_minimal/core/RtkOutageCausalReferenceBuilder.h"
#include "offline_lc_minimal/core/RtkOutagePreOutageVerticalFenceBuilder.h"
#include "offline_lc_minimal/core/RtkOutageRecoveryConstraintBuilder.h"
#include "offline_lc_minimal/core/RtkOutageSegmentedBatchRunner.h"
#include "offline_lc_minimal/core/RtkOutageSmoothingConstraintBuilder.h"
#include "offline_lc_minimal/core/RtkVelocityConstraintBuilder.h"
#include "offline_lc_minimal/core/RtkOutageWindowPlanner.h"
#include "offline_lc_minimal/core/RtkVerticalDriftReferenceEstimator.h"
#include "offline_lc_minimal/core/Stage1OutageBodyYEnvelopeConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage2AttitudeHoldBuilder.h"
#include "offline_lc_minimal/core/Stage2HorizontalHoldBuilder.h"
#include "offline_lc_minimal/core/Stage2VehicleNHCConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage2VelocityOptimizationRunner.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"
#include "offline_lc_minimal/core/Stage3JumpRegularizerConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceConstraintBuilder.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceOptimizationRunner.h"
#include "offline_lc_minimal/core/Stage3VerticalReferenceTimelineAligner.h"
#include "offline_lc_minimal/core/Stage1YawRefinementRunner.h"
#include "offline_lc_minimal/core/Stage2LowfreqVerticalReferenceOptimizationRunner.h"
#include "offline_lc_minimal/core/TrajectoryResultBuilder.h"
#include "offline_lc_minimal/core/TrajectoryInitializer.h"
#include "offline_lc_minimal/core/VerticalAdaptiveReweightingLoop.h"
#include "offline_lc_minimal/core/VerticalAccelBiasGmConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalJumpBiasConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalJumpImpulseConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalJumpImuMasker.h"
#include "offline_lc_minimal/core/VerticalJumpShapeConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalMotionConstraintBuilder.h"
#include "offline_lc_minimal/core/VerticalMotionStabilityEstimator.h"
#include "offline_lc_minimal/core/VerticalPositionVelocityConsistencyConstraintBuilder.h"
#include "offline_lc_minimal/factor/AngularRateFactor.h"
#include "offline_lc_minimal/factor/BiasGmTransitionFactor.h"
#include "offline_lc_minimal/factor/GlobalAccelBiasFactor.h"
#include "offline_lc_minimal/factor/GlobalGyroBiasFactor.h"
#include "offline_lc_minimal/factor/GlobalPlanarAccelBiasFactor.h"
#include "offline_lc_minimal/factor/SegmentBiasFeedbackFactor.h"
#include "offline_lc_minimal/factor/StaticZeroAngularRateFactor.h"
#include "offline_lc_minimal/factor/StaticSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticVerticalSpecificForceFactor.h"
#include "offline_lc_minimal/factor/StaticAttitudeDriftFactor.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {

namespace {

constexpr double kNumericalSigmaFloorM = 1e-4;

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kTimeEpsilonS = 1e-9;
constexpr double kAngularRateSigmaRadps = 0.1;
constexpr double kInterpolatorQcVariance = 10000.0;
constexpr double kDisabledAccBiasPriorSigmaMps2 = 1e6;
constexpr double kDisabledGyroBiasPriorSigmaRadps = 1e6;

double ComputeBiasDecay(const double dt_s, const double tau_s) {
  return std::exp(-std::max(dt_s, 0.0) / std::max(tau_s, 1e-9));
}

gtsam::Matrix3 ComputeBiasPhi(const double dt_s, const double tau_s) {
  return ComputeBiasDecay(dt_s, tau_s) * gtsam::I_3x3;
}

gtsam::Matrix66 ComputeBiasProcessCovariance(const double dt_s, const OfflineRunnerConfig &config) {
  const double bounded_dt_s = std::max(dt_s, 1e-6);
  const double acc_variance =
    std::pow(config.bias_acc_sigma, 2.0) * config.bias_process_noise_acc_scale *
    std::max(1.0 - std::exp(-2.0 * bounded_dt_s / std::max(config.tau_acc_bias_s, 1e-9)), 1e-9);
  const double gyro_variance =
    std::pow(config.bias_gyro_sigma, 2.0) * config.bias_process_noise_gyro_scale *
    std::max(1.0 - std::exp(-2.0 * bounded_dt_s / std::max(config.tau_gyro_bias_s, 1e-9)), 1e-12);

  gtsam::Matrix66 covariance = gtsam::Matrix66::Zero();
  covariance.block<3, 3>(0, 0) = acc_variance * gtsam::I_3x3;
  covariance.block<3, 3>(3, 3) = gyro_variance * gtsam::I_3x3;
  return covariance;
}

bool Stage2ReferenceMatchesGraphTimeline(
  const Stage2VelocityReference &reference,
  const std::vector<double> &state_timestamps) {
  if (reference.trajectory.size() != state_timestamps.size()) {
    return false;
  }
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    if (!std::isfinite(reference.trajectory[index].time_s) ||
        std::abs(reference.trajectory[index].time_s - state_timestamps[index]) > 1.0e-6) {
      return false;
    }
  }
  return true;
}

}  // namespace

OfflineBatchRunner::OfflineBatchRunner(OfflineRunnerConfig config)
    : config_(std::move(config)) {}

OfflineBatchRunner::OfflineBatchRunner(
  OfflineRunnerConfig config,
  std::shared_ptr<const Stage2VelocityReference> stage2_reference)
    : config_(std::move(config)),
      stage2_reference_(std::move(stage2_reference)) {}

OfflineBatchRunner::OfflineBatchRunner(
  OfflineRunnerConfig config,
  std::shared_ptr<const Stage2VelocityReference> stage2_reference,
  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference)
    : config_(std::move(config)),
      stage2_reference_(std::move(stage2_reference)),
      stage1_body_y_reference_(std::move(stage1_body_y_reference)) {}

OfflineBatchRunner::OfflineBatchRunner(
  OfflineRunnerConfig config,
  std::shared_ptr<const Stage2VelocityReference> stage2_reference,
  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference,
  std::shared_ptr<const Stage3VerticalReference> stage3_vertical_reference)
    : config_(std::move(config)),
      stage2_reference_(std::move(stage2_reference)),
      stage1_body_y_reference_(std::move(stage1_body_y_reference)),
      stage3_vertical_reference_(std::move(stage3_vertical_reference)) {}

OfflineBatchRunner::OfflineBatchRunner(
  OfflineRunnerConfig config,
  std::shared_ptr<const Stage2VelocityReference> stage2_reference,
  std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference,
  std::shared_ptr<const Stage3VerticalReference> stage3_vertical_reference,
  std::shared_ptr<const Stage3VerticalReference> stage2_lowpass_vertical_reference)
    : config_(std::move(config)),
      stage2_reference_(std::move(stage2_reference)),
      stage1_body_y_reference_(std::move(stage1_body_y_reference)),
      stage3_vertical_reference_(std::move(stage3_vertical_reference)),
      stage2_lowpass_vertical_reference_(std::move(stage2_lowpass_vertical_reference)) {}

double OfflineBatchRunner::CorrectedGnssTime(const GnssSolutionSample &sample) const {
  return sample.time_s - config_.gnss_time_offset_s;
}

bool OfflineBatchRunner::IsAllowedGnssFixType(const GnssFixType fix_type) const {
  if (!config_.drop_non_rtkfix) {
    return true;
  }
  return fix_type == GnssFixType::kRtkFix;
}

bool OfflineBatchRunner::PassesGnssQualityFilters(const GnssSolutionSample &sample) const {
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
  if (!IsAllowedGnssFixType(sample.fix_type())) {
    return false;
  }
  if (config_.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return true;
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
  if (!PassesGnssQualityFilters(sample)) {
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

std::size_t OfflineBatchRunner::FindNavigationStartIndex(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const std::vector<ImuSample> &imu_samples,
  const std::size_t origin_index,
  const double navigation_start_min_time_s) const {
  for (std::size_t index = origin_index; index < gnss_samples.size(); ++index) {
    if (!CanUseGnssSampleForInitialization(gnss_samples[index], imu_samples)) {
      continue;
    }
    if (CorrectedGnssTime(gnss_samples[index]) + kTimeEpsilonS < navigation_start_min_time_s) {
      continue;
    }
    return index;
  }
  throw std::runtime_error(
    "failed to find a navigation start GNSS sample after the requested static alignment duration");
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
  if (!IsAllowedGnssFixType(sample.fix_type())) {
    ++run_summary.dropped_non_rtkfix_count;
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
  if (!std::isfinite(sigma.x())) {
    sigma.x() = config_.position_sigma_ceiling_m;
  }
  if (!std::isfinite(sigma.y())) {
    sigma.y() = config_.position_sigma_ceiling_m;
  }
  if (config_.gnss_vertical_sigma_mode == GnssVerticalSigmaMode::kFixed) {
    sigma.z() = config_.gnss_vertical_fixed_sigma_m;
  } else if (!std::isfinite(sigma.z())) {
    sigma.z() = config_.position_sigma_ceiling_m;
  }
  sigma.x() *= config_.gnss_sigma_scale_horizontal;
  sigma.y() *= config_.gnss_sigma_scale_horizontal;
  sigma.z() *= config_.gnss_sigma_scale_up;
  sigma.x() = std::clamp(
    sigma.x(),
    std::max(config_.position_sigma_floor_horizontal_m, kNumericalSigmaFloorM),
    config_.position_sigma_ceiling_m);
  sigma.y() = std::clamp(
    sigma.y(),
    std::max(config_.position_sigma_floor_horizontal_m, kNumericalSigmaFloorM),
    config_.position_sigma_ceiling_m);
  sigma.z() = std::clamp(
    sigma.z(),
    std::max(config_.position_sigma_floor_up_m, kNumericalSigmaFloorM),
    config_.position_sigma_ceiling_m);
  sigma *= GnssFixScale(sample.fix_type());
  return sigma;
}

OfflineRunResult OfflineBatchRunner::Run(DataSet dataset) const {
  if (config_.enable_stage2_lowfreq_vertical_reference_optimization &&
      stage2_reference_ == nullptr &&
      stage3_vertical_reference_ == nullptr &&
      stage2_lowpass_vertical_reference_ == nullptr) {
    Stage2LowfreqVerticalReferenceOptimizationRequest request;
    request.config = config_;
    request.dataset = std::move(dataset);
    request.run_once = [](
                         const OfflineRunnerConfig &config,
                         std::shared_ptr<const Stage3VerticalReference> lowpass_reference,
                         DataSet run_dataset) {
      return OfflineBatchRunner(
        config,
        nullptr,
        nullptr,
        nullptr,
        std::move(lowpass_reference)).Run(std::move(run_dataset));
    };
    return Stage2LowfreqVerticalReferenceOptimizationRunner(std::move(request)).Run();
  }
  if (config_.enable_stage3_vertical_reference_optimization &&
      stage2_reference_ == nullptr &&
      stage3_vertical_reference_ == nullptr) {
    Stage3VerticalReferenceOptimizationRequest request;
    request.config = config_;
    request.dataset = std::move(dataset);
    request.run_once = [](
                         const OfflineRunnerConfig &config,
                         std::shared_ptr<const Stage2VelocityReference> stage2_reference,
                         std::shared_ptr<const Stage3VerticalReference> stage3_vertical_reference,
                         DataSet run_dataset) {
      return OfflineBatchRunner(
        config,
        std::move(stage2_reference),
        nullptr,
        std::move(stage3_vertical_reference)).Run(std::move(run_dataset));
    };
    return Stage3VerticalReferenceOptimizationRunner(std::move(request)).Run();
  }
  if (config_.enable_stage2_velocity_optimization && stage2_reference_ == nullptr) {
    auto stage2_lowpass_vertical_reference = stage2_lowpass_vertical_reference_;
    Stage2VelocityOptimizationRequest request;
    request.config = config_;
    request.dataset = std::move(dataset);
    request.run_once = [stage2_lowpass_vertical_reference](
                         const OfflineRunnerConfig &config,
                         std::shared_ptr<const Stage2VelocityReference> stage2_reference,
                         std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference,
                         DataSet run_dataset) {
      return OfflineBatchRunner(
        config,
        std::move(stage2_reference),
        std::move(stage1_body_y_reference),
        nullptr,
        stage2_lowpass_vertical_reference).Run(std::move(run_dataset));
    };
    return Stage2VelocityOptimizationRunner(std::move(request)).Run();
  }
  if (config_.enable_stage1_yaw_refinement) {
    Stage1YawRefinementRequest request;
    request.config = config_;
    request.dataset = std::move(dataset);
    request.body_y_envelope_reference = stage1_body_y_reference_;
    request.run_once = [](
                         const OfflineRunnerConfig &config,
                         std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference,
                         DataSet run_dataset) {
      return OfflineBatchRunner(
        config,
        nullptr,
        std::move(stage1_body_y_reference),
        nullptr).Run(std::move(run_dataset));
    };
    return Stage1YawRefinementRunner(std::move(request)).Run();
  }

  if (dataset.imu_samples.empty()) {
    throw std::runtime_error("offline runner received an empty IMU data set");
  }
  if (dataset.gnss_samples.empty()) {
    throw std::runtime_error("offline runner received an empty GNSS data set");
  }

  OfflineRunResult run_result;
  run_result.data_summary = dataset.summary;
  run_result.run_summary.gnss_enabled = config_.enable_gnss;
  run_result.run_summary.gnss_vertical_reference_source =
    ToString(config_.gnss_vertical_reference_source);
  run_result.run_summary.initial_static_constraints_enabled =
    config_.enable_initial_static_zupt_zaru ||
    config_.enable_initial_static_zero_specific_force ||
    config_.enable_initial_static_vertical_specific_force ||
    config_.enable_initial_static_vertical_bias_soft_prior ||
    config_.enable_initial_static_vertical_bias_gm_tightening ||
    config_.enable_initial_static_vertical_position_hold ||
    config_.enable_initial_static_rtk_height_reference;
  run_result.run_summary.initial_static_subgraph_enabled = config_.enable_initial_static_subgraph;

  const std::size_t origin_index = FindOriginIndex(dataset.gnss_samples, dataset.imu_samples);
  const auto &origin_sample = dataset.gnss_samples[origin_index];
  const double alignment_start_time_s = CorrectedGnssTime(origin_sample);
  const double static_alignment_end_time_s =
    alignment_start_time_s + config_.static_alignment_duration_s;
  double navigation_start_min_time_s = static_alignment_end_time_s;
  if (config_.processing_start_time_s > 0.0) {
    navigation_start_min_time_s =
      std::max(navigation_start_min_time_s, config_.processing_start_time_s);
  }
  const std::size_t navigation_start_index =
    FindNavigationStartIndex(dataset.gnss_samples, dataset.imu_samples, origin_index, navigation_start_min_time_s);
  const auto &navigation_start_sample = dataset.gnss_samples[navigation_start_index];
  const double alignment_end_time_s =
    config_.static_alignment_duration_s > 0.0
      ? static_alignment_end_time_s
      : CorrectedGnssTime(navigation_start_sample);
  GeoReference geo_reference(origin_sample.lat_rad, origin_sample.lon_rad, origin_sample.h_m);
  PopulateEnuPositions(dataset.gnss_samples, geo_reference);

  run_result.run_summary.origin_lat_rad = geo_reference.origin_lat_rad();
  run_result.run_summary.origin_lon_rad = geo_reference.origin_lon_rad();
  run_result.run_summary.origin_h_m = geo_reference.origin_h_m();
  run_result.run_summary.alignment_start_time_s = alignment_start_time_s;
  run_result.run_summary.static_alignment_duration_s = config_.static_alignment_duration_s;

  const std::vector<std::size_t> initialization_candidate_indices =
    CollectInitializationCandidateIndices(dataset.gnss_samples, dataset.imu_samples);
  const double dynamic_start_time_s = CorrectedGnssTime(navigation_start_sample);
  run_result.run_summary.navigation_start_time_s = alignment_start_time_s;
  run_result.run_summary.dynamic_start_time_s = dynamic_start_time_s;
  run_result.run_summary.processing_start_time_s = config_.processing_start_time_s;
  const InitialPoseEstimate initial_pose =
    TrajectoryInitializer::Estimate(
      dataset.imu_samples,
      dataset.gnss_samples,
      navigation_start_index,
      alignment_start_time_s,
      alignment_end_time_s,
      dynamic_start_time_s,
      geo_reference.EarthRateEnu(),
      initialization_candidate_indices,
      config_);
  run_result.run_summary.yaw_source = initial_pose.yaw_source;
  const InitialStaticConstraintData initial_static_constraint_data =
    InitialStaticConstraintBuilder::Collect(
      dataset.imu_samples,
      alignment_start_time_s,
      alignment_end_time_s,
      config_);
  run_result.run_summary.initial_static_constraint_sample_count =
    initial_static_constraint_data.window_summary.sample_count;
  const InitialStaticRtkHeightReference initial_static_rtk_height_reference =
    InitialStaticRtkHeightConstraintBuilder::BuildReference(
      dataset.gnss_samples,
      alignment_start_time_s,
      alignment_end_time_s,
      config_);
  run_result.run_summary.initial_static_rtk_height_reference_sample_count =
    initial_static_rtk_height_reference.sample_count;
  if (initial_static_rtk_height_reference.valid) {
    run_result.run_summary.initial_static_rtk_height_reference_up_m =
      initial_static_rtk_height_reference.reference_up_m;
  }
  if (config_.enable_rtk_vertical_drift_reference && !initial_static_rtk_height_reference.valid) {
    throw std::runtime_error(
      "RTK vertical drift reference requires a valid initial static RTK height reference");
  }
  AccumulateStaticSpecificForceWindowMetrics(
    dataset.imu_samples,
    alignment_start_time_s,
    alignment_end_time_s,
    1.0 / std::max(config_.state_frequency_hz, 1e-9),
    run_result.run_summary);

  double end_time_s = std::numeric_limits<double>::lowest();
  for (const auto &sample : dataset.gnss_samples) {
    if (!sample.has_valid_position()) {
      continue;
    }
    if (config_.drop_non_rtkfix && !IsAllowedGnssFixType(sample.fix_type())) {
      continue;
    }
    const double corrected_time_s = CorrectedGnssTime(sample);
    if (IsWithinImuCoverage(dataset.imu_samples, corrected_time_s)) {
      end_time_s = std::max(end_time_s, corrected_time_s);
    }
  }
  if (!std::isfinite(end_time_s) || end_time_s <= dynamic_start_time_s) {
    throw std::runtime_error("failed to find a valid offline processing time range");
  }
  if (config_.processing_end_time_s > 0.0) {
    if (config_.processing_end_time_s <= dynamic_start_time_s) {
      throw std::runtime_error("processing_end_time_s must be after the dynamic navigation start time");
    }
    end_time_s = std::min(end_time_s, config_.processing_end_time_s);
  }
  run_result.run_summary.processing_end_time_s = end_time_s;

  const GraphTimeline graph_timeline = BuildGraphTimeline(
    alignment_start_time_s,
    alignment_end_time_s,
    dynamic_start_time_s,
    end_time_s,
    config_);
  const std::vector<double> &state_timestamps = graph_timeline.timestamps_s;
  run_result.run_summary.initial_static_state_count = graph_timeline.initial_static_state_count;
  std::map<std::size_t, double> state_timestamp_map;
  for (std::size_t index = 0; index < state_timestamps.size(); ++index) {
    state_timestamp_map.emplace(index, state_timestamps[index]);
  }
  std::shared_ptr<const Stage2VelocityReference> aligned_stage2_reference;
  std::shared_ptr<const Stage3VerticalReference> aligned_stage3_vertical_reference;
  const Stage2VelocityReference *active_stage2_reference = stage2_reference_.get();
  const Stage3VerticalReference *active_stage3_vertical_reference =
    stage3_vertical_reference_.get();
  if (stage2_reference_ != nullptr && stage3_vertical_reference_ != nullptr) {
    Stage3VerticalReferenceTimelineAlignResult aligned_references =
      AlignStage3VerticalReferencesToTimeline(
        *stage2_reference_,
        *stage3_vertical_reference_,
        state_timestamps);
    aligned_stage2_reference =
      std::make_shared<Stage2VelocityReference>(
        std::move(aligned_references.stage2_reference));
    aligned_stage3_vertical_reference =
      std::make_shared<Stage3VerticalReference>(
        std::move(aligned_references.stage3_reference));
    active_stage2_reference = aligned_stage2_reference.get();
    active_stage3_vertical_reference = aligned_stage3_vertical_reference.get();
  }
  const bool collect_error_diagnostics = config_.write_error_diagnostics;
  const bool collect_segment_error_diagnostics =
    config_.write_segment_error_diagnostics ||
    config_.enable_segment_error_feedback;
  const bool collect_reference_states =
    collect_error_diagnostics ||
    collect_segment_error_diagnostics ||
    config_.gnss_consistency_gate_mode != GnssConsistencyGateMode::kNone ||
    config_.enable_body_z_jump_detection ||
    config_.enable_attitude_reference_constraint ||
    (config_.enable_rtk_outage_smoothing && config_.enable_rtk_outage_attitude_hold) ||
    active_stage2_reference != nullptr;
  run_result.run_summary.error_state_count = 0;

  const auto imu_params = gtsam::PreintegrationCombinedParams::MakeSharedU(config_.gravity_mps2);
  imu_params->accelerometerCovariance = std::pow(config_.imu_sigma_acc, 2.0) * gtsam::I_3x3;
  imu_params->gyroscopeCovariance = std::pow(config_.imu_sigma_gyro, 2.0) * gtsam::I_3x3;
  imu_params->integrationCovariance = std::pow(config_.integration_sigma, 2.0) * gtsam::I_3x3;
  imu_params->biasAccCovariance = std::pow(config_.bias_acc_sigma, 2.0) * gtsam::I_3x3;
  if (config_.enable_vertical_acc_bias_gm_process) {
    // In the Stage 2 route, let the explicit ba_z GM factor be the dominant
    // cross-node model for vertical accelerometer bias instead of stacking two
    // equally strong z-bias evolution assumptions.
    imu_params->biasAccCovariance(2, 2) *= 1e6;
  }
  imu_params->biasOmegaCovariance = std::pow(config_.bias_gyro_sigma, 2.0) * gtsam::I_3x3;
  imu_params->setOmegaCoriolis(geo_reference.EarthRateEnu());
  imu_params->setUse2ndOrderCoriolis(true);

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;

  const Eigen::Vector3d initial_position_enu =
    graph_timeline.dynamic_start_index > 0U ? origin_sample.enu_position_m : navigation_start_sample.enu_position_m;
  const gtsam::Pose3 initial_pose_world(
    initial_pose.orientation,
    gtsam::Point3(
      initial_position_enu.x(),
      initial_position_enu.y(),
      initial_position_enu.z()));
  const gtsam::Vector3 initial_velocity = gtsam::Vector3::Zero();
  const gtsam::imuBias::ConstantBias initial_bias = initial_pose.imu_bias;
  const std::size_t initial_imu_index = FindNearestImuIndex(dataset.imu_samples, state_timestamps.front());
  const gtsam::Vector3 initial_omega =
    initial_bias.correctGyroscope(dataset.imu_samples[initial_imu_index].gyro_radps);
  const gtsam::Key global_acc_bias_key = gtsam::Symbol('a', 0);
  const gtsam::Key global_gyro_bias_key = gtsam::Symbol('g', 0);
  const gtsam::Key stage2_mount_leakage_key = gtsam::Symbol('m', 0);

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
      (gtsam::Vector6() << (config_.enable_global_acc_bias ? kDisabledAccBiasPriorSigmaMps2 : config_.bias_acc_prior_sigma),
        (config_.enable_global_acc_bias ? kDisabledAccBiasPriorSigmaMps2 : config_.bias_acc_prior_sigma),
        (config_.enable_global_acc_bias ? kDisabledAccBiasPriorSigmaMps2 : config_.bias_acc_prior_sigma),
        (config_.enable_global_gyro_bias ? kDisabledGyroBiasPriorSigmaRadps : config_.bias_gyro_prior_sigma),
        (config_.enable_global_gyro_bias ? kDisabledGyroBiasPriorSigmaRadps : config_.bias_gyro_prior_sigma),
        (config_.enable_global_gyro_bias ? kDisabledGyroBiasPriorSigmaRadps : config_.bias_gyro_prior_sigma)).finished())));
  graph.add(gtsam::PriorFactor<gtsam::Vector3>(
    W(0),
    initial_omega,
    gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));
  graph.add(factor::AngularRateFactor(
    W(0),
    B(0),
    dataset.imu_samples[initial_imu_index].gyro_radps,
    gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));
  if (config_.enable_global_acc_bias) {
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      global_acc_bias_key,
      initial_bias.accelerometer(),
      gtsam::noiseModel::Isotropic::Sigma(3, config_.bias_acc_prior_sigma)));
    if (config_.enable_vertical_acc_bias_gm_process) {
      graph.add(factor::GlobalPlanarAccelBiasFactor(
        B(0),
        global_acc_bias_key,
        gtsam::noiseModel::Isotropic::Sigma(2, config_.global_acc_bias_tie_sigma_xy_mps2)));
    } else {
      graph.add(factor::GlobalAccelBiasFactor(
        B(0),
        global_acc_bias_key,
        gtsam::noiseModel::Isotropic::Sigma(3, config_.global_acc_bias_tie_sigma_mps2)));
    }
  }
  if (config_.enable_global_gyro_bias) {
    graph.add(gtsam::PriorFactor<gtsam::Vector3>(
      global_gyro_bias_key,
      initial_bias.gyroscope(),
      gtsam::noiseModel::Isotropic::Sigma(3, config_.bias_gyro_prior_sigma)));
    graph.add(factor::GlobalGyroBiasFactor(
      B(0),
      global_gyro_bias_key,
      gtsam::noiseModel::Isotropic::Sigma(3, config_.global_gyro_bias_tie_sigma_radps)));
  }
  InitialStaticConstraintBuilder::AddFactors(
    initial_static_constraint_data,
    geo_reference.EarthRateEnu(),
    config_,
    graph,
    X(0),
    V(0),
    B(0));
  if (InitialStaticBiasConstraintBuilder::AddVerticalAccelBiasSoftPrior(
    config_,
    graph,
    B(0),
    global_acc_bias_key)) {
    ++run_result.run_summary.initial_static_vertical_bias_prior_factor_count;
  }
  if (InitialStaticRtkHeightConstraintBuilder::AddVerticalReference(
    initial_static_rtk_height_reference,
    config_,
    graph,
    X(0))) {
    ++run_result.run_summary.initial_static_rtk_height_reference_factor_count;
  }

  initial_values.insert(X(0), initial_pose_world);
  initial_values.insert(V(0), initial_velocity);
  initial_values.insert(B(0), initial_bias);
  initial_values.insert(W(0), initial_omega);
  if (config_.enable_global_acc_bias) {
    initial_values.insert(global_acc_bias_key, initial_bias.accelerometer());
  }
  if (config_.enable_global_gyro_bias) {
    initial_values.insert(global_gyro_bias_key, initial_bias.gyroscope());
  }

  std::vector<ReferenceNodeState> reference_node_states;
  if (collect_reference_states) {
    reference_node_states.reserve(state_timestamps.size());
    reference_node_states.push_back(MakeReferenceNodeState(
      state_timestamps.front(),
      gtsam::NavState(initial_pose_world, initial_velocity),
      initial_bias,
      initial_omega));
  }

  gtsam::NavState previous_nav_state(initial_pose_world, initial_velocity);
  gtsam::imuBias::ConstantBias previous_bias = initial_bias;
  double previous_time_s = state_timestamps.front();
  std::vector<std::optional<std::size_t>> trajectory_row_index_by_state(state_timestamps.size(), std::nullopt);
  std::vector<VerticalVelocityDeltaPropagationRecord> vertical_velocity_delta_records;
  vertical_velocity_delta_records.reserve(state_timestamps.size() > 0U ? state_timestamps.size() - 1U : 0U);
  std::vector<VelocityDeltaPropagationRecord> velocity_delta_records;
  velocity_delta_records.reserve(state_timestamps.size() > 0U ? state_timestamps.size() - 1U : 0U);
  std::vector<VerticalAccelBiasGmTransitionRecord> vertical_acc_bias_gm_records;
  vertical_acc_bias_gm_records.reserve(state_timestamps.size() > 0U ? state_timestamps.size() - 1U : 0U);
  std::vector<VerticalJumpImuIntervalRecord> vertical_jump_imu_interval_records;
  vertical_jump_imu_interval_records.reserve(state_timestamps.size() > 0U ? state_timestamps.size() - 1U : 0U);
  run_result.trajectory.reserve(state_timestamps.size());
  const auto graph_state_to_trajectory_row = [&](const std::size_t state_index) -> long long {
    if (state_index >= trajectory_row_index_by_state.size()) {
      return -1;
    }
    if (!trajectory_row_index_by_state[state_index].has_value()) {
      return -1;
    }
    return static_cast<long long>(*trajectory_row_index_by_state[state_index]);
  };
  const auto append_trajectory_row =
    [&](const std::size_t state_index,
        const gtsam::NavState &nav_state,
        const gtsam::imuBias::ConstantBias &bias,
        const gtsam::Vector3 &omega,
        const double time_s) {
      TrajectoryRow row;
      row.time_s = time_s;
      row.enu_position_m =
        Eigen::Vector3d(nav_state.position().x(), nav_state.position().y(), nav_state.position().z());
      row.enu_velocity_mps = Eigen::Vector3d(nav_state.v().x(), nav_state.v().y(), nav_state.v().z());
      row.ypr_rad = Rot3ToYpr(nav_state.pose().rotation());
      row.omega_radps = Eigen::Vector3d(omega.x(), omega.y(), omega.z());
      row.bias_acc = bias.accelerometer();
      row.bias_gyro = bias.gyroscope();
      trajectory_row_index_by_state[state_index] = run_result.trajectory.size();
      run_result.trajectory.push_back(row);
    };
  append_trajectory_row(0U, previous_nav_state, initial_bias, initial_omega, state_timestamps.front());

  for (std::size_t state_index = 1; state_index < state_timestamps.size(); ++state_index) {
    const double current_time_s = state_timestamps[state_index];
    const auto imu_window =
      IntegrateImuWindow(dataset.imu_samples, previous_time_s, current_time_s, imu_params, previous_bias);

    const std::size_t imu_factor_index = graph.size();
    if (config_.enable_segment_error_feedback) {
      graph.add(gtsam::ImuFactor(
        X(state_index - 1U),
        V(state_index - 1U),
        X(state_index),
        V(state_index),
        B(state_index - 1U),
        imu_window.preintegrated_imu_measurements));
    } else {
      graph.add(gtsam::CombinedImuFactor(
        X(state_index - 1U),
        V(state_index - 1U),
        X(state_index),
        V(state_index),
        B(state_index - 1U),
        B(state_index),
        imu_window.preintegrated_measurements));
      vertical_jump_imu_interval_records.push_back(VerticalJumpImuIntervalRecord{
        state_index - 1U,
        state_index,
        previous_time_s,
        current_time_s,
        imu_factor_index,
        imu_window.preintegrated_measurements});
    }

    const gtsam::NavState predicted_state =
      config_.enable_segment_error_feedback
        ? imu_window.preintegrated_imu_measurements.predict(previous_nav_state, previous_bias)
        : imu_window.preintegrated_measurements.predict(previous_nav_state, previous_bias);
    vertical_velocity_delta_records.push_back(VerticalVelocityDeltaPropagationRecord{
      state_index - 1U,
      state_index,
      previous_time_s,
      current_time_s,
      predicted_state.v().z() - previous_nav_state.v().z(),
      previous_bias.accelerometer().z()});
    velocity_delta_records.push_back(VelocityDeltaPropagationRecord{
      state_index - 1U,
      state_index,
      previous_time_s,
      current_time_s,
      predicted_state.v() - previous_nav_state.v()});
    initial_values.insert(X(state_index), predicted_state.pose());
    initial_values.insert(V(state_index), predicted_state.v());
    initial_values.insert(B(state_index), previous_bias);
    initial_values.insert(W(state_index), imu_window.end_gyro_radps);
    if (collect_reference_states) {
      reference_node_states.push_back(MakeReferenceNodeState(
        current_time_s,
        predicted_state,
        previous_bias,
        imu_window.end_gyro_radps));
    }
    if (config_.enable_global_acc_bias) {
      if (config_.enable_vertical_acc_bias_gm_process) {
        graph.add(factor::GlobalPlanarAccelBiasFactor(
          B(state_index),
          global_acc_bias_key,
          gtsam::noiseModel::Isotropic::Sigma(2, config_.global_acc_bias_tie_sigma_xy_mps2)));
      } else {
        graph.add(factor::GlobalAccelBiasFactor(
          B(state_index),
          global_acc_bias_key,
          gtsam::noiseModel::Isotropic::Sigma(3, config_.global_acc_bias_tie_sigma_mps2)));
      }
    }
    if (config_.enable_global_gyro_bias) {
      graph.add(factor::GlobalGyroBiasFactor(
        B(state_index),
        global_gyro_bias_key,
        gtsam::noiseModel::Isotropic::Sigma(3, config_.global_gyro_bias_tie_sigma_radps)));
    }
    if (config_.enable_vertical_acc_bias_gm_process) {
      const bool is_initial_static_interval =
        config_.enable_initial_static_subgraph &&
        current_time_s <= alignment_end_time_s + kTimeEpsilonS;
      vertical_acc_bias_gm_records.push_back(VerticalAccelBiasGmTransitionRecord{
        state_index - 1U,
        state_index,
        previous_time_s,
        current_time_s,
        is_initial_static_interval});
    }
    if (config_.enable_segment_error_feedback) {
      const double delta_time_s = current_time_s - previous_time_s;
      const gtsam::Matrix3 phi_acc = ComputeBiasPhi(delta_time_s, config_.tau_acc_bias_s);
      const gtsam::Matrix3 phi_gyro = ComputeBiasPhi(delta_time_s, config_.tau_gyro_bias_s);
      graph.add(factor::BiasGmTransitionFactor(
        B(state_index - 1U),
        B(state_index),
        phi_acc,
        phi_gyro,
        gtsam::noiseModel::Gaussian::Covariance(ComputeBiasProcessCovariance(delta_time_s, config_))));
      if (config_.enable_segment_local_error_feedback) {
        const gtsam::Vector6 segment_feedback_sigmas =
          (gtsam::Vector6() << config_.segment_feedback_acc_sigma_mps2,
            config_.segment_feedback_acc_sigma_mps2,
            config_.segment_feedback_acc_sigma_mps2,
            config_.segment_feedback_gyro_sigma_radps,
            config_.segment_feedback_gyro_sigma_radps,
            config_.segment_feedback_gyro_sigma_radps).finished();
        graph.add(factor::SegmentBiasFeedbackFactor(
          X(state_index - 1U),
          V(state_index - 1U),
          B(state_index - 1U),
          X(state_index),
          V(state_index),
          B(state_index),
          imu_window.preintegrated_imu_measurements,
          phi_acc,
          phi_gyro,
          delta_time_s,
          config_.segment_feedback_attitude_gain,
          config_.segment_feedback_velocity_gain,
          config_.segment_feedback_position_gain,
          gtsam::noiseModel::Diagonal::Sigmas(segment_feedback_sigmas)));
      }
    }

    graph.add(factor::AngularRateFactor(
      W(state_index),
      B(state_index),
      imu_window.end_gyro_radps,
      gtsam::noiseModel::Isotropic::Sigma(3, kAngularRateSigmaRadps)));
    if (config_.enable_initial_static_subgraph && state_timestamps[state_index] <= alignment_end_time_s + kTimeEpsilonS) {
      if (config_.enable_initial_static_zupt_zaru && initial_static_constraint_data.valid) {
        double static_zaru_sigma = config_.initial_static_zaru_sigma_radps;
        if (imu_window.imu_segments > 0U && initial_static_constraint_data.window_summary.sample_count > 0U) {
          static_zaru_sigma *= std::sqrt(
            static_cast<double>(initial_static_constraint_data.window_summary.sample_count) /
            static_cast<double>(imu_window.imu_segments));
        }
        graph.add(gtsam::PriorFactor<gtsam::Vector3>(
          V(state_index),
          gtsam::Vector3::Zero(),
          gtsam::noiseModel::Isotropic::Sigma(3, config_.initial_static_zupt_velocity_sigma_mps)));
        graph.add(factor::StaticZeroAngularRateFactor(
          X(state_index),
          B(state_index),
          imu_window.mean_gyro_radps,
          geo_reference.EarthRateEnu(),
          gtsam::noiseModel::Isotropic::Sigma(3, static_zaru_sigma)));
      }
      if (config_.enable_initial_static_zero_specific_force && initial_static_constraint_data.valid) {
        double static_specific_force_sigma = config_.initial_static_specific_force_sigma_mps2;
        if (imu_window.imu_segments > 0U && initial_static_constraint_data.window_summary.sample_count > 0U) {
          static_specific_force_sigma *= std::sqrt(
            static_cast<double>(initial_static_constraint_data.window_summary.sample_count) /
            static_cast<double>(imu_window.imu_segments));
        }
        graph.add(factor::StaticSpecificForceFactor(
          X(state_index),
          B(state_index),
          imu_window.mean_acc_mps2,
          Eigen::Vector3d(0.0, 0.0, config_.gravity_mps2),
          gtsam::noiseModel::Isotropic::Sigma(3, static_specific_force_sigma)));
      }
      if (config_.enable_initial_static_vertical_specific_force && initial_static_constraint_data.valid) {
        double static_vertical_specific_force_sigma =
          config_.initial_static_vertical_specific_force_sigma_mps2;
        if (imu_window.imu_segments > 0U && initial_static_constraint_data.window_summary.sample_count > 0U) {
          static_vertical_specific_force_sigma *= std::sqrt(
            static_cast<double>(initial_static_constraint_data.window_summary.sample_count) /
            static_cast<double>(imu_window.imu_segments));
        }
        graph.add(factor::StaticVerticalSpecificForceFactor(
          X(state_index),
          B(state_index),
          imu_window.mean_acc_mps2.z(),
          Eigen::Vector3d(0.0, 0.0, config_.gravity_mps2),
          gtsam::noiseModel::Isotropic::Sigma(1, static_vertical_specific_force_sigma)));
      }
      if (InitialStaticBiasConstraintBuilder::AddVerticalAccelBiasSoftPrior(
        config_,
        graph,
        B(state_index),
        global_acc_bias_key)) {
        ++run_result.run_summary.initial_static_vertical_bias_prior_factor_count;
      }
      if (InitialStaticPositionConstraintBuilder::AddVerticalPositionHold(
        config_,
        graph,
        X(0),
        X(state_index))) {
        ++run_result.run_summary.initial_static_vertical_position_hold_factor_count;
      }
      if (InitialStaticPositionConstraintBuilder::AddPositionHold(
        config_,
        graph,
        X(0),
        X(state_index))) {
        ++run_result.run_summary.initial_static_position_hold_factor_count;
      }
      if (InitialStaticRtkHeightConstraintBuilder::AddVerticalReference(
        initial_static_rtk_height_reference,
        config_,
        graph,
        X(state_index))) {
        ++run_result.run_summary.initial_static_rtk_height_reference_factor_count;
      }
      graph.add(factor::StaticAttitudeDriftFactor(
        X(state_index - 1U),
        X(state_index),
        gtsam::noiseModel::Isotropic::Sigma(3, config_.initial_static_attitude_drift_sigma_rad)));
    }
    append_trajectory_row(state_index, predicted_state, previous_bias, imu_window.end_gyro_radps, current_time_s);

    previous_nav_state = predicted_state;
    previous_time_s = current_time_s;
  }

  std::vector<ReferenceNodeState> stage2_fixed_reference_states;
  const bool stage2_reference_matches_graph =
    active_stage2_reference != nullptr &&
    Stage2ReferenceMatchesGraphTimeline(*active_stage2_reference, state_timestamps);
  const bool defer_stage2_reference_for_segmented_batch =
    active_stage2_reference != nullptr &&
    !stage2_reference_matches_graph &&
    config_.enable_rtk_outage_segmented_batch &&
    config_.enable_rtk_outage_smoothing;
  if (active_stage2_reference != nullptr && !defer_stage2_reference_for_segmented_batch) {
    const Stage2ReferenceApplicationOptions stage2_reference_options =
      active_stage3_vertical_reference != nullptr
        ? Stage2ReferenceApplicationOptions{}
        : Stage2AttitudeHorizontalReferenceApplicationOptions();
    ApplyStage2ReferenceTrajectoryToInitialValues(
      *active_stage2_reference,
      state_timestamps,
      initial_values,
      stage2_reference_options);
    UpdateTrajectoryRowsFromOptimizedValues(initial_values, run_result.trajectory);
    stage2_fixed_reference_states =
      BuildReferenceStatesFromOptimizedValues(state_timestamps, initial_values);
    run_result.attitude_reference_states = stage2_fixed_reference_states;
    if (collect_reference_states) {
      reference_node_states = stage2_fixed_reference_states;
    }
  }

  run_result.run_summary.state_count = run_result.trajectory.size();
  if (collect_reference_states) {
    run_result.reference_node_trajectory.reserve(reference_node_states.size());
    for (const auto &reference_state : reference_node_states) {
      run_result.reference_node_trajectory.push_back(MakeReferenceNodeRow(reference_state));
    }
  }

  const bool collect_gnss_consistency =
    config_.enable_gnss &&
    (collect_error_diagnostics || collect_segment_error_diagnostics ||
     config_.gnss_consistency_gate_mode != GnssConsistencyGateMode::kNone);
  const gp::GPWNOJInterpolator base_interpolator(
    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance)));
  const gtsam::NonlinearFactorGraph base_graph = graph;
  const gtsam::Values base_initial_values = initial_values;
  const std::vector<TrajectoryRow> base_dynamic_trajectory = run_result.trajectory;
  const RunSummary base_run_summary = run_result.run_summary;
  gtsam::LevenbergMarquardtParams optimizer_params;
  optimizer_params.maxIterations = config_.lm_max_iterations;
  optimizer_params.lambdaInitial = config_.lm_lambda_initial;
  optimizer_params.setVerbosity(config_.verbose ? "ERROR" : "SILENT");
  optimizer_params.setVerbosityLM(config_.verbose ? "TRYLAMBDA" : "SILENT");
  const auto add_vertical_acc_bias_gm_constraints =
    [&](gtsam::NonlinearFactorGraph &target_graph,
        RunSummary &target_summary,
        const std::vector<BodyZBiasReestimateSegmentRow> *bias_reestimate_segments,
        const VerticalMotionStabilityProfile *stability_profile) {
      VerticalAccelBiasGmConstraintBuildRequest request;
      request.config = &config_;
      request.records = &vertical_acc_bias_gm_records;
      request.bias_reestimate_segments = bias_reestimate_segments;
      request.stability_profile = stability_profile;
      request.global_acc_bias_key = global_acc_bias_key;
      request.graph = &target_graph;
      request.run_summary = &target_summary;
      VerticalAccelBiasGmConstraintBuilder(std::move(request)).Build();
    };

  if (config_.enable_body_z_jump_detection) {
    gtsam::NonlinearFactorGraph body_z_base_graph = base_graph;
    RunSummary body_z_seed_summary = base_run_summary;
    add_vertical_acc_bias_gm_constraints(body_z_base_graph, body_z_seed_summary, nullptr, nullptr);
    BodyZWindowPipelineRequest body_z_request;
    body_z_request.config = &config_;
    body_z_request.imu_samples = &dataset.imu_samples;
    body_z_request.gnss_samples = &dataset.gnss_samples;
    body_z_request.state_timestamps = &state_timestamps;
    body_z_request.base_graph = &body_z_base_graph;
    body_z_request.base_initial_values = &base_initial_values;
    body_z_request.optimizer_params = optimizer_params;
    body_z_request.navigation_start_index = navigation_start_index;
    body_z_request.dynamic_start_time_s = dynamic_start_time_s;
    body_z_request.end_time_s = state_timestamps.back();
    body_z_request.passes_gnss_quality_filters = [&](const GnssSolutionSample &sample) {
      return PassesGnssQualityFilters(sample);
    };
    body_z_request.is_within_imu_coverage = [&](const double corrected_time_s) {
      return IsWithinImuCoverage(dataset.imu_samples, corrected_time_s);
    };
    body_z_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
      return CorrectedGnssTime(sample);
    };
    body_z_request.clamped_sigma_m = [&](const GnssSolutionSample &sample) {
      return ClampGnssSigma(sample);
    };
    body_z_request.find_state_for_time_s = [&](const double corrected_time_s) {
      return FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
    };
    const BodyZWindowPipelineResult body_z_result = BodyZWindowPipeline(std::move(body_z_request)).Run();
    run_result.seed_body_z_acc_diagnostics = body_z_result.imu_diagnostics;
    run_result.body_z_seed_jump_windows = body_z_result.jump_windows;
    run_result.body_z_seed_bias_windows = body_z_result.bias_windows;
    run_result.attitude_reference_states = body_z_result.seed_reference_states;
  }
  if (active_stage2_reference != nullptr) {
    run_result.attitude_reference_states = stage2_fixed_reference_states;
  }
  run_result.body_z_bias_reestimate_segments = PlanBodyZBiasReestimateSegments(
    run_result.body_z_seed_bias_windows,
    run_result.body_z_seed_jump_windows,
    BodyZBiasReestimatePlannerOptions{
      config_.vertical_velocity_delta_jump_padding_s,
      config_.vertical_jump_segmented_bias_min_segment_s,
      config_.body_z_long_bias_min_duration_s});

  std::vector<RtkOutageWindowRow> planned_rtk_outage_windows;
  std::vector<BodyZSeedJumpWindowRow> nhc_constraint_windows =
    run_result.body_z_seed_jump_windows;
  if (config_.enable_rtk_outage_smoothing) {
    RtkOutageWindowPlanRequest outage_request;
    outage_request.config = &config_;
    outage_request.gnss_samples = &dataset.gnss_samples;
    outage_request.state_timestamps = &state_timestamps;
    outage_request.body_z_jump_windows = &run_result.body_z_seed_jump_windows;
    outage_request.navigation_start_index = navigation_start_index;
    outage_request.passes_gnss_quality_filters = [&](const GnssSolutionSample &sample) {
      return PassesGnssQualityFilters(sample);
    };
    outage_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
      return CorrectedGnssTime(sample);
    };
    planned_rtk_outage_windows =
      RtkOutageWindowPlanner(std::move(outage_request)).Plan();
    nhc_constraint_windows = BuildRtkOutageNHCWindows(
      run_result.body_z_seed_jump_windows,
      planned_rtk_outage_windows);
    if (config_.enable_rtk_outage_baz_reestimate) {
      run_result.body_z_bias_reestimate_segments =
        PlanRtkOutageBazReestimateSegments(
          run_result.body_z_bias_reestimate_segments,
          planned_rtk_outage_windows,
          run_result.body_z_seed_jump_windows,
          RtkOutageBazReestimatePlannerOptions{
            config_.vertical_velocity_delta_jump_padding_s,
            config_.vertical_jump_segmented_bias_min_segment_s});
    }
  }

  auto populate_initial_dynamic_static_summary = [&]() {
    run_result.run_summary.initial_dynamic_static_detection_enabled =
      config_.enable_initial_dynamic_static_detection;
    run_result.run_summary.initial_dynamic_static_lowpass_protection_enabled =
      config_.enable_initial_dynamic_static_lowpass_protection;
    run_result.run_summary.initial_dynamic_static_vz_constraint_enabled =
      config_.enable_initial_dynamic_static_vz_constraint;
    run_result.run_summary.initial_dynamic_static_feature_window_count =
      run_result.initial_dynamic_static_feature_diagnostics.size();
    run_result.run_summary.initial_dynamic_static_valid_feature_window_count =
      static_cast<std::size_t>(
        std::count_if(
          run_result.initial_dynamic_static_feature_diagnostics.begin(),
          run_result.initial_dynamic_static_feature_diagnostics.end(),
          [](const LateStaticFeatureDiagnosticRow &row) {
            return row.valid_features;
          }));
    run_result.run_summary.initial_dynamic_static_window_count =
      run_result.initial_dynamic_static_windows.size();
    run_result.run_summary.initial_dynamic_static_vz_factor_count = 0U;
    run_result.run_summary.initial_dynamic_static_up_factor_count = 0U;
    run_result.run_summary.initial_dynamic_static_height_hold_factor_count = 0U;
    run_result.run_summary.initial_dynamic_static_rtk_speed_threshold_mps =
      std::numeric_limits<double>::quiet_NaN();
    run_result.run_summary.initial_dynamic_static_gyro_rms_threshold_radps =
      std::numeric_limits<double>::quiet_NaN();
    run_result.run_summary.initial_dynamic_static_acc_std_threshold_mps2 =
      std::numeric_limits<double>::quiet_NaN();
    for (const auto &row : run_result.initial_dynamic_static_threshold_diagnostics) {
      if (!row.valid || !std::isfinite(row.threshold_value)) {
        continue;
      }
      if (row.feature_name == "rtk_horizontal_speed_rms_mps") {
        run_result.run_summary.initial_dynamic_static_rtk_speed_threshold_mps =
          row.threshold_value;
      } else if (row.feature_name == "imu_gyro_norm_rms_radps") {
        run_result.run_summary.initial_dynamic_static_gyro_rms_threshold_radps =
          row.threshold_value;
      } else if (row.feature_name == "imu_acc_norm_std_mps2") {
        run_result.run_summary.initial_dynamic_static_acc_std_threshold_mps2 =
          row.threshold_value;
      }
    }
  };

  if (config_.enable_initial_dynamic_static_detection) {
    InitialDynamicStaticDetectionRequest initial_dynamic_static_request;
    initial_dynamic_static_request.config = &config_;
    initial_dynamic_static_request.imu_samples = &dataset.imu_samples;
    initial_dynamic_static_request.gnss_samples = &dataset.gnss_samples;
    initial_dynamic_static_request.rtk_outage_windows = &planned_rtk_outage_windows;
    initial_dynamic_static_request.alignment_start_time_s = alignment_start_time_s;
    initial_dynamic_static_request.alignment_end_time_s = alignment_end_time_s;
    initial_dynamic_static_request.dynamic_start_time_s = dynamic_start_time_s;
    initial_dynamic_static_request.processing_end_time_s = end_time_s;
    initial_dynamic_static_request.should_use_rtkfix_sample =
      [&](const GnssSolutionSample &sample) {
        return sample.has_enu_position && PassesGnssQualityFilters(sample);
      };
    initial_dynamic_static_request.corrected_time_s =
      [&](const GnssSolutionSample &sample) {
        return CorrectedGnssTime(sample);
      };
    InitialDynamicStaticDetectionResult initial_dynamic_static_result =
      InitialDynamicStaticDetector(std::move(initial_dynamic_static_request)).Detect();
    run_result.initial_dynamic_static_feature_diagnostics =
      std::move(initial_dynamic_static_result.feature_diagnostics);
    run_result.initial_dynamic_static_threshold_diagnostics =
      std::move(initial_dynamic_static_result.threshold_diagnostics);
    run_result.initial_dynamic_static_windows =
      std::move(initial_dynamic_static_result.windows);
  }
  populate_initial_dynamic_static_summary();

  if (config_.enable_rtk_outage_segmented_batch &&
      config_.enable_rtk_outage_smoothing &&
      !planned_rtk_outage_windows.empty()) {
    RtkOutageSegmentedBatchRunRequest segmented_request;
    segmented_request.base_config =
      active_stage2_reference != nullptr && active_stage2_reference->source_config != nullptr
        ? *active_stage2_reference->source_config
        : config_;
    segmented_request.config = config_;
    segmented_request.dataset = dataset;
    segmented_request.stage2_reference = stage2_reference_;
    segmented_request.outage_windows = planned_rtk_outage_windows;
    segmented_request.bias_reestimate_segments = run_result.body_z_bias_reestimate_segments;
    segmented_request.state_timestamps = state_timestamps;
    segmented_request.dynamic_start_time_s = dynamic_start_time_s;
    segmented_request.processing_end_time_s = end_time_s;
    auto stage2_lowpass_vertical_reference = stage2_lowpass_vertical_reference_;
    segmented_request.run_once = [stage2_lowpass_vertical_reference](
                                   OfflineRunnerConfig segment_config,
                                   std::shared_ptr<const Stage2VelocityReference> stage2_reference,
                                   std::shared_ptr<const Stage1OutageBodyYEnvelopeReference> stage1_body_y_reference,
                                   DataSet segment_dataset) {
      return OfflineBatchRunner(
        std::move(segment_config),
        std::move(stage2_reference),
        std::move(stage1_body_y_reference),
        nullptr,
        stage2_lowpass_vertical_reference).Run(std::move(segment_dataset));
    };
    return RtkOutageSegmentedBatchRunner(std::move(segmented_request)).Run();
  }
  if (defer_stage2_reference_for_segmented_batch) {
    throw std::runtime_error(
      "segmented Stage2 reference does not match the full graph and no RTK outage segment was planned");
  }

  RtkOutageCausalReferenceResult causal_reference_result;
  if (config_.enable_rtk_outage_causal_drift_reference &&
      config_.enable_rtk_outage_smoothing &&
      !planned_rtk_outage_windows.empty()) {
    RtkOutageCausalReferenceBuildRequest causal_request;
    causal_request.config = &config_;
    causal_request.prefix_base_config =
      active_stage2_reference != nullptr && active_stage2_reference->source_config != nullptr
        ? active_stage2_reference->source_config.get()
        : &config_;
    causal_request.dataset = &dataset;
    causal_request.outage_windows = &planned_rtk_outage_windows;
    causal_request.dynamic_start_time_s = dynamic_start_time_s;
    causal_request.should_use_sample = [&](const GnssSolutionSample &sample) {
      return sample.has_enu_position && PassesGnssQualityFilters(sample);
    };
    causal_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
      return CorrectedGnssTime(sample);
    };
    causal_request.is_within_imu_coverage = [&](const double corrected_time_s) {
      return IsWithinImuCoverage(dataset.imu_samples, corrected_time_s);
    };
    causal_request.run_prefix = [](
                                  OfflineRunnerConfig prefix_config,
                                  DataSet prefix_dataset) {
      return OfflineBatchRunner(std::move(prefix_config)).Run(std::move(prefix_dataset));
    };
    causal_reference_result =
      RtkOutageCausalReferenceBuilder(std::move(causal_request)).Build();
  }
  run_result.rtk_outage_causal_nav_reference_diagnostics =
    causal_reference_result.nav_reference_rows;

  if (config_.enable_late_static_detection && active_stage2_reference != nullptr) {
    LateStaticDetectionRequest late_static_request;
    late_static_request.config = &config_;
    late_static_request.imu_samples = &dataset.imu_samples;
    late_static_request.gnss_samples = &dataset.gnss_samples;
    late_static_request.rtk_outage_windows = &planned_rtk_outage_windows;
    late_static_request.processing_start_time_s = config_.processing_start_time_s;
    late_static_request.processing_end_time_s =
      config_.processing_end_time_s > 0.0 ? config_.processing_end_time_s : end_time_s;
    late_static_request.alignment_start_time_s = alignment_start_time_s;
    late_static_request.alignment_end_time_s = alignment_end_time_s;
    late_static_request.should_use_rtkfix_sample =
      [&](const GnssSolutionSample &sample) {
        return sample.has_enu_position && PassesGnssQualityFilters(sample);
      };
    late_static_request.corrected_time_s =
      [&](const GnssSolutionSample &sample) {
        return CorrectedGnssTime(sample);
      };
    run_result.late_static_feature_diagnostics =
      LateStaticFeatureExtractor(std::move(late_static_request)).Extract();
    const LateStaticThresholdSet late_static_thresholds =
      DataDrivenStaticThresholdEstimator(config_).Estimate(
        run_result.late_static_feature_diagnostics);
    run_result.late_static_threshold_diagnostics =
      late_static_thresholds.diagnostics;
    run_result.late_static_windows =
      LateStaticWindowDetector(config_).Detect(
        late_static_thresholds,
        &run_result.late_static_feature_diagnostics);
    run_result.run_summary.late_static_detection_enabled = true;
    run_result.run_summary.late_static_feature_window_count =
      run_result.late_static_feature_diagnostics.size();
    run_result.run_summary.late_static_valid_feature_window_count =
      static_cast<std::size_t>(
        std::count_if(
          run_result.late_static_feature_diagnostics.begin(),
          run_result.late_static_feature_diagnostics.end(),
          [](const LateStaticFeatureDiagnosticRow &row) {
            return row.valid_features;
          }));
    run_result.run_summary.late_static_window_count =
      run_result.late_static_windows.size();
    run_result.run_summary.late_static_rtk_speed_threshold_mps =
      late_static_thresholds.rtk_speed_rms_threshold_mps;
    run_result.run_summary.late_static_gyro_rms_threshold_radps =
      late_static_thresholds.gyro_rms_threshold_radps;
  } else {
    run_result.run_summary.late_static_detection_enabled =
      config_.enable_late_static_detection && active_stage2_reference != nullptr;
  }

  std::vector<VerticalMotionAdaptiveReweightingDiagnosticRow> adaptive_reweighting_diagnostics;
  VerticalMotionStabilityProfile active_stability_profile;
  const VerticalMotionStabilityProfile *active_stability_profile_ptr = nullptr;
  std::vector<RtkVerticalDriftReferenceDiagnosticRow> active_rtk_vertical_drift_profile;
  const std::vector<RtkVerticalDriftReferenceDiagnosticRow> *active_rtk_vertical_drift_profile_ptr =
    nullptr;
  gtsam::Values adaptive_initial_values = base_initial_values;
  bool adaptive_converged = false;
  const int adaptive_extra_iterations =
    config_.enable_vertical_motion_adaptive_reweighting
      ? std::max(0, config_.vertical_motion_adaptive_outer_iterations)
      : 0;
  const int rtk_drift_extra_iterations =
    config_.enable_rtk_vertical_drift_reference
      ? std::max(0, config_.rtk_vertical_drift_outer_iterations)
      : 0;
  const int adaptive_pass_limit =
    1 + std::max(adaptive_extra_iterations, rtk_drift_extra_iterations);
  int completed_adaptive_pass_count = 0;

  for (int adaptive_pass = 0; adaptive_pass < adaptive_pass_limit; ++adaptive_pass) {
  run_result.run_summary = base_run_summary;
  run_result.run_summary.vertical_motion_adaptive_reweighting_enabled =
    config_.enable_vertical_motion_adaptive_reweighting;
  run_result.run_summary.vertical_motion_adaptive_pass_count =
    config_.enable_vertical_motion_adaptive_reweighting ? adaptive_pass + 1 : 1;
  run_result.run_summary.vertical_motion_adaptive_converged = adaptive_converged;
  run_result.run_summary.rtk_vertical_drift_reference_enabled =
    config_.enable_rtk_vertical_drift_reference;
  run_result.run_summary.rtk_vertical_drift_reference_pass_count =
    config_.enable_rtk_vertical_drift_reference ? adaptive_pass + 1 : 0;
  run_result.run_summary.rtk_vertical_drift_reference_valid_count = 0;
  run_result.run_summary.rtk_vertical_drift_static_range_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_vertical_drift_static_std_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_vertical_drift_white_residual_std_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_vertical_drift_max_abs_correction_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_vertical_drift_first20_mean_correction_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_vertical_drift_first20_max_abs_correction_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_vertical_drift_outage_segmentation_enabled = false;
  run_result.run_summary.rtk_vertical_drift_segment_count = 0;
  run_result.run_summary.rtk_vertical_drift_outage_boundary_count = 0;
  run_result.run_summary.rtk_vertical_drift_cross_outage_lowpass_blocked = false;
  run_result.run_summary.rtk_vertical_drift_causal_reference_enabled = false;
  run_result.run_summary.rtk_vertical_drift_causal_reference_sample_count = 0;
  run_result.run_summary.rtk_vertical_drift_causal_reference_max_full_delta_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_outage_causal_reference_prefix_run_count =
    causal_reference_result.prefix_run_count;
  run_result.run_summary.rtk_outage_causal_reference_boundary_time_s =
    causal_reference_result.boundary_time_s;
  run_result.run_summary.rtk_outage_smoothing_enabled =
    config_.enable_rtk_outage_smoothing;
  run_result.run_summary.rtk_outage_segmented_batch_enabled = false;
  run_result.run_summary.rtk_outage_batch_segment_count = 0;
  run_result.run_summary.rtk_outage_segmented_batch_run_count = 0;
  run_result.run_summary.rtk_outage_segmented_batch_vertical_boundary_jump_allowed =
    config_.rtk_outage_segmented_batch_allow_vertical_boundary_jump;
  run_result.run_summary.rtk_outage_baz_reestimate_enabled =
    config_.enable_rtk_outage_smoothing &&
    config_.enable_rtk_outage_baz_reestimate;
  run_result.run_summary.rtk_outage_window_count = 0;
  run_result.run_summary.rtk_outage_window_with_body_z_jump_count = 0;
  run_result.run_summary.rtk_outage_baz_reestimate_segment_count = 0;
  run_result.run_summary.rtk_outage_baz_reestimate_boundary_break_count = 0;
  run_result.run_summary.rtk_outage_baz_reestimate_prior_factor_count = 0;
  run_result.run_summary.rtk_outage_boundary_constraints_enabled =
    config_.enable_rtk_outage_boundary_constraints;
  run_result.run_summary.rtk_outage_boundary_reference_count = 0;
  run_result.run_summary.rtk_outage_boundary_up_factor_count = 0;
  run_result.run_summary.rtk_outage_boundary_vz_factor_count = 0;
  run_result.run_summary.rtk_outage_boundary_baz_factor_count = 0;
  run_result.run_summary.rtk_outage_preoutage_vertical_fence_enabled =
    config_.enable_rtk_outage_preoutage_vertical_fence && causal_reference_result.valid;
  run_result.run_summary.rtk_outage_preoutage_vertical_fence_factor_count = 0;
  run_result.run_summary.rtk_outage_preoutage_vertical_fence_max_delta_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_outage_preoutage_vertical_fence_max_vz_delta_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_outage_position_ramp_factor_count = 0;
  run_result.run_summary.rtk_outage_velocity_delta_factor_count = 0;
  run_result.run_summary.rtk_outage_velocity_delta_skipped_body_z_jump_count = 0;
  run_result.run_summary.rtk_outage_attitude_hold_factor_count = 0;
  run_result.run_summary.rtk_outage_relative_attitude_factor_count = 0;
  run_result.run_summary.rtk_outage_velocity_delta_3d_factor_count = 0;
  run_result.run_summary.rtk_outage_attitude_hold_max_abs_residual_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_outage_relative_attitude_max_abs_residual_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.rtk_outage_velocity_delta_3d_rms_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.gnss_factor_count = 0;
  run_result.run_summary.gnss_synced_factor_count = 0;
  run_result.run_summary.gnss_interpolated_factor_count = 0;
  run_result.run_summary.gnss_dropped_count = 0;
  run_result.run_summary.gnss_cached_count = 0;
  run_result.run_summary.gnss_vertical_reference_source =
    ToString(config_.gnss_vertical_reference_source);
  run_result.run_summary.gnss_vertical_reference_selected_count = 0;
  run_result.run_summary.gnss_vertical_reference_skipped_count = 0;
  run_result.run_summary.dropped_non_rtkfix_count = 0;
  run_result.run_summary.dropped_no_solution_count = 0;
  run_result.run_summary.dropped_nonfinite_sigma_count = 0;
  run_result.run_summary.dropped_bad_status_count = 0;
  run_result.run_summary.dropped_out_of_imu_coverage_count = 0;
  run_result.run_summary.gnss_nis_mean = std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.gnss_nis_median = std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.gnss_nis_p95 = std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.axis_2sigma_pass_rate = std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.vertical_velocity_delta_factor_count = 0;
  run_result.run_summary.vertical_velocity_delta_static_factor_count = 0;
  run_result.run_summary.vertical_velocity_delta_skipped_disabled_count = 0;
  run_result.run_summary.vertical_velocity_delta_skipped_static_count = 0;
  run_result.run_summary.vertical_velocity_delta_skipped_jump_count = 0;
  run_result.run_summary.vertical_velocity_delta_skipped_gnss_support_count = 0;
  run_result.run_summary.vertical_velocity_delta_skipped_invalid_count = 0;
  run_result.run_summary.vertical_velocity_delta_target_clamped_count = 0;
  run_result.run_summary.vertical_velocity_delta_bias_aware_target_enabled = false;
  run_result.run_summary.vertical_velocity_delta_bias_aware_factor_count = 0;
  run_result.run_summary.vertical_position_velocity_consistency_factor_count = 0;
  run_result.run_summary.vertical_position_velocity_window_consistency_factor_count = 0;
  run_result.run_summary.vertical_position_velocity_consistency_skipped_invalid_count = 0;
  run_result.run_summary.vertical_position_velocity_window_consistency_skipped_invalid_count = 0;
  run_result.run_summary.attitude_reference_factor_count = 0;
  run_result.run_summary.body_z_nhc_velocity_factor_count = 0;
  run_result.run_summary.body_z_nhc_displacement_factor_count = 0;
  run_result.run_summary.body_z_nhc_window_count = 0;
  run_result.run_summary.body_z_nhc_skipped_short_window_count = 0;
  run_result.run_summary.body_z_nhc_skipped_invalid_count = 0;
  run_result.run_summary.body_z_nhc_strict_effective_weighting_enabled =
    config_.enable_body_z_nhc_strict_effective_weighting;
  run_result.run_summary.body_z_nhc_unique_velocity_factor_count = 0;
  run_result.run_summary.body_z_nhc_velocity_duplicate_state_count = 0;
  run_result.run_summary.body_z_nhc_interval_overlap_count = 0;
  run_result.run_summary.body_z_nhc_jump_velocity_factor_count = 0;
  run_result.run_summary.body_z_nhc_global_velocity_factor_count = 0;
  run_result.run_summary.body_z_nhc_jump_displacement_factor_count = 0;
  run_result.run_summary.body_z_nhc_global_displacement_factor_count = 0;
  run_result.run_summary.body_z_nhc_horizontal_leakage_correction_enabled = false;
  run_result.run_summary.body_z_nhc_horizontal_leakage_estimate_valid = false;
  run_result.run_summary.body_z_nhc_horizontal_leakage_sample_count = 0;
  run_result.run_summary.body_z_nhc_horizontal_leakage_skipped_window_count = 0;
  run_result.run_summary.body_z_nhc_horizontal_leakage_skipped_low_speed_count = 0;
  run_result.run_summary.body_z_nhc_horizontal_leakage_skipped_invalid_count = 0;
  run_result.run_summary.body_z_nhc_leakage_corrected_velocity_factor_count = 0;
  run_result.run_summary.body_z_nhc_leakage_corrected_displacement_factor_count = 0;
  run_result.run_summary.body_z_nhc_horizontal_leakage_x_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.body_z_nhc_horizontal_leakage_y_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.body_z_nhc_corrected_max_velocity_residual_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.body_z_nhc_corrected_max_abs_displacement_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage1_outage_body_y_envelope_enabled =
    config_.enable_stage1_outage_body_y_envelope &&
    stage1_body_y_reference_ != nullptr;
  run_result.run_summary.stage1_outage_body_y_envelope_count = 0;
  run_result.run_summary.stage1_outage_body_y_envelope_valid_count = 0;
  run_result.run_summary.stage1_outage_body_y_velocity_factor_count = 0;
  run_result.run_summary.stage1_outage_body_y_mean_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage1_outage_body_y_rmse_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage1_outage_body_y_deadband_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_velocity_optimization_enabled =
    active_stage2_reference != nullptr && config_.enable_stage2_velocity_optimization;
  run_result.run_summary.stage2_attitude_hold_factor_count = 0;
  run_result.run_summary.stage2_horizontal_position_hold_factor_count = 0;
  run_result.run_summary.stage2_horizontal_velocity_hold_factor_count = 0;
  run_result.run_summary.stage2_vehicle_y_nhc_velocity_factor_count = 0;
  run_result.run_summary.stage2_vehicle_y_nhc_displacement_factor_count = 0;
  run_result.run_summary.stage2_vehicle_z_nhc_velocity_factor_count = 0;
  run_result.run_summary.stage2_vehicle_z_nhc_displacement_factor_count = 0;
  run_result.run_summary.stage2_vehicle_nhc_window_count = 0;
  run_result.run_summary.stage2_vehicle_nhc_skipped_short_window_count = 0;
  run_result.run_summary.stage2_vehicle_nhc_skipped_invalid_count = 0;
  run_result.run_summary.stage2_vehicle_nhc_unique_velocity_factor_count = 0;
  run_result.run_summary.stage2_vehicle_nhc_velocity_duplicate_state_count = 0;
  run_result.run_summary.stage2_vehicle_nhc_interval_overlap_count = 0;
  run_result.run_summary.stage2_mount_initial_k_zx_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_mount_initial_k_zy_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_mount_initial_k_yx_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_mount_k_zx_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_mount_k_zy_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_mount_k_yx_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_max_abs_yaw_delta_rad =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage2_lowfreq_vertical_reference_optimization_enabled =
    config_.enable_stage2_lowfreq_vertical_reference_optimization;
  run_result.run_summary.stage2_lowfreq_vertical_reference_source =
    ToString(config_.stage2_lowfreq_vertical_reference_source);
  run_result.run_summary.stage2_lowfreq_vertical_reference_cutoff_hz =
    config_.stage2_lowfreq_vertical_reference_cutoff_hz;
  run_result.run_summary.stage3_vertical_reference_optimization_enabled =
    active_stage3_vertical_reference != nullptr;
  run_result.run_summary.stage3_vertical_reference_smoothing_method =
    active_stage3_vertical_reference != nullptr
      ? ToString(config_.stage3_vertical_reference_smoothing_method)
      : "disabled";
  run_result.run_summary.stage3_vertical_reference_lowpass_cutoff_hz =
    active_stage3_vertical_reference != nullptr
      ? config_.stage3_vertical_reference_lowpass_cutoff_hz
      : std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_reference_spline_knot_spacing_m =
    active_stage3_vertical_reference != nullptr
      ? config_.stage3_vertical_reference_spline_knot_spacing_m
      : std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_anchor_sigma_m =
    active_stage3_vertical_reference != nullptr
      ? config_.stage3_vertical_anchor_sigma_m
      : std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_reference_constraint_mode =
    active_stage3_vertical_reference != nullptr
      ? ToString(config_.stage3_vertical_reference_constraint_mode)
      : "disabled";
  run_result.run_summary.stage3_vertical_reference_factor_count = 0;
  run_result.run_summary.stage3_vertical_reference_center_pull_factor_count = 0;
  run_result.run_summary.stage3_vertical_reference_total_factor_count = 0;
  run_result.run_summary.stage3_vertical_reference_skipped_count = 0;
  run_result.run_summary.stage3_vertical_reference_mean_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_reference_max_abs_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_reference_max_abs_lowpass_delta_m =
    std::numeric_limits<double>::quiet_NaN();
  const bool stage3_envelope_summary_enabled =
    active_stage3_vertical_reference != nullptr &&
    config_.stage3_vertical_reference_constraint_mode ==
      Stage3VerticalReferenceConstraintMode::kEnvelope;
  run_result.run_summary.stage3_vertical_envelope_half_width_m =
    stage3_envelope_summary_enabled
      ? config_.stage3_vertical_envelope_half_width_m
      : std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_envelope_sigma_m =
    stage3_envelope_summary_enabled
      ? config_.stage3_vertical_envelope_sigma_m
      : std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_envelope_center_sigma_m =
    stage3_envelope_summary_enabled && config_.enable_stage3_vertical_envelope_center_pull
      ? config_.stage3_vertical_envelope_center_sigma_m
      : std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_envelope_center_deadband_m =
    stage3_envelope_summary_enabled && config_.enable_stage3_vertical_envelope_center_pull
      ? config_.stage3_vertical_envelope_center_deadband_m
      : std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_envelope_outside_gate_count = 0;
  run_result.run_summary.stage3_vertical_envelope_max_abs_overflow_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_vertical_envelope_max_abs_center_pull_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_velocity_smoothness_regularizer_enabled =
    config_.enable_stage3_jump_velocity_smoothness_regularizer;
  run_result.run_summary.stage3_jump_velocity_smoothness_factor_count = 0;
  run_result.run_summary.stage3_jump_velocity_smoothness_skipped_count = 0;
  run_result.run_summary.stage3_jump_velocity_smoothness_max_abs_residual_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_height_highfreq_deadband_enabled =
    config_.enable_stage3_jump_height_highfreq_deadband;
  run_result.run_summary.stage3_jump_height_highfreq_deadband_factor_count = 0;
  run_result.run_summary.stage3_jump_height_highfreq_deadband_skipped_count = 0;
  run_result.run_summary.stage3_jump_height_highfreq_deadband_max_abs_raw_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_height_highfreq_deadband_max_abs_overflow_residual_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_adaptive_context_envelope_enabled =
    config_.enable_stage3_jump_adaptive_context_envelope;
  run_result.run_summary.stage3_jump_context_envelope_profile_count = 0;
  run_result.run_summary.stage3_jump_context_envelope_fallback_count = 0;
  run_result.run_summary.stage3_jump_velocity_context_envelope_factor_count = 0;
  run_result.run_summary.stage3_jump_velocity_context_envelope_skipped_count = 0;
  run_result.run_summary.stage3_jump_velocity_context_envelope_max_abs_overflow_residual_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_context_velocity_deadband_min_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_context_velocity_deadband_max_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_context_velocity_delta_deadband_min_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_context_velocity_delta_deadband_max_mps =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_context_height_deadband_min_m =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.stage3_jump_context_height_deadband_max_m =
    std::numeric_limits<double>::quiet_NaN();
  populate_initial_dynamic_static_summary();
  run_result.run_summary.vertical_jump_combined_imu_factor_count = 0;
  run_result.run_summary.vertical_jump_masked_imu_factor_count = 0;
  run_result.run_summary.vertical_jump_impulse_factor_count = 0;
  run_result.run_summary.vertical_jump_impulse_prior_factor_count = 0;
  run_result.run_summary.vertical_jump_impulse_replaced_imu_factor_count = 0;
  run_result.run_summary.vertical_jump_impulse_skipped_count = 0;
  run_result.run_summary.vertical_jump_bias_velocity_factor_count = 0;
  run_result.run_summary.vertical_jump_bias_prior_factor_count = 0;
  run_result.run_summary.vertical_jump_bias_replaced_imu_factor_count = 0;
  run_result.run_summary.vertical_jump_bias_position_velocity_factor_count = 0;
  run_result.run_summary.vertical_jump_bias_skipped_count = 0;
  run_result.run_summary.vertical_jump_bias_segment_count = 0;
  run_result.run_summary.vertical_jump_bias_highfreq_inflated_factor_count = 0;
  run_result.run_summary.body_z_bias_reestimate_segment_count = 0;
  run_result.run_summary.body_z_bias_reestimate_boundary_break_count = 0;
  run_result.run_summary.body_z_bias_reestimate_prior_factor_count = 0;
  run_result.run_summary.body_z_bias_reestimate_initialized_state_count = 0;
  run_result.run_summary.body_z_bias_reestimate_gm_skipped_count = 0;
  run_result.run_summary.vertical_jump_spectral_bias_relaxation_enabled = false;
  run_result.run_summary.vertical_jump_spectral_relaxed_segment_count = 0;
  run_result.run_summary.vertical_jump_spectral_max_response_ratio =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.vertical_jump_spectral_max_effective_prior_sigma_mps2 =
    std::numeric_limits<double>::quiet_NaN();
  run_result.run_summary.vertical_jump_velocity_ramp_factor_count = 0;
  run_result.run_summary.vertical_jump_position_ramp_factor_count = 0;
  run_result.run_summary.vertical_jump_velocity_height_slope_factor_count = 0;
  run_result.run_summary.vertical_jump_velocity_continuity_factor_count = 0;
  run_result.run_summary.vertical_jump_position_velocity_consistency_factor_count = 0;
  run_result.run_summary.vertical_jump_continuity_skipped_count = 0;
  run_result.run_summary.vertical_jump_velocity_ramp_skipped_count = 0;
  run_result.trajectory = base_dynamic_trajectory;
  run_result.gnss_factor_records.clear();
  run_result.gnss_consistency_records.clear();
  run_result.vertical_envelope_diagnostics.clear();
  run_result.rtk_vertical_drift_reference_diagnostics =
    active_rtk_vertical_drift_profile_ptr != nullptr
      ? *active_rtk_vertical_drift_profile_ptr
      : std::vector<RtkVerticalDriftReferenceDiagnosticRow>{};
  run_result.rtk_outage_causal_nav_reference_diagnostics =
    causal_reference_result.nav_reference_rows;
  run_result.rtk_outage_boundary_diagnostics.clear();
  run_result.rtk_velocity_diagnostics.clear();
  run_result.rtk_outage_attitude_hold_diagnostics.clear();
  run_result.rtk_outage_velocity_delta_3d_diagnostics.clear();
  run_result.vertical_velocity_delta_diagnostics.clear();
  run_result.vertical_motion_adaptive_reweighting_diagnostics.clear();
  run_result.vertical_position_velocity_consistency_diagnostics.clear();
  run_result.attitude_reference_diagnostics.clear();
  run_result.relative_yaw_reference_diagnostics.clear();
  run_result.body_z_nhc_horizontal_leakage_diagnostics.clear();
  run_result.body_z_nhc_diagnostics.clear();
  run_result.stage1_outage_body_y_envelopes.clear();
  run_result.stage1_outage_body_y_state_diagnostics.clear();
  run_result.stage2_mount_leakage_diagnostics.clear();
  run_result.stage2_vehicle_nhc_state_diagnostics.clear();
  run_result.stage3_vertical_reference_diagnostics.clear();
  run_result.stage3_jump_regularizer_diagnostics.clear();
  run_result.stage3_jump_context_envelope_profiles.clear();
  run_result.vertical_jump_masked_imu_diagnostics.clear();
  run_result.vertical_jump_impulse_diagnostics.clear();
  run_result.vertical_jump_bias_diagnostics.clear();
  run_result.vertical_jump_velocity_ramp_diagnostics.clear();
  run_result.vertical_jump_continuity_diagnostics.clear();
  run_result.vertical_state_corrections.clear();
  run_result.rtk_outage_windows = planned_rtk_outage_windows;

  gtsam::NonlinearFactorGraph graph_with_gnss = base_graph;
  add_vertical_acc_bias_gm_constraints(
    graph_with_gnss,
    run_result.run_summary,
    &run_result.body_z_bias_reestimate_segments,
    active_stability_profile_ptr);
  gtsam::Values optimization_initial_values = adaptive_initial_values;
  RtkOutageInitialValueSmoothRequest rtk_outage_initial_request;
  rtk_outage_initial_request.state_timestamps = &state_timestamps;
  rtk_outage_initial_request.gnss_samples = &dataset.gnss_samples;
  rtk_outage_initial_request.propagation_records = &vertical_velocity_delta_records;
  rtk_outage_initial_request.initial_values = &optimization_initial_values;
  rtk_outage_initial_request.outage_windows = &run_result.rtk_outage_windows;
  RtkOutageInitialValueSmoother(std::move(rtk_outage_initial_request)).Apply();

  if (!run_result.body_z_bias_reestimate_segments.empty()) {
    BodyZBiasReestimateConstraintBuildRequest bias_reestimate_request;
    bias_reestimate_request.config = &config_;
    bias_reestimate_request.state_timestamps = &state_timestamps;
    bias_reestimate_request.segments = &run_result.body_z_bias_reestimate_segments;
    bias_reestimate_request.imu_intervals = &vertical_jump_imu_interval_records;
    bias_reestimate_request.graph = &graph_with_gnss;
    bias_reestimate_request.initial_values = &optimization_initial_values;
    bias_reestimate_request.run_summary = &run_result.run_summary;
    BodyZBiasReestimateConstraintBuilder(std::move(bias_reestimate_request)).Apply();
  }

  if (active_stage2_reference != nullptr &&
      !active_stage2_reference->boundary_references.empty()) {
    RtkOutageBoundaryConstraintBuildRequest boundary_request;
    boundary_request.config = &config_;
    boundary_request.state_timestamps = &state_timestamps;
    boundary_request.boundary_references = &active_stage2_reference->boundary_references;
    boundary_request.graph = &graph_with_gnss;
    boundary_request.run_summary = &run_result.run_summary;
    boundary_request.diagnostics = &run_result.rtk_outage_boundary_diagnostics;
    RtkOutageBoundaryConstraintBuilder(std::move(boundary_request)).Build();
  }

  if (causal_reference_result.valid) {
    RtkOutagePreOutageVerticalFenceBuildRequest fence_request;
    fence_request.config = &config_;
    fence_request.state_references = &causal_reference_result.state_reference_rows;
    fence_request.graph = &graph_with_gnss;
    fence_request.run_summary = &run_result.run_summary;
    RtkOutagePreOutageVerticalFenceBuilder(std::move(fence_request)).Build();
  }

  if (config_.enable_vertical_jump_impulse) {
    VerticalJumpImpulseConstraintBuildRequest vertical_jump_impulse_request;
    vertical_jump_impulse_request.config = &config_;
    vertical_jump_impulse_request.state_timestamps = &state_timestamps;
    vertical_jump_impulse_request.jump_windows = &run_result.body_z_seed_jump_windows;
    vertical_jump_impulse_request.imu_intervals = &vertical_jump_imu_interval_records;
    vertical_jump_impulse_request.propagation_records = &vertical_velocity_delta_records;
    vertical_jump_impulse_request.graph = &graph_with_gnss;
    vertical_jump_impulse_request.initial_values = &optimization_initial_values;
    vertical_jump_impulse_request.run_summary = &run_result.run_summary;
    vertical_jump_impulse_request.diagnostics = &run_result.vertical_jump_impulse_diagnostics;
    VerticalJumpImpulseConstraintBuilder(std::move(vertical_jump_impulse_request)).Build();
  }
  if (config_.enable_vertical_jump_bias) {
    VerticalJumpBiasConstraintBuildRequest vertical_jump_bias_request;
    vertical_jump_bias_request.config = &config_;
    vertical_jump_bias_request.state_timestamps = &state_timestamps;
    vertical_jump_bias_request.jump_windows = &run_result.body_z_seed_jump_windows;
    vertical_jump_bias_request.body_z_diagnostics = &run_result.seed_body_z_acc_diagnostics;
    vertical_jump_bias_request.imu_intervals = &vertical_jump_imu_interval_records;
    vertical_jump_bias_request.propagation_records = &vertical_velocity_delta_records;
    vertical_jump_bias_request.graph = &graph_with_gnss;
    vertical_jump_bias_request.initial_values = &optimization_initial_values;
    vertical_jump_bias_request.run_summary = &run_result.run_summary;
    vertical_jump_bias_request.diagnostics = &run_result.vertical_jump_bias_diagnostics;
    VerticalJumpBiasConstraintBuilder(std::move(vertical_jump_bias_request)).Build();
  }
  if (config_.enable_vertical_jump_masked_imu) {
    VerticalJumpImuMaskRequest vertical_jump_mask_request;
    vertical_jump_mask_request.config = &config_;
    vertical_jump_mask_request.intervals = &vertical_jump_imu_interval_records;
    vertical_jump_mask_request.jump_windows = &run_result.body_z_seed_jump_windows;
    vertical_jump_mask_request.graph = &graph_with_gnss;
    vertical_jump_mask_request.run_summary = &run_result.run_summary;
    vertical_jump_mask_request.diagnostics = &run_result.vertical_jump_masked_imu_diagnostics;
    VerticalJumpImuMasker(std::move(vertical_jump_mask_request)).Apply();
  }

  GnssFactorBuildRequest gnss_request;
  gnss_request.config = &config_;
  gnss_request.gnss_samples = &dataset.gnss_samples;
  gnss_request.navigation_start_index = navigation_start_index;
  gnss_request.graph = &graph_with_gnss;
  gnss_request.trajectory = &run_result.trajectory;
  gnss_request.run_summary = &run_result.run_summary;
  gnss_request.factor_records = &run_result.gnss_factor_records;
  gnss_request.consistency_records = &run_result.gnss_consistency_records;
  gnss_request.vertical_envelope_diagnostics = &run_result.vertical_envelope_diagnostics;
  gnss_request.rtk_vertical_drift_reference_profile = active_rtk_vertical_drift_profile_ptr;
  gnss_request.stage2_lowpass_vertical_reference =
    stage2_lowpass_vertical_reference_.get();
  gnss_request.collect_consistency_records = collect_gnss_consistency;
  gnss_request.dynamic_start_time_s = dynamic_start_time_s;
  gnss_request.disable_vertical_factors = active_stage3_vertical_reference != nullptr;
  gnss_request.should_use_sample = [&](const GnssSolutionSample &sample) {
    return ShouldUseGnssFactor(sample, run_result.run_summary);
  };
  gnss_request.is_within_imu_coverage = [&](const double corrected_time_s) {
    return IsWithinImuCoverage(dataset.imu_samples, corrected_time_s);
  };
  gnss_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
    return CorrectedGnssTime(sample);
  };
  gnss_request.clamped_sigma_m = [&](const GnssSolutionSample &sample) {
    return ClampGnssSigma(sample);
  };
  gnss_request.find_state_for_time_s = [&](const double corrected_time_s) {
    return FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
  };
  gnss_request.trajectory_row_index_for_state = graph_state_to_trajectory_row;
  GnssFactorBuilder(std::move(gnss_request)).Build();

  if (active_stage3_vertical_reference != nullptr) {
    Stage3VerticalReferenceConstraintBuildRequest stage3_request;
    stage3_request.config = &config_;
    stage3_request.reference = active_stage3_vertical_reference;
    stage3_request.state_timestamps = &state_timestamps;
    stage3_request.dynamic_start_index = graph_timeline.dynamic_start_index;
    stage3_request.graph = &graph_with_gnss;
    stage3_request.run_summary = &run_result.run_summary;
    stage3_request.diagnostics = &run_result.stage3_vertical_reference_diagnostics;
    Stage3VerticalReferenceConstraintBuilder(std::move(stage3_request)).Build();

    Stage3JumpRegularizerConstraintBuildRequest jump_regularizer_request;
    jump_regularizer_request.config = &config_;
    jump_regularizer_request.reference = active_stage3_vertical_reference;
    jump_regularizer_request.state_timestamps = &state_timestamps;
    jump_regularizer_request.jump_windows = &run_result.body_z_seed_jump_windows;
    jump_regularizer_request.initial_values = &optimization_initial_values;
    jump_regularizer_request.dynamic_start_index = graph_timeline.dynamic_start_index;
    jump_regularizer_request.graph = &graph_with_gnss;
    jump_regularizer_request.run_summary = &run_result.run_summary;
    jump_regularizer_request.diagnostics =
      &run_result.stage3_jump_regularizer_diagnostics;
    jump_regularizer_request.context_profiles =
      &run_result.stage3_jump_context_envelope_profiles;
    Stage3JumpRegularizerConstraintBuilder(
      std::move(jump_regularizer_request)).Build();
  }

  if (!run_result.initial_dynamic_static_windows.empty()) {
    InitialDynamicStaticConstraintBuildRequest initial_dynamic_static_constraint_request;
    initial_dynamic_static_constraint_request.config = &config_;
    initial_dynamic_static_constraint_request.state_timestamps = &state_timestamps;
    initial_dynamic_static_constraint_request.dynamic_start_index =
      graph_timeline.dynamic_start_index;
    initial_dynamic_static_constraint_request.graph = &graph_with_gnss;
    initial_dynamic_static_constraint_request.run_summary = &run_result.run_summary;
    initial_dynamic_static_constraint_request.windows =
      &run_result.initial_dynamic_static_windows;
    InitialDynamicStaticConstraintBuilder(
      std::move(initial_dynamic_static_constraint_request)).Build();
  }

  RtkVelocityConstraintBuildRequest rtk_velocity_request;
  rtk_velocity_request.config = &config_;
  rtk_velocity_request.gnss_samples = &dataset.gnss_samples;
  rtk_velocity_request.state_timestamps = &state_timestamps;
  rtk_velocity_request.graph = &graph_with_gnss;
  rtk_velocity_request.run_summary = &run_result.run_summary;
  rtk_velocity_request.diagnostics = &run_result.rtk_velocity_diagnostics;
  rtk_velocity_request.dynamic_start_index = graph_timeline.dynamic_start_index;
  rtk_velocity_request.should_use_sample = [&](const GnssSolutionSample &sample) {
    return sample.has_enu_position && PassesGnssQualityFilters(sample);
  };
  rtk_velocity_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
    return CorrectedGnssTime(sample);
  };
  rtk_velocity_request.find_state_for_time_s = [&](const double corrected_time_s) {
    return FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
  };
  RtkVelocityConstraintBuilder(std::move(rtk_velocity_request)).Build();

  std::optional<double> vertical_velocity_delta_support_end_time_s;
  for (const auto &record : run_result.gnss_factor_records) {
    if (!record.factor_used || !std::isfinite(record.corrected_time_s)) {
      continue;
    }
    if (!vertical_velocity_delta_support_end_time_s.has_value()) {
      vertical_velocity_delta_support_end_time_s = record.corrected_time_s;
      continue;
    }
    vertical_velocity_delta_support_end_time_s =
      std::max(*vertical_velocity_delta_support_end_time_s, record.corrected_time_s);
  }

  VerticalMotionConstraintBuildRequest vertical_motion_request;
  vertical_motion_request.config = &config_;
  vertical_motion_request.propagation_records = &vertical_velocity_delta_records;
  vertical_motion_request.jump_windows = &run_result.body_z_seed_jump_windows;
  vertical_motion_request.rtk_outage_windows = &run_result.rtk_outage_windows;
  vertical_motion_request.bias_reestimate_segments = &run_result.body_z_bias_reestimate_segments;
  vertical_motion_request.stability_profile = active_stability_profile_ptr;
  vertical_motion_request.gnss_support_end_time_s = vertical_velocity_delta_support_end_time_s;
  vertical_motion_request.dynamic_start_index = graph_timeline.dynamic_start_index;
  vertical_motion_request.outer_pass = adaptive_pass;
  vertical_motion_request.graph = &graph_with_gnss;
  vertical_motion_request.run_summary = &run_result.run_summary;
  vertical_motion_request.diagnostics = &run_result.vertical_velocity_delta_diagnostics;
  VerticalMotionConstraintBuilder(std::move(vertical_motion_request)).Build();

  VerticalPositionVelocityConsistencyBuildRequest vertical_position_velocity_request;
  vertical_position_velocity_request.config = &config_;
  vertical_position_velocity_request.state_timestamps = &state_timestamps;
  vertical_position_velocity_request.jump_windows = &run_result.body_z_seed_jump_windows;
  vertical_position_velocity_request.initial_values = &optimization_initial_values;
  vertical_position_velocity_request.dynamic_start_index = graph_timeline.dynamic_start_index;
  vertical_position_velocity_request.graph = &graph_with_gnss;
  vertical_position_velocity_request.run_summary = &run_result.run_summary;
  vertical_position_velocity_request.diagnostics =
    &run_result.vertical_position_velocity_consistency_diagnostics;
  VerticalPositionVelocityConsistencyConstraintBuilder(
    std::move(vertical_position_velocity_request)).Build();

  RtkOutageRecoveryConstraintBuildRequest rtk_outage_recovery_request;
  rtk_outage_recovery_request.config = &config_;
  rtk_outage_recovery_request.state_timestamps = &state_timestamps;
  rtk_outage_recovery_request.outage_windows = &run_result.rtk_outage_windows;
  rtk_outage_recovery_request.reference_states = &reference_node_states;
  rtk_outage_recovery_request.velocity_delta_records = &velocity_delta_records;
  rtk_outage_recovery_request.graph = &graph_with_gnss;
  rtk_outage_recovery_request.run_summary = &run_result.run_summary;
  rtk_outage_recovery_request.attitude_diagnostics =
    &run_result.rtk_outage_attitude_hold_diagnostics;
  rtk_outage_recovery_request.velocity_diagnostics =
    &run_result.rtk_outage_velocity_delta_3d_diagnostics;
  RtkOutageRecoveryConstraintBuilder(std::move(rtk_outage_recovery_request)).Build();

  RtkOutageSmoothingConstraintBuildRequest rtk_outage_request;
  rtk_outage_request.config = &config_;
  rtk_outage_request.state_timestamps = &state_timestamps;
  rtk_outage_request.body_z_jump_windows = &run_result.body_z_seed_jump_windows;
  rtk_outage_request.propagation_records = &vertical_velocity_delta_records;
  rtk_outage_request.graph = &graph_with_gnss;
  rtk_outage_request.run_summary = &run_result.run_summary;
  rtk_outage_request.outage_windows = &run_result.rtk_outage_windows;
  RtkOutageSmoothingConstraintBuilder(std::move(rtk_outage_request)).Build();

  if (stage1_body_y_reference_ != nullptr) {
    Stage1OutageBodyYEnvelopeConstraintBuildRequest body_y_request;
    body_y_request.config = &config_;
    body_y_request.state_timestamps = &state_timestamps;
    body_y_request.reference = stage1_body_y_reference_.get();
    body_y_request.dynamic_start_index = graph_timeline.dynamic_start_index;
    body_y_request.graph = &graph_with_gnss;
    body_y_request.run_summary = &run_result.run_summary;
    body_y_request.envelopes = &run_result.stage1_outage_body_y_envelopes;
    body_y_request.state_diagnostics =
      &run_result.stage1_outage_body_y_state_diagnostics;
    Stage1OutageBodyYEnvelopeConstraintBuilder(std::move(body_y_request)).Build();
  }

  BodyZNHCConstraintBuildRequest body_z_nhc_request;
  body_z_nhc_request.config = &config_;
  body_z_nhc_request.state_timestamps = &state_timestamps;
  body_z_nhc_request.jump_windows = &nhc_constraint_windows;
  body_z_nhc_request.initial_values = &optimization_initial_values;
  body_z_nhc_request.reference_states = &run_result.attitude_reference_states;
  body_z_nhc_request.dynamic_start_index = graph_timeline.dynamic_start_index;
  body_z_nhc_request.graph = &graph_with_gnss;
  body_z_nhc_request.run_summary = &run_result.run_summary;
  body_z_nhc_request.diagnostics = &run_result.body_z_nhc_diagnostics;
  body_z_nhc_request.horizontal_leakage_diagnostics =
    &run_result.body_z_nhc_horizontal_leakage_diagnostics;
  BodyZNHCConstraintBuilder(std::move(body_z_nhc_request)).Build();

  if (active_stage2_reference != nullptr) {
    Stage2VehicleNHCConstraintBuildRequest stage2_vehicle_request;
    stage2_vehicle_request.config = &config_;
    stage2_vehicle_request.state_timestamps = &state_timestamps;
    stage2_vehicle_request.jump_windows = &nhc_constraint_windows;
    stage2_vehicle_request.initial_values = &optimization_initial_values;
    stage2_vehicle_request.reference_states = &stage2_fixed_reference_states;
    stage2_vehicle_request.dynamic_start_index = graph_timeline.dynamic_start_index;
    stage2_vehicle_request.mount_leakage_key = stage2_mount_leakage_key;
    stage2_vehicle_request.graph = &graph_with_gnss;
    stage2_vehicle_request.run_summary = &run_result.run_summary;
    stage2_vehicle_request.mount_diagnostics =
      &run_result.stage2_mount_leakage_diagnostics;
    stage2_vehicle_request.state_diagnostics =
      &run_result.stage2_vehicle_nhc_state_diagnostics;
    Stage2VehicleNHCConstraintBuilder(std::move(stage2_vehicle_request)).Build();

    LateStaticVerticalConstraintBuildRequest late_static_vertical_request;
    late_static_vertical_request.config = &config_;
    late_static_vertical_request.state_timestamps = &state_timestamps;
    late_static_vertical_request.dynamic_start_index = graph_timeline.dynamic_start_index;
    late_static_vertical_request.graph = &graph_with_gnss;
    late_static_vertical_request.run_summary = &run_result.run_summary;
    late_static_vertical_request.windows = &run_result.late_static_windows;
    LateStaticVerticalConstraintBuilder(std::move(late_static_vertical_request)).Build();
  }

  AttitudeReferenceConstraintBuildRequest attitude_reference_request;
  attitude_reference_request.config = &config_;
  attitude_reference_request.state_timestamps = &state_timestamps;
  attitude_reference_request.reference_states = &run_result.attitude_reference_states;
  attitude_reference_request.relative_yaw_reference_states = &reference_node_states;
  attitude_reference_request.dynamic_start_index = graph_timeline.dynamic_start_index;
  attitude_reference_request.graph = &graph_with_gnss;
  attitude_reference_request.run_summary = &run_result.run_summary;
  attitude_reference_request.diagnostics = &run_result.attitude_reference_diagnostics;
  attitude_reference_request.relative_yaw_diagnostics =
    &run_result.relative_yaw_reference_diagnostics;
  AttitudeReferenceConstraintBuilder(std::move(attitude_reference_request)).Build();

  if (active_stage2_reference != nullptr) {
    Stage2AttitudeHoldBuildRequest attitude_hold_request;
    attitude_hold_request.config = &config_;
    attitude_hold_request.state_timestamps = &state_timestamps;
    attitude_hold_request.reference_states = &stage2_fixed_reference_states;
    attitude_hold_request.graph = &graph_with_gnss;
    attitude_hold_request.run_summary = &run_result.run_summary;
    Stage2AttitudeHoldBuilder(std::move(attitude_hold_request)).Build();

    Stage2HorizontalHoldBuildRequest horizontal_hold_request;
    horizontal_hold_request.config = &config_;
    horizontal_hold_request.state_timestamps = &state_timestamps;
    horizontal_hold_request.reference_states = &stage2_fixed_reference_states;
    horizontal_hold_request.graph = &graph_with_gnss;
    horizontal_hold_request.run_summary = &run_result.run_summary;
    Stage2HorizontalHoldBuilder(std::move(horizontal_hold_request)).Build();
  }

  if (config_.enable_vertical_jump_velocity_ramp_smoothing ||
      config_.enable_vertical_jump_position_ramp_smoothing ||
      config_.enable_vertical_jump_velocity_continuity ||
      config_.enable_vertical_jump_velocity_context_mean ||
      config_.enable_vertical_jump_context_mean_continuity ||
      config_.enable_vertical_jump_position_velocity_consistency ||
      config_.enable_vertical_jump_velocity_height_slope_constraint) {
    VerticalJumpShapeConstraintBuildRequest vertical_ramp_request;
    vertical_ramp_request.config = &config_;
    vertical_ramp_request.state_timestamps = &state_timestamps;
    vertical_ramp_request.jump_windows = &run_result.body_z_seed_jump_windows;
    vertical_ramp_request.graph = &graph_with_gnss;
    vertical_ramp_request.run_summary = &run_result.run_summary;
    vertical_ramp_request.diagnostics = &run_result.vertical_jump_velocity_ramp_diagnostics;
    vertical_ramp_request.continuity_diagnostics = &run_result.vertical_jump_continuity_diagnostics;
    VerticalJumpShapeConstraintBuilder(std::move(vertical_ramp_request)).Build();
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(
    graph_with_gnss,
    optimization_initial_values,
    optimizer_params);
  run_result.run_summary.initial_error = optimizer.error();
  const gtsam::Values optimized_values = optimizer.optimize();
  run_result.run_summary.final_error = graph_with_gnss.error(optimized_values);
  RtkVelocityConstraintBuilder::PopulateDiagnostics(
    optimized_values,
    run_result.rtk_velocity_diagnostics,
    run_result.run_summary);
  PopulateRtkOutageRecoveryDiagnostics(
    optimized_values,
    run_result.rtk_outage_attitude_hold_diagnostics,
    run_result.rtk_outage_velocity_delta_3d_diagnostics,
    run_result.run_summary);
  PopulateRtkOutageBoundaryDiagnostics(
    optimized_values,
    run_result.rtk_outage_boundary_diagnostics);
  PopulateStage3VerticalReferenceDiagnostics(
    optimized_values,
    run_result.stage3_vertical_reference_diagnostics,
    run_result.run_summary);
  PopulateStage3JumpRegularizerDiagnostics(
    optimized_values,
    run_result.stage3_jump_regularizer_diagnostics,
    run_result.run_summary);
  PopulateInitialDynamicStaticDiagnostics(
    optimized_values,
    run_result.initial_dynamic_static_windows,
    run_result.run_summary);
  PopulateStage1OutageBodyYEnvelopeDiagnostics(
    optimized_values,
    run_result.stage1_outage_body_y_state_diagnostics,
    run_result.run_summary);
  PopulateLateStaticVerticalDiagnostics(
    optimized_values,
    run_result.late_static_windows,
    run_result.run_summary);
  completed_adaptive_pass_count = adaptive_pass + 1;

  bool finalize_adaptive_pass = adaptive_pass + 1 >= adaptive_pass_limit;
  bool continue_with_reweighted_graph = false;
  if (!finalize_adaptive_pass && config_.enable_rtk_vertical_drift_reference) {
    RtkVerticalDriftReferenceEstimateRequest drift_request;
    drift_request.config = &config_;
    drift_request.gnss_samples = &dataset.gnss_samples;
    drift_request.rtk_outage_windows = &planned_rtk_outage_windows;
    drift_request.late_static_windows = &run_result.late_static_windows;
    drift_request.causal_nav_reference_profile =
      causal_reference_result.valid ? &causal_reference_result.nav_reference_rows : nullptr;
    drift_request.optimized_values = &optimized_values;
    drift_request.causal_nav_reference_end_time_s = causal_reference_result.boundary_time_s;
    drift_request.alignment_start_time_s = alignment_start_time_s;
    drift_request.alignment_end_time_s = alignment_end_time_s;
    drift_request.static_reference_up_m = initial_static_rtk_height_reference.reference_up_m;
    drift_request.dynamic_start_time_s = dynamic_start_time_s;
    drift_request.pass_index = adaptive_pass + 1;
    drift_request.should_use_sample = [&](const GnssSolutionSample &sample) {
      return sample.has_enu_position && PassesGnssQualityFilters(sample);
    };
    drift_request.is_within_imu_coverage = [&](const double corrected_time_s) {
      return IsWithinImuCoverage(dataset.imu_samples, corrected_time_s);
    };
    drift_request.corrected_time_s = [&](const GnssSolutionSample &sample) {
      return CorrectedGnssTime(sample);
    };
    drift_request.clamped_sigma_m = [&](const GnssSolutionSample &sample) {
      return ClampGnssSigma(sample);
    };
    drift_request.find_state_for_time_s = [&](const double corrected_time_s) {
      return FindStateForMeasurement(state_timestamp_map, corrected_time_s, config_);
    };
    RtkVerticalDriftReferenceEstimateResult next_drift_profile =
      RtkVerticalDriftReferenceEstimator(std::move(drift_request)).Estimate(
        active_rtk_vertical_drift_profile_ptr);
    if (active_rtk_vertical_drift_profile_ptr != nullptr &&
        next_drift_profile.max_abs_profile_delta_m <=
          config_.rtk_vertical_drift_convergence_threshold_m) {
      // Current graph already used a converged fixed drift profile.
    } else {
      active_rtk_vertical_drift_profile = std::move(next_drift_profile.profile);
      active_rtk_vertical_drift_profile_ptr = &active_rtk_vertical_drift_profile;
      run_result.rtk_vertical_drift_reference_diagnostics = active_rtk_vertical_drift_profile;
      continue_with_reweighted_graph = true;
    }
  }
  if (!finalize_adaptive_pass && config_.enable_vertical_motion_adaptive_reweighting) {
    VerticalMotionStabilityEstimateRequest stability_request;
    stability_request.config = &config_;
    stability_request.state_timestamps = &state_timestamps;
    stability_request.propagation_records = &vertical_velocity_delta_records;
    stability_request.bias_gm_records = &vertical_acc_bias_gm_records;
    stability_request.imu_samples = &dataset.imu_samples;
    stability_request.jump_windows = &run_result.body_z_seed_jump_windows;
    stability_request.optimized_values = &optimized_values;
    stability_request.outer_pass = adaptive_pass + 1;
    VerticalMotionStabilityProfile next_profile =
      VerticalMotionStabilityEstimator(std::move(stability_request)).Estimate();
    if (active_stability_profile_ptr != nullptr &&
        MaxMotionScoreDelta(active_stability_profile, next_profile) <=
          config_.vertical_motion_adaptive_convergence_score_epsilon) {
      adaptive_converged = true;
    } else {
      adaptive_reweighting_diagnostics.insert(
        adaptive_reweighting_diagnostics.end(),
        next_profile.begin(),
        next_profile.end());
      active_stability_profile = std::move(next_profile);
      active_stability_profile_ptr = &active_stability_profile;
      continue_with_reweighted_graph = true;
    }
  }
  if (continue_with_reweighted_graph) {
    adaptive_initial_values = optimized_values;
    continue;
  }
  run_result.run_summary.vertical_motion_adaptive_pass_count =
    config_.enable_vertical_motion_adaptive_reweighting ? completed_adaptive_pass_count : 1;
  run_result.run_summary.vertical_motion_adaptive_converged = adaptive_converged;
  run_result.run_summary.rtk_vertical_drift_reference_pass_count =
    config_.enable_rtk_vertical_drift_reference ? completed_adaptive_pass_count : 0;
  run_result.vertical_motion_adaptive_reweighting_diagnostics = adaptive_reweighting_diagnostics;
  if (active_rtk_vertical_drift_profile_ptr != nullptr) {
    run_result.rtk_vertical_drift_reference_diagnostics = *active_rtk_vertical_drift_profile_ptr;
  }

  PopulateGnssPostfitResiduals(
    optimized_values,
    base_interpolator,
    trajectory_row_index_by_state,
    run_result.gnss_factor_records,
    collect_gnss_consistency ? &run_result.gnss_consistency_records : nullptr,
    &run_result.trajectory);
  PopulateVerticalEnvelopeDiagnostics(
    optimized_values,
    base_interpolator,
    run_result.vertical_envelope_diagnostics);
  PopulateRtkVerticalDriftReferenceSummary(
    config_,
    run_result.rtk_vertical_drift_reference_diagnostics,
    run_result.run_summary);
  PopulateRtkOutagePreOutageVerticalFenceSummary(
    optimized_values,
    causal_reference_result.state_reference_rows,
    run_result.run_summary);
  PopulateVerticalVelocityDeltaDiagnostics(
    optimized_values,
    run_result.vertical_velocity_delta_diagnostics);
  AccumulateAdaptiveReweightingSummary(
    active_stability_profile_ptr != nullptr
      ? *active_stability_profile_ptr
      : VerticalMotionStabilityProfile{},
    run_result.vertical_velocity_delta_diagnostics,
    dynamic_start_time_s,
    run_result.run_summary);
  PopulateVerticalPositionVelocityConsistencyDiagnostics(
    optimized_values,
    run_result.vertical_position_velocity_consistency_diagnostics,
    run_result.run_summary);
  PopulateBodyZNHCDiagnostics(
    optimization_initial_values,
    optimized_values,
    state_timestamps,
    run_result.body_z_nhc_diagnostics,
    &run_result.body_z_nhc_state_diagnostics,
    &run_result.attitude_reference_states,
    &run_result.run_summary);
  if (active_stage2_reference != nullptr) {
    PopulateStage2VehicleNHCDiagnostics(
      optimization_initial_values,
      optimized_values,
      state_timestamps,
      stage2_fixed_reference_states,
      stage2_mount_leakage_key,
      run_result.stage2_mount_leakage_diagnostics,
      run_result.stage2_vehicle_nhc_state_diagnostics,
      run_result.run_summary);
    run_result.run_summary.stage2_max_abs_yaw_delta_rad =
      ComputeMaxAbsYawDeltaRad(stage2_fixed_reference_states, optimized_values);
  }
  PopulateAttitudeReferenceDiagnostics(
    optimized_values,
    run_result.attitude_reference_diagnostics);
  PopulateRelativeYawReferenceDiagnostics(
    optimized_values,
    run_result.relative_yaw_reference_diagnostics);
  PopulateVerticalJumpVelocityRampDiagnostics(
    optimized_values,
    run_result.vertical_jump_velocity_ramp_diagnostics);
  PopulateVerticalJumpContinuityDiagnostics(
    optimized_values,
    state_timestamps,
    run_result.vertical_jump_continuity_diagnostics);
  PopulateVerticalJumpImpulseDiagnostics(
    optimized_values,
    run_result.vertical_jump_impulse_diagnostics);
  PopulateVerticalJumpBiasDiagnostics(
    optimized_values,
    run_result.vertical_jump_bias_diagnostics);

  if (collect_reference_states) {
    const std::vector<ReferenceNodeState> optimized_reference_states =
      BuildReferenceStatesFromOptimizedValues(state_timestamps, optimized_values);
    run_result.reference_node_trajectory.clear();
    run_result.reference_node_trajectory.reserve(optimized_reference_states.size());
    for (const auto &reference_state : optimized_reference_states) {
      run_result.reference_node_trajectory.push_back(MakeReferenceNodeRow(reference_state));
    }
  }
  AccumulateStaticConsistencyMetrics(optimized_values, graph_timeline, run_result.run_summary);
  run_result.static_alignment_validation = BuildStaticAlignmentValidation(
    optimized_values,
    graph_timeline,
    global_acc_bias_key,
    config_.vertical_acc_bias_tau_s,
    run_result.run_summary.initial_static_rtk_height_reference_up_m,
    run_result.run_summary);
  run_result.error_state_trajectory.clear();
  if (graph_timeline.dynamic_start_index < state_timestamps.size()) {
    const std::size_t first_dynamic_index = graph_timeline.dynamic_start_index;
    const auto first_dynamic_pose = optimized_values.at<gtsam::Pose3>(X(first_dynamic_index));
    const auto first_dynamic_velocity = optimized_values.at<gtsam::Vector3>(V(first_dynamic_index));
    const auto first_dynamic_bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(first_dynamic_index));
    const ForwardDriftSummary drift_summary = ComputeFeedbackForwardDriftSummary(
      dataset.imu_samples,
      first_dynamic_pose,
      first_dynamic_velocity,
      first_dynamic_bias,
      state_timestamps[first_dynamic_index],
      state_timestamps.back(),
      config_.gravity_mps2);
    run_result.run_summary.feedback_forward_up_slope_10s = drift_summary.up_slope_10s;
    run_result.run_summary.feedback_forward_up_slope_30s = drift_summary.up_slope_30s;
    run_result.run_summary.feedback_forward_horizontal_slope_10s = drift_summary.horizontal_slope_10s;
    run_result.run_summary.feedback_forward_horizontal_slope_30s = drift_summary.horizontal_slope_30s;
    const auto first_dynamic_forward_rows = BuildForwardTrajectoryRows(
      dataset.imu_samples,
      first_dynamic_pose,
      first_dynamic_velocity,
      first_dynamic_bias,
      state_timestamps[first_dynamic_index],
      30.0,
      config_.gravity_mps2);
    AccumulateForwardTrajectoryVariationSummary(
      first_dynamic_forward_rows,
      run_result.run_summary.forward_first30s_up_total_variation_m,
      run_result.run_summary.forward_first30s_vz_total_variation_mps);
  }

  UpdateTrajectoryRowsFromOptimizedValues(optimized_values, run_result.trajectory);

  run_result.initial_static_trajectory = BuildInitialStaticTrajectoryRows(
    state_timestamps,
    optimized_values,
    graph_timeline.initial_static_state_count);
  run_result.run_summary.initial_static_trajectory_count = run_result.initial_static_trajectory.size();
  run_result.optimized_static_terminal_forward_trajectory.clear();
  std::optional<TrajectoryRow> optimized_last_static_row;
  if (!run_result.initial_static_trajectory.empty()) {
    const std::size_t static_terminal_graph_index = graph_timeline.initial_static_state_count - 1U;
    const auto static_terminal_pose = optimized_values.at<gtsam::Pose3>(X(static_terminal_graph_index));
    const auto static_terminal_velocity = optimized_values.at<gtsam::Vector3>(V(static_terminal_graph_index));
    const auto static_terminal_bias =
      optimized_values.at<gtsam::imuBias::ConstantBias>(B(static_terminal_graph_index));
    optimized_last_static_row = run_result.initial_static_trajectory.back();
    const auto &row = *optimized_last_static_row;
    run_result.optimized_static_terminal_forward_trajectory = BuildForwardTrajectoryRows(
      dataset.imu_samples,
      static_terminal_pose,
      static_terminal_velocity,
      static_terminal_bias,
      row.time_s,
      20.0,
      config_.gravity_mps2);
  }

  std::vector<TrajectoryRow> dynamic_trajectory_rows;
  if (graph_timeline.dynamic_start_index < run_result.trajectory.size()) {
    dynamic_trajectory_rows.assign(
      run_result.trajectory.begin() + static_cast<std::ptrdiff_t>(graph_timeline.dynamic_start_index),
      run_result.trajectory.end());
  }
  AccumulateInitialDynamicConsistencyMetrics(
    dynamic_trajectory_rows,
    initial_bias,
    optimized_last_static_row,
    run_result.optimized_static_terminal_forward_trajectory,
    run_result.run_summary);

  if (config_.write_imu_rate_avp) {
    std::vector<OptimizedNodeState> optimized_node_states;
    optimized_node_states.reserve(run_result.trajectory.size());
    for (std::size_t index = graph_timeline.dynamic_start_index; index < state_timestamps.size(); ++index) {
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
    run_result.run_summary.imu_rate_interval_count =
      optimized_node_states.size() > 1U ? optimized_node_states.size() - 1U : 0U;
    run_result.run_summary.imu_rate_skipped_interval_count =
      std::count_if(
        run_result.imu_rate_interval_diagnostics.begin(),
        run_result.imu_rate_interval_diagnostics.end(),
        [](const ImuRateIntervalDiagnostic &diagnostic) { return !diagnostic.used_interval; });
  }

  if (config_.enable_gnss) {
    if (collect_gnss_consistency) {
      AccumulateGnssConsistencySummary(
        run_result.gnss_consistency_records,
        config_,
        run_result.run_summary);
    }

    run_result.vertical_state_corrections = BuildVerticalStateCorrections(
      state_timestamps,
      optimized_values,
      run_result.gnss_factor_records,
      run_result.gnss_consistency_records,
      collect_gnss_consistency);
  }

  if (collect_segment_error_diagnostics) {
    run_result.segment_error_diagnostics = BuildSegmentErrorDiagnostics(
      state_timestamps,
      dataset.imu_samples,
      imu_params,
      optimized_values,
      run_result.gnss_factor_records,
      run_result.gnss_consistency_records,
      config_);
    run_result.run_summary.segment_error_count = run_result.segment_error_diagnostics.size();
  }

  break;
  }

  return run_result;
}

}  // namespace offline_lc_minimal
