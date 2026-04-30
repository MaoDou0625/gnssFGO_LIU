#include "offline_lc_minimal/core/BodyZWindowPipeline.h"

#include <algorithm>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "offline_lc_minimal/factor/GPInterpolatedGPSFactor.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {
namespace {

using gtsam::symbol_shorthand::B;
using gtsam::symbol_shorthand::V;
using gtsam::symbol_shorthand::W;
using gtsam::symbol_shorthand::X;

constexpr double kInterpolatorQcVariance = 10000.0;

std::vector<ReferenceNodeState> BuildReferenceStatesFromOptimizedValues(
  const std::vector<double> &state_timestamps,
  const gtsam::Values &optimized_values) {
  std::vector<ReferenceNodeState> states;
  states.reserve(state_timestamps.size());
  for (std::size_t state_index = 0; state_index < state_timestamps.size(); ++state_index) {
    ReferenceNodeState state;
    state.time_s = state_timestamps[state_index];
    state.pose = optimized_values.at<gtsam::Pose3>(X(state_index));
    state.velocity = optimized_values.at<gtsam::Vector3>(V(state_index));
    state.bias = optimized_values.at<gtsam::imuBias::ConstantBias>(B(state_index));
    state.omega = optimized_values.at<gtsam::Vector3>(W(state_index));
    states.push_back(state);
  }
  return states;
}

gtsam::SharedNoiseModel MakeRtkSeedNoiseModel(
  const BodyZWindowPipelineRequest &request,
  const GnssSolutionSample &sample) {
  const Eigen::Vector3d sigma_m = request.clamped_sigma_m(sample);
  return gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector3(sigma_m.x(), sigma_m.y(), sigma_m.z()));
}

std::vector<ReferenceNodeState> BuildRtkSeedReferenceStates(const BodyZWindowPipelineRequest &request) {
  if (!request.config->enable_vertical_rtk_seed_pass) {
    return {};
  }
  if (request.gnss_samples == nullptr || request.base_graph == nullptr ||
      request.base_initial_values == nullptr || !request.passes_gnss_quality_filters ||
      !request.is_within_imu_coverage || !request.corrected_time_s ||
      !request.clamped_sigma_m || !request.find_state_for_time_s) {
    throw std::runtime_error("BodyZWindowPipeline RTK seed pass received an incomplete request");
  }

  gtsam::NonlinearFactorGraph rtk_seed_graph = *request.base_graph;
  std::size_t rtk_seed_factor_count = 0U;
  const gp::GPWNOJInterpolator base_interpolator(
    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance)));

  const std::size_t first_sample =
    std::min(request.navigation_start_index + 1U, request.gnss_samples->size());
  for (std::size_t sample_index = first_sample; sample_index < request.gnss_samples->size(); ++sample_index) {
    const auto &gnss_sample = (*request.gnss_samples)[sample_index];
    if (request.config->body_z_seed_jump_use_fix_only && gnss_sample.fix_type() != GnssFixType::kRtkFix) {
      continue;
    }
    if (!request.passes_gnss_quality_filters(gnss_sample) || !gnss_sample.has_enu_position) {
      continue;
    }
    const double corrected_time_s = request.corrected_time_s(gnss_sample);
    if (!request.is_within_imu_coverage(corrected_time_s)) {
      continue;
    }
    const StateMeasSyncResult sync_result = request.find_state_for_time_s(corrected_time_s);
    if (sync_result.status == StateMeasSyncStatus::kSynchronizedI ||
        sync_result.status == StateMeasSyncStatus::kSynchronizedJ) {
      const std::size_t sync_state_index =
        sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i : sync_result.key_index_j;
      rtk_seed_graph.add(gtsam::GPSFactor(
        X(sync_state_index),
        gtsam::Point3(
          gnss_sample.enu_position_m.x(),
          gnss_sample.enu_position_m.y(),
          gnss_sample.enu_position_m.z()),
        MakeRtkSeedNoiseModel(request, gnss_sample)));
      ++rtk_seed_factor_count;
    } else if (
      sync_result.status == StateMeasSyncStatus::kInterpolated &&
      request.config->enable_gp_interpolated_gnss &&
      sync_result.state_j_exists()) {
      gp::GPWNOJInterpolator interpolator = base_interpolator;
      interpolator.Recalculate(
        sync_result.timestamp_j_s - sync_result.timestamp_i_s,
        sync_result.duration_from_state_i_s);
      rtk_seed_graph.add(factor::GPInterpolatedGPSFactor(
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
        MakeRtkSeedNoiseModel(request, gnss_sample),
        interpolator));
      ++rtk_seed_factor_count;
    }
  }

  if (rtk_seed_factor_count == 0U) {
    return {};
  }

  gtsam::LevenbergMarquardtOptimizer optimizer(
    rtk_seed_graph,
    *request.base_initial_values,
    request.optimizer_params);
  const gtsam::Values optimized_values = optimizer.optimize();
  return BuildReferenceStatesFromOptimizedValues(*request.state_timestamps, optimized_values);
}

BodyZSeedImuDiagnosticRow MakeImuDiagnosticRow(const BodyZJumpSignalSample &sample) {
  BodyZSeedImuDiagnosticRow row;
  row.time_s = sample.time_s;
  row.relative_time_s = sample.relative_time_s;
  row.body_z_specific_force_mps2 = sample.body_z_specific_force_mps2;
  row.gravity_projection_z_mps2 = sample.gravity_projection_z_mps2;
  row.body_z_acc_mps2 = sample.body_z_acc_mps2;
  row.body_z_acc_1s_smooth_mps2 = sample.body_z_acc_1s_smooth_mps2;
  row.integrated_body_z_velocity_mps = sample.integrated_body_z_velocity_mps;
  row.integrated_body_z_velocity_0p2s_smooth_mps =
    sample.integrated_body_z_velocity_0p2s_smooth_mps;
  row.integrated_body_z_velocity_1s_smooth_mps =
    sample.integrated_body_z_velocity_1s_smooth_mps;
  row.signed_step_metric_mps = sample.signed_step_metric_mps;
  row.downward_score_mps = sample.downward_score_mps;
  row.upward_score_mps = sample.upward_score_mps;
  row.body_z_axis_nav_z = sample.body_z_axis_nav_z;
  return row;
}

BodyZSeedJumpWindowRow MakeJumpWindowRow(const BodyZJumpWindowCandidate &window) {
  BodyZSeedJumpWindowRow row;
  row.direction = window.direction;
  row.selection_level = window.selection_level;
  row.start_state_index = static_cast<long long>(window.start_state_index);
  row.center_state_index = static_cast<long long>(window.center_state_index);
  row.end_state_index = static_cast<long long>(window.end_state_index);
  row.start_time_s = window.start_time_s;
  row.center_time_s = window.center_time_s;
  row.end_time_s = window.end_time_s;
  row.start_relative_time_s = window.start_relative_time_s;
  row.center_relative_time_s = window.center_relative_time_s;
  row.end_relative_time_s = window.end_relative_time_s;
  row.duration_s = window.duration_s;
  row.pre_velocity_mps = window.pre_velocity_mps;
  row.post_velocity_mps = window.post_velocity_mps;
  row.signed_delta_velocity_mps = window.signed_delta_velocity_mps;
  row.direction_score_mps = window.direction_score_mps;
  row.signed_step_metric_mps = window.signed_step_metric_mps;
  row.level_threshold_mps = window.level_threshold_mps;
  row.level_max_peak_mps = window.level_max_peak_mps;
  row.level_noise_floor_mps = window.level_noise_floor_mps;
  row.min_acc_mps2 = window.min_acc_mps2;
  row.max_acc_mps2 = window.max_acc_mps2;
  row.mean_acc_mps2 = window.mean_acc_mps2;
  row.body_z_axis_nav_z = window.body_z_axis_nav_z;
  row.delta_vz_init_mps = window.delta_vz_init_mps;
  return row;
}

}  // namespace

BodyZWindowPipeline::BodyZWindowPipeline(BodyZWindowPipelineRequest request)
    : request_(std::move(request)) {}

BodyZWindowPipelineResult BodyZWindowPipeline::Run() const {
  if (request_.config == nullptr || request_.imu_samples == nullptr ||
      request_.state_timestamps == nullptr) {
    throw std::runtime_error("BodyZWindowPipeline received an incomplete request");
  }

  BodyZWindowPipelineResult result;
  if (!request_.config->enable_body_z_seed_jump_windows) {
    return result;
  }

  const std::vector<ReferenceNodeState> seed_reference_states = BuildRtkSeedReferenceStates(request_);
  if (seed_reference_states.empty()) {
    return result;
  }

  BodyZBidirectionalJumpDetector detector(*request_.config);
  result.detection = detector.Detect(
    *request_.imu_samples,
    seed_reference_states,
    *request_.state_timestamps,
    request_.dynamic_start_time_s,
    request_.end_time_s);

  result.imu_diagnostics.reserve(result.detection.signal.size());
  for (const auto &sample : result.detection.signal) {
    result.imu_diagnostics.push_back(MakeImuDiagnosticRow(sample));
  }

  result.jump_windows.reserve(result.detection.windows.size());
  for (const auto &window : result.detection.windows) {
    result.jump_windows.push_back(MakeJumpWindowRow(window));
  }
  return result;
}

}  // namespace offline_lc_minimal
