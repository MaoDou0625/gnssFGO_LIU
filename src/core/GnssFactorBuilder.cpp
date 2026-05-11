#include "offline_lc_minimal/core/GnssFactorBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/factor/GPInterpolatedHorizontalPositionFactor.h"
#include "offline_lc_minimal/factor/HorizontalPositionFactor.h"
#include "offline_lc_minimal/core/RtkVerticalLatentReferenceBuilder.h"
#include "offline_lc_minimal/core/RtkVerticalLowpassReferenceBuilder.h"
#include "offline_lc_minimal/gp/GPWNOJInterpolator.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kInterpolatorQcVariance = 10000.0;

gtsam::SharedNoiseModel WrapHorizontalNoiseModel(
  const OfflineRunnerConfig &config,
  const gtsam::SharedNoiseModel &gaussian_model) {
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

gtsam::SharedNoiseModel MakeHorizontalNoiseModel(
  const OfflineRunnerConfig &config,
  const Eigen::Vector3d &sigma_m) {
  const gtsam::Vector2 variances(
    sigma_m.x() * sigma_m.x(),
    sigma_m.y() * sigma_m.y());
  return WrapHorizontalNoiseModel(config, gtsam::noiseModel::Diagonal::Variances(variances));
}

GnssFactorRecord MakeBaseFactorRecord(
  const GnssSolutionSample &sample,
  std::size_t sample_index,
  double corrected_time_s) {
  GnssFactorRecord record;
  record.sample_index = sample_index;
  record.raw_time_s = sample.time_s;
  record.corrected_time_s = corrected_time_s;
  record.gnss_fix_type = sample.fix_type();
  if (sample.has_enu_position) {
    record.measurement_enu_m = sample.enu_position_m;
  }
  return record;
}

GnssConsistencyRecord MakeBaseConsistencyRecord(
  const GnssSolutionSample &sample,
  std::size_t sample_index,
  double corrected_time_s) {
  GnssConsistencyRecord record;
  record.sample_index = sample_index;
  record.raw_time_s = sample.time_s;
  record.corrected_time_s = corrected_time_s;
  record.gnss_fix_type = sample.fix_type();
  record.raw_sigma_h_m = sample.sigma_h_m;
  record.postfit_residual_enu_m =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  return record;
}

void FillSyncFields(
  const StateMeasSyncResult &sync_result,
  const std::function<long long(std::size_t)> &trajectory_row_index_for_state,
  GnssFactorRecord *record,
  GnssConsistencyRecord *consistency_record) {
  record->sync_status = sync_result.status;
  record->state_index_i = sync_result.key_index_i;
  record->state_index_j = sync_result.key_index_j;
  record->trajectory_row_index_i = trajectory_row_index_for_state(sync_result.key_index_i);
  record->trajectory_row_index_j = trajectory_row_index_for_state(sync_result.key_index_j);
  record->state_time_i_s = sync_result.timestamp_i_s;
  record->state_time_j_s = sync_result.timestamp_j_s;
  record->duration_from_state_i_s = sync_result.duration_from_state_i_s;
  if (sync_result.status == StateMeasSyncStatus::kSynchronizedI ||
      sync_result.status == StateMeasSyncStatus::kSynchronizedJ) {
    record->synchronized_state_index =
      sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i
                                                                : sync_result.key_index_j;
    record->synchronized_trajectory_row_index =
      trajectory_row_index_for_state(record->synchronized_state_index);
  }

  consistency_record->sync_status = sync_result.status;
}

void FillSigmaFields(
  const Eigen::Vector3d &sigma_m,
  const VerticalConstraintPolicy &vertical_policy,
  GnssFactorRecord *factor_record,
  GnssConsistencyRecord *record) {
  factor_record->vertical_direct_position_factor_used = vertical_policy.UsesDirectPositionFactor();
  record->sigma_e_m = sigma_m.x();
  record->sigma_n_m = sigma_m.y();
  record->sigma_u_m = sigma_m.z();
  record->effective_sigma_u_m = sigma_m.z();
  record->vertical_sigma_u_used_m = vertical_policy.VerticalSigmaUsedM(sigma_m);
  record->vertical_direct_position_factor_used = vertical_policy.UsesDirectPositionFactor();
}

}  // namespace

GnssFactorBuilder::GnssFactorBuilder(GnssFactorBuildRequest request)
    : request_(std::move(request)) {}

void GnssFactorBuilder::Validate() const {
  if (request_.config == nullptr || request_.gnss_samples == nullptr || request_.graph == nullptr ||
      request_.trajectory == nullptr || request_.run_summary == nullptr ||
      request_.factor_records == nullptr || !request_.should_use_sample ||
      !request_.is_within_imu_coverage || !request_.corrected_time_s ||
      !request_.clamped_sigma_m || !request_.find_state_for_time_s ||
      !request_.trajectory_row_index_for_state) {
    throw std::runtime_error("GnssFactorBuilder received an incomplete request");
  }
  if (request_.collect_consistency_records && request_.consistency_records == nullptr) {
    throw std::runtime_error("GnssFactorBuilder consistency records requested without storage");
  }
  if (request_.config->vertical_constraint_mode == VerticalConstraintMode::kEnvelope &&
      request_.vertical_envelope_diagnostics == nullptr) {
    throw std::runtime_error("envelope vertical constraints requested without diagnostics storage");
  }
  if (request_.config->enable_rtk_vertical_lowpass_reference &&
      request_.rtk_vertical_lowpass_reference_diagnostics == nullptr) {
    throw std::runtime_error("RTK vertical low-pass reference requested without diagnostics storage");
  }
  if (request_.config->enable_rtk_vertical_latent_reference &&
      (request_.initial_values == nullptr ||
       request_.rtk_vertical_latent_reference_diagnostics == nullptr)) {
    throw std::runtime_error("RTK vertical latent reference requested without initial values or diagnostics storage");
  }
}

void GnssFactorBuilder::Build() const {
  Validate();
  if (!request_.config->enable_gnss) {
    return;
  }
  const std::unique_ptr<VerticalConstraintPolicy> vertical_policy =
    CreateVerticalConstraintPolicy(*request_.config);

  const std::size_t first_sample =
    request_.config->enable_rtk_vertical_latent_reference
      ? 0U
      : std::min(request_.navigation_start_index + 1U, request_.gnss_samples->size());
  std::optional<RtkVerticalLatentReferenceBuildResult> latent_result;
  if (request_.config->enable_rtk_vertical_latent_reference) {
    RtkVerticalLatentReferenceBuildRequest latent_request;
    latent_request.config = request_.config;
    latent_request.gnss_samples = request_.gnss_samples;
    latent_request.first_sample_index = first_sample;
    latent_request.dynamic_start_time_s = request_.dynamic_start_time_s;
    if (first_sample < request_.gnss_samples->size()) {
      latent_request.reference_epoch_s =
        request_.corrected_time_s((*request_.gnss_samples)[first_sample]);
    }
    latent_request.graph = request_.graph;
    latent_request.initial_values = request_.initial_values;
    latent_request.run_summary = request_.run_summary;
    latent_request.is_within_imu_coverage = request_.is_within_imu_coverage;
    latent_request.corrected_time_s = request_.corrected_time_s;
    latent_request.clamped_sigma_m = request_.clamped_sigma_m;
    latent_result = RtkVerticalLatentReferenceBuilder(std::move(latent_request)).Build();
    *request_.rtk_vertical_latent_reference_diagnostics =
      latent_result->diagnostics;
  }
  if (request_.config->enable_rtk_vertical_lowpass_reference) {
    RtkVerticalLowpassReferenceBuildRequest lowpass_request;
    lowpass_request.config = request_.config;
    lowpass_request.gnss_samples = request_.gnss_samples;
    lowpass_request.first_sample_index = first_sample;
    lowpass_request.is_within_imu_coverage = request_.is_within_imu_coverage;
    lowpass_request.corrected_time_s = request_.corrected_time_s;
    lowpass_request.clamped_sigma_m = request_.clamped_sigma_m;
    RtkVerticalLowpassReferenceBuildResult lowpass_result =
      RtkVerticalLowpassReferenceBuilder(std::move(lowpass_request)).Build();
    request_.run_summary->rtk_vertical_lowpass_reference_enabled = true;
    request_.run_summary->rtk_vertical_lowpass_valid_count = lowpass_result.valid_count;
    request_.run_summary->rtk_vertical_lowpass_raw_minus_lowpass_std_m =
      lowpass_result.raw_minus_lowpass_std_m;
    request_.run_summary->rtk_vertical_lowpass_raw_minus_lowpass_max_abs_m =
      lowpass_result.raw_minus_lowpass_max_abs_m;
    *request_.rtk_vertical_lowpass_reference_diagnostics = std::move(lowpass_result.rows);
  }
  for (std::size_t sample_index = first_sample; sample_index < request_.gnss_samples->size();
       ++sample_index) {
    const GnssSolutionSample &sample = (*request_.gnss_samples)[sample_index];
    const double corrected_time_s = request_.corrected_time_s(sample);
    GnssFactorRecord factor_record = MakeBaseFactorRecord(sample, sample_index, corrected_time_s);
    GnssConsistencyRecord consistency_record =
      MakeBaseConsistencyRecord(sample, sample_index, corrected_time_s);

    if (!request_.should_use_sample(sample)) {
      factor_record.sync_status = StateMeasSyncStatus::kDropped;
      consistency_record.sync_status = StateMeasSyncStatus::kDropped;
      request_.factor_records->push_back(factor_record);
      if (request_.collect_consistency_records) {
        request_.consistency_records->push_back(consistency_record);
      }
      continue;
    }

    if (!request_.is_within_imu_coverage(corrected_time_s)) {
      ++request_.run_summary->dropped_out_of_imu_coverage_count;
      ++request_.run_summary->gnss_dropped_count;
      factor_record.sync_status = StateMeasSyncStatus::kDropped;
      consistency_record.sync_status = StateMeasSyncStatus::kDropped;
      request_.factor_records->push_back(factor_record);
      if (request_.collect_consistency_records) {
        request_.consistency_records->push_back(consistency_record);
      }
      continue;
    }

    const StateMeasSyncResult sync_result = request_.find_state_for_time_s(corrected_time_s);
    FillSyncFields(
      sync_result,
      request_.trajectory_row_index_for_state,
      &factor_record,
      &consistency_record);

    Eigen::Vector3d sigma_m = request_.clamped_sigma_m(sample);
    const double elapsed_s = corrected_time_s - request_.dynamic_start_time_s;
    if (request_.config->early_gnss_relaxation_duration_s > 0.0 &&
        elapsed_s >= 0.0 &&
        elapsed_s < request_.config->early_gnss_relaxation_duration_s) {
      const double alpha = 1.0 - (elapsed_s / request_.config->early_gnss_relaxation_duration_s);
      sigma_m *= 1.0 + alpha * (request_.config->early_gnss_relaxation_scale - 1.0);
    }
    FillSigmaFields(sigma_m, *vertical_policy, &factor_record, &consistency_record);

    bool factor_used = false;
    if (sync_result.status == StateMeasSyncStatus::kSynchronizedI ||
        sync_result.status == StateMeasSyncStatus::kSynchronizedJ) {
      AddSynchronizedFactors(
        sample,
        sample_index,
        corrected_time_s,
        sync_result,
        sigma_m,
        *vertical_policy,
        latent_result ? &latent_result->sample_references : nullptr);
      factor_used = true;
      ++request_.run_summary->gnss_synced_factor_count;
    } else if (sync_result.status == StateMeasSyncStatus::kInterpolated &&
               request_.config->enable_gp_interpolated_gnss) {
      AddInterpolatedFactors(
        sample,
        sample_index,
        corrected_time_s,
        sync_result,
        sigma_m,
        *vertical_policy,
        latent_result ? &latent_result->sample_references : nullptr);
      factor_used = true;
      ++request_.run_summary->gnss_interpolated_factor_count;
    } else if (sync_result.status == StateMeasSyncStatus::kCached) {
      ++request_.run_summary->gnss_cached_count;
    } else {
      ++request_.run_summary->gnss_dropped_count;
    }

    factor_record.factor_used = factor_used;
    consistency_record.factor_used = factor_used;
    if (factor_used) {
      ++request_.run_summary->gnss_factor_count;
      UpdateTrajectoryRows(sample, factor_record);
    }

    request_.factor_records->push_back(factor_record);
    if (request_.collect_consistency_records) {
      request_.consistency_records->push_back(consistency_record);
    }
  }
}

void GnssFactorBuilder::AddSynchronizedFactors(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const double corrected_time_s,
  const StateMeasSyncResult &sync_result,
  const Eigen::Vector3d &sigma_m,
  const VerticalConstraintPolicy &vertical_policy,
  const std::vector<RtkVerticalLatentReferenceSampleReference> *latent_references) const {
  const std::size_t state_index =
    sync_result.status == StateMeasSyncStatus::kSynchronizedI ? sync_result.key_index_i
                                                              : sync_result.key_index_j;
  const gtsam::Point2 horizontal_measurement(
    sample.enu_position_m.x(),
    sample.enu_position_m.y());
  const auto horizontal_noise = MakeHorizontalNoiseModel(*request_.config, sigma_m);
  request_.graph->add(factor::HorizontalPositionFactor(
    symbol::X(state_index),
    horizontal_measurement,
    horizontal_noise));
  VerticalConstraintPolicyContext context;
  context.graph = request_.graph;
  context.run_summary = request_.run_summary;
  context.envelope_diagnostics = request_.vertical_envelope_diagnostics;
  if (request_.config->enable_rtk_vertical_lowpass_reference) {
    context.rtk_lowpass_references = request_.rtk_vertical_lowpass_reference_diagnostics;
  }
  if (request_.config->enable_rtk_vertical_latent_reference) {
    context.rtk_latent_references = latent_references;
  }
  vertical_policy.AddSynchronized(sample, sample_index, corrected_time_s, sync_result, sigma_m, context);
}

void GnssFactorBuilder::AddInterpolatedFactors(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const double corrected_time_s,
  const StateMeasSyncResult &sync_result,
  const Eigen::Vector3d &sigma_m,
  const VerticalConstraintPolicy &vertical_policy,
  const std::vector<RtkVerticalLatentReferenceSampleReference> *latent_references) const {
  const auto qc_model =
    gtsam::noiseModel::Diagonal::Variances(gtsam::Vector6::Constant(kInterpolatorQcVariance));
  const gp::GPWNOJInterpolator interpolator(
    qc_model,
    sync_result.timestamp_j_s - sync_result.timestamp_i_s,
    sync_result.duration_from_state_i_s);
  const gtsam::Point2 horizontal_measurement(
    sample.enu_position_m.x(),
    sample.enu_position_m.y());
  const auto horizontal_noise = MakeHorizontalNoiseModel(*request_.config, sigma_m);
  request_.graph->add(factor::GPInterpolatedHorizontalPositionFactor(
    symbol::X(sync_result.key_index_i),
    symbol::V(sync_result.key_index_i),
    symbol::W(sync_result.key_index_i),
    symbol::X(sync_result.key_index_j),
    symbol::V(sync_result.key_index_j),
    symbol::W(sync_result.key_index_j),
    horizontal_measurement,
    gtsam::Vector3::Zero(),
    horizontal_noise,
    interpolator));
  VerticalConstraintPolicyContext context;
  context.graph = request_.graph;
  context.run_summary = request_.run_summary;
  context.envelope_diagnostics = request_.vertical_envelope_diagnostics;
  if (request_.config->enable_rtk_vertical_lowpass_reference) {
    context.rtk_lowpass_references = request_.rtk_vertical_lowpass_reference_diagnostics;
  }
  if (request_.config->enable_rtk_vertical_latent_reference) {
    context.rtk_latent_references = latent_references;
  }
  vertical_policy.AddInterpolated(sample, sample_index, corrected_time_s, sync_result, sigma_m, interpolator, context);
}

void GnssFactorBuilder::UpdateTrajectoryRows(
  const GnssSolutionSample &sample,
  const GnssFactorRecord &record) const {
  const auto mark_row = [this, &sample](const long long row_index) {
    if (row_index < 0) {
      return;
    }
    const std::size_t index = static_cast<std::size_t>(row_index);
    if (index >= request_.trajectory->size()) {
      return;
    }
    (*request_.trajectory)[index].gnss_factor_used = true;
    (*request_.trajectory)[index].gnss_fix_type = sample.fix_type();
  };

  if (record.sync_status == StateMeasSyncStatus::kSynchronizedI ||
      record.sync_status == StateMeasSyncStatus::kSynchronizedJ) {
    mark_row(record.synchronized_trajectory_row_index);
  } else if (record.sync_status == StateMeasSyncStatus::kInterpolated) {
    mark_row(record.trajectory_row_index_i);
    mark_row(record.trajectory_row_index_j);
  }
}

}  // namespace offline_lc_minimal
