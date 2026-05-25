#include "offline_lc_minimal/core/VerticalConstraintPolicy.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalRtkFactors.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

struct EnvelopePolicyParams {
  double gate_sigma_multiple = 2.0;
  double min_half_width_m = 0.10;
  double factor_sigma_m = 0.20;
  bool enable_center_pull = false;
  double center_sigma_m = 0.60;
  VerticalEnvelopeCenterSigmaMode center_sigma_mode = VerticalEnvelopeCenterSigmaMode::kFixed;
  double center_deadband_m = 0.01;
  bool enable_rtk_vertical_drift_reference = false;
  bool rtk_vertical_drift_use_for_center_pull = true;
  bool enable_rtk_vertical_lowpass_reference = false;
};

struct CenterPullReference {
  double up_m = std::numeric_limits<double>::quiet_NaN();
  std::string type = "raw_rtk";
  double rtk_drift_estimate_m = std::numeric_limits<double>::quiet_NaN();
};

gtsam::SharedNoiseModel MakeDirectVerticalNoiseModel(const Eigen::Vector3d &sigma_m) {
  return gtsam::noiseModel::Isotropic::Sigma(1, sigma_m.z());
}

gtsam::SharedNoiseModel MakeEnvelopeNoiseModel(const EnvelopePolicyParams &params) {
  return gtsam::noiseModel::Isotropic::Sigma(1, params.factor_sigma_m);
}

gtsam::SharedNoiseModel MakeEnvelopeCenterNoiseModel(const double center_sigma_m) {
  return gtsam::noiseModel::Isotropic::Sigma(1, center_sigma_m);
}

double ComputeEnvelopeHalfWidthM(const EnvelopePolicyParams &params, const Eigen::Vector3d &sigma_m) {
  return std::max(
    params.min_half_width_m,
    params.gate_sigma_multiple * sigma_m.z());
}

double ComputeEnvelopeCenterSigmaM(
  const EnvelopePolicyParams &params,
  const double half_width_m) {
  if (params.center_sigma_mode == VerticalEnvelopeCenterSigmaMode::kGateSigma) {
    return half_width_m / params.gate_sigma_multiple;
  }
  return params.center_sigma_m;
}

CenterPullReference SelectCenterPullReference(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const EnvelopePolicyParams &params,
  const VerticalConstraintPolicyContext &context) {
  CenterPullReference reference;
  reference.up_m =
    context.vertical_reference != nullptr && context.vertical_reference->valid
      ? context.vertical_reference->selected_up_m
      : sample.enu_position_m.z();
  if (context.vertical_reference != nullptr &&
      context.vertical_reference->valid &&
      context.vertical_reference->source != "raw_rtk") {
    reference.type = context.vertical_reference->source;
    reference.rtk_drift_estimate_m =
      context.vertical_reference->rtk_drift_estimate_m;
    return reference;
  }
  if (!params.enable_rtk_vertical_drift_reference ||
      !params.rtk_vertical_drift_use_for_center_pull ||
      context.rtk_vertical_drift_reference_profile == nullptr ||
      sample_index >= context.rtk_vertical_drift_reference_profile->size()) {
    return reference;
  }
  const auto &row = (*context.rtk_vertical_drift_reference_profile)[sample_index];
  if (!row.valid || !std::isfinite(row.corrected_center_up_m)) {
    return reference;
  }
  if (params.enable_rtk_vertical_lowpass_reference &&
      row.lowpass_applied &&
      std::isfinite(row.lowpass_center_up_m)) {
    reference.up_m = row.lowpass_center_up_m;
    reference.type = "rtk_drift_lowpass";
  } else {
    reference.up_m = row.corrected_center_up_m;
    reference.type = "rtk_drift_corrected";
  }
  reference.rtk_drift_estimate_m = row.drift_estimate_m;
  return reference;
}

VerticalEnvelopeDiagnosticRow MakeEnvelopeDiagnosticRow(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const double corrected_time_s,
  const StateMeasSyncResult &sync_result,
  const Eigen::Vector3d &sigma_m,
  const double half_width_m,
  const double center_sigma_m,
  const double envelope_center_up_m,
  const CenterPullReference &center_reference,
  const EnvelopePolicyParams &params) {
  VerticalEnvelopeDiagnosticRow row;
  row.sample_index = sample_index;
  row.raw_time_s = sample.time_s;
  row.corrected_time_s = corrected_time_s;
  row.sync_status = sync_result.status;
  row.state_index_i = sync_result.key_index_i;
  row.state_index_j = sync_result.key_index_j;
  row.state_time_i_s = sync_result.timestamp_i_s;
  row.state_time_j_s = sync_result.timestamp_j_s;
  row.duration_from_state_i_s = sync_result.duration_from_state_i_s;
  if (sync_result.status == StateMeasSyncStatus::kSynchronizedI ||
      sync_result.status == StateMeasSyncStatus::kSynchronizedJ) {
    row.synchronized_state_index =
      sync_result.status == StateMeasSyncStatus::kSynchronizedI
        ? sync_result.key_index_i
        : sync_result.key_index_j;
  }
  row.factor_used = true;
  row.rtk_up_m = envelope_center_up_m;
  row.sigma_u_m = sigma_m.z();
  row.half_width_m = half_width_m;
  row.center_pull_factor_used = params.enable_center_pull;
  row.center_pull_reference_type = center_reference.type;
  row.center_pull_reference_up_m = center_reference.up_m;
  row.rtk_drift_estimate_m = center_reference.rtk_drift_estimate_m;
  if (params.enable_center_pull) {
    row.center_pull_sigma_m = center_sigma_m;
    row.center_pull_deadband_m = params.center_deadband_m;
  }
  return row;
}

void RequireGraph(const VerticalConstraintPolicyContext &context) {
  if (context.graph == nullptr) {
    throw std::runtime_error("vertical constraint policy requires a graph");
  }
}

class DirectVerticalConstraintPolicy final : public VerticalConstraintPolicy {
 public:
  DirectVerticalConstraintPolicy() = default;

  [[nodiscard]] bool UsesDirectPositionFactor() const override { return true; }

  [[nodiscard]] double VerticalSigmaUsedM(const Eigen::Vector3d &sigma_m) const override {
    return sigma_m.z();
  }

  void AddSynchronized(
    const GnssSolutionSample &sample,
    const std::size_t /*sample_index*/,
    const double /*corrected_time_s*/,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const VerticalConstraintPolicyContext &context) const override {
    RequireGraph(context);
    const std::size_t state_index =
      sync_result.status == StateMeasSyncStatus::kSynchronizedI
        ? sync_result.key_index_i
        : sync_result.key_index_j;
    const double vertical_up_m =
      context.vertical_reference != nullptr && context.vertical_reference->valid
        ? context.vertical_reference->selected_up_m
        : sample.enu_position_m.z();
    context.graph->add(factor::VerticalPositionFactor(
      symbol::X(state_index),
      vertical_up_m,
      MakeDirectVerticalNoiseModel(sigma_m)));
  }

  void AddInterpolated(
    const GnssSolutionSample &sample,
    const std::size_t /*sample_index*/,
    const double /*corrected_time_s*/,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const gp::GPWNOJInterpolator &interpolator,
    const VerticalConstraintPolicyContext &context) const override {
    RequireGraph(context);
    const double vertical_up_m =
      context.vertical_reference != nullptr && context.vertical_reference->valid
        ? context.vertical_reference->selected_up_m
        : sample.enu_position_m.z();
    context.graph->add(factor::GPInterpolatedVerticalPositionFactor(
      symbol::X(sync_result.key_index_i),
      symbol::V(sync_result.key_index_i),
      symbol::W(sync_result.key_index_i),
      symbol::X(sync_result.key_index_j),
      symbol::V(sync_result.key_index_j),
      symbol::W(sync_result.key_index_j),
      vertical_up_m,
      interpolator,
      MakeDirectVerticalNoiseModel(sigma_m)));
  }
};

class EnvelopeVerticalConstraintPolicy final : public VerticalConstraintPolicy {
 public:
  explicit EnvelopeVerticalConstraintPolicy(const OfflineRunnerConfig &config)
      : params_{
          config.vertical_envelope_gate_sigma_multiple,
          config.vertical_envelope_min_half_width_m,
          config.vertical_envelope_factor_sigma_m,
          config.enable_vertical_envelope_center_pull,
          config.vertical_envelope_center_sigma_m,
          config.vertical_envelope_center_sigma_mode,
          config.vertical_envelope_center_deadband_m,
          config.enable_rtk_vertical_drift_reference,
          config.rtk_vertical_drift_use_for_center_pull,
          config.enable_rtk_vertical_lowpass_reference} {}

  [[nodiscard]] bool UsesDirectPositionFactor() const override { return false; }

  [[nodiscard]] double VerticalSigmaUsedM(const Eigen::Vector3d & /*sigma_m*/) const override {
    return params_.factor_sigma_m;
  }

  void AddSynchronized(
    const GnssSolutionSample &sample,
    const std::size_t sample_index,
    const double corrected_time_s,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const VerticalConstraintPolicyContext &context) const override {
    RequireGraph(context);
    if (context.envelope_diagnostics == nullptr) {
      throw std::runtime_error("envelope vertical constraints require diagnostics storage");
    }
    const std::size_t state_index =
      sync_result.status == StateMeasSyncStatus::kSynchronizedI
        ? sync_result.key_index_i
        : sync_result.key_index_j;
    const double half_width_m = ComputeEnvelopeHalfWidthM(params_, sigma_m);
    const double center_sigma_m = ComputeEnvelopeCenterSigmaM(params_, half_width_m);
    const CenterPullReference center_reference =
      SelectCenterPullReference(sample, sample_index, params_, context);
    const double envelope_center_up_m =
      context.vertical_reference != nullptr && context.vertical_reference->valid
        ? context.vertical_reference->selected_up_m
        : sample.enu_position_m.z();
    context.graph->add(factor::VerticalEnvelopeFactor(
      symbol::X(state_index),
      envelope_center_up_m,
      half_width_m,
      MakeEnvelopeNoiseModel(params_)));
    if (params_.enable_center_pull) {
      context.graph->add(factor::VerticalEnvelopeCenterPullFactor(
        symbol::X(state_index),
        center_reference.up_m,
        half_width_m,
        params_.center_deadband_m,
        MakeEnvelopeCenterNoiseModel(center_sigma_m)));
    }
    context.envelope_diagnostics->push_back(
      MakeEnvelopeDiagnosticRow(
        sample,
        sample_index,
        corrected_time_s,
        sync_result,
        sigma_m,
        half_width_m,
        center_sigma_m,
        envelope_center_up_m,
        center_reference,
        params_));
  }

  void AddInterpolated(
    const GnssSolutionSample &sample,
    const std::size_t sample_index,
    const double corrected_time_s,
    const StateMeasSyncResult &sync_result,
    const Eigen::Vector3d &sigma_m,
    const gp::GPWNOJInterpolator &interpolator,
    const VerticalConstraintPolicyContext &context) const override {
    RequireGraph(context);
    if (context.envelope_diagnostics == nullptr) {
      throw std::runtime_error("envelope vertical constraints require diagnostics storage");
    }
    const double half_width_m = ComputeEnvelopeHalfWidthM(params_, sigma_m);
    const double center_sigma_m = ComputeEnvelopeCenterSigmaM(params_, half_width_m);
    const CenterPullReference center_reference =
      SelectCenterPullReference(sample, sample_index, params_, context);
    const double envelope_center_up_m =
      context.vertical_reference != nullptr && context.vertical_reference->valid
        ? context.vertical_reference->selected_up_m
        : sample.enu_position_m.z();
    context.graph->add(factor::GPInterpolatedVerticalEnvelopeFactor(
      symbol::X(sync_result.key_index_i),
      symbol::V(sync_result.key_index_i),
      symbol::W(sync_result.key_index_i),
      symbol::X(sync_result.key_index_j),
      symbol::V(sync_result.key_index_j),
      symbol::W(sync_result.key_index_j),
      envelope_center_up_m,
      half_width_m,
      interpolator,
      MakeEnvelopeNoiseModel(params_)));
    if (params_.enable_center_pull) {
      context.graph->add(factor::GPInterpolatedVerticalEnvelopeCenterPullFactor(
        symbol::X(sync_result.key_index_i),
        symbol::V(sync_result.key_index_i),
        symbol::W(sync_result.key_index_i),
        symbol::X(sync_result.key_index_j),
        symbol::V(sync_result.key_index_j),
        symbol::W(sync_result.key_index_j),
        center_reference.up_m,
        half_width_m,
        params_.center_deadband_m,
        interpolator,
        MakeEnvelopeCenterNoiseModel(center_sigma_m)));
    }
    context.envelope_diagnostics->push_back(
      MakeEnvelopeDiagnosticRow(
        sample,
        sample_index,
        corrected_time_s,
        sync_result,
        sigma_m,
        half_width_m,
        center_sigma_m,
        envelope_center_up_m,
        center_reference,
        params_));
  }

 private:
  EnvelopePolicyParams params_;
};

}  // namespace

std::unique_ptr<VerticalConstraintPolicy> CreateVerticalConstraintPolicy(
  const OfflineRunnerConfig &config) {
  switch (config.vertical_constraint_mode) {
    case VerticalConstraintMode::kDirectZ:
      return std::make_unique<DirectVerticalConstraintPolicy>();
    case VerticalConstraintMode::kEnvelope:
      return std::make_unique<EnvelopeVerticalConstraintPolicy>(config);
    default:
      throw std::runtime_error("unsupported vertical constraint mode");
  }
}

}  // namespace offline_lc_minimal
