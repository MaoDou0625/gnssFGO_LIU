#include "offline_lc_minimal/core/VerticalConstraintPolicy.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
};

gtsam::SharedNoiseModel MakeDirectVerticalNoiseModel(const Eigen::Vector3d &sigma_m) {
  return gtsam::noiseModel::Isotropic::Sigma(1, sigma_m.z());
}

gtsam::SharedNoiseModel MakeEnvelopeNoiseModel(const EnvelopePolicyParams &params) {
  return gtsam::noiseModel::Isotropic::Sigma(1, params.factor_sigma_m);
}

gtsam::SharedNoiseModel MakeEnvelopeCenterNoiseModel(const EnvelopePolicyParams &params) {
  return gtsam::noiseModel::Isotropic::Sigma(1, params.center_sigma_m);
}

double ComputeEnvelopeHalfWidthM(const EnvelopePolicyParams &params, const Eigen::Vector3d &sigma_m) {
  return std::max(
    params.min_half_width_m,
    params.gate_sigma_multiple * sigma_m.z());
}

VerticalEnvelopeDiagnosticRow MakeEnvelopeDiagnosticRow(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const double corrected_time_s,
  const StateMeasSyncResult &sync_result,
  const Eigen::Vector3d &sigma_m,
  const double half_width_m,
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
  row.rtk_up_m = sample.enu_position_m.z();
  row.sigma_u_m = sigma_m.z();
  row.half_width_m = half_width_m;
  row.center_pull_factor_used = params.enable_center_pull;
  if (params.enable_center_pull) {
    row.center_pull_sigma_m = params.center_sigma_m;
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
    context.graph->add(factor::VerticalPositionFactor(
      symbol::X(state_index),
      sample.enu_position_m.z(),
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
    context.graph->add(factor::GPInterpolatedVerticalPositionFactor(
      symbol::X(sync_result.key_index_i),
      symbol::V(sync_result.key_index_i),
      symbol::W(sync_result.key_index_i),
      symbol::X(sync_result.key_index_j),
      symbol::V(sync_result.key_index_j),
      symbol::W(sync_result.key_index_j),
      sample.enu_position_m.z(),
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
          config.vertical_envelope_center_sigma_m} {}

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
    context.graph->add(factor::VerticalEnvelopeFactor(
      symbol::X(state_index),
      sample.enu_position_m.z(),
      half_width_m,
      MakeEnvelopeNoiseModel(params_)));
    if (params_.enable_center_pull) {
      context.graph->add(factor::VerticalEnvelopeCenterPullFactor(
        symbol::X(state_index),
        sample.enu_position_m.z(),
        half_width_m,
        MakeEnvelopeCenterNoiseModel(params_)));
    }
    context.envelope_diagnostics->push_back(
      MakeEnvelopeDiagnosticRow(sample, sample_index, corrected_time_s, sync_result, sigma_m, half_width_m, params_));
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
    context.graph->add(factor::GPInterpolatedVerticalEnvelopeFactor(
      symbol::X(sync_result.key_index_i),
      symbol::V(sync_result.key_index_i),
      symbol::W(sync_result.key_index_i),
      symbol::X(sync_result.key_index_j),
      symbol::V(sync_result.key_index_j),
      symbol::W(sync_result.key_index_j),
      sample.enu_position_m.z(),
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
        sample.enu_position_m.z(),
        half_width_m,
        interpolator,
        MakeEnvelopeCenterNoiseModel(params_)));
    }
    context.envelope_diagnostics->push_back(
      MakeEnvelopeDiagnosticRow(sample, sample_index, corrected_time_s, sync_result, sigma_m, half_width_m, params_));
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
