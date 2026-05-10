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
  bool enable_lowpass_reference = false;
  bool lowpass_use_for_center_pull = true;
  bool enable_latent_reference = false;
  bool latent_use_for_center_pull = true;
  bool latent_use_for_envelope_gate = true;
};

struct VerticalReferenceSelection {
  bool reference_available = true;
  double reference_up_m = std::numeric_limits<double>::quiet_NaN();
  std::string reference_type = "raw_rtk";
  std::string skip_reason = "NONE";
  std::size_t latent_key_index = 0;
  gtsam::Key latent_key = 0;
  bool latent_reference_used = false;
};

struct CenterPullReference {
  bool factor_used = false;
  VerticalReferenceSelection selection;
};

VerticalReferenceSelection RawReference(const GnssSolutionSample &sample) {
  VerticalReferenceSelection reference;
  reference.reference_available = true;
  reference.reference_up_m = sample.enu_position_m.z();
  reference.reference_type = "raw_rtk";
  return reference;
}

VerticalReferenceSelection LatentReference(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const VerticalConstraintPolicyContext &context) {
  VerticalReferenceSelection reference = RawReference(sample);
  reference.reference_available = false;
  reference.reference_type = "rtk_latent";
  reference.skip_reason = "LATENT_REFERENCE_MISSING";
  if (context.rtk_latent_references == nullptr ||
      sample_index >= context.rtk_latent_references->size()) {
    return reference;
  }
  const RtkVerticalLatentReferenceSampleReference &sample_reference =
    (*context.rtk_latent_references)[sample_index];
  reference.latent_key_index = sample_reference.key_index;
  reference.latent_key = sample_reference.key;
  reference.latent_reference_used = sample_reference.valid;
  reference.reference_up_m = sample_reference.initial_reference_up_m;
  if (!sample_reference.valid || !std::isfinite(sample_reference.initial_reference_up_m)) {
    reference.skip_reason =
      sample_reference.skip_reason.empty() ? "LATENT_REFERENCE_INVALID" : sample_reference.skip_reason;
    return reference;
  }
  reference.reference_available = true;
  reference.skip_reason = "NONE";
  return reference;
}

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

VerticalEnvelopeDiagnosticRow MakeEnvelopeDiagnosticRow(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const double corrected_time_s,
  const StateMeasSyncResult &sync_result,
  const Eigen::Vector3d &sigma_m,
  const double half_width_m,
  const double center_sigma_m,
  const double center_deadband_m,
  const double gate_reference_up_m,
  const std::string &gate_reference_type,
  const std::string &gate_skip_reason,
  const std::size_t latent_reference_key_index,
  const bool latent_reference_used,
  const double center_reference_up_m,
  const std::string &center_reference_type,
  const std::string &center_skip_reason,
  const bool center_factor_used) {
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
  row.gate_reference_up_m = gate_reference_up_m;
  row.gate_reference_type = gate_reference_type;
  row.raw_minus_gate_reference_m = sample.enu_position_m.z() - gate_reference_up_m;
  row.gate_reference_skip_reason = gate_skip_reason;
  row.latent_reference_key_index = latent_reference_key_index;
  row.latent_reference_used = latent_reference_used;
  row.center_pull_factor_used = center_factor_used;
  row.center_pull_reference_up_m = center_reference_up_m;
  row.center_pull_reference_type = center_reference_type;
  row.center_pull_skip_reason = center_skip_reason;
  if (center_factor_used) {
    row.center_pull_sigma_m = center_sigma_m;
    row.center_pull_deadband_m = center_deadband_m;
  }
  return row;
}

CenterPullReference SelectCenterPullReference(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const EnvelopePolicyParams &params,
  const VerticalConstraintPolicyContext &context,
  const VerticalReferenceSelection &gate_reference) {
  CenterPullReference reference;
  reference.factor_used = params.enable_center_pull;
  reference.selection = RawReference(sample);

  if (!params.enable_center_pull) {
    reference.factor_used = false;
    reference.selection.skip_reason = "CENTER_PULL_DISABLED";
    return reference;
  }
  if (params.enable_latent_reference && params.latent_use_for_center_pull) {
    reference.selection = gate_reference.latent_reference_used
                            ? gate_reference
                            : LatentReference(sample, sample_index, context);
    reference.factor_used = reference.selection.reference_available;
    return reference;
  }
  if (!params.enable_lowpass_reference || !params.lowpass_use_for_center_pull) {
    return reference;
  }
  if (context.rtk_lowpass_references == nullptr ||
      sample_index >= context.rtk_lowpass_references->size()) {
    reference.factor_used = false;
    reference.selection.reference_type = "rtk_lowpass";
    reference.selection.skip_reason = "LOWPASS_REFERENCE_MISSING";
    return reference;
  }

  const RtkVerticalLowpassReferenceRow &lowpass_row =
    (*context.rtk_lowpass_references)[sample_index];
  reference.selection.reference_type = "rtk_lowpass";
  reference.selection.reference_up_m = lowpass_row.lowpass_up_m;
  if (!lowpass_row.lowpass_valid || !std::isfinite(lowpass_row.lowpass_up_m)) {
    reference.factor_used = false;
    reference.selection.skip_reason =
      lowpass_row.skip_reason.empty() ? "LOWPASS_REFERENCE_INVALID" : lowpass_row.skip_reason;
  }
  return reference;
}

VerticalReferenceSelection SelectGateReference(
  const GnssSolutionSample &sample,
  const std::size_t sample_index,
  const EnvelopePolicyParams &params,
  const VerticalConstraintPolicyContext &context) {
  if (params.enable_latent_reference && params.latent_use_for_envelope_gate) {
    return LatentReference(sample, sample_index, context);
  }
  return RawReference(sample);
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
          config.vertical_envelope_center_sigma_m,
          config.vertical_envelope_center_sigma_mode,
          config.vertical_envelope_center_deadband_m,
          config.enable_rtk_vertical_lowpass_reference,
          config.rtk_vertical_lowpass_use_for_center_pull,
          config.enable_rtk_vertical_latent_reference,
          config.rtk_vertical_latent_reference_use_for_center_pull,
          config.rtk_vertical_latent_reference_use_for_envelope_gate} {}

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
    const VerticalReferenceSelection gate_reference =
      SelectGateReference(sample, sample_index, params_, context);
    if (!gate_reference.reference_available) {
      throw std::runtime_error("latent RTK gate reference is missing for a synchronized GNSS factor");
    }
    if (gate_reference.latent_reference_used) {
      context.graph->add(factor::VerticalEnvelopeLatentReferenceFactor(
        symbol::X(state_index),
        gate_reference.latent_key,
        half_width_m,
        MakeEnvelopeNoiseModel(params_)));
      if (context.run_summary != nullptr) {
        ++context.run_summary->rtk_vertical_latent_reference_envelope_factor_count;
      }
    } else {
      context.graph->add(factor::VerticalEnvelopeFactor(
        symbol::X(state_index),
        gate_reference.reference_up_m,
        half_width_m,
        MakeEnvelopeNoiseModel(params_)));
    }
    const CenterPullReference center_reference =
      SelectCenterPullReference(sample, sample_index, params_, context, gate_reference);
    if (center_reference.factor_used) {
      if (center_reference.selection.latent_reference_used) {
        context.graph->add(factor::VerticalEnvelopeLatentCenterPullFactor(
          symbol::X(state_index),
          center_reference.selection.latent_key,
          half_width_m,
          params_.center_deadband_m,
          MakeEnvelopeCenterNoiseModel(center_sigma_m)));
        if (context.run_summary != nullptr) {
          ++context.run_summary->rtk_vertical_latent_reference_center_pull_factor_count;
        }
      } else {
        context.graph->add(factor::VerticalEnvelopeCenterPullFactor(
          symbol::X(state_index),
          center_reference.selection.reference_up_m,
          half_width_m,
          params_.center_deadband_m,
          MakeEnvelopeCenterNoiseModel(center_sigma_m)));
        if (context.run_summary != nullptr &&
            center_reference.selection.reference_type == "rtk_lowpass") {
          ++context.run_summary->rtk_vertical_lowpass_center_pull_factor_count;
        }
      }
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
        params_.center_deadband_m,
        gate_reference.reference_up_m,
        gate_reference.reference_type,
        gate_reference.skip_reason,
        gate_reference.latent_key_index,
        gate_reference.latent_reference_used,
        center_reference.selection.reference_up_m,
        center_reference.selection.reference_type,
        center_reference.selection.skip_reason,
        center_reference.factor_used));
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
    const VerticalReferenceSelection gate_reference =
      SelectGateReference(sample, sample_index, params_, context);
    if (!gate_reference.reference_available) {
      throw std::runtime_error("latent RTK gate reference is missing for an interpolated GNSS factor");
    }
    if (gate_reference.latent_reference_used) {
      context.graph->add(factor::GPInterpolatedVerticalEnvelopeLatentReferenceFactor(
        symbol::X(sync_result.key_index_i),
        symbol::V(sync_result.key_index_i),
        symbol::W(sync_result.key_index_i),
        symbol::X(sync_result.key_index_j),
        symbol::V(sync_result.key_index_j),
        symbol::W(sync_result.key_index_j),
        gate_reference.latent_key,
        half_width_m,
        interpolator,
        MakeEnvelopeNoiseModel(params_)));
      if (context.run_summary != nullptr) {
        ++context.run_summary->rtk_vertical_latent_reference_envelope_factor_count;
      }
    } else {
      context.graph->add(factor::GPInterpolatedVerticalEnvelopeFactor(
        symbol::X(sync_result.key_index_i),
        symbol::V(sync_result.key_index_i),
        symbol::W(sync_result.key_index_i),
        symbol::X(sync_result.key_index_j),
        symbol::V(sync_result.key_index_j),
        symbol::W(sync_result.key_index_j),
        gate_reference.reference_up_m,
        half_width_m,
        interpolator,
        MakeEnvelopeNoiseModel(params_)));
    }
    const CenterPullReference center_reference =
      SelectCenterPullReference(sample, sample_index, params_, context, gate_reference);
    if (center_reference.factor_used) {
      if (center_reference.selection.latent_reference_used) {
        context.graph->add(factor::GPInterpolatedVerticalEnvelopeLatentCenterPullFactor(
          symbol::X(sync_result.key_index_i),
          symbol::V(sync_result.key_index_i),
          symbol::W(sync_result.key_index_i),
          symbol::X(sync_result.key_index_j),
          symbol::V(sync_result.key_index_j),
          symbol::W(sync_result.key_index_j),
          center_reference.selection.latent_key,
          half_width_m,
          params_.center_deadband_m,
          interpolator,
          MakeEnvelopeCenterNoiseModel(center_sigma_m)));
        if (context.run_summary != nullptr) {
          ++context.run_summary->rtk_vertical_latent_reference_center_pull_factor_count;
        }
      } else {
        context.graph->add(factor::GPInterpolatedVerticalEnvelopeCenterPullFactor(
          symbol::X(sync_result.key_index_i),
          symbol::V(sync_result.key_index_i),
          symbol::W(sync_result.key_index_i),
          symbol::X(sync_result.key_index_j),
          symbol::V(sync_result.key_index_j),
          symbol::W(sync_result.key_index_j),
          center_reference.selection.reference_up_m,
          half_width_m,
          params_.center_deadband_m,
          interpolator,
          MakeEnvelopeCenterNoiseModel(center_sigma_m)));
        if (context.run_summary != nullptr &&
            center_reference.selection.reference_type == "rtk_lowpass") {
          ++context.run_summary->rtk_vertical_lowpass_center_pull_factor_count;
        }
      }
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
        params_.center_deadband_m,
        gate_reference.reference_up_m,
        gate_reference.reference_type,
        gate_reference.skip_reason,
        gate_reference.latent_key_index,
        gate_reference.latent_reference_used,
        center_reference.selection.reference_up_m,
        center_reference.selection.reference_type,
        center_reference.selection.skip_reason,
        center_reference.factor_used));
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
