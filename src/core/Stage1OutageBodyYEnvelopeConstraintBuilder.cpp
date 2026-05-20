#include "offline_lc_minimal/core/Stage1OutageBodyYEnvelopeConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/BodyZLeakageCorrectedVelocityFactor.h"
#include "offline_lc_minimal/factor/FixedAxisBodyYVelocityEnvelopeFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

bool TimeInOutageEnvelope(
  const double time_s,
  const Stage1OutageBodyYEnvelopeRow &envelope) {
  return std::isfinite(time_s) &&
         time_s + kTimeEpsilonS >= envelope.outage_start_time_s &&
         time_s <= envelope.outage_end_time_s + kTimeEpsilonS;
}

gtsam::Vector3 BodyYAxisForReferenceState(const ReferenceNodeState &state) {
  return factor::BodyFrameAxesNavFromPose(state.pose).body_y_axis_nav;
}

void AccumulateSummary(
  const std::vector<Stage1OutageBodyYEnvelopeRow> &envelopes,
  RunSummary &summary) {
  summary.stage1_outage_body_y_envelope_count = envelopes.size();
  summary.stage1_outage_body_y_envelope_valid_count = 0U;
  summary.stage1_outage_body_y_mean_mps = std::numeric_limits<double>::quiet_NaN();
  summary.stage1_outage_body_y_rmse_mps = std::numeric_limits<double>::quiet_NaN();
  summary.stage1_outage_body_y_deadband_mps = std::numeric_limits<double>::quiet_NaN();
  for (const auto &envelope : envelopes) {
    if (!envelope.valid) {
      continue;
    }
    ++summary.stage1_outage_body_y_envelope_valid_count;
    if (!std::isfinite(summary.stage1_outage_body_y_rmse_mps)) {
      summary.stage1_outage_body_y_mean_mps = envelope.mean_body_y_mps;
      summary.stage1_outage_body_y_rmse_mps = envelope.rmse_body_y_mps;
      summary.stage1_outage_body_y_deadband_mps = envelope.deadband_mps;
    }
  }
}

}  // namespace

Stage1OutageBodyYEnvelopeConstraintBuilder::
  Stage1OutageBodyYEnvelopeConstraintBuilder(
    Stage1OutageBodyYEnvelopeConstraintBuildRequest request)
    : request_(std::move(request)) {}

void Stage1OutageBodyYEnvelopeConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.envelopes == nullptr || request_.state_diagnostics == nullptr) {
    throw std::runtime_error(
      "Stage1OutageBodyYEnvelopeConstraintBuilder received an incomplete request");
  }
  request_.run_summary->stage1_outage_body_y_envelope_enabled =
    request_.config->enable_stage1_outage_body_y_envelope;
  request_.envelopes->clear();
  request_.state_diagnostics->clear();
  if (!request_.config->enable_stage1_outage_body_y_envelope ||
      request_.reference == nullptr ||
      request_.reference->envelopes.empty()) {
    return;
  }
  if (request_.reference->reference_states.size() != request_.state_timestamps->size()) {
    throw std::runtime_error(
      "stage1 outage body-y envelope requires one fixed reference state per graph state");
  }

  *request_.envelopes = request_.reference->envelopes;
  AccumulateSummary(*request_.envelopes, *request_.run_summary);

  for (auto &envelope : *request_.envelopes) {
    if (!envelope.valid) {
      continue;
    }
    const auto base_noise =
      gtsam::noiseModel::Isotropic::Sigma(1, envelope.sigma_mps);
    const auto robust_noise =
      gtsam::noiseModel::Robust::Create(
        gtsam::noiseModel::mEstimator::Huber::Create(envelope.huber_k),
        base_noise);
    for (std::size_t state_index = request_.dynamic_start_index;
         state_index < request_.state_timestamps->size();
         ++state_index) {
      const double time_s = (*request_.state_timestamps)[state_index];
      if (!TimeInOutageEnvelope(time_s, envelope)) {
        continue;
      }
      const gtsam::Vector3 body_y_axis =
        BodyYAxisForReferenceState(request_.reference->reference_states[state_index]);
      request_.graph->add(factor::FixedAxisBodyYVelocityEnvelopeFactor(
        symbol::V(state_index),
        body_y_axis,
        envelope.mean_body_y_mps,
        envelope.deadband_mps,
        robust_noise));
      ++envelope.factor_count;
      ++request_.run_summary->stage1_outage_body_y_velocity_factor_count;

      Stage1OutageBodyYStateDiagnosticRow row;
      row.window_index = envelope.window_index;
      row.state_index = state_index;
      row.time_s = time_s;
      row.factor_added = true;
      row.skip_reason = "OK";
      row.mean_body_y_mps = envelope.mean_body_y_mps;
      row.rmse_body_y_mps = envelope.rmse_body_y_mps;
      row.deadband_mps = envelope.deadband_mps;
      row.sigma_mps = envelope.sigma_mps;
      row.body_y_axis_x = body_y_axis.x();
      row.body_y_axis_y = body_y_axis.y();
      row.body_y_axis_z = body_y_axis.z();
      request_.state_diagnostics->push_back(row);
    }
  }
}

void PopulateStage1OutageBodyYEnvelopeDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<Stage1OutageBodyYStateDiagnosticRow> &state_diagnostics,
  RunSummary &run_summary) {
  for (auto &row : state_diagnostics) {
    if (!row.factor_added) {
      continue;
    }
    const auto velocity = optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index));
    const gtsam::Vector3 body_y_axis(row.body_y_axis_x, row.body_y_axis_y, row.body_y_axis_z);
    row.optimized_vx_mps = velocity.x();
    row.optimized_vy_mps = velocity.y();
    row.optimized_vz_mps = velocity.z();
    row.optimized_body_y_mps = body_y_axis.dot(velocity);
    row.centered_body_y_mps = row.optimized_body_y_mps - row.mean_body_y_mps;
    row.deadband_residual_mps =
      factor::FixedAxisBodyYEnvelopeResidualMps(
        body_y_axis,
        velocity,
        row.mean_body_y_mps,
        row.deadband_mps);
    row.normalized_residual =
      row.sigma_mps > 0.0 ? row.deadband_residual_mps / row.sigma_mps :
                            std::numeric_limits<double>::quiet_NaN();
  }
  run_summary.stage1_outage_body_y_velocity_factor_count =
    std::count_if(
      state_diagnostics.begin(),
      state_diagnostics.end(),
      [](const Stage1OutageBodyYStateDiagnosticRow &row) {
        return row.factor_added;
      });
}

}  // namespace offline_lc_minimal
