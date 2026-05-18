#include "offline_lc_minimal/core/RtkOutagePreOutageVerticalFenceBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
#include "offline_lc_minimal/factor/VerticalVelocityPriorFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

bool ShouldAddFenceAtTime(
  const double time_s,
  const double last_added_time_s,
  const double stride_s) {
  return !std::isfinite(last_added_time_s) ||
         time_s >= last_added_time_s + stride_s - kTimeEpsilonS;
}

}  // namespace

RtkOutagePreOutageVerticalFenceBuilder::RtkOutagePreOutageVerticalFenceBuilder(
  RtkOutagePreOutageVerticalFenceBuildRequest request)
    : request_(std::move(request)) {}

void RtkOutagePreOutageVerticalFenceBuilder::Build() const {
  if (request_.config == nullptr || request_.state_references == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr) {
    throw std::runtime_error("RtkOutagePreOutageVerticalFenceBuilder received an incomplete request");
  }
  request_.run_summary->rtk_outage_preoutage_vertical_fence_enabled =
    request_.config->enable_rtk_outage_preoutage_vertical_fence;
  if (!request_.config->enable_rtk_outage_preoutage_vertical_fence ||
      request_.state_references->empty()) {
    return;
  }

  const auto up_noise =
    gtsam::noiseModel::Isotropic::Sigma(1, request_.config->rtk_outage_preoutage_fence_up_sigma_m);
  const auto vz_noise =
    gtsam::noiseModel::Isotropic::Sigma(1, request_.config->rtk_outage_preoutage_fence_vz_sigma_mps);
  double last_added_time_s = std::numeric_limits<double>::quiet_NaN();
  std::size_t last_valid_index = request_.state_references->size();

  for (std::size_t row_index = 0; row_index < request_.state_references->size(); ++row_index) {
    auto &row = (*request_.state_references)[row_index];
    row.fence_factor_added = false;
    if (!row.valid ||
        !std::isfinite(row.time_s) ||
        !std::isfinite(row.reference_up_m) ||
        !std::isfinite(row.reference_vz_mps)) {
      continue;
    }
    last_valid_index = row_index;
    if (!ShouldAddFenceAtTime(
          row.time_s,
          last_added_time_s,
          request_.config->rtk_outage_preoutage_fence_stride_s)) {
      continue;
    }
    request_.graph->add(factor::VerticalPositionFactor(
      symbol::X(row.state_index),
      row.reference_up_m,
      up_noise));
    request_.graph->add(factor::VerticalVelocityPriorFactor(
      symbol::V(row.state_index),
      row.reference_vz_mps,
      vz_noise));
    row.fence_factor_added = true;
    last_added_time_s = row.time_s;
    request_.run_summary->rtk_outage_preoutage_vertical_fence_factor_count += 2U;
  }

  if (last_valid_index < request_.state_references->size() &&
      !(*request_.state_references)[last_valid_index].fence_factor_added) {
    auto &row = (*request_.state_references)[last_valid_index];
    request_.graph->add(factor::VerticalPositionFactor(
      symbol::X(row.state_index),
      row.reference_up_m,
      up_noise));
    request_.graph->add(factor::VerticalVelocityPriorFactor(
      symbol::V(row.state_index),
      row.reference_vz_mps,
      vz_noise));
    row.fence_factor_added = true;
    request_.run_summary->rtk_outage_preoutage_vertical_fence_factor_count += 2U;
  }
}

void PopulateRtkOutagePreOutageVerticalFenceSummary(
  const gtsam::Values &optimized_values,
  const std::vector<RtkOutageCausalStateReferenceRow> &state_references,
  RunSummary &run_summary) {
  double max_abs_up_delta_m = 0.0;
  double max_abs_vz_delta_mps = 0.0;
  bool has_delta = false;
  for (const auto &row : state_references) {
    if (!row.valid || !row.fence_factor_added) {
      continue;
    }
    const gtsam::Key x_key = symbol::X(row.state_index);
    const gtsam::Key v_key = symbol::V(row.state_index);
    if (!optimized_values.exists(x_key) || !optimized_values.exists(v_key)) {
      continue;
    }
    const double up_delta_m =
      optimized_values.at<gtsam::Pose3>(x_key).translation().z() - row.reference_up_m;
    const double vz_delta_mps =
      optimized_values.at<gtsam::Vector3>(v_key).z() - row.reference_vz_mps;
    max_abs_up_delta_m = std::max(max_abs_up_delta_m, std::abs(up_delta_m));
    max_abs_vz_delta_mps = std::max(max_abs_vz_delta_mps, std::abs(vz_delta_mps));
    has_delta = true;
  }
  run_summary.rtk_outage_preoutage_vertical_fence_max_delta_m =
    has_delta ? max_abs_up_delta_m : std::numeric_limits<double>::quiet_NaN();
  run_summary.rtk_outage_preoutage_vertical_fence_max_vz_delta_mps =
    has_delta ? max_abs_vz_delta_mps : std::numeric_limits<double>::quiet_NaN();
}

}  // namespace offline_lc_minimal
