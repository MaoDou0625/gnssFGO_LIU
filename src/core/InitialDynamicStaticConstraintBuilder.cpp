#include "offline_lc_minimal/core/InitialDynamicStaticConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalVelocityPriorFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

bool TimeInWindow(const double time_s, const LateStaticWindowRow &window) {
  return std::isfinite(time_s) &&
         time_s + kTimeEpsilonS >= window.start_time_s &&
         time_s <= window.end_time_s + kTimeEpsilonS;
}

void AccumulateSummary(
  const std::vector<LateStaticWindowRow> &windows,
  RunSummary &summary) {
  summary.initial_dynamic_static_window_count =
    static_cast<std::size_t>(
      std::count_if(
        windows.begin(),
        windows.end(),
        [](const LateStaticWindowRow &row) { return row.valid; }));
  summary.initial_dynamic_static_vz_factor_count = 0U;
  for (const auto &window : windows) {
    summary.initial_dynamic_static_vz_factor_count += window.vz_factor_count;
  }
}

}  // namespace

InitialDynamicStaticConstraintBuilder::InitialDynamicStaticConstraintBuilder(
  InitialDynamicStaticConstraintBuildRequest request)
    : request_(std::move(request)) {}

void InitialDynamicStaticConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.windows == nullptr) {
    throw std::runtime_error(
      "InitialDynamicStaticConstraintBuilder received an incomplete request");
  }
  request_.run_summary->initial_dynamic_static_detection_enabled =
    request_.config->enable_initial_dynamic_static_detection;
  request_.run_summary->initial_dynamic_static_vz_constraint_enabled =
    request_.config->enable_initial_dynamic_static_vz_constraint;
  if (!request_.config->enable_initial_dynamic_static_vz_constraint ||
      request_.windows->empty()) {
    AccumulateSummary(*request_.windows, *request_.run_summary);
    return;
  }

  const auto vz_noise =
    gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->initial_dynamic_static_vz_sigma_mps);
  for (auto &window : *request_.windows) {
    window.vz_factor_count = 0U;
    window.up_factor_count = 0U;
    window.height_hold_factor_count = 0U;
    window.vz_factor_state_indices.clear();
    window.up_factor_state_indices.clear();
    window.height_hold_factor_state_index_pairs.clear();
    window.vz_sigma_mps = request_.config->initial_dynamic_static_vz_sigma_mps;
    window.up_sigma_m = std::numeric_limits<double>::quiet_NaN();
    window.height_hold_sigma_m = std::numeric_limits<double>::quiet_NaN();
    if (!window.valid) {
      continue;
    }
    for (std::size_t state_index = request_.dynamic_start_index;
         state_index < request_.state_timestamps->size();
         ++state_index) {
      if (!TimeInWindow((*request_.state_timestamps)[state_index], window)) {
        continue;
      }
      request_.graph->add(
        factor::VerticalVelocityPriorFactor(
          symbol::V(state_index),
          0.0,
          vz_noise));
      window.vz_factor_state_indices.push_back(state_index);
      ++window.vz_factor_count;
    }
  }
  AccumulateSummary(*request_.windows, *request_.run_summary);
}

void PopulateInitialDynamicStaticDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<LateStaticWindowRow> &windows,
  RunSummary &run_summary) {
  run_summary.initial_dynamic_static_window_count = 0U;
  run_summary.initial_dynamic_static_vz_factor_count = 0U;
  for (auto &window : windows) {
    if (!window.valid) {
      continue;
    }
    ++run_summary.initial_dynamic_static_window_count;
    run_summary.initial_dynamic_static_vz_factor_count += window.vz_factor_count;
    double max_abs_vz = 0.0;
    bool has_vz = false;
    for (const std::size_t state_index : window.vz_factor_state_indices) {
      const gtsam::Key key = symbol::V(state_index);
      if (!optimized_values.exists(key)) {
        continue;
      }
      const auto velocity = optimized_values.at<gtsam::Vector3>(key);
      max_abs_vz = std::max(max_abs_vz, std::abs(velocity.z()));
      has_vz = true;
    }
    window.max_abs_vz_residual_mps =
      has_vz ? max_abs_vz : std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace offline_lc_minimal
