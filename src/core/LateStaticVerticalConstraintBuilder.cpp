#include "offline_lc_minimal/core/LateStaticVerticalConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/StaticVerticalPositionHoldFactor.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
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
  summary.late_static_window_count =
    static_cast<std::size_t>(
      std::count_if(
        windows.begin(),
        windows.end(),
        [](const LateStaticWindowRow &row) { return row.valid; }));
  summary.late_static_vz_factor_count = 0U;
  summary.late_static_up_factor_count = 0U;
  summary.late_static_height_hold_factor_count = 0U;
  for (const auto &window : windows) {
    summary.late_static_vz_factor_count += window.vz_factor_count;
    summary.late_static_up_factor_count += window.up_factor_count;
    summary.late_static_height_hold_factor_count += window.height_hold_factor_count;
  }
}

}  // namespace

LateStaticVerticalConstraintBuilder::LateStaticVerticalConstraintBuilder(
  LateStaticVerticalConstraintBuildRequest request)
    : request_(std::move(request)) {}

void LateStaticVerticalConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.windows == nullptr) {
    throw std::runtime_error("LateStaticVerticalConstraintBuilder received an incomplete request");
  }
  request_.run_summary->late_static_detection_enabled =
    request_.config->enable_late_static_detection;
  if (!request_.config->enable_late_static_detection || request_.windows->empty()) {
    return;
  }

  const auto vz_noise =
    gtsam::noiseModel::Isotropic::Sigma(1, request_.config->late_static_vz_sigma_mps);
  const auto up_noise =
    gtsam::noiseModel::Isotropic::Sigma(1, request_.config->late_static_up_sigma_m);
  const auto height_hold_noise =
    gtsam::noiseModel::Isotropic::Sigma(
      1,
      request_.config->late_static_height_hold_sigma_m);

  for (auto &window : *request_.windows) {
    window.vz_factor_count = 0U;
    window.up_factor_count = 0U;
    window.height_hold_factor_count = 0U;
    window.vz_factor_state_indices.clear();
    window.up_factor_state_indices.clear();
    window.height_hold_factor_state_index_pairs.clear();
    if (!window.valid || !std::isfinite(window.rtk_median_up_m)) {
      continue;
    }
    window.vz_sigma_mps = request_.config->late_static_vz_sigma_mps;
    window.up_sigma_m = request_.config->late_static_up_sigma_m;
    window.height_hold_sigma_m = request_.config->late_static_height_hold_sigma_m;
    std::size_t previous_static_state_index = std::numeric_limits<std::size_t>::max();
    for (std::size_t state_index = request_.dynamic_start_index;
         state_index < request_.state_timestamps->size();
         ++state_index) {
      const double time_s = (*request_.state_timestamps)[state_index];
      if (!TimeInWindow(time_s, window)) {
        continue;
      }
      request_.graph->add(
        factor::VerticalVelocityPriorFactor(
          symbol::V(state_index),
          0.0,
          vz_noise));
      window.vz_factor_state_indices.push_back(state_index);
      ++window.vz_factor_count;

      request_.graph->add(
        factor::VerticalPositionFactor(
          symbol::X(state_index),
          window.rtk_median_up_m,
          up_noise));
      window.up_factor_state_indices.push_back(state_index);
      ++window.up_factor_count;

      if (previous_static_state_index != std::numeric_limits<std::size_t>::max()) {
        request_.graph->add(
          factor::StaticVerticalPositionHoldFactor(
            symbol::X(previous_static_state_index),
            symbol::X(state_index),
            height_hold_noise));
        window.height_hold_factor_state_index_pairs.push_back(
          {previous_static_state_index, state_index});
        ++window.height_hold_factor_count;
      }
      previous_static_state_index = state_index;
    }
  }
  AccumulateSummary(*request_.windows, *request_.run_summary);
}

void PopulateLateStaticVerticalDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<LateStaticWindowRow> &windows,
  RunSummary &run_summary) {
  run_summary.late_static_window_count = 0U;
  run_summary.late_static_vz_factor_count = 0U;
  run_summary.late_static_up_factor_count = 0U;
  run_summary.late_static_height_hold_factor_count = 0U;
  for (auto &window : windows) {
    if (!window.valid) {
      continue;
    }
    ++run_summary.late_static_window_count;
    run_summary.late_static_vz_factor_count += window.vz_factor_count;
    run_summary.late_static_up_factor_count += window.up_factor_count;
    run_summary.late_static_height_hold_factor_count += window.height_hold_factor_count;

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

    double max_abs_up = 0.0;
    bool has_up = false;
    for (const std::size_t state_index : window.up_factor_state_indices) {
      const gtsam::Key key = symbol::X(state_index);
      if (!optimized_values.exists(key)) {
        continue;
      }
      const auto pose = optimized_values.at<gtsam::Pose3>(key);
      max_abs_up =
        std::max(max_abs_up, std::abs(pose.translation().z() - window.rtk_median_up_m));
      has_up = true;
    }
    window.max_abs_up_residual_m =
      has_up ? max_abs_up : std::numeric_limits<double>::quiet_NaN();

    double max_abs_height_hold = 0.0;
    bool has_height_hold = false;
    for (const auto &[state_i, state_j] : window.height_hold_factor_state_index_pairs) {
      const gtsam::Key key_i = symbol::X(state_i);
      const gtsam::Key key_j = symbol::X(state_j);
      if (!optimized_values.exists(key_i) || !optimized_values.exists(key_j)) {
        continue;
      }
      const auto pose_i = optimized_values.at<gtsam::Pose3>(key_i);
      const auto pose_j = optimized_values.at<gtsam::Pose3>(key_j);
      max_abs_height_hold =
        std::max(
          max_abs_height_hold,
          std::abs(pose_j.translation().z() - pose_i.translation().z()));
      has_height_hold = true;
    }
    window.max_abs_height_hold_residual_m =
      has_height_hold ? max_abs_height_hold : std::numeric_limits<double>::quiet_NaN();
  }
}

}  // namespace offline_lc_minimal
