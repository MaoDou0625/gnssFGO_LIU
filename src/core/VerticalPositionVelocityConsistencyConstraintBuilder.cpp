#include "offline_lc_minimal/core/VerticalPositionVelocityConsistencyConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalPositionVelocityConsistencyFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;

double JumpPaddingS(const OfflineRunnerConfig &config) {
  return std::max(
    {0.0,
     config.vertical_velocity_delta_jump_padding_s,
     config.body_z_nhc_jump_padding_s,
     config.vertical_jump_masked_imu_padding_s,
     config.vertical_jump_bias_padding_s});
}

bool Overlaps(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s + kTimeEpsilonS &&
         right_start_s <= left_end_s + kTimeEpsilonS;
}

void FillMismatchFields(
  const gtsam::Pose3 &pose_i,
  const gtsam::Vector3 &velocity_i,
  const gtsam::Pose3 &pose_j,
  const gtsam::Vector3 &velocity_j,
  const double dt_s,
  double &delta_z_m,
  double &trapezoid_vz_integral_m,
  double &mismatch_m) {
  delta_z_m = pose_j.translation().z() - pose_i.translation().z();
  trapezoid_vz_integral_m = 0.5 * dt_s * (velocity_i.z() + velocity_j.z());
  mismatch_m = delta_z_m - trapezoid_vz_integral_m;
}

bool HasStateValues(const gtsam::Values &values, const std::size_t state_i, const std::size_t state_j) {
  return values.exists(symbol::X(state_i)) &&
         values.exists(symbol::V(state_i)) &&
         values.exists(symbol::X(state_j)) &&
         values.exists(symbol::V(state_j));
}

}  // namespace

VerticalPositionVelocityConsistencyConstraintBuilder::
  VerticalPositionVelocityConsistencyConstraintBuilder(
    VerticalPositionVelocityConsistencyBuildRequest request)
    : request_(std::move(request)) {}

void VerticalPositionVelocityConsistencyConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.jump_windows == nullptr || request_.initial_values == nullptr ||
      request_.graph == nullptr || request_.run_summary == nullptr ||
      request_.diagnostics == nullptr) {
    throw std::runtime_error(
      "VerticalPositionVelocityConsistencyConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_vertical_position_velocity_consistency_all_states) {
    return;
  }

  request_.diagnostics->reserve(
    request_.diagnostics->size() +
    (request_.state_timestamps->size() > 0U ? request_.state_timestamps->size() - 1U : 0U));
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_position_velocity_consistency_sigma_m);

  for (std::size_t state_i = 0; state_i + 1U < request_.state_timestamps->size(); ++state_i) {
    const std::size_t state_j = state_i + 1U;
    VerticalPositionVelocityConsistencyDiagnosticRow row;
    row.state_index_i = state_i;
    row.state_index_j = state_j;
    row.start_time_s = (*request_.state_timestamps)[state_i];
    row.end_time_s = (*request_.state_timestamps)[state_j];
    row.dt_s = row.end_time_s - row.start_time_s;
    row.interval_type = IntervalType(state_i, state_j, row.start_time_s, row.end_time_s);
    row.sigma_m = request_.config->vertical_position_velocity_consistency_sigma_m;

    if (!std::isfinite(row.dt_s) || row.dt_s <= 0.0) {
      row.skip_reason = "INVALID_DT";
      ++request_.run_summary->vertical_position_velocity_consistency_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }
    if (!HasStateValues(*request_.initial_values, state_i, state_j)) {
      row.skip_reason = "MISSING_INITIAL_VALUE";
      ++request_.run_summary->vertical_position_velocity_consistency_skipped_invalid_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    FillMismatchFields(
      request_.initial_values->at<gtsam::Pose3>(symbol::X(state_i)),
      request_.initial_values->at<gtsam::Vector3>(symbol::V(state_i)),
      request_.initial_values->at<gtsam::Pose3>(symbol::X(state_j)),
      request_.initial_values->at<gtsam::Vector3>(symbol::V(state_j)),
      row.dt_s,
      row.initial_delta_z_m,
      row.initial_trapezoid_vz_integral_m,
      row.initial_mismatch_m);

    request_.graph->add(factor::VerticalPositionVelocityConsistencyFactor(
      symbol::X(state_i),
      symbol::V(state_i),
      symbol::X(state_j),
      symbol::V(state_j),
      row.dt_s,
      noise));
    row.factor_added = true;
    row.skip_reason = "ADDED";
    ++request_.run_summary->vertical_position_velocity_consistency_factor_count;
    request_.diagnostics->push_back(row);
  }
}

bool VerticalPositionVelocityConsistencyConstraintBuilder::OverlapsJumpPadding(
  const double start_time_s,
  const double end_time_s) const {
  const double padding_s = JumpPaddingS(*request_.config);
  for (const auto &window : *request_.jump_windows) {
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s)) {
      continue;
    }
    if (Overlaps(
          start_time_s,
          end_time_s,
          window.start_time_s - padding_s,
          window.end_time_s + padding_s)) {
      return true;
    }
  }
  return false;
}

std::string VerticalPositionVelocityConsistencyConstraintBuilder::IntervalType(
  const std::size_t state_i,
  const std::size_t state_j,
  const double start_time_s,
  const double end_time_s) const {
  if (state_i + 1U == request_.dynamic_start_index &&
      state_j == request_.dynamic_start_index) {
    return "static_dynamic_boundary";
  }
  if (OverlapsJumpPadding(start_time_s, end_time_s)) {
    return "jump_padding";
  }
  if (state_j < request_.dynamic_start_index) {
    return "static";
  }
  return "dynamic";
}

void PopulateVerticalPositionVelocityConsistencyDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<VerticalPositionVelocityConsistencyDiagnosticRow> &diagnostics,
  RunSummary &run_summary) {
  bool has_max_mismatch = false;
  bool has_jump_padding_mismatch = false;
  double max_abs_mismatch_m = 0.0;
  double jump_padding_max_abs_mismatch_m = 0.0;
  for (auto &row : diagnostics) {
    if (!row.factor_added ||
        !HasStateValues(optimized_values, row.state_index_i, row.state_index_j)) {
      continue;
    }

    FillMismatchFields(
      optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_i)),
      optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index_i)),
      optimized_values.at<gtsam::Pose3>(symbol::X(row.state_index_j)),
      optimized_values.at<gtsam::Vector3>(symbol::V(row.state_index_j)),
      row.dt_s,
      row.optimized_delta_z_m,
      row.optimized_trapezoid_vz_integral_m,
      row.optimized_mismatch_m);
    row.normalized_residual =
      row.sigma_m > 0.0 ? row.optimized_mismatch_m / row.sigma_m : std::numeric_limits<double>::quiet_NaN();

    const double abs_mismatch_m = std::abs(row.optimized_mismatch_m);
    max_abs_mismatch_m = has_max_mismatch ? std::max(max_abs_mismatch_m, abs_mismatch_m) : abs_mismatch_m;
    has_max_mismatch = true;

    if (row.interval_type == "static_dynamic_boundary") {
      run_summary.vertical_position_velocity_consistency_static_dynamic_boundary_mismatch_m =
        row.optimized_mismatch_m;
    } else if (row.interval_type == "jump_padding") {
      jump_padding_max_abs_mismatch_m = has_jump_padding_mismatch
        ? std::max(jump_padding_max_abs_mismatch_m, abs_mismatch_m)
        : abs_mismatch_m;
      has_jump_padding_mismatch = true;
    }
  }

  if (has_max_mismatch) {
    run_summary.vertical_position_velocity_consistency_max_abs_mismatch_m =
      max_abs_mismatch_m;
  }
  if (has_jump_padding_mismatch) {
    run_summary.vertical_position_velocity_consistency_jump_padding_max_abs_mismatch_m =
      jump_padding_max_abs_mismatch_m;
  }
}

}  // namespace offline_lc_minimal
