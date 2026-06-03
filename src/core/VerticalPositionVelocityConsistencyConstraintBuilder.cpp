#include "offline_lc_minimal/core/VerticalPositionVelocityConsistencyConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalPositionVelocityConsistencyFactor.h"
#include "offline_lc_minimal/factor/VerticalPositionVelocityWindowConsistencyFactor.h"

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

bool HasWindowStateValues(const gtsam::Values &values, const std::size_t state_i, const std::size_t state_j) {
  if (!values.exists(symbol::X(state_i)) || !values.exists(symbol::X(state_j))) {
    return false;
  }
  for (std::size_t state_index = state_i; state_index <= state_j; ++state_index) {
    if (!values.exists(symbol::V(state_index))) {
      return false;
    }
  }
  return true;
}

void FillWindowMismatchFields(
  const gtsam::Values &values,
  const std::size_t state_i,
  const std::size_t state_j,
  const std::vector<double> &state_timestamps,
  double &delta_z_m,
  double &trapezoid_vz_integral_m,
  double &mismatch_m) {
  delta_z_m =
    values.at<gtsam::Pose3>(symbol::X(state_j)).translation().z() -
    values.at<gtsam::Pose3>(symbol::X(state_i)).translation().z();
  trapezoid_vz_integral_m = 0.0;
  for (std::size_t state_index = state_i + 1U; state_index <= state_j; ++state_index) {
    const std::size_t window_offset = state_index - state_i;
    const double dt_s = state_timestamps[window_offset] - state_timestamps[window_offset - 1U];
    trapezoid_vz_integral_m +=
      0.5 * dt_s *
      (values.at<gtsam::Vector3>(symbol::V(state_index - 1U)).z() +
       values.at<gtsam::Vector3>(symbol::V(state_index)).z());
  }
  mismatch_m = delta_z_m - trapezoid_vz_integral_m;
}

std::vector<gtsam::Key> BuildVelocityKeys(const std::size_t state_i, const std::size_t state_j) {
  std::vector<gtsam::Key> keys;
  keys.reserve(state_j - state_i + 1U);
  for (std::size_t state_index = state_i; state_index <= state_j; ++state_index) {
    keys.push_back(symbol::V(state_index));
  }
  return keys;
}

std::vector<double> SliceStateTimes(
  const std::vector<double> &state_timestamps,
  const std::size_t state_i,
  const std::size_t state_j) {
  return std::vector<double>(
    state_timestamps.begin() + static_cast<std::ptrdiff_t>(state_i),
    state_timestamps.begin() + static_cast<std::ptrdiff_t>(state_j + 1U));
}

std::size_t FindFirstStateAtOrAfter(
  const std::vector<double> &state_timestamps,
  const std::size_t start_index,
  const double target_time_s) {
  for (std::size_t state_index = start_index; state_index < state_timestamps.size(); ++state_index) {
    if (state_timestamps[state_index] + kTimeEpsilonS >= target_time_s) {
      return state_index;
    }
  }
  return state_timestamps.size();
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
  if (!request_.config->enable_vertical_position_velocity_consistency_all_states &&
      !request_.config->enable_vertical_position_velocity_window_consistency) {
    return;
  }

  BuildAdjacentFactors();
  BuildWindowFactors();
}

void VerticalPositionVelocityConsistencyConstraintBuilder::BuildAdjacentFactors() const {
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
    row.state_count = 2U;
    row.start_time_s = (*request_.state_timestamps)[state_i];
    row.end_time_s = (*request_.state_timestamps)[state_j];
    row.dt_s = row.end_time_s - row.start_time_s;
    row.state_times_s = {row.start_time_s, row.end_time_s};
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

void VerticalPositionVelocityConsistencyConstraintBuilder::BuildWindowFactors() const {
  if (!request_.config->enable_vertical_position_velocity_window_consistency) {
    return;
  }
  if (request_.state_timestamps->size() < 2U) {
    return;
  }

  const auto noise = gtsam::noiseModel::Isotropic::Sigma(
    1,
    request_.config->vertical_position_velocity_window_sigma_m);

  for (std::size_t state_i = 0; state_i + 1U < request_.state_timestamps->size();) {
    const double start_time_s = (*request_.state_timestamps)[state_i];
    const std::size_t state_j = FindFirstStateAtOrAfter(
      *request_.state_timestamps,
      state_i + 1U,
      start_time_s + request_.config->vertical_position_velocity_window_s);
    if (state_j >= request_.state_timestamps->size()) {
      break;
    }

    VerticalPositionVelocityConsistencyDiagnosticRow row;
    row.constraint_type = "window";
    row.state_index_i = state_i;
    row.state_index_j = state_j;
    row.state_count = state_j - state_i + 1U;
    row.start_time_s = start_time_s;
    row.end_time_s = (*request_.state_timestamps)[state_j];
    row.dt_s = row.end_time_s - row.start_time_s;
    row.state_times_s = SliceStateTimes(*request_.state_timestamps, state_i, state_j);
    row.interval_type = IntervalType(state_i, state_j, row.start_time_s, row.end_time_s);
    row.sigma_m = request_.config->vertical_position_velocity_window_sigma_m;

    if (!std::isfinite(row.dt_s) || row.dt_s <= 0.0) {
      row.skip_reason = "INVALID_DT";
      ++request_.run_summary->vertical_position_velocity_window_consistency_skipped_invalid_count;
      request_.diagnostics->push_back(row);
    } else if (!HasWindowStateValues(*request_.initial_values, state_i, state_j)) {
      row.skip_reason = "MISSING_INITIAL_VALUE";
      ++request_.run_summary->vertical_position_velocity_window_consistency_skipped_invalid_count;
      request_.diagnostics->push_back(row);
    } else {
      FillWindowMismatchFields(
        *request_.initial_values,
        state_i,
        state_j,
        row.state_times_s,
        row.initial_delta_z_m,
        row.initial_trapezoid_vz_integral_m,
        row.initial_mismatch_m);

      request_.graph->add(factor::VerticalPositionVelocityWindowConsistencyFactor(
        symbol::X(state_i),
        symbol::X(state_j),
        BuildVelocityKeys(state_i, state_j),
        row.state_times_s,
        noise));
      row.factor_added = true;
      row.skip_reason = "ADDED";
      ++request_.run_summary->vertical_position_velocity_consistency_factor_count;
      ++request_.run_summary->vertical_position_velocity_window_consistency_factor_count;
      request_.diagnostics->push_back(row);
    }

    const std::size_t next_state_i = FindFirstStateAtOrAfter(
      *request_.state_timestamps,
      state_i + 1U,
      start_time_s + request_.config->vertical_position_velocity_window_stride_s);
    state_i = next_state_i > state_i ? next_state_i : state_i + 1U;
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
  if (state_i < request_.dynamic_start_index &&
      state_j >= request_.dynamic_start_index) {
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
  bool has_window_max_mismatch = false;
  bool has_jump_padding_mismatch = false;
  double max_abs_mismatch_m = 0.0;
  double window_max_abs_mismatch_m = 0.0;
  double jump_padding_max_abs_mismatch_m = 0.0;
  for (auto &row : diagnostics) {
    if (!row.factor_added) {
      continue;
    }

    if (row.constraint_type == "window") {
      if (!HasWindowStateValues(optimized_values, row.state_index_i, row.state_index_j)) {
        continue;
      }
      if (row.state_times_s.size() != row.state_count) {
        continue;
      }
      FillWindowMismatchFields(
        optimized_values,
        row.state_index_i,
        row.state_index_j,
        row.state_times_s,
        row.optimized_delta_z_m,
        row.optimized_trapezoid_vz_integral_m,
        row.optimized_mismatch_m);
    } else {
      if (!HasStateValues(optimized_values, row.state_index_i, row.state_index_j)) {
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
    }
    row.normalized_residual =
      row.sigma_m > 0.0 ? row.optimized_mismatch_m / row.sigma_m : std::numeric_limits<double>::quiet_NaN();

    const double abs_mismatch_m = std::abs(row.optimized_mismatch_m);
    max_abs_mismatch_m = has_max_mismatch ? std::max(max_abs_mismatch_m, abs_mismatch_m) : abs_mismatch_m;
    has_max_mismatch = true;
    if (row.constraint_type == "window") {
      window_max_abs_mismatch_m = has_window_max_mismatch
        ? std::max(window_max_abs_mismatch_m, abs_mismatch_m)
        : abs_mismatch_m;
      has_window_max_mismatch = true;
    }

    if (row.constraint_type == "adjacent" &&
        row.interval_type == "static_dynamic_boundary") {
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
  if (has_window_max_mismatch) {
    run_summary.vertical_position_velocity_window_consistency_max_abs_mismatch_m =
      window_max_abs_mismatch_m;
  }
  if (has_jump_padding_mismatch) {
    run_summary.vertical_position_velocity_consistency_jump_padding_max_abs_mismatch_m =
      jump_padding_max_abs_mismatch_m;
  }
}

}  // namespace offline_lc_minimal
