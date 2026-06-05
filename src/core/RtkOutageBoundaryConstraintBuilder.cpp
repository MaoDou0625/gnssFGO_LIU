#include "offline_lc_minimal/core/RtkOutageBoundaryConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>

#include "offline_lc_minimal/common/Units.h"
#include "offline_lc_minimal/factor/AttitudeReferenceFactor.h"
#include "offline_lc_minimal/factor/AttitudeHoldFactor.h"
#include "offline_lc_minimal/factor/HorizontalHoldFactor.h"
#include "offline_lc_minimal/factor/VerticalAccelBiasPriorFactor.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
#include "offline_lc_minimal/factor/VerticalVelocityPriorFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

std::size_t BoundaryStateIndex(
  const std::vector<double> &state_timestamps,
  const RtkOutageBoundaryReferenceRow &reference) {
  if (reference.target_state_index >= 0 &&
      static_cast<std::size_t>(reference.target_state_index) < state_timestamps.size()) {
    return static_cast<std::size_t>(reference.target_state_index);
  }
  if (reference.boundary_role == "OUTAGE_START") {
    const auto lower = std::lower_bound(
      state_timestamps.begin(),
      state_timestamps.end(),
      reference.target_time_s);
    return lower == state_timestamps.end()
      ? state_timestamps.size() - 1U
      : static_cast<std::size_t>(lower - state_timestamps.begin());
  }
  if (reference.boundary_role == "OUTAGE_END" ||
      reference.boundary_role == "POST_START") {
    const auto upper = std::upper_bound(
      state_timestamps.begin(),
      state_timestamps.end(),
      reference.target_time_s);
    return upper == state_timestamps.begin()
      ? 0U
      : static_cast<std::size_t>((upper - state_timestamps.begin()) - 1);
  }
  const auto lower = std::lower_bound(
    state_timestamps.begin(),
    state_timestamps.end(),
    reference.target_time_s);
  if (lower == state_timestamps.begin()) {
    return 0U;
  }
  if (lower == state_timestamps.end()) {
    return state_timestamps.size() - 1U;
  }
  const std::size_t upper_index =
    static_cast<std::size_t>(lower - state_timestamps.begin());
  const std::size_t lower_index = upper_index - 1U;
  return std::abs(state_timestamps[lower_index] - reference.target_time_s) <=
             std::abs(state_timestamps[upper_index] - reference.target_time_s)
    ? lower_index
    : upper_index;
}

bool CanAddScalarFactor(
  const bool enabled,
  const bool has_value,
  const double value,
  const double sigma) {
  return enabled && has_value && std::isfinite(value) &&
         std::isfinite(sigma) && sigma > 0.0;
}

bool CanAddVector2Factor(
  const bool enabled,
  const bool has_value,
  const Eigen::Vector2d &value,
  const double sigma) {
  return enabled && has_value && value.allFinite() &&
         std::isfinite(sigma) && sigma > 0.0;
}

Eigen::Vector3d Rot3ToYpr(const gtsam::Rot3 &rotation) {
  const auto ypr = rotation.ypr();
  return Eigen::Vector3d(ypr.x(), ypr.y(), ypr.z());
}

gtsam::Vector3 NavZBody(const gtsam::Rot3 &rotation) {
  return rotation.unrotate(gtsam::Vector3::UnitZ());
}

bool CanAddAttitudeFactor(
  const RtkOutageBoundaryReferenceRow &reference) {
  return reference.add_attitude_constraint &&
         reference.has_attitude &&
         std::isfinite(reference.attitude_sigma_rad) &&
         reference.attitude_sigma_rad > 0.0 &&
         reference.reference_rotation.matrix().allFinite();
}

RtkOutageBoundaryDiagnosticRow MakeDiagnostic(
  const RtkOutageBoundaryReferenceRow &reference,
  const std::size_t state_index,
  const double state_time_s) {
  RtkOutageBoundaryDiagnosticRow row;
  row.window_index = reference.window_index;
  row.boundary_role = reference.boundary_role;
  row.source_type = reference.source_type;
  row.target_state_index = state_index;
  row.target_time_s = state_time_s;
  row.valid = reference.valid;
  row.reference_horizontal_position_m = reference.reference_horizontal_position_m;
  row.horizontal_position_sigma_m = reference.horizontal_position_sigma_m;
  row.reference_horizontal_velocity_mps = reference.reference_horizontal_velocity_mps;
  row.horizontal_velocity_sigma_mps = reference.horizontal_velocity_sigma_mps;
  row.reference_up_m = reference.reference_up_m;
  row.up_sigma_m = reference.up_sigma_m;
  row.reference_vz_mps = reference.reference_vz_mps;
  row.vz_sigma_mps = reference.vz_sigma_mps;
  row.reference_ba_z_ug = Mps2ToMicroG(reference.reference_ba_z_mps2);
  row.ba_z_sigma_ug = Mps2ToMicroG(reference.ba_z_sigma_mps2);
  if (reference.has_attitude) {
    row.reference_rotation = reference.reference_rotation;
    row.reference_ypr_rad = Rot3ToYpr(reference.reference_rotation);
  }
  row.attitude_sigma_rad = reference.attitude_sigma_rad;
  row.skip_reason = reference.skip_reason;
  return row;
}

}  // namespace

RtkOutageBoundaryConstraintBuilder::RtkOutageBoundaryConstraintBuilder(
  RtkOutageBoundaryConstraintBuildRequest request)
    : request_(std::move(request)) {}

void RtkOutageBoundaryConstraintBuilder::Build() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.boundary_references == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr) {
    throw std::runtime_error(
      "RtkOutageBoundaryConstraintBuilder received an incomplete request");
  }
  if (!request_.config->enable_rtk_outage_boundary_constraints ||
      request_.boundary_references->empty()) {
    return;
  }
  if (request_.state_timestamps->empty()) {
    throw std::runtime_error("RTK outage boundary constraints require graph timestamps");
  }
  const bool use_base_tilt_reference =
    request_.config->enable_base_graph_tilt_reference_constraint;
  const bool needs_boundary_tilt_reference = std::any_of(
    request_.boundary_references->begin(),
    request_.boundary_references->end(),
    [](const RtkOutageBoundaryReferenceRow &reference) {
      return reference.valid && CanAddAttitudeFactor(reference);
    });
  if (use_base_tilt_reference && needs_boundary_tilt_reference) {
    if (request_.tilt_reference_states == nullptr ||
        request_.tilt_reference_states->empty()) {
      throw std::runtime_error(
        "RTK outage boundary tilt constraint requires optimized base graph reference states");
    }
    if (request_.tilt_reference_states->size() != request_.state_timestamps->size()) {
      throw std::runtime_error(
        "RTK outage boundary tilt reference state count does not match graph state count");
    }
  }

  request_.run_summary->rtk_outage_boundary_constraints_enabled = true;
  request_.run_summary->rtk_outage_boundary_reference_count +=
    request_.boundary_references->size();

  for (const auto &reference : *request_.boundary_references) {
    const std::size_t state_index =
      BoundaryStateIndex(*request_.state_timestamps, reference);
    RtkOutageBoundaryDiagnosticRow diagnostic =
      MakeDiagnostic(reference, state_index, (*request_.state_timestamps)[state_index]);
    if (!reference.valid) {
      if (diagnostic.skip_reason == "UNSET") {
        diagnostic.skip_reason = "invalid_reference";
      }
      request_.diagnostics->push_back(diagnostic);
      continue;
    }

    const bool add_up = CanAddScalarFactor(
      reference.add_up_constraint,
      reference.has_up,
      reference.reference_up_m,
      reference.up_sigma_m);
    if (add_up) {
      request_.graph->add(factor::VerticalPositionFactor(
        symbol::X(state_index),
        reference.reference_up_m,
        gtsam::noiseModel::Isotropic::Sigma(1, reference.up_sigma_m)));
      diagnostic.up_factor_added = true;
      ++request_.run_summary->rtk_outage_boundary_up_factor_count;
    }

    const bool add_vz = CanAddScalarFactor(
      reference.add_vz_constraint,
      reference.has_vz,
      reference.reference_vz_mps,
      reference.vz_sigma_mps);
    if (add_vz) {
      request_.graph->add(factor::VerticalVelocityPriorFactor(
        symbol::V(state_index),
        reference.reference_vz_mps,
        gtsam::noiseModel::Isotropic::Sigma(1, reference.vz_sigma_mps)));
      diagnostic.vz_factor_added = true;
      ++request_.run_summary->rtk_outage_boundary_vz_factor_count;
    }

    const bool add_ba_z = CanAddScalarFactor(
      reference.add_ba_z_constraint,
      reference.has_ba_z,
      reference.reference_ba_z_mps2,
      reference.ba_z_sigma_mps2);
    if (add_ba_z) {
      request_.graph->add(factor::VerticalAccelBiasPriorFactor(
        symbol::B(state_index),
        reference.reference_ba_z_mps2,
        gtsam::noiseModel::Isotropic::Sigma(1, reference.ba_z_sigma_mps2)));
      diagnostic.ba_z_factor_added = true;
      ++request_.run_summary->rtk_outage_boundary_baz_factor_count;
    }

    const bool add_horizontal_position = CanAddVector2Factor(
      reference.add_horizontal_position_constraint,
      reference.has_horizontal_position,
      reference.reference_horizontal_position_m,
      reference.horizontal_position_sigma_m);
    if (add_horizontal_position) {
      request_.graph->add(factor::HorizontalPositionHoldFactor(
        symbol::X(state_index),
        reference.reference_horizontal_position_m,
        gtsam::noiseModel::Isotropic::Sigma(2, reference.horizontal_position_sigma_m)));
      diagnostic.horizontal_position_factor_added = true;
      ++request_.run_summary->rtk_outage_boundary_horizontal_position_factor_count;
    }

    const bool add_horizontal_velocity = CanAddVector2Factor(
      reference.add_horizontal_velocity_constraint,
      reference.has_horizontal_velocity,
      reference.reference_horizontal_velocity_mps,
      reference.horizontal_velocity_sigma_mps);
    if (add_horizontal_velocity) {
      request_.graph->add(factor::HorizontalVelocityHoldFactor(
        symbol::V(state_index),
        reference.reference_horizontal_velocity_mps,
        gtsam::noiseModel::Isotropic::Sigma(2, reference.horizontal_velocity_sigma_mps)));
      diagnostic.horizontal_velocity_factor_added = true;
      ++request_.run_summary->rtk_outage_boundary_horizontal_velocity_factor_count;
    }

    const bool add_attitude = CanAddAttitudeFactor(reference);
    if (add_attitude) {
      if (use_base_tilt_reference) {
        const auto &tilt_reference_state = (*request_.tilt_reference_states)[state_index];
        request_.graph->add(factor::TiltReferenceFactor(
          symbol::X(state_index),
          tilt_reference_state.pose.rotation(),
          gtsam::noiseModel::Isotropic::Sigma(
            2,
            request_.config->base_graph_tilt_reference_sigma_rad)));
        diagnostic.attitude_constraint_type = "tilt";
        diagnostic.reference_rotation = tilt_reference_state.pose.rotation();
        diagnostic.reference_ypr_rad = Rot3ToYpr(tilt_reference_state.pose.rotation());
        diagnostic.attitude_sigma_rad =
          request_.config->base_graph_tilt_reference_sigma_rad;
      } else {
        request_.graph->add(factor::AttitudeHoldFactor(
          symbol::X(state_index),
          reference.reference_rotation,
          gtsam::noiseModel::Isotropic::Sigma(3, reference.attitude_sigma_rad)));
        diagnostic.attitude_constraint_type = "absolute";
      }
      diagnostic.attitude_factor_added = true;
      ++request_.run_summary->rtk_outage_boundary_attitude_factor_count;
    }

    if (add_up || add_vz || add_ba_z || add_horizontal_position ||
        add_horizontal_velocity || add_attitude) {
      diagnostic.skip_reason = "ADDED";
    } else if (diagnostic.skip_reason == "UNSET" || diagnostic.skip_reason == "OK") {
      diagnostic.skip_reason = "no_enabled_boundary_reference";
    }
    request_.diagnostics->push_back(diagnostic);
  }
}

void PopulateRtkOutageBoundaryDiagnostics(
  const gtsam::Values &optimized_values,
  std::vector<RtkOutageBoundaryDiagnosticRow> &diagnostics) {
  for (auto &row : diagnostics) {
    if (!row.valid) {
      continue;
    }
    if (row.up_factor_added) {
      const auto pose = optimized_values.at<gtsam::Pose3>(symbol::X(row.target_state_index));
      row.optimized_up_m = pose.translation().z();
      row.up_residual_m = row.optimized_up_m - row.reference_up_m;
    }
    if (row.horizontal_position_factor_added) {
      const auto pose = optimized_values.at<gtsam::Pose3>(symbol::X(row.target_state_index));
      row.optimized_horizontal_position_m =
        Eigen::Vector2d(pose.translation().x(), pose.translation().y());
      row.horizontal_position_residual_m =
        row.optimized_horizontal_position_m - row.reference_horizontal_position_m;
      row.horizontal_position_residual_norm_m =
        row.horizontal_position_residual_m.norm();
    }
    if (row.horizontal_velocity_factor_added) {
      const auto velocity =
        optimized_values.at<gtsam::Vector3>(symbol::V(row.target_state_index));
      row.optimized_horizontal_velocity_mps =
        Eigen::Vector2d(velocity.x(), velocity.y());
      row.horizontal_velocity_residual_mps =
        row.optimized_horizontal_velocity_mps - row.reference_horizontal_velocity_mps;
      row.horizontal_velocity_residual_norm_mps =
        row.horizontal_velocity_residual_mps.norm();
    }
    if (row.vz_factor_added) {
      const auto velocity =
        optimized_values.at<gtsam::Vector3>(symbol::V(row.target_state_index));
      row.optimized_vz_mps = velocity.z();
      row.vz_residual_mps = row.optimized_vz_mps - row.reference_vz_mps;
    }
    if (row.ba_z_factor_added) {
      const auto bias = optimized_values.at<gtsam::imuBias::ConstantBias>(
        symbol::B(row.target_state_index));
      row.optimized_ba_z_ug = Mps2ToMicroG(bias.accelerometer().z());
      row.ba_z_residual_ug = row.optimized_ba_z_ug - row.reference_ba_z_ug;
    }
    if (row.attitude_factor_added) {
      const auto pose = optimized_values.at<gtsam::Pose3>(symbol::X(row.target_state_index));
      row.optimized_ypr_rad = Rot3ToYpr(pose.rotation());
      if (row.attitude_constraint_type == "tilt") {
        const gtsam::Vector3 residual =
          NavZBody(pose.rotation()) - NavZBody(row.reference_rotation);
        row.attitude_residual_rad = Eigen::Vector3d(
          residual.x(),
          residual.y(),
          std::numeric_limits<double>::quiet_NaN());
        row.attitude_residual_norm_rad = std::hypot(residual.x(), residual.y());
      } else {
        row.attitude_residual_rad =
          gtsam::Rot3::Logmap(row.reference_rotation.between(pose.rotation()));
        row.attitude_residual_norm_rad = row.attitude_residual_rad.norm();
      }
    }
  }
}

}  // namespace offline_lc_minimal
