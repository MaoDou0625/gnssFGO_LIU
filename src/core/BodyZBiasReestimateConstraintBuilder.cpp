#include "offline_lc_minimal/core/BodyZBiasReestimateConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

#include <boost/make_shared.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/BazContinuityBreakCombinedImuFactor.h"
#include "offline_lc_minimal/factor/VerticalAccelBiasPriorFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

constexpr double kTimeEpsilonS = 1.0e-9;
constexpr double kBoundaryTimeToleranceS = 1.0e-6;

bool ContainsTime(
  const BodyZBiasReestimateSegmentRow &segment,
  const double time_s) {
  return std::isfinite(time_s) &&
         time_s + kTimeEpsilonS >= segment.start_time_s &&
         time_s <= segment.end_time_s + kTimeEpsilonS;
}

gtsam::imuBias::ConstantBias WithUpdatedBaz(
  const gtsam::imuBias::ConstantBias &bias,
  const double ba_z_mps2) {
  gtsam::Vector3 accelerometer = bias.accelerometer();
  accelerometer.z() = ba_z_mps2;
  return gtsam::imuBias::ConstantBias(accelerometer, bias.gyroscope());
}

bool IsRtkOutageLinked(const BodyZBiasReestimateSegmentRow &segment) {
  return segment.source_outage_window_index >= 0 ||
         segment.source_type == "RTK_OUTAGE";
}

}  // namespace

BodyZBiasReestimateConstraintBuilder::BodyZBiasReestimateConstraintBuilder(
  BodyZBiasReestimateConstraintBuildRequest request)
    : request_(std::move(request)) {}

void BodyZBiasReestimateConstraintBuilder::Apply() const {
  if (request_.config == nullptr || request_.state_timestamps == nullptr ||
      request_.segments == nullptr || request_.imu_intervals == nullptr ||
      request_.graph == nullptr || request_.initial_values == nullptr ||
      request_.run_summary == nullptr) {
    throw std::runtime_error("BodyZBiasReestimateConstraintBuilder received an incomplete request");
  }
  if (request_.segments->empty()) {
    return;
  }

  request_.run_summary->body_z_bias_reestimate_segment_count = request_.segments->size();
  request_.run_summary->rtk_outage_baz_reestimate_enabled =
    request_.config->enable_rtk_outage_smoothing &&
    request_.config->enable_rtk_outage_baz_reestimate;
  request_.run_summary->rtk_outage_baz_reestimate_segment_count =
    static_cast<std::size_t>(std::count_if(
      request_.segments->begin(),
      request_.segments->end(),
      [](const BodyZBiasReestimateSegmentRow &segment) {
        return IsRtkOutageLinked(segment);
      }));
  for (auto &segment : *request_.segments) {
    segment.start_state_index = -1;
    segment.end_state_index = -1;
    segment.anchor_state_index = -1;
    segment.reference_ba_z_mps2 = std::numeric_limits<double>::quiet_NaN();
    segment.prior_target_ba_z_mps2 = std::numeric_limits<double>::quiet_NaN();
    segment.prior_sigma_mps2 = std::numeric_limits<double>::quiet_NaN();
    segment.initialized_state_count = 0;
    segment.prior_factor_added = false;
    segment.skip_reason = "UNSET";

    const std::vector<std::size_t> state_indices = StateIndicesInSegment(segment);
    if (state_indices.empty()) {
      segment.skip_reason = "NO_STATES";
      continue;
    }
    segment.start_state_index = static_cast<long long>(state_indices.front());
    segment.end_state_index = static_cast<long long>(state_indices.back());
    segment.anchor_state_index = static_cast<long long>(state_indices.front());

    if (!std::isfinite(segment.detected_bias_delta_mps2)) {
      segment.skip_reason = "MISSING_DETECTED_BIAS";
      continue;
    }

    const gtsam::Key anchor_key = symbol::B(state_indices.front());
    const auto anchor_bias = request_.initial_values->at<gtsam::imuBias::ConstantBias>(anchor_key);
    const RtkOutageBoundaryReferenceRow *boundary_ba_z_reference =
      PostStartBazReferenceForSegment(segment);
    segment.prior_sigma_mps2 =
      boundary_ba_z_reference != nullptr &&
      std::isfinite(boundary_ba_z_reference->ba_z_sigma_mps2) &&
      boundary_ba_z_reference->ba_z_sigma_mps2 > 0.0
        ? boundary_ba_z_reference->ba_z_sigma_mps2
        : request_.config->vertical_jump_bias_prior_sigma_mps2;
    segment.reference_ba_z_mps2 =
      boundary_ba_z_reference != nullptr
        ? boundary_ba_z_reference->reference_ba_z_mps2
        : anchor_bias.accelerometer().z();
    segment.prior_target_ba_z_mps2 =
      segment.reference_ba_z_mps2 +
      (boundary_ba_z_reference != nullptr ? 0.0 : segment.detected_bias_delta_mps2);

    for (const std::size_t state_index : state_indices) {
      const gtsam::Key bias_key = symbol::B(state_index);
      const auto current_bias = request_.initial_values->at<gtsam::imuBias::ConstantBias>(bias_key);
      request_.initial_values->update(
        bias_key,
        WithUpdatedBaz(current_bias, segment.prior_target_ba_z_mps2));
      ++segment.initialized_state_count;
      ++request_.run_summary->body_z_bias_reestimate_initialized_state_count;
    }

    const auto prior_noise = gtsam::noiseModel::Isotropic::Sigma(
      1,
      segment.prior_sigma_mps2);
    request_.graph->add(factor::VerticalAccelBiasPriorFactor(
      anchor_key,
      segment.prior_target_ba_z_mps2,
      prior_noise));
    segment.prior_factor_added = true;
    segment.skip_reason = "ADDED";
    ++request_.run_summary->body_z_bias_reestimate_prior_factor_count;
    if (IsRtkOutageLinked(segment)) {
      ++request_.run_summary->rtk_outage_baz_reestimate_prior_factor_count;
    }
  }

  std::set<std::size_t> replaced_factor_indices;
  for (const auto &interval : *request_.imu_intervals) {
    if (!CrossesReestimateBoundary(interval) ||
        interval.graph_factor_index >= request_.graph->size() ||
        !replaced_factor_indices.insert(interval.graph_factor_index).second) {
      continue;
    }
    request_.graph->replace(
      interval.graph_factor_index,
      boost::make_shared<factor::BazContinuityBreakCombinedImuFactor>(
        symbol::X(interval.state_index_i),
        symbol::V(interval.state_index_i),
        symbol::X(interval.state_index_j),
        symbol::V(interval.state_index_j),
        symbol::B(interval.state_index_i),
        symbol::B(interval.state_index_j),
        interval.preintegrated_measurements));
    ++request_.run_summary->body_z_bias_reestimate_boundary_break_count;
    if (BoundaryTouchesRtkOutageSegment(interval)) {
      ++request_.run_summary->rtk_outage_baz_reestimate_boundary_break_count;
    }
  }
}

std::vector<std::size_t> BodyZBiasReestimateConstraintBuilder::StateIndicesInSegment(
  const BodyZBiasReestimateSegmentRow &segment) const {
  std::vector<std::size_t> indices;
  for (std::size_t index = 0; index < request_.state_timestamps->size(); ++index) {
    if (ContainsTime(segment, (*request_.state_timestamps)[index])) {
      indices.push_back(index);
    }
  }
  return indices;
}

long long BodyZBiasReestimateConstraintBuilder::SegmentIndexForState(
  const std::size_t state_index) const {
  const BodyZBiasReestimateSegmentRow *segment = SegmentForState(state_index);
  if (segment == nullptr) {
    return -1;
  }
  return static_cast<long long>(segment->segment_index);
}

const BodyZBiasReestimateSegmentRow *BodyZBiasReestimateConstraintBuilder::SegmentForState(
  const std::size_t state_index) const {
  if (state_index >= request_.state_timestamps->size()) {
    return nullptr;
  }
  const double time_s = (*request_.state_timestamps)[state_index];
  for (const auto &segment : *request_.segments) {
    if (ContainsTime(segment, time_s)) {
      return &segment;
    }
  }
  return nullptr;
}

const RtkOutageBoundaryReferenceRow *
BodyZBiasReestimateConstraintBuilder::PostStartBazReferenceForSegment(
  const BodyZBiasReestimateSegmentRow &segment) const {
  if (request_.boundary_references == nullptr ||
      !std::isfinite(segment.start_time_s)) {
    return nullptr;
  }
  for (const auto &reference : *request_.boundary_references) {
    if (reference.boundary_role != "POST_START" ||
        !reference.has_ba_z ||
        !reference.add_ba_z_constraint ||
        !std::isfinite(reference.target_time_s) ||
        !std::isfinite(reference.reference_ba_z_mps2)) {
      continue;
    }
    if (std::abs(reference.target_time_s - segment.start_time_s) <=
        kBoundaryTimeToleranceS) {
      return &reference;
    }
  }
  return nullptr;
}

bool BodyZBiasReestimateConstraintBuilder::CrossesReestimateBoundary(
  const VerticalJumpImuIntervalRecord &interval) const {
  const long long left_segment_index = SegmentIndexForState(interval.state_index_i);
  const long long right_segment_index = SegmentIndexForState(interval.state_index_j);
  return left_segment_index != right_segment_index &&
         (left_segment_index >= 0 || right_segment_index >= 0);
}

bool BodyZBiasReestimateConstraintBuilder::BoundaryTouchesRtkOutageSegment(
  const VerticalJumpImuIntervalRecord &interval) const {
  const BodyZBiasReestimateSegmentRow *left_segment = SegmentForState(interval.state_index_i);
  const BodyZBiasReestimateSegmentRow *right_segment = SegmentForState(interval.state_index_j);
  if (left_segment == right_segment) {
    return false;
  }
  return (left_segment != nullptr && IsRtkOutageLinked(*left_segment)) ||
         (right_segment != nullptr && IsRtkOutageLinked(*right_segment));
}

}  // namespace offline_lc_minimal
