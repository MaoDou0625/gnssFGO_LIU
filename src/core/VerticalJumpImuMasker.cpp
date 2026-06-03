#include "offline_lc_minimal/core/VerticalJumpImuMasker.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include <boost/make_shared.hpp>
#include <gtsam/inference/Symbol.h>

#include "offline_lc_minimal/factor/VerticalMaskedCombinedImuFactor.h"

namespace offline_lc_minimal {
namespace {

namespace symbol = gtsam::symbol_shorthand;

bool IntervalsOverlap(
  const double left_start_s,
  const double left_end_s,
  const double right_start_s,
  const double right_end_s) {
  return left_start_s <= right_end_s && right_start_s <= left_end_s;
}

}  // namespace

VerticalJumpImuMasker::VerticalJumpImuMasker(VerticalJumpImuMaskRequest request)
    : request_(std::move(request)) {}

void VerticalJumpImuMasker::Apply() const {
  if (request_.config == nullptr || request_.intervals == nullptr ||
      request_.jump_windows == nullptr || request_.graph == nullptr ||
      request_.run_summary == nullptr || request_.diagnostics == nullptr) {
    throw std::runtime_error("VerticalJumpImuMasker received an incomplete request");
  }

  request_.diagnostics->reserve(request_.diagnostics->size() + request_.intervals->size());
  for (const auto &interval : *request_.intervals) {
    VerticalJumpMaskedImuDiagnosticRow row;
    row.state_index_i = interval.state_index_i;
    row.state_index_j = interval.state_index_j;
    row.start_time_s = interval.start_time_s;
    row.end_time_s = interval.end_time_s;
    row.overlap_jump_padding = OverlapsJumpPadding(interval.start_time_s, interval.end_time_s);

    if (!request_.config->enable_vertical_jump_masked_imu || !row.overlap_jump_padding) {
      row.factor_type = "combined";
      ++request_.run_summary->vertical_jump_combined_imu_factor_count;
      request_.diagnostics->push_back(row);
      continue;
    }

    if (interval.graph_factor_index >= request_.graph->size()) {
      row.factor_type = "invalid_factor_index";
      request_.diagnostics->push_back(row);
      continue;
    }

    request_.graph->replace(
      interval.graph_factor_index,
      boost::make_shared<factor::VerticalMaskedCombinedImuFactor>(
        symbol::X(interval.state_index_i),
        symbol::V(interval.state_index_i),
        symbol::X(interval.state_index_j),
        symbol::V(interval.state_index_j),
        symbol::B(interval.state_index_i),
        symbol::B(interval.state_index_j),
        interval.preintegrated_measurements));
    row.factor_type = "vertical_masked";
    row.masked_z_position = true;
    row.masked_vz = true;
    ++request_.run_summary->vertical_jump_masked_imu_factor_count;
    request_.diagnostics->push_back(row);
  }
}

bool VerticalJumpImuMasker::OverlapsJumpPadding(
  const double start_time_s,
  const double end_time_s) const {
  const double padding_s = request_.config->vertical_jump_masked_imu_padding_s;
  for (const auto &window : *request_.jump_windows) {
    if (!std::isfinite(window.start_time_s) || !std::isfinite(window.end_time_s)) {
      continue;
    }
    if (IntervalsOverlap(start_time_s, end_time_s, window.start_time_s - padding_s, window.end_time_s + padding_s)) {
      return true;
    }
  }
  return false;
}

}  // namespace offline_lc_minimal
