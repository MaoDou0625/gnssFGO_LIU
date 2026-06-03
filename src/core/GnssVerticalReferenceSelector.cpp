#include "offline_lc_minimal/core/GnssVerticalReferenceSelector.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

bool HasRawUp(const GnssSolutionSample &sample) {
  return sample.has_enu_position && std::isfinite(sample.enu_position_m.z());
}

GnssVerticalReferenceSelection MakeInvalidSelection(
  const GnssSolutionSample &sample,
  const GnssVerticalReferenceSource source,
  std::string skip_reason) {
  GnssVerticalReferenceSelection selection;
  selection.source = ToString(source);
  selection.raw_up_m =
    sample.has_enu_position ? sample.enu_position_m.z()
                            : std::numeric_limits<double>::quiet_NaN();
  selection.skip_reason = std::move(skip_reason);
  return selection;
}

GnssVerticalReferenceSelection MakeValidSelection(
  const GnssSolutionSample &sample,
  const GnssVerticalReferenceSource source,
  const double selected_up_m,
  const double rtk_drift_estimate_m =
    std::numeric_limits<double>::quiet_NaN()) {
  GnssVerticalReferenceSelection selection;
  selection.valid = true;
  selection.source = ToString(source);
  selection.skip_reason = "OK";
  selection.raw_up_m = sample.enu_position_m.z();
  selection.selected_up_m = selected_up_m;
  selection.highfreq_residual_m = selection.raw_up_m - selection.selected_up_m;
  selection.rtk_drift_estimate_m = rtk_drift_estimate_m;
  return selection;
}

const Stage3VerticalReferenceDiagnosticRow *FindFirstFiniteRow(
  const Stage3VerticalReference &reference) {
  for (const auto &row : reference.rows) {
    if (std::isfinite(row.time_s) && std::isfinite(row.stage2_lowpass_up_m)) {
      return &row;
    }
  }
  return nullptr;
}

const Stage3VerticalReferenceDiagnosticRow *FindLastFiniteRow(
  const Stage3VerticalReference &reference) {
  for (auto iter = reference.rows.rbegin(); iter != reference.rows.rend(); ++iter) {
    if (std::isfinite(iter->time_s) && std::isfinite(iter->stage2_lowpass_up_m)) {
      return &(*iter);
    }
  }
  return nullptr;
}

bool InterpolateStage2LowpassUp(
  const Stage3VerticalReference &reference,
  const double corrected_time_s,
  double &up_m) {
  if (reference.rows.empty() || !std::isfinite(corrected_time_s)) {
    return false;
  }
  const auto *first = FindFirstFiniteRow(reference);
  const auto *last = FindLastFiniteRow(reference);
  if (first == nullptr || last == nullptr) {
    return false;
  }
  if (corrected_time_s <= first->time_s) {
    up_m = first->stage2_lowpass_up_m;
    return true;
  }
  if (corrected_time_s >= last->time_s) {
    up_m = last->stage2_lowpass_up_m;
    return true;
  }

  auto upper = std::lower_bound(
    reference.rows.begin(),
    reference.rows.end(),
    corrected_time_s,
    [](const Stage3VerticalReferenceDiagnosticRow &row, const double time_s) {
      return row.time_s < time_s;
    });
  while (upper != reference.rows.end() &&
         (!std::isfinite(upper->time_s) ||
          !std::isfinite(upper->stage2_lowpass_up_m))) {
    ++upper;
  }
  if (upper == reference.rows.end()) {
    return false;
  }
  auto lower = upper;
  while (lower != reference.rows.begin()) {
    --lower;
    if (std::isfinite(lower->time_s) &&
        std::isfinite(lower->stage2_lowpass_up_m)) {
      break;
    }
  }
  if (!std::isfinite(lower->time_s) ||
      !std::isfinite(lower->stage2_lowpass_up_m)) {
    return false;
  }
  if (upper->time_s <= lower->time_s) {
    up_m = lower->stage2_lowpass_up_m;
    return true;
  }
  const double alpha =
    (corrected_time_s - lower->time_s) / (upper->time_s - lower->time_s);
  up_m =
    lower->stage2_lowpass_up_m +
    alpha * (upper->stage2_lowpass_up_m - lower->stage2_lowpass_up_m);
  return std::isfinite(up_m);
}

}  // namespace

GnssVerticalReferenceSelector::GnssVerticalReferenceSelector(
  GnssVerticalReferenceSelectionRequest request)
    : request_(std::move(request)) {}

GnssVerticalReferenceSelection GnssVerticalReferenceSelector::Select() const {
  if (request_.sample == nullptr) {
    throw std::runtime_error(
      "GnssVerticalReferenceSelector received a null GNSS sample");
  }
  const GnssSolutionSample &sample = *request_.sample;
  if (!HasRawUp(sample)) {
    return MakeInvalidSelection(
      sample,
      request_.source,
      "RAW_RTK_UP_UNAVAILABLE");
  }

  switch (request_.source) {
    case GnssVerticalReferenceSource::kRawRtk:
      return MakeValidSelection(
        sample,
        GnssVerticalReferenceSource::kRawRtk,
        sample.enu_position_m.z());
    case GnssVerticalReferenceSource::kStage2Lowpass: {
      if (request_.stage2_lowpass_reference == nullptr) {
        return MakeInvalidSelection(
          sample,
          request_.source,
          "STAGE2_LOWPASS_REFERENCE_UNAVAILABLE");
      }
      double selected_up_m = std::numeric_limits<double>::quiet_NaN();
      if (!InterpolateStage2LowpassUp(
            *request_.stage2_lowpass_reference,
            request_.corrected_time_s,
            selected_up_m)) {
        return MakeInvalidSelection(
          sample,
          request_.source,
          "STAGE2_LOWPASS_REFERENCE_UNAVAILABLE");
      }
      return MakeValidSelection(sample, request_.source, selected_up_m);
    }
    case GnssVerticalReferenceSource::kRtkDriftLowpass: {
      if (request_.rtk_vertical_drift_reference_profile == nullptr ||
          request_.sample_index >=
            request_.rtk_vertical_drift_reference_profile->size()) {
        return MakeInvalidSelection(
          sample,
          request_.source,
          "RTK_DRIFT_LOWPASS_REFERENCE_UNAVAILABLE");
      }
      const auto &row =
        (*request_.rtk_vertical_drift_reference_profile)[request_.sample_index];
      if (!row.valid ||
          !row.lowpass_applied ||
          !std::isfinite(row.lowpass_center_up_m)) {
        return MakeInvalidSelection(
          sample,
          request_.source,
          "RTK_DRIFT_LOWPASS_REFERENCE_UNAVAILABLE");
      }
      return MakeValidSelection(
        sample,
        request_.source,
        row.lowpass_center_up_m,
        row.drift_estimate_m);
    }
    default:
      return MakeInvalidSelection(
        sample,
        request_.source,
        "UNSUPPORTED_VERTICAL_REFERENCE_SOURCE");
  }
}

}  // namespace offline_lc_minimal
