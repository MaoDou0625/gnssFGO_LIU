#include "offline_lc_minimal/core/RtkVerticalLowpassReferenceFilter.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace offline_lc_minimal {
namespace {

constexpr double kTwoPi = 6.28318530717958647692;

bool CanFilterRow(const RtkVerticalDriftReferenceDiagnosticRow &row) {
  return row.valid && std::isfinite(row.time_s) && std::isfinite(row.corrected_center_up_m);
}

double LowpassAlpha(const double dt_s, const double tau_s) {
  if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
    return 0.0;
  }
  return 1.0 - std::exp(-dt_s / tau_s);
}

void FilterSegment(
  const OfflineRunnerConfig &config,
  const std::vector<std::size_t> &indices,
  std::vector<RtkVerticalDriftReferenceDiagnosticRow> *profile,
  RtkVerticalLowpassReferenceFilterSummary *summary) {
  if (indices.empty()) {
    return;
  }

  const double tau_s = 1.0 / (kTwoPi * config.rtk_vertical_lowpass_reference_cutoff_hz);
  std::vector<double> forward(indices.size(), 0.0);
  forward.front() = (*profile)[indices.front()].corrected_center_up_m;

  for (std::size_t i = 1; i < indices.size(); ++i) {
    const auto &prev_row = (*profile)[indices[i - 1U]];
    const auto &row = (*profile)[indices[i]];
    const double alpha = LowpassAlpha(row.time_s - prev_row.time_s, tau_s);
    forward[i] = forward[i - 1U] + alpha * (row.corrected_center_up_m - forward[i - 1U]);
  }

  std::vector<double> zero_phase = forward;
  for (std::size_t reverse = indices.size() - 1U; reverse > 0U; --reverse) {
    const std::size_t i = reverse - 1U;
    const auto &row = (*profile)[indices[i]];
    const auto &next_row = (*profile)[indices[i + 1U]];
    const double alpha = LowpassAlpha(next_row.time_s - row.time_s, tau_s);
    zero_phase[i] = zero_phase[i + 1U] + alpha * (forward[i] - zero_phase[i + 1U]);
  }

  for (std::size_t i = 0; i < indices.size(); ++i) {
    auto &row = (*profile)[indices[i]];
    row.lowpass_center_up_m = zero_phase[i];
    row.lowpass_delta_m = row.lowpass_center_up_m - row.corrected_center_up_m;
    row.lowpass_cutoff_hz = config.rtk_vertical_lowpass_reference_cutoff_hz;
    row.lowpass_applied = true;
    ++summary->valid_count;
    summary->max_abs_delta_m =
      std::max(summary->max_abs_delta_m, std::abs(row.lowpass_delta_m));
  }
}

}  // namespace

RtkVerticalLowpassReferenceFilterSummary ApplyRtkVerticalLowpassReferenceFilter(
  const OfflineRunnerConfig &config,
  std::vector<RtkVerticalDriftReferenceDiagnosticRow> *profile) {
  if (profile == nullptr) {
    throw std::runtime_error("RTK vertical lowpass reference filter received a null profile");
  }
  RtkVerticalLowpassReferenceFilterSummary summary;
  if (!config.enable_rtk_vertical_lowpass_reference || profile->empty()) {
    return summary;
  }

  std::vector<std::size_t> segment;
  segment.reserve(profile->size());
  for (std::size_t index = 0; index < profile->size(); ++index) {
    if (!CanFilterRow((*profile)[index])) {
      FilterSegment(config, segment, profile, &summary);
      segment.clear();
      continue;
    }
    if (!segment.empty()) {
      const auto &prev_row = (*profile)[segment.back()];
      const auto &row = (*profile)[index];
      if (row.time_s <= prev_row.time_s) {
        FilterSegment(config, segment, profile, &summary);
        segment.clear();
      }
    }
    segment.push_back(index);
  }
  FilterSegment(config, segment, profile, &summary);
  return summary;
}

}  // namespace offline_lc_minimal
