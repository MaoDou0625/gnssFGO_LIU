#include "offline_lc_minimal/core/RtkOutageRecoveryReferenceBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED" || window.skip_reason == "ADDED";
}

bool HasFiniteEnuUp(const GnssSolutionSample &sample) {
  return sample.has_enu_position && std::isfinite(sample.enu_position_m.z());
}

struct FitSample {
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double up_m = std::numeric_limits<double>::quiet_NaN();
};

std::vector<FitSample> CollectSamplesFromGnssSamples(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const RtkOutageRecoveryReferenceBuildRequest &request,
  const double fit_start_time_s,
  const double fit_end_time_s) {
  std::vector<FitSample> samples;
  for (const auto &sample : gnss_samples) {
    const double corrected_time_s = request.corrected_time_s(sample);
    if (!std::isfinite(corrected_time_s) ||
        corrected_time_s + kTimeEpsilonS < fit_start_time_s ||
        corrected_time_s > fit_end_time_s + kTimeEpsilonS) {
      continue;
    }
    if (sample.fix_type() != GnssFixType::kRtkFix ||
        !request.passes_gnss_quality_filters(sample) ||
        !HasFiniteEnuUp(sample)) {
      continue;
    }
    samples.push_back(FitSample{corrected_time_s, sample.enu_position_m.z()});
  }
  return samples;
}

std::vector<FitSample> CollectSamplesFromGnssFactorRecords(
  const std::vector<GnssFactorRecord> &records,
  const double fit_start_time_s,
  const double fit_end_time_s) {
  std::vector<FitSample> samples;
  for (const auto &record : records) {
    if (!record.factor_used ||
        record.gnss_fix_type != GnssFixType::kRtkFix ||
        !std::isfinite(record.corrected_time_s) ||
        record.corrected_time_s + kTimeEpsilonS < fit_start_time_s ||
        record.corrected_time_s > fit_end_time_s + kTimeEpsilonS ||
        !std::isfinite(record.measurement_enu_m.z())) {
      continue;
    }
    samples.push_back(FitSample{record.corrected_time_s, record.measurement_enu_m.z()});
  }
  return samples;
}

void FitUpAndVzAtBoundary(
  const std::vector<FitSample> &samples,
  const double boundary_time_s,
  double &reference_up_m,
  double &reference_vz_mps) {
  double sum_t = 0.0;
  double sum_u = 0.0;
  for (const auto &sample : samples) {
    sum_t += sample.time_s - boundary_time_s;
    sum_u += sample.up_m;
  }
  const double count = static_cast<double>(samples.size());
  const double mean_t = sum_t / count;
  const double mean_u = sum_u / count;
  double numerator = 0.0;
  double denominator = 0.0;
  for (const auto &sample : samples) {
    const double centered_t = (sample.time_s - boundary_time_s) - mean_t;
    numerator += centered_t * (sample.up_m - mean_u);
    denominator += centered_t * centered_t;
  }
  reference_vz_mps = denominator > 1.0e-12 ? numerator / denominator : 0.0;
  reference_up_m = mean_u - reference_vz_mps * mean_t;
}

}  // namespace

RtkOutageRecoveryReferenceBuilder::RtkOutageRecoveryReferenceBuilder(
  RtkOutageRecoveryReferenceBuildRequest request)
    : request_(std::move(request)) {}

std::vector<RtkOutageRecoveryReferenceRow>
RtkOutageRecoveryReferenceBuilder::Build() const {
  if (request_.config == nullptr || request_.outage_windows == nullptr ||
      (request_.gnss_samples == nullptr && request_.gnss_factor_records == nullptr)) {
    throw std::runtime_error(
      "RtkOutageRecoveryReferenceBuilder received an incomplete request");
  }
  if (request_.gnss_samples != nullptr &&
      (!request_.passes_gnss_quality_filters || !request_.corrected_time_s)) {
    throw std::runtime_error(
      "RtkOutageRecoveryReferenceBuilder received incomplete GNSS sample callbacks");
  }

  std::vector<RtkOutageRecoveryReferenceRow> rows;
  rows.reserve(request_.outage_windows->size());
  for (const auto &window : *request_.outage_windows) {
    RtkOutageRecoveryReferenceRow row;
    row.window_index = window.window_index;
    row.outage_end_time_s = window.end_time_s;
    row.min_fix_sample_count =
      request_.config->rtk_outage_recovery_reference_min_fix_samples;
    row.fit_start_time_s = window.end_time_s;
    row.fit_end_time_s =
      window.end_time_s + request_.config->rtk_outage_recovery_reference_max_duration_s;

    if (!IsPlannedOutage(window) || !std::isfinite(window.end_time_s)) {
      row.skip_reason = "invalid_outage";
      rows.push_back(row);
      continue;
    }

    std::vector<FitSample> samples;
    if (request_.gnss_samples != nullptr) {
      samples = CollectSamplesFromGnssSamples(
        *request_.gnss_samples,
        request_,
        row.fit_start_time_s,
        row.fit_end_time_s);
    }
    if (samples.size() <
          static_cast<std::size_t>(
            request_.config->rtk_outage_recovery_reference_min_fix_samples) &&
        request_.gnss_factor_records != nullptr) {
      samples = CollectSamplesFromGnssFactorRecords(
        *request_.gnss_factor_records,
        row.fit_start_time_s,
        row.fit_end_time_s);
    }

    row.valid_fix_sample_count = samples.size();
    if (samples.size() <
        static_cast<std::size_t>(request_.config->rtk_outage_recovery_reference_min_fix_samples)) {
      row.skip_reason = "insufficient_rtkfix_samples";
      rows.push_back(row);
      continue;
    }

    std::sort(
      samples.begin(),
      samples.end(),
      [](const FitSample &left, const FitSample &right) {
        return left.time_s < right.time_s;
      });
    row.first_sample_time_s = samples.front().time_s;
    row.last_sample_time_s = samples.back().time_s;
    row.first_sample_up_m = samples.front().up_m;
    row.last_sample_up_m = samples.back().up_m;
    FitUpAndVzAtBoundary(
      samples,
      window.end_time_s,
      row.reference_up_m,
      row.reference_vz_mps);
    row.valid = true;
    row.skip_reason = "OK";
    rows.push_back(row);
  }
  return rows;
}

RtkOutageBoundaryReferenceRow MakePostStartBoundaryReferenceFromRecovery(
  const OfflineRunnerConfig &config,
  const RtkOutageRecoveryReferenceRow &reference) {
  RtkOutageBoundaryReferenceRow row;
  row.window_index = reference.window_index;
  row.boundary_role = "POST_START";
  row.source_type = "POST_RECOVERY_RTK";
  row.target_time_s = reference.outage_end_time_s;
  row.valid = reference.valid;
  row.has_up = reference.valid;
  row.has_vz = reference.valid;
  row.has_ba_z = false;
  row.add_up_constraint = reference.valid;
  row.add_vz_constraint = reference.valid;
  row.add_ba_z_constraint = false;
  row.reference_up_m = reference.reference_up_m;
  row.reference_vz_mps = reference.reference_vz_mps;
  row.up_sigma_m = config.rtk_outage_boundary_up_sigma_m;
  row.vz_sigma_mps = config.rtk_outage_boundary_vz_sigma_mps;
  row.ba_z_sigma_mps2 = config.rtk_outage_boundary_baz_sigma_mps2;
  row.skip_reason = reference.valid ? "OK" : reference.skip_reason;
  return row;
}

}  // namespace offline_lc_minimal
