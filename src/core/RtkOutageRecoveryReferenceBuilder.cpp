#include "offline_lc_minimal/core/RtkOutageRecoveryReferenceBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool IsPlannedOutage(const RtkOutageWindowRow &window) {
  return window.skip_reason == "PLANNED" || window.skip_reason == "ADDED";
}

bool HasFiniteEnuPosition(const GnssSolutionSample &sample) {
  return sample.has_enu_position && sample.enu_position_m.allFinite();
}

struct FitSample {
  double time_s = std::numeric_limits<double>::quiet_NaN();
  Eigen::Vector3d enu_position_m =
    Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
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
        !HasFiniteEnuPosition(sample)) {
      continue;
    }
    samples.push_back(FitSample{corrected_time_s, sample.enu_position_m});
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
        !record.measurement_enu_m.allFinite()) {
      continue;
    }
    samples.push_back(FitSample{record.corrected_time_s, record.measurement_enu_m});
  }
  return samples;
}

void FitPositionAndVelocityAtBoundary(
  const std::vector<FitSample> &samples,
  const double boundary_time_s,
  Eigen::Vector3d &reference_position_m,
  Eigen::Vector3d &reference_velocity_mps) {
  double sum_t = 0.0;
  Eigen::Vector3d sum_position = Eigen::Vector3d::Zero();
  for (const auto &sample : samples) {
    sum_t += sample.time_s - boundary_time_s;
    sum_position += sample.enu_position_m;
  }
  const double count = static_cast<double>(samples.size());
  const double mean_t = sum_t / count;
  const Eigen::Vector3d mean_position = sum_position / count;
  Eigen::Vector3d numerator = Eigen::Vector3d::Zero();
  double denominator = 0.0;
  for (const auto &sample : samples) {
    const double centered_t = (sample.time_s - boundary_time_s) - mean_t;
    numerator += centered_t * (sample.enu_position_m - mean_position);
    denominator += centered_t * centered_t;
  }
  if (denominator > 1.0e-12) {
    reference_velocity_mps = numerator / denominator;
  } else {
    reference_velocity_mps = Eigen::Vector3d::Zero();
  }
  reference_position_m = mean_position - reference_velocity_mps * mean_t;
}

RtkOutageBoundaryReferenceRow MakeBoundaryReferenceFromRecovery(
  const OfflineRunnerConfig &config,
  const RtkOutageRecoveryReferenceRow &reference,
  const std::string &boundary_role) {
  RtkOutageBoundaryReferenceRow row;
  row.window_index = reference.window_index;
  row.boundary_role = boundary_role;
  row.source_type = "POST_RECOVERY_RTK";
  row.target_time_s = reference.outage_end_time_s;
  row.valid = reference.valid;
  row.has_up = reference.valid;
  row.has_vz = reference.valid;
  row.has_ba_z = false;
  row.has_horizontal_position =
    reference.valid && reference.reference_horizontal_position_m.allFinite();
  row.has_horizontal_velocity =
    reference.valid && reference.reference_horizontal_velocity_mps.allFinite();
  row.add_up_constraint = reference.valid;
  row.add_vz_constraint = reference.valid;
  row.add_ba_z_constraint = false;
  row.add_horizontal_position_constraint = row.has_horizontal_position;
  row.add_horizontal_velocity_constraint = row.has_horizontal_velocity;
  row.reference_up_m = reference.reference_up_m;
  row.reference_vz_mps = reference.reference_vz_mps;
  row.reference_horizontal_position_m = reference.reference_horizontal_position_m;
  row.reference_horizontal_velocity_mps = reference.reference_horizontal_velocity_mps;
  row.up_sigma_m = config.rtk_outage_boundary_up_sigma_m;
  row.vz_sigma_mps = config.rtk_outage_boundary_vz_sigma_mps;
  row.ba_z_sigma_mps2 = config.rtk_outage_boundary_baz_sigma_mps2;
  row.horizontal_position_sigma_m = config.stage2_horizontal_position_hold_sigma_m;
  row.horizontal_velocity_sigma_mps = config.stage2_horizontal_velocity_hold_sigma_mps;
  row.skip_reason = reference.valid ? "OK" : reference.skip_reason;
  return row;
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

    std::sort(
      samples.begin(),
      samples.end(),
      [](const FitSample &left, const FitSample &right) {
        return left.time_s < right.time_s;
      });
    row.valid_fix_sample_count = samples.size();
    if (!samples.empty()) {
      row.first_sample_time_s = samples.front().time_s;
      row.last_sample_time_s = samples.back().time_s;
      row.first_sample_up_m = samples.front().enu_position_m.z();
      row.last_sample_up_m = samples.back().enu_position_m.z();
      row.first_sample_horizontal_position_m = samples.front().enu_position_m.head<2>();
      row.last_sample_horizontal_position_m = samples.back().enu_position_m.head<2>();
    }
    if (samples.size() <
        static_cast<std::size_t>(request_.config->rtk_outage_recovery_reference_min_fix_samples)) {
      row.skip_reason = "insufficient_rtkfix_samples";
      rows.push_back(row);
      continue;
    }

    Eigen::Vector3d reference_position_m =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    Eigen::Vector3d reference_velocity_mps =
      Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
    FitPositionAndVelocityAtBoundary(
      samples,
      window.end_time_s,
      reference_position_m,
      reference_velocity_mps);
    row.reference_horizontal_position_m = reference_position_m.head<2>();
    row.reference_up_m = reference_position_m.z();
    row.reference_horizontal_velocity_mps = reference_velocity_mps.head<2>();
    row.reference_vz_mps = reference_velocity_mps.z();
    row.valid = true;
    row.skip_reason = "OK";
    rows.push_back(row);
  }
  return rows;
}

RtkOutageBoundaryReferenceRow MakePostStartBoundaryReferenceFromRecovery(
  const OfflineRunnerConfig &config,
  const RtkOutageRecoveryReferenceRow &reference) {
  return MakeBoundaryReferenceFromRecovery(config, reference, "POST_START");
}

RtkOutageBoundaryReferenceRow MakeOutageEndBoundaryReferenceFromRecovery(
  const OfflineRunnerConfig &config,
  const RtkOutageRecoveryReferenceRow &reference) {
  return MakeBoundaryReferenceFromRecovery(config, reference, "OUTAGE_END");
}

}  // namespace offline_lc_minimal
