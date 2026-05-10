#include "offline_lc_minimal/core/RtkVerticalLatentReferenceBuilder.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <stdexcept>
#include <utility>

#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalRtkFactors.h"

namespace offline_lc_minimal {
namespace {

constexpr double kMinSigmaM = 1.0e-6;

struct CandidateSample {
  std::size_t sample_index = 0;
  std::size_t bin_index = 0;
  double time_s = std::numeric_limits<double>::quiet_NaN();
  double up_m = std::numeric_limits<double>::quiet_NaN();
  double sigma_u_m = std::numeric_limits<double>::quiet_NaN();
  double half_width_m = std::numeric_limits<double>::quiet_NaN();
};

struct BinAccumulator {
  std::size_t bin_index = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  std::vector<CandidateSample> samples;
};

bool IsAllowedFixType(const OfflineRunnerConfig &config, const GnssFixType fix_type) {
  if (!config.drop_non_rtkfix) {
    return true;
  }
  return fix_type == GnssFixType::kRtkFix;
}

bool PassesGnssQualityFiltersWithoutCounters(
  const GnssSolutionSample &sample,
  const OfflineRunnerConfig &config) {
  if (!sample.has_valid_position() || !sample.has_enu_position) {
    return false;
  }
  if (config.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config.required_best_sol_status_code) {
    return false;
  }
  if (config.drop_no_solution && sample.fix_type() == GnssFixType::kNoSolution) {
    return false;
  }
  if (!IsAllowedFixType(config, sample.fix_type())) {
    return false;
  }
  if (config.drop_nonfinite_sigma && !sample.has_finite_sigma()) {
    return false;
  }
  return true;
}

Eigen::Vector3d ApplyEarlyRelaxation(
  const OfflineRunnerConfig &config,
  const Eigen::Vector3d &sigma_m,
  const double corrected_time_s,
  const double dynamic_start_time_s) {
  if (config.early_gnss_relaxation_duration_s <= 0.0) {
    return sigma_m;
  }
  const double elapsed_s = corrected_time_s - dynamic_start_time_s;
  if (elapsed_s < 0.0 || elapsed_s >= config.early_gnss_relaxation_duration_s) {
    return sigma_m;
  }
  const double alpha = 1.0 - (elapsed_s / config.early_gnss_relaxation_duration_s);
  return sigma_m * (1.0 + alpha * (config.early_gnss_relaxation_scale - 1.0));
}

double ComputeHalfWidthM(const OfflineRunnerConfig &config, const double sigma_u_m) {
  return std::max(
    config.vertical_envelope_min_half_width_m,
    config.vertical_envelope_gate_sigma_multiple * sigma_u_m);
}

double ReferenceMeasurementSigmaM(
  const OfflineRunnerConfig &config,
  const double half_width_m) {
  switch (config.rtk_vertical_latent_reference_measurement_sigma_mode) {
    case RtkVerticalLatentReferenceMeasurementSigmaMode::kGateSigma:
    default:
      return std::max(half_width_m / config.vertical_envelope_gate_sigma_multiple, kMinSigmaM);
  }
}

gtsam::SharedNoiseModel MakeReferenceMeasurementNoise(
  const OfflineRunnerConfig &config,
  const double half_width_m) {
  const auto gaussian = gtsam::noiseModel::Isotropic::Sigma(
    1,
    ReferenceMeasurementSigmaM(config, half_width_m));
  return gtsam::noiseModel::Robust::Create(
    gtsam::noiseModel::mEstimator::Huber::Create(
      config.rtk_vertical_latent_reference_measurement_huber_sigma_m),
    gaussian);
}

double Mean(const std::vector<double> &values) {
  if (values.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  return std::accumulate(values.begin(), values.end(), 0.0) / static_cast<double>(values.size());
}

double StdDev(const std::vector<double> &values, const double mean) {
  if (values.empty() || !std::isfinite(mean)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double variance = 0.0;
  for (const double value : values) {
    const double centered = value - mean;
    variance += centered * centered;
  }
  return std::sqrt(variance / static_cast<double>(values.size()));
}

double WeightedMedian(std::vector<std::pair<double, double>> value_weights) {
  if (value_weights.empty()) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  std::sort(
    value_weights.begin(),
    value_weights.end(),
    [](const auto &lhs, const auto &rhs) {
      return lhs.first < rhs.first;
    });
  double total_weight = 0.0;
  for (const auto &[_, weight] : value_weights) {
    total_weight += weight;
  }
  if (!(total_weight > 0.0)) {
    return value_weights[value_weights.size() / 2U].first;
  }
  const double half_weight = 0.5 * total_weight;
  double accumulated_weight = 0.0;
  for (const auto &[value, weight] : value_weights) {
    accumulated_weight += weight;
    if (accumulated_weight >= half_weight) {
      return value;
    }
  }
  return value_weights.back().first;
}

double InitialReferenceUpM(const std::vector<CandidateSample> &samples) {
  std::vector<std::pair<double, double>> value_weights;
  value_weights.reserve(samples.size());
  for (const CandidateSample &sample : samples) {
    const double sigma = std::max(std::abs(sample.sigma_u_m), kMinSigmaM);
    value_weights.emplace_back(sample.up_m, 1.0 / (sigma * sigma));
  }
  return WeightedMedian(std::move(value_weights));
}

RtkVerticalLatentReferenceDiagnosticRow MakeDiagnosticRow(
  const BinAccumulator &bin,
  const std::size_t key_index,
  const double initial_reference_up_m,
  const int min_sample_count) {
  RtkVerticalLatentReferenceDiagnosticRow row;
  row.bin_index = bin.bin_index;
  row.key_index = key_index;
  row.bin_start_time_s = bin.start_time_s;
  row.bin_end_time_s = bin.end_time_s;
  row.bin_center_time_s = 0.5 * (bin.start_time_s + bin.end_time_s);
  row.sample_count = static_cast<int>(bin.samples.size());
  row.sample_indices.reserve(bin.samples.size());
  for (const CandidateSample &sample : bin.samples) {
    row.sample_indices.push_back(sample.sample_index);
  }
  row.low_sample_count = row.sample_count < min_sample_count;
  row.initial_reference_up_m = initial_reference_up_m;
  row.raw_median_up_m = initial_reference_up_m;
  row.skip_reason = row.low_sample_count ? "LOW_SAMPLE_COUNT" : "NONE";

  std::vector<double> up_values;
  up_values.reserve(bin.samples.size());
  for (const CandidateSample &sample : bin.samples) {
    up_values.push_back(sample.up_m);
  }
  row.raw_mean_up_m = Mean(up_values);
  row.raw_std_up_m = StdDev(up_values, row.raw_mean_up_m);
  if (!up_values.empty()) {
    const auto [min_it, max_it] = std::minmax_element(up_values.begin(), up_values.end());
    row.raw_range_up_m = *max_it - *min_it;
  }
  return row;
}

}  // namespace

gtsam::Key RtkVerticalLatentReferenceKey(const std::size_t key_index) {
  return gtsam::Symbol('r', key_index);
}

RtkVerticalLatentReferenceBuilder::RtkVerticalLatentReferenceBuilder(
    RtkVerticalLatentReferenceBuildRequest request)
    : request_(std::move(request)) {}

void RtkVerticalLatentReferenceBuilder::Validate() const {
  if (request_.config == nullptr || request_.gnss_samples == nullptr ||
      request_.graph == nullptr || request_.initial_values == nullptr ||
      request_.run_summary == nullptr || !request_.is_within_imu_coverage ||
      !request_.corrected_time_s || !request_.clamped_sigma_m) {
    throw std::runtime_error("RtkVerticalLatentReferenceBuilder received an incomplete request");
  }
}

RtkVerticalLatentReferenceBuildResult RtkVerticalLatentReferenceBuilder::Build() const {
  Validate();

  RtkVerticalLatentReferenceBuildResult result;
  result.sample_references.resize(request_.gnss_samples->size());
  for (std::size_t index = 0; index < result.sample_references.size(); ++index) {
    result.sample_references[index].sample_index = index;
    result.sample_references[index].skip_reason = "NOT_EVALUATED";
  }

  double reference_epoch_s = request_.reference_epoch_s;
  if (!std::isfinite(reference_epoch_s) && request_.first_sample_index < request_.gnss_samples->size()) {
    reference_epoch_s = request_.corrected_time_s((*request_.gnss_samples)[request_.first_sample_index]);
  }

  std::vector<CandidateSample> candidates;
  candidates.reserve(request_.gnss_samples->size());
  for (std::size_t sample_index = request_.first_sample_index;
       sample_index < request_.gnss_samples->size();
       ++sample_index) {
    RtkVerticalLatentReferenceSampleReference &sample_ref = result.sample_references[sample_index];
    const GnssSolutionSample &sample = (*request_.gnss_samples)[sample_index];
    if (!PassesGnssQualityFiltersWithoutCounters(sample, *request_.config)) {
      sample_ref.skip_reason = "SAMPLE_REJECTED";
      continue;
    }

    const double corrected_time_s = request_.corrected_time_s(sample);
    if (!request_.is_within_imu_coverage(corrected_time_s)) {
      sample_ref.skip_reason = "OUT_OF_IMU_COVERAGE";
      continue;
    }
    Eigen::Vector3d sigma_m = request_.clamped_sigma_m(sample);
    sigma_m = ApplyEarlyRelaxation(
      *request_.config,
      sigma_m,
      corrected_time_s,
      request_.dynamic_start_time_s);
    if (!std::isfinite(corrected_time_s) || !std::isfinite(reference_epoch_s) ||
        !std::isfinite(sample.enu_position_m.z()) || !std::isfinite(sigma_m.z()) ||
        sigma_m.z() <= 0.0) {
      sample_ref.skip_reason = "INVALID_SAMPLE";
      continue;
    }

    const double bin_offset = (corrected_time_s - reference_epoch_s) /
                              request_.config->rtk_vertical_latent_reference_bin_s;
    if (!std::isfinite(bin_offset)) {
      sample_ref.skip_reason = "INVALID_BIN";
      continue;
    }
    const auto signed_bin_index = static_cast<long long>(std::floor(bin_offset + 1.0e-9));
    if (signed_bin_index < 0) {
      sample_ref.skip_reason = "BEFORE_REFERENCE_EPOCH";
      continue;
    }
    candidates.push_back(CandidateSample{
      sample_index,
      static_cast<std::size_t>(signed_bin_index),
      corrected_time_s,
      sample.enu_position_m.z(),
      sigma_m.z(),
      ComputeHalfWidthM(*request_.config, sigma_m.z())});
  }

  std::map<std::size_t, BinAccumulator> bins;
  for (const CandidateSample &candidate : candidates) {
    BinAccumulator &bin = bins[candidate.bin_index];
    if (bin.samples.empty()) {
      bin.bin_index = candidate.bin_index;
      bin.start_time_s = reference_epoch_s +
                         static_cast<double>(candidate.bin_index) *
                           request_.config->rtk_vertical_latent_reference_bin_s;
      bin.end_time_s = bin.start_time_s + request_.config->rtk_vertical_latent_reference_bin_s;
    }
    bin.samples.push_back(candidate);
  }

  std::vector<std::size_t> ordered_bin_indices;
  ordered_bin_indices.reserve(bins.size());
  for (const auto &[bin_index, _] : bins) {
    ordered_bin_indices.push_back(bin_index);
  }

  std::map<std::size_t, std::size_t> bin_to_key_index;
  for (std::size_t ordered_index = 0; ordered_index < ordered_bin_indices.size(); ++ordered_index) {
    const std::size_t bin_index = ordered_bin_indices[ordered_index];
    const BinAccumulator &bin = bins.at(bin_index);
    const double initial_reference_up_m = InitialReferenceUpM(bin.samples);
    if (!std::isfinite(initial_reference_up_m)) {
      continue;
    }

    const std::size_t key_index = ordered_index;
    bin_to_key_index[bin_index] = key_index;
    const gtsam::Key reference_key = RtkVerticalLatentReferenceKey(key_index);
    double initial_value_up_m = initial_reference_up_m;
    if (!request_.initial_values->exists(reference_key)) {
      request_.initial_values->insert(reference_key, initial_reference_up_m);
    } else {
      initial_value_up_m = request_.initial_values->at<double>(reference_key);
    }

    RtkVerticalLatentReferenceDiagnosticRow diagnostic = MakeDiagnosticRow(
      bin,
      key_index,
      initial_value_up_m,
      request_.config->rtk_vertical_latent_reference_min_sample_count);
    diagnostic.raw_median_up_m = initial_reference_up_m;
    if (diagnostic.low_sample_count) {
      ++request_.run_summary->rtk_vertical_latent_reference_low_sample_bin_count;
    }
    result.diagnostics.push_back(diagnostic);

    for (const CandidateSample &sample : bin.samples) {
      const auto noise = MakeReferenceMeasurementNoise(*request_.config, sample.half_width_m);
      request_.graph->add(factor::RtkVerticalReferenceMeasurementFactor(
        reference_key,
        sample.up_m,
        noise));
      ++request_.run_summary->rtk_vertical_latent_reference_measurement_factor_count;

      RtkVerticalLatentReferenceSampleReference &sample_ref =
        result.sample_references[sample.sample_index];
      sample_ref.valid = true;
      sample_ref.bin_index = bin_index;
      sample_ref.key_index = key_index;
      sample_ref.key = reference_key;
      sample_ref.initial_reference_up_m = initial_value_up_m;
      sample_ref.low_sample_count = diagnostic.low_sample_count;
      sample_ref.skip_reason = "NONE";
    }
  }

  for (std::size_t index = 1U; index < ordered_bin_indices.size(); ++index) {
    const std::size_t prev_bin_index = ordered_bin_indices[index - 1U];
    const std::size_t curr_bin_index = ordered_bin_indices[index];
    if (!bin_to_key_index.contains(prev_bin_index) || !bin_to_key_index.contains(curr_bin_index)) {
      continue;
    }
    const std::size_t bin_gap = std::max<std::size_t>(1U, curr_bin_index - prev_bin_index);
    const double smooth_sigma_m =
      request_.config->rtk_vertical_latent_reference_smooth_sigma_m *
      std::sqrt(static_cast<double>(bin_gap));
    request_.graph->add(factor::RtkVerticalReferenceSmoothnessFactor(
      RtkVerticalLatentReferenceKey(bin_to_key_index.at(prev_bin_index)),
      RtkVerticalLatentReferenceKey(bin_to_key_index.at(curr_bin_index)),
      gtsam::noiseModel::Isotropic::Sigma(1, smooth_sigma_m)));
    ++request_.run_summary->rtk_vertical_latent_reference_smoothness_factor_count;
    if (index - 1U < result.diagnostics.size()) {
      result.diagnostics[index - 1U].smoothness_sigma_to_next_m = smooth_sigma_m;
    }
  }

  request_.run_summary->rtk_vertical_latent_reference_enabled = true;
  request_.run_summary->rtk_vertical_latent_reference_bin_count = result.diagnostics.size();
  return result;
}

}  // namespace offline_lc_minimal
