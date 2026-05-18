#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/ResultOutputWriters.h"
#include "offline_lc_minimal/core/RtkVerticalDriftGateWeighting.h"
#include "offline_lc_minimal/core/RtkVerticalDriftReferenceEstimator.h"
#include "offline_lc_minimal/core/RtkVerticalLowpassReferenceFilter.h"

namespace {

template <typename Function>
void RunTest(const std::string &name, Function &&function) {
  try {
    function();
  } catch (const std::exception &exception) {
    throw std::runtime_error(name + ": " + exception.what());
  }
}

void ExpectTrue(const bool condition, const std::string &message) {
  if (!condition) {
    throw std::runtime_error(message);
  }
}

void ExpectNear(const double actual, const double expected, const double tolerance, const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(message);
  }
}

std::vector<offline_lc_minimal::GnssSolutionSample> MakeStaticSamples(
  const std::vector<double> &up_values_m) {
  std::vector<offline_lc_minimal::GnssSolutionSample> samples;
  samples.reserve(up_values_m.size());
  for (std::size_t index = 0; index < up_values_m.size(); ++index) {
    offline_lc_minimal::GnssSolutionSample sample;
    sample.time_s = static_cast<double>(index);
    sample.lat_rad = 1.0;
    sample.lon_rad = 1.0;
    sample.h_m = up_values_m[index];
    sample.sigma_lat_m = 0.01;
    sample.sigma_lon_m = 0.01;
    sample.sigma_h_m = 0.01;
    sample.best_sol_status_code = 1;
    sample.gnssfgo_type_code = 1;
    sample.enu_position_m.z() = up_values_m[index];
    sample.has_enu_position = true;
    samples.push_back(sample);
  }
  return samples;
}

offline_lc_minimal::RtkVerticalDriftReferenceEstimateRequest MakeRequest(
  const offline_lc_minimal::OfflineRunnerConfig &config,
  const std::vector<offline_lc_minimal::GnssSolutionSample> &samples) {
  offline_lc_minimal::RtkVerticalDriftReferenceEstimateRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.alignment_start_time_s = 0.0;
  request.alignment_end_time_s = 200.0;
  request.static_reference_up_m = 0.0;
  request.should_use_sample = [](const offline_lc_minimal::GnssSolutionSample &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const offline_lc_minimal::GnssSolutionSample &sample) {
    return sample.time_s;
  };
  request.clamped_sigma_m = [](const offline_lc_minimal::GnssSolutionSample &sample) {
    return Eigen::Vector3d(sample.sigma_lat_m, sample.sigma_lon_m, sample.sigma_h_m);
  };
  return request;
}

void TestNoDriftKeepsCorrectionNearZero() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.rtk_vertical_drift_sigma_m = 0.010;
  config.rtk_vertical_white_noise_sigma_m = 0.002;
  config.rtk_vertical_drift_correlation_time_s = 5.3;
  const std::vector<double> up_values(50U, 1.234);
  const auto samples = MakeStaticSamples(up_values);
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(MakeRequest(config, samples)).Estimate(nullptr);
  double max_abs_correction = 0.0;
  for (const auto &row : result.profile) {
    ExpectTrue(row.valid, "all static samples should be valid");
    max_abs_correction = std::max(max_abs_correction, std::abs(row.drift_estimate_m));
  }
  ExpectTrue(max_abs_correction < 1.0e-9, "constant RTK offset should be treated as bias, not drift");
}

void TestSlowDriftReducesCorrectedCenterRange() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.rtk_vertical_drift_sigma_m = 0.010;
  config.rtk_vertical_white_noise_sigma_m = 0.002;
  config.rtk_vertical_drift_correlation_time_s = 5.3;
  std::vector<double> up_values;
  for (std::size_t index = 0; index < 100U; ++index) {
    const double t = static_cast<double>(index);
    up_values.push_back(2.0 + 0.010 * std::sin(2.0 * 3.14159265358979323846 * t / 50.0));
  }
  const auto samples = MakeStaticSamples(up_values);
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(MakeRequest(config, samples)).Estimate(nullptr);
  std::vector<double> corrected;
  corrected.reserve(result.profile.size());
  for (const auto &row : result.profile) {
    ExpectTrue(row.valid, "sinusoidal static samples should be valid");
    corrected.push_back(row.corrected_center_up_m);
  }
  const auto raw_minmax = std::minmax_element(up_values.begin(), up_values.end());
  const auto corrected_minmax = std::minmax_element(corrected.begin(), corrected.end());
  const double raw_range = *raw_minmax.second - *raw_minmax.first;
  const double corrected_range = *corrected_minmax.second - *corrected_minmax.first;
  ExpectTrue(corrected_range < raw_range, "drift correction should reduce slow RTK range");
}

void TestCorrectionIsClipped() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.rtk_vertical_drift_sigma_m = 0.10;
  config.rtk_vertical_white_noise_sigma_m = 0.001;
  config.rtk_vertical_drift_correlation_time_s = 100.0;
  config.rtk_vertical_drift_max_abs_correction_m = 0.050;
  std::vector<double> up_values(100U, 0.0);
  for (std::size_t index = 50U; index < up_values.size(); ++index) {
    up_values[index] = 0.20;
  }
  const auto samples = MakeStaticSamples(up_values);
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(MakeRequest(config, samples)).Estimate(nullptr);
  double max_abs_correction = 0.0;
  for (const auto &row : result.profile) {
    if (row.valid) {
      max_abs_correction = std::max(max_abs_correction, std::abs(row.drift_estimate_m));
    }
  }
  ExpectTrue(max_abs_correction <= 0.050 + 1.0e-12, "drift correction should respect max clip");
}

void TestLowpassReferenceReducesHighFrequencyCenterMotion() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_vertical_lowpass_reference = true;
  config.rtk_vertical_lowpass_reference_cutoff_hz = 0.10;

  std::vector<offline_lc_minimal::RtkVerticalDriftReferenceDiagnosticRow> profile(200U);
  for (std::size_t index = 0; index < profile.size(); ++index) {
    const double time_s = 0.2 * static_cast<double>(index);
    auto &row = profile[index];
    row.sample_index = index;
    row.time_s = time_s;
    row.corrected_center_up_m =
      10.0 + 0.020 * std::sin(2.0 * 3.14159265358979323846 * 0.5 * time_s);
    row.valid = true;
  }

  const auto summary =
    offline_lc_minimal::ApplyRtkVerticalLowpassReferenceFilter(config, &profile);
  std::vector<double> raw;
  std::vector<double> filtered;
  raw.reserve(profile.size());
  filtered.reserve(profile.size());
  for (const auto &row : profile) {
    ExpectTrue(row.lowpass_applied, "valid rows should receive lowpass center references");
    raw.push_back(row.corrected_center_up_m);
    filtered.push_back(row.lowpass_center_up_m);
  }
  const auto raw_minmax = std::minmax_element(raw.begin(), raw.end());
  const auto filtered_minmax = std::minmax_element(filtered.begin(), filtered.end());
  const double raw_range = *raw_minmax.second - *raw_minmax.first;
  const double filtered_range = *filtered_minmax.second - *filtered_minmax.first;
  ExpectTrue(summary.valid_count == profile.size(), "all valid rows should be counted");
  ExpectTrue(summary.max_abs_delta_m > 0.0, "lowpass should report a finite delta");
  ExpectTrue(filtered_range < 0.6 * raw_range, "0.1 Hz lowpass should suppress 0.5 Hz height motion");
}

void TestGateWeightFormula() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.vertical_envelope_min_half_width_m = 0.03;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.rtk_vertical_white_noise_sigma_m = 0.002;
  config.rtk_vertical_drift_gate_weight_floor = 0.05;

  const auto inside =
    offline_lc_minimal::ComputeRtkVerticalDriftGateWeighting(config, 0.020, 0.015);
  ExpectNear(inside.gate_half_width_m, 0.030, 1.0e-15, "gate half width should match envelope rule");
  ExpectNear(inside.gate_weight, 1.0, 1.0e-15, "inside-gate observation should keep full weight");
  ExpectNear(inside.gate_violation_m, 0.0, 1.0e-15, "inside-gate observation should have no violation");
  ExpectNear(
    inside.effective_white_sigma_m,
    config.rtk_vertical_white_noise_sigma_m,
    1.0e-15,
    "full-weight observation should preserve white sigma");

  const auto half_weight =
    offline_lc_minimal::ComputeRtkVerticalDriftGateWeighting(config, 0.060, 0.015);
  ExpectNear(half_weight.gate_weight, 0.5, 1.0e-15, "2x gate observation should get half weight");
  ExpectNear(half_weight.gate_violation_m, 0.030, 1.0e-15, "2x gate observation should report excess");
  ExpectNear(
    half_weight.effective_white_sigma_m,
    config.rtk_vertical_white_noise_sigma_m / std::sqrt(0.5),
    1.0e-15,
    "effective white sigma should scale by inverse sqrt weight");
}

void TestGateWeightFloor() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.vertical_envelope_min_half_width_m = 0.03;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.rtk_vertical_white_noise_sigma_m = 0.002;
  config.rtk_vertical_drift_gate_weight_floor = 0.20;

  const auto result =
    offline_lc_minimal::ComputeRtkVerticalDriftGateWeighting(config, 1.0, 0.015);
  ExpectNear(result.gate_weight, 0.20, 1.0e-15, "gate weight should respect configured floor");
}

void TestEstimatorWritesGateDiagnostics() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.vertical_envelope_min_half_width_m = 0.03;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.rtk_vertical_drift_gate_weight_floor = 0.05;
  config.rtk_vertical_white_noise_sigma_m = 0.002;

  const auto samples = MakeStaticSamples({0.0, 0.0, 0.060, 0.0, 0.0});
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(MakeRequest(config, samples)).Estimate(nullptr);
  const auto &row = result.profile[2U];
  ExpectTrue(row.valid, "out-of-gate sample should still be valid");
  ExpectNear(row.gate_half_width_m, 0.03, 1.0e-15, "diagnostic should store gate half width");
  ExpectNear(row.gate_observation_m, 0.060, 1.0e-12, "diagnostic should store drift observation");
  ExpectNear(row.gate_violation_m, 0.030, 1.0e-12, "diagnostic should store gate violation");
  ExpectNear(row.gate_weight, 0.5, 1.0e-12, "diagnostic should store gate weight");
  ExpectNear(
    row.effective_white_sigma_m,
    config.rtk_vertical_white_noise_sigma_m / std::sqrt(0.5),
    1.0e-12,
    "diagnostic should store effective white sigma");
}

void TestEstimatorGateWeightingReducesLargeDriftEstimate() {
  offline_lc_minimal::OfflineRunnerConfig weighted_config;
  weighted_config.vertical_envelope_min_half_width_m = 0.03;
  weighted_config.vertical_envelope_gate_sigma_multiple = 2.0;
  weighted_config.rtk_vertical_drift_sigma_m = 0.050;
  weighted_config.rtk_vertical_white_noise_sigma_m = 0.030;
  weighted_config.rtk_vertical_drift_huber_sigma_m = 1.0;
  weighted_config.rtk_vertical_drift_correlation_time_s = 100.0;
  weighted_config.rtk_vertical_drift_max_abs_correction_m = 1.0;
  weighted_config.rtk_vertical_drift_gate_weight_floor = 0.05;

  offline_lc_minimal::OfflineRunnerConfig unweighted_config = weighted_config;
  unweighted_config.enable_rtk_vertical_drift_gate_weighting = false;

  std::vector<double> up_values(40U, 0.0);
  for (std::size_t index = 20U; index < up_values.size(); ++index) {
    up_values[index] = 0.60;
  }
  const auto samples = MakeStaticSamples(up_values);

  const auto weighted =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(MakeRequest(weighted_config, samples)).Estimate(nullptr);
  const auto unweighted =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(MakeRequest(unweighted_config, samples)).Estimate(nullptr);
  double weighted_max_abs = 0.0;
  double unweighted_max_abs = 0.0;
  for (const auto &row : weighted.profile) {
    if (row.valid) {
      weighted_max_abs = std::max(weighted_max_abs, std::abs(row.drift_estimate_m));
    }
  }
  for (const auto &row : unweighted.profile) {
    if (row.valid) {
      unweighted_max_abs = std::max(unweighted_max_abs, std::abs(row.drift_estimate_m));
    }
  }
  ExpectTrue(
    weighted_max_abs < unweighted_max_abs,
    "gate weighting should reduce drift estimate for large out-of-gate observations");
}

void TestGateDiagnosticsWriterIncludesColumns() {
  std::vector<offline_lc_minimal::RtkVerticalDriftReferenceDiagnosticRow> rows(1U);
  rows.front().sample_index = 7U;
  rows.front().time_s = 1.0;
  rows.front().gate_half_width_m = 0.03;
  rows.front().gate_observation_m = 0.06;
  rows.front().gate_violation_m = 0.03;
  rows.front().gate_weight = 0.5;
  rows.front().effective_white_sigma_m = 0.004;

  const auto path =
    std::filesystem::temp_directory_path() / "rtk_vertical_drift_reference_writer_test.csv";
  offline_lc_minimal::WriteRtkVerticalDriftReferenceDiagnosticsCsv(path, rows);
  std::ifstream stream(path);
  std::string header;
  std::getline(stream, header);
  stream.close();
  std::filesystem::remove(path);
  ExpectTrue(
    header.find("gate_half_width_m,gate_observation_m,gate_violation_m,gate_weight,effective_white_sigma_m") !=
      std::string::npos,
    "CSV writer should include gate weighting diagnostics");
}

}  // namespace

int main() {
  try {
    RunTest("TestNoDriftKeepsCorrectionNearZero", TestNoDriftKeepsCorrectionNearZero);
    RunTest("TestSlowDriftReducesCorrectedCenterRange", TestSlowDriftReducesCorrectedCenterRange);
    RunTest("TestCorrectionIsClipped", TestCorrectionIsClipped);
    RunTest(
      "TestLowpassReferenceReducesHighFrequencyCenterMotion",
      TestLowpassReferenceReducesHighFrequencyCenterMotion);
    RunTest("TestGateWeightFormula", TestGateWeightFormula);
    RunTest("TestGateWeightFloor", TestGateWeightFloor);
    RunTest("TestEstimatorWritesGateDiagnostics", TestEstimatorWritesGateDiagnostics);
    RunTest(
      "TestEstimatorGateWeightingReducesLargeDriftEstimate",
      TestEstimatorGateWeightingReducesLargeDriftEstimate);
    RunTest("TestGateDiagnosticsWriterIncludesColumns", TestGateDiagnosticsWriterIncludesColumns);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
