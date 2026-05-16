#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

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

}  // namespace

int main() {
  try {
    RunTest("TestNoDriftKeepsCorrectionNearZero", TestNoDriftKeepsCorrectionNearZero);
    RunTest("TestSlowDriftReducesCorrectedCenterRange", TestSlowDriftReducesCorrectedCenterRange);
    RunTest("TestCorrectionIsClipped", TestCorrectionIsClipped);
    RunTest(
      "TestLowpassReferenceReducesHighFrequencyCenterMotion",
      TestLowpassReferenceReducesHighFrequencyCenterMotion);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
