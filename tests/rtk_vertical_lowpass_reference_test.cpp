#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "offline_lc_minimal/core/RtkVerticalLowpassReferenceBuilder.h"

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

void ExpectNear(
  const double actual,
  const double expected,
  const double tolerance,
  const std::string &message) {
  if (std::abs(actual - expected) > tolerance) {
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
  }
}

offline_lc_minimal::GnssSolutionSample MakeSample(const double time_s, const double up_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 0.1;
  sample.lon_rad = 0.2;
  sample.h_m = 10.0 + up_m;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.01;
  sample.gnssfgo_type_code = 1;
  sample.best_sol_status_code = 1;
  sample.has_enu_position = true;
  sample.enu_position_m = Eigen::Vector3d(0.0, 0.0, up_m);
  return sample;
}

offline_lc_minimal::RtkVerticalLowpassReferenceBuildRequest MakeRequest(
  const offline_lc_minimal::OfflineRunnerConfig &config,
  const std::vector<offline_lc_minimal::GnssSolutionSample> &samples) {
  offline_lc_minimal::RtkVerticalLowpassReferenceBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.first_sample_index = 0;
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.01, 0.01, 0.01); };
  return request;
}

void TestStableInputReturnsRawHeight() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_vertical_lowpass_window_s = 10.0;
  config.rtk_vertical_lowpass_min_sample_count = 3;
  config.rtk_vertical_lowpass_huber_sigma_m = 0.03;
  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.0, 4.0),
    MakeSample(1.0, 4.0),
    MakeSample(2.0, 4.0),
  };

  const auto result =
    offline_lc_minimal::RtkVerticalLowpassReferenceBuilder(MakeRequest(config, samples)).Build();

  ExpectNear(static_cast<double>(result.valid_count), 3.0, 0.0, "all rows should be valid");
  for (const auto &row : result.rows) {
    ExpectTrue(row.lowpass_valid, "constant RTK row should be valid");
    ExpectNear(row.lowpass_up_m, row.raw_up_m, 1e-12, "constant RTK should pass through unchanged");
    ExpectNear(row.raw_minus_lowpass_m, 0.0, 1e-12, "constant high-pass residual should be zero");
  }
}

void TestSinglePulseDoesNotPolluteLowpass() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_vertical_lowpass_window_s = 10.0;
  config.rtk_vertical_lowpass_min_sample_count = 3;
  config.rtk_vertical_lowpass_huber_sigma_m = 0.03;
  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.0, 0.0),
    MakeSample(1.0, 0.0),
    MakeSample(2.0, 1.0),
    MakeSample(3.0, 0.0),
    MakeSample(4.0, 0.0),
  };

  const auto result =
    offline_lc_minimal::RtkVerticalLowpassReferenceBuilder(MakeRequest(config, samples)).Build();

  ExpectNear(static_cast<double>(result.valid_count), 5.0, 0.0, "all pulse-test rows should be valid");
  ExpectTrue(result.rows[2].lowpass_valid, "pulse row should still have a low-pass reference");
  ExpectTrue(
    std::abs(result.rows[2].lowpass_up_m) < 0.02,
    "Huber low-pass reference should stay near the neighboring baseline");
  ExpectTrue(
    result.rows[2].raw_minus_lowpass_m > 0.98,
    "the pulse should remain in the raw-minus-lowpass diagnostic");
}

void TestInsufficientSamplesAreInvalid() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.rtk_vertical_lowpass_window_s = 1.0;
  config.rtk_vertical_lowpass_min_sample_count = 5;
  config.rtk_vertical_lowpass_huber_sigma_m = 0.03;
  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.0, 1.0),
    MakeSample(1.0, 1.0),
    MakeSample(2.0, 1.0),
  };

  const auto result =
    offline_lc_minimal::RtkVerticalLowpassReferenceBuilder(MakeRequest(config, samples)).Build();

  ExpectNear(static_cast<double>(result.valid_count), 0.0, 0.0, "no row should be valid");
  for (const auto &row : result.rows) {
    ExpectTrue(!row.lowpass_valid, "insufficient sample row should be invalid");
    ExpectTrue(
      row.skip_reason == "INSUFFICIENT_WINDOW_SAMPLES",
      "invalid row should record insufficient window samples");
  }
}

}  // namespace

int main() {
  try {
    RunTest("TestStableInputReturnsRawHeight", TestStableInputReturnsRawHeight);
    RunTest("TestSinglePulseDoesNotPolluteLowpass", TestSinglePulseDoesNotPolluteLowpass);
    RunTest("TestInsufficientSamplesAreInvalid", TestInsufficientSamplesAreInvalid);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
