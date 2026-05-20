#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/LateStaticDetector.h"
#include "offline_lc_minimal/core/LateStaticVerticalConstraintBuilder.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"
#include "offline_lc_minimal/factor/VerticalVelocityPriorFactor.h"

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

offline_lc_minimal::GnssSolutionSample MakeGnssSample(
  const double time_s,
  const double east_m,
  const double up_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 1.0;
  sample.lon_rad = 1.0;
  sample.h_m = up_m;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.01;
  sample.best_sol_status_code = 1;
  sample.gnssfgo_type_code = 1;
  sample.enu_position_m = Eigen::Vector3d(east_m, 0.0, up_m);
  sample.has_enu_position = true;
  return sample;
}

std::vector<offline_lc_minimal::GnssSolutionSample> MakeStaticDetectionGnss() {
  std::vector<offline_lc_minimal::GnssSolutionSample> samples;
  for (int t = 0; t <= 60; ++t) {
    const double time_s = static_cast<double>(t);
    const double east_m =
      time_s < 35.0 ? time_s : 35.0 + 0.001 * std::sin(0.5 * time_s);
    samples.push_back(MakeGnssSample(time_s, east_m, 2.0 + 0.001 * std::sin(time_s)));
  }
  return samples;
}

std::vector<offline_lc_minimal::ImuSample> MakeStaticDetectionImu() {
  std::vector<offline_lc_minimal::ImuSample> samples;
  for (int i = 0; i <= 120; ++i) {
    const double time_s = 0.5 * static_cast<double>(i);
    offline_lc_minimal::ImuSample sample;
    sample.time_s = time_s;
    const double gyro = time_s < 35.0 ? 0.10 : 0.0001;
    sample.gyro_radps = Eigen::Vector3d(gyro, 0.0, 0.0);
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81 + 0.001 * std::sin(time_s));
    samples.push_back(sample);
  }
  return samples;
}

std::vector<offline_lc_minimal::GnssSolutionSample> MakeInitialPrefixGnss() {
  std::vector<offline_lc_minimal::GnssSolutionSample> samples;
  for (int t = 0; t <= 80; ++t) {
    const double time_s = static_cast<double>(t);
    double east_m = 0.0;
    if (time_s >= 30.0 && time_s < 50.0) {
      east_m = time_s - 30.0;
    } else if (time_s >= 50.0) {
      east_m = 20.0 + 0.001 * std::sin(0.5 * time_s);
    } else {
      east_m = 0.001 * std::sin(0.5 * time_s);
    }
    samples.push_back(MakeGnssSample(time_s, east_m, 2.0 + 0.001 * std::sin(time_s)));
  }
  return samples;
}

std::vector<offline_lc_minimal::ImuSample> MakeInitialPrefixImu() {
  std::vector<offline_lc_minimal::ImuSample> samples;
  for (int i = 0; i <= 160; ++i) {
    const double time_s = 0.5 * static_cast<double>(i);
    offline_lc_minimal::ImuSample sample;
    sample.time_s = time_s;
    const double gyro = (time_s >= 30.0 && time_s < 50.0) ? 0.10 : 0.0001;
    sample.gyro_radps = Eigen::Vector3d(gyro, 0.0, 0.0);
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81 + 0.001 * std::sin(time_s));
    samples.push_back(sample);
  }
  return samples;
}

offline_lc_minimal::OfflineRunnerConfig MakeLateStaticConfig() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_late_static_detection = true;
  config.late_static_window_s = 5.0;
  config.late_static_stride_s = 5.0;
  config.late_static_min_duration_s = 8.0;
  config.late_static_min_rtkfix_samples = 5;
  config.late_static_merge_gap_s = 5.0;
  config.late_static_exclude_initial_static = true;
  config.late_static_exclude_rtk_outage = true;
  config.late_static_vz_sigma_mps = 0.002;
  config.late_static_up_sigma_m = 0.02;
  return config;
}

void TestDataDrivenDetectorFindsLateStaticAndRejectsDynamic() {
  auto config = MakeLateStaticConfig();
  const auto gnss = MakeStaticDetectionGnss();
  const auto imu = MakeStaticDetectionImu();

  offline_lc_minimal::LateStaticDetectionRequest request;
  request.config = &config;
  request.gnss_samples = &gnss;
  request.imu_samples = &imu;
  request.processing_start_time_s = 0.0;
  request.processing_end_time_s = 60.0;
  request.alignment_start_time_s = -100.0;
  request.alignment_end_time_s = -90.0;
  request.should_use_rtkfix_sample =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.best_sol_status_code == 1;
    };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  std::vector<offline_lc_minimal::LateStaticFeatureDiagnosticRow> features =
    offline_lc_minimal::LateStaticFeatureExtractor(request).Extract();
  const auto thresholds =
    offline_lc_minimal::DataDrivenStaticThresholdEstimator(config).Estimate(features);
  const auto windows =
    offline_lc_minimal::LateStaticWindowDetector(config).Detect(thresholds, &features);

  ExpectTrue(thresholds.valid, "log-Otsu thresholds should be valid for separated data");
  ExpectTrue(!windows.empty(), "late static detector should accept the terminal static period");
  ExpectTrue(windows.front().start_time_s >= 30.0,
             "accepted static window should not start inside the dynamic period");
  for (const auto &feature : features) {
    if (feature.window_center_time_s >= 10.0 && feature.window_center_time_s <= 30.0) {
      ExpectTrue(!feature.pass_all, "moving windows should not pass static gates");
    }
  }
}

void TestDetectorExcludesInitialStaticAndOutageWindows() {
  auto config = MakeLateStaticConfig();
  config.late_static_min_duration_s = 4.0;
  const auto gnss = MakeStaticDetectionGnss();
  const auto imu = MakeStaticDetectionImu();
  std::vector<offline_lc_minimal::RtkOutageWindowRow> outages(1U);
  outages.front().start_time_s = 40.0;
  outages.front().end_time_s = 50.0;

  offline_lc_minimal::LateStaticDetectionRequest request;
  request.config = &config;
  request.gnss_samples = &gnss;
  request.imu_samples = &imu;
  request.rtk_outage_windows = &outages;
  request.processing_start_time_s = 0.0;
  request.processing_end_time_s = 60.0;
  request.alignment_start_time_s = 35.0;
  request.alignment_end_time_s = 40.0;
  request.should_use_rtkfix_sample =
    [](const offline_lc_minimal::GnssSolutionSample &) { return true; };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  std::vector<offline_lc_minimal::LateStaticFeatureDiagnosticRow> features =
    offline_lc_minimal::LateStaticFeatureExtractor(request).Extract();
  const auto thresholds =
    offline_lc_minimal::DataDrivenStaticThresholdEstimator(config).Estimate(features);
  const auto windows =
    offline_lc_minimal::LateStaticWindowDetector(config).Detect(thresholds, &features);

  for (const auto &window : windows) {
    ExpectTrue(window.start_time_s >= 50.0,
               "accepted late-static windows should exclude initial static and outage spans");
  }
}

void TestInitialStaticPrefixIsSuppressedUntilDynamicEvidence() {
  auto config = MakeLateStaticConfig();
  const auto gnss = MakeInitialPrefixGnss();
  const auto imu = MakeInitialPrefixImu();

  offline_lc_minimal::LateStaticDetectionRequest request;
  request.config = &config;
  request.gnss_samples = &gnss;
  request.imu_samples = &imu;
  request.processing_start_time_s = 0.0;
  request.processing_end_time_s = 80.0;
  request.alignment_start_time_s = 0.0;
  request.alignment_end_time_s = 10.0;
  request.should_use_rtkfix_sample =
    [](const offline_lc_minimal::GnssSolutionSample &) { return true; };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  std::vector<offline_lc_minimal::LateStaticFeatureDiagnosticRow> features =
    offline_lc_minimal::LateStaticFeatureExtractor(request).Extract();
  const auto thresholds =
    offline_lc_minimal::DataDrivenStaticThresholdEstimator(config).Estimate(features);
  const auto windows =
    offline_lc_minimal::LateStaticWindowDetector(config).Detect(thresholds, &features);

  ExpectTrue(thresholds.valid, "thresholds should be valid for initial-prefix fixture");
  ExpectTrue(!windows.empty(), "terminal late static should still be accepted");
  ExpectTrue(windows.front().start_time_s >= 50.0,
             "initial static tail should be suppressed before dynamic evidence");
  bool saw_prefix_skip = false;
  for (const auto &feature : features) {
    if (feature.skip_reason == "INITIAL_STATIC_PREFIX") {
      saw_prefix_skip = true;
    }
  }
  ExpectTrue(saw_prefix_skip, "diagnostics should mark skipped initial static prefix rows");
}

void TestLateStaticVerticalBuilderAddsOnlyUpAndVzFactors() {
  auto config = MakeLateStaticConfig();
  std::vector<double> timestamps{0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0};
  std::vector<offline_lc_minimal::LateStaticWindowRow> windows(1U);
  windows.front().start_time_s = 2.0;
  windows.front().end_time_s = 5.0;
  windows.front().duration_s = 3.0;
  windows.front().valid = true;
  windows.front().rtk_median_up_m = 7.0;
  windows.front().skip_reason = "OK";

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  offline_lc_minimal::LateStaticVerticalConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.dynamic_start_index = 0U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.windows = &windows;
  offline_lc_minimal::LateStaticVerticalConstraintBuilder(request).Build();

  ExpectTrue(windows.front().vz_factor_count == 4U, "builder should add one vz prior per static state");
  ExpectTrue(windows.front().up_factor_count == 4U, "builder should add 1 Hz up priors at 1 Hz states");
  ExpectTrue(graph.size() == 8U, "builder should add only up and vz priors");
  const auto first_vz =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalVelocityPriorFactor>(graph.at(0));
  const auto first_up =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalPositionFactor>(graph.at(1));
  ExpectTrue(first_vz.get() != nullptr, "first factor should constrain vertical velocity");
  ExpectTrue(first_up.get() != nullptr, "second factor should constrain vertical position");
  ExpectTrue(summary.late_static_vz_factor_count == 4U,
             "summary should count vz priors");
  ExpectTrue(summary.late_static_up_factor_count == 4U,
             "summary should count up priors");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestDataDrivenDetectorFindsLateStaticAndRejectsDynamic",
      TestDataDrivenDetectorFindsLateStaticAndRejectsDynamic);
    RunTest(
      "TestDetectorExcludesInitialStaticAndOutageWindows",
      TestDetectorExcludesInitialStaticAndOutageWindows);
    RunTest(
      "TestInitialStaticPrefixIsSuppressedUntilDynamicEvidence",
      TestInitialStaticPrefixIsSuppressedUntilDynamicEvidence);
    RunTest(
      "TestLateStaticVerticalBuilderAddsOnlyUpAndVzFactors",
      TestLateStaticVerticalBuilderAddsOnlyUpAndVzFactors);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
