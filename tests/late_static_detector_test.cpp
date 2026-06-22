#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/core/InitialDynamicStaticConstraintBuilder.h"
#include "offline_lc_minimal/core/InitialDynamicStaticDetector.h"
#include "offline_lc_minimal/core/LateStaticDetector.h"
#include "offline_lc_minimal/core/LateStaticReferenceEstimator.h"
#include "offline_lc_minimal/core/LateStaticVerticalConstraintBuilder.h"
#include "offline_lc_minimal/factor/StaticVerticalPositionHoldFactor.h"
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

std::vector<offline_lc_minimal::GnssSolutionSample> MakeInitialDynamicStaticGnss() {
  std::vector<offline_lc_minimal::GnssSolutionSample> samples;
  for (int t = 0; t <= 130; ++t) {
    const double time_s = static_cast<double>(t);
    const double east_m =
      time_s <= 110.0
        ? 0.001 * std::sin(0.2 * time_s)
        : 0.001 * std::sin(0.2 * 110.0) + 0.8 * (time_s - 110.0);
    samples.push_back(
      MakeGnssSample(time_s, east_m, 2.0 + 0.001 * std::sin(0.5 * time_s)));
  }
  return samples;
}

std::vector<offline_lc_minimal::ImuSample> MakeInitialDynamicStaticImu() {
  std::vector<offline_lc_minimal::ImuSample> samples;
  for (int i = 0; i <= 260; ++i) {
    const double time_s = 0.5 * static_cast<double>(i);
    offline_lc_minimal::ImuSample sample;
    sample.time_s = time_s;
    const double gyro = time_s <= 110.0 ? 0.0001 : 0.08;
    sample.gyro_radps = Eigen::Vector3d(gyro, 0.0, 0.0);
    const double accel_noise = time_s <= 110.0 ? 0.001 * std::sin(time_s)
                                               : 0.08 * std::sin(3.0 * time_s);
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81 + accel_noise);
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
  config.late_static_gyro_threshold_scale = 1.05;
  config.late_static_acc_norm_std_threshold_mps2 = 0.001;
  config.late_static_min_rtkfix_samples = 5;
  config.late_static_merge_gap_s = 5.0;
  config.late_static_exclude_initial_static = true;
  config.late_static_exclude_rtk_outage = true;
  config.late_static_vz_sigma_mps = 0.0005;
  config.late_static_up_sigma_m = 0.02;
  config.late_static_height_hold_sigma_m = 0.001;
  config.enable_initial_dynamic_static_detection = true;
  config.initial_dynamic_static_search_duration_s = 20.0;
  config.initial_dynamic_static_threshold_multiplier = 3.0;
  config.initial_dynamic_static_min_duration_s = 5.0;
  config.initial_dynamic_static_merge_gap_s = 2.0;
  config.enable_initial_dynamic_static_lowpass_protection = true;
  config.initial_dynamic_static_lowpass_blend_s = 1.0;
  config.enable_initial_dynamic_static_vz_constraint = true;
  config.initial_dynamic_static_vz_sigma_mps = 0.0005;
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

offline_lc_minimal::LateStaticFeatureDiagnosticRow MakeFeatureRow(
  const double start_time_s,
  const double end_time_s,
  const double rtk_speed_rms_mps,
  const double rtk_range_m,
  const double gyro_rms_radps,
  const double gyro_p95_radps,
  const double acc_std_mps2) {
  offline_lc_minimal::LateStaticFeatureDiagnosticRow row;
  row.window_start_time_s = start_time_s;
  row.window_end_time_s = end_time_s;
  row.window_center_time_s = 0.5 * (start_time_s + end_time_s);
  row.valid_features = true;
  row.rtk_horizontal_speed_rms_mps = rtk_speed_rms_mps;
  row.rtk_horizontal_range_m = rtk_range_m;
  row.rtk_up_median_m = 1.0;
  row.rtk_up_range_m = 0.002;
  row.imu_gyro_norm_rms_radps = gyro_rms_radps;
  row.imu_gyro_norm_p95_radps = gyro_p95_radps;
  row.imu_acc_norm_std_mps2 = acc_std_mps2;
  return row;
}

void TestLateStaticGyroScaleAndAccelStdGate() {
  auto config = MakeLateStaticConfig();
  config.late_static_stride_s = 0.5;
  config.late_static_merge_gap_s = 2.0;
  config.late_static_min_duration_s = 8.0;

  std::vector<offline_lc_minimal::LateStaticFeatureDiagnosticRow> features;
  for (int i = 0; i < 24; ++i) {
    const double start_time_s = 0.5 * static_cast<double>(i);
    const bool stop_transition = i < 4;
    features.push_back(
      MakeFeatureRow(
        start_time_s,
        start_time_s + 5.0,
        0.01,
        0.01,
        0.000124,
        0.000195,
        stop_transition ? 0.12 : 0.0004));
  }

  offline_lc_minimal::LateStaticThresholdSet thresholds;
  thresholds.valid = true;
  thresholds.rtk_speed_rms_threshold_mps = 0.13;
  thresholds.rtk_horizontal_range_threshold_m = 0.12;
  thresholds.gyro_rms_threshold_radps = 0.00012182 * config.late_static_gyro_threshold_scale;
  thresholds.gyro_p95_threshold_radps = 0.00019201 * config.late_static_gyro_threshold_scale;
  thresholds.acc_norm_std_threshold_mps2 = config.late_static_acc_norm_std_threshold_mps2;

  const auto windows =
    offline_lc_minimal::LateStaticWindowDetector(config).Detect(thresholds, &features);

  ExpectTrue(!windows.empty(), "gyro scale should accept quiet terminal static windows");
  ExpectTrue(windows.front().start_time_s >= 2.0,
             "acc norm std gate should reject stop-transition windows");
  for (const auto &feature : features) {
    if (feature.window_start_time_s < 2.0) {
      ExpectTrue(!feature.pass_acc_std, "high accel std transition windows should fail");
      ExpectTrue(!feature.pass_all, "transition windows should not pass all gates");
    }
  }
}

void TestLateStaticEstimatorAppliesGyroScale() {
  auto base_config = MakeLateStaticConfig();
  base_config.late_static_gyro_threshold_scale = 1.0;
  auto scaled_config = base_config;
  scaled_config.late_static_gyro_threshold_scale = 1.05;

  std::vector<offline_lc_minimal::LateStaticFeatureDiagnosticRow> features;
  for (int i = 0; i < 12; ++i) {
    features.push_back(
      MakeFeatureRow(
        static_cast<double>(i),
        static_cast<double>(i) + 5.0,
        0.01,
        0.01,
        0.0001,
        0.00018,
        0.0004));
  }
  for (int i = 12; i < 24; ++i) {
    features.push_back(
      MakeFeatureRow(
        static_cast<double>(i),
        static_cast<double>(i) + 5.0,
        1.0,
        5.0,
        0.01,
        0.02,
        0.12));
  }

  const auto base_thresholds =
    offline_lc_minimal::DataDrivenStaticThresholdEstimator(base_config).Estimate(features);
  const auto scaled_thresholds =
    offline_lc_minimal::DataDrivenStaticThresholdEstimator(scaled_config).Estimate(features);

  ExpectTrue(base_thresholds.valid && scaled_thresholds.valid,
             "threshold estimator should produce valid thresholds");
  ExpectTrue(
    std::abs(
      scaled_thresholds.gyro_rms_threshold_radps -
      base_thresholds.gyro_rms_threshold_radps * 1.05) < 1e-15,
    "gyro RMS threshold should be scaled");
  ExpectTrue(
    std::abs(
      scaled_thresholds.gyro_p95_threshold_radps -
      base_thresholds.gyro_p95_threshold_radps * 1.05) < 1e-15,
    "gyro P95 threshold should be scaled");
  ExpectTrue(
    std::abs(
      scaled_thresholds.acc_norm_std_threshold_mps2 -
      scaled_config.late_static_acc_norm_std_threshold_mps2) < 1e-15,
    "acc norm std threshold should come from config");
}

void TestLateStaticEstimatorUsesInitialStaticAccelStdThreshold() {
  auto config = MakeLateStaticConfig();
  config.late_static_initial_acc_norm_std_scale = 1.2;
  config.late_static_initial_acc_norm_std_min_sample_count = 10;

  std::vector<offline_lc_minimal::LateStaticFeatureDiagnosticRow> features;
  for (int i = 0; i < 12; ++i) {
    features.push_back(
      MakeFeatureRow(
        static_cast<double>(i),
        static_cast<double>(i) + 5.0,
        0.01,
        0.01,
        0.0001,
        0.00018,
        0.04));
  }
  for (int i = 12; i < 24; ++i) {
    features.push_back(
      MakeFeatureRow(
        static_cast<double>(i),
        static_cast<double>(i) + 5.0,
        1.0,
        5.0,
        0.01,
        0.02,
        0.12));
  }

  std::vector<offline_lc_minimal::ImuSample> imu_samples;
  for (int i = 0; i < 120; ++i) {
    offline_lc_minimal::ImuSample sample;
    sample.time_s = 0.5 * static_cast<double>(i);
    sample.gyro_radps = Eigen::Vector3d::Zero();
    const double alternating_noise_mps2 = (i % 2 == 0) ? -0.1 : 0.1;
    sample.accel_mps2 = Eigen::Vector3d(0.0, 0.0, 9.81 + alternating_noise_mps2);
    imu_samples.push_back(sample);
  }

  const auto thresholds =
    offline_lc_minimal::DataDrivenStaticThresholdEstimator(config).Estimate(
      features,
      &imu_samples,
      0.0,
      60.0);

  ExpectTrue(thresholds.valid, "threshold estimator should accept initial static IMU data");
  ExpectTrue(
    std::abs(thresholds.acc_norm_std_threshold_mps2 - 0.12) < 1e-12,
    "acc norm std threshold should scale the initial static accel norm std");
  ExpectTrue(
    thresholds.diagnostics.back().method == "initial_static_scaled",
    "acc norm std diagnostic should report initial-static threshold source");
  ExpectTrue(
    thresholds.diagnostics.back().sample_count == 120U,
    "acc norm std diagnostic should report initial static IMU sample count");
}

void TestLateStaticReferenceEstimatorUsesWindowRtkHuberMean() {
  std::vector<offline_lc_minimal::LateStaticWindowRow> windows(1U);
  windows.front().start_time_s = 10.0;
  windows.front().end_time_s = 15.0;
  windows.front().duration_s = 5.0;
  windows.front().valid = true;
  windows.front().rtk_median_up_m = 1.0;
  windows.front().rtk_up_range_m = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> gnss;
  const std::vector<double> up_values = {0.000, 0.001, 0.002, 0.003, 0.004, 0.100};
  for (std::size_t i = 0; i < up_values.size(); ++i) {
    auto sample = MakeGnssSample(10.0 + static_cast<double>(i), 0.0, up_values[i]);
    sample.enu_position_m.z() = up_values[i];
    gnss.push_back(sample);
  }
  auto outside = MakeGnssSample(20.0, 0.0, 5.0);
  outside.enu_position_m.z() = 5.0;
  gnss.push_back(outside);
  auto no_solution = MakeGnssSample(12.0, 0.0, -5.0);
  no_solution.enu_position_m.z() = -5.0;
  no_solution.gnssfgo_type_code = 4;
  gnss.push_back(no_solution);

  offline_lc_minimal::LateStaticReferenceEstimationRequest request;
  request.gnss_samples = &gnss;
  request.windows = &windows;
  request.should_use_rtkfix_sample =
    [](const offline_lc_minimal::GnssSolutionSample &) { return true; };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  offline_lc_minimal::LateStaticReferenceEstimator(std::move(request)).Estimate();

  ExpectTrue(windows.front().valid, "window should remain valid with RTK samples");
  ExpectTrue(windows.front().rtk_median_up_m > 0.0,
             "Huber reference should use in-window RTK samples");
  ExpectTrue(windows.front().rtk_median_up_m < 0.010,
             "Huber reference should not be pulled to the large outlier");
  ExpectTrue(std::abs(windows.front().rtk_up_range_m - 0.100) < 1e-12,
             "RTK Up range should use all in-window RTKFIX samples");
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
  ExpectTrue(windows.front().up_factor_count == 4U, "builder should add one up prior per static state");
  ExpectTrue(
    windows.front().height_hold_factor_count == 3U,
    "builder should add one height hold factor from the reference state to each later static state");
  ExpectTrue(
    windows.front().height_hold_factor_state_index_pairs.size() == 3U,
    "builder should record height hold factor state pairs");
  ExpectTrue(
    windows.front().height_hold_factor_state_index_pairs[0U] == std::make_pair<std::size_t, std::size_t>(2U, 3U) &&
      windows.front().height_hold_factor_state_index_pairs[1U] == std::make_pair<std::size_t, std::size_t>(2U, 4U) &&
      windows.front().height_hold_factor_state_index_pairs[2U] == std::make_pair<std::size_t, std::size_t>(2U, 5U),
    "late-static height hold should use the window's first state as the fixed reference");
  ExpectTrue(graph.size() == 11U, "builder should add up, vz, and height hold priors");
  const auto first_vz =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalVelocityPriorFactor>(graph.at(0));
  const auto first_up =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalPositionFactor>(graph.at(1));
  const auto first_hold =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::StaticVerticalPositionHoldFactor>(graph.at(4));
  ExpectTrue(first_vz.get() != nullptr, "first factor should constrain vertical velocity");
  ExpectTrue(first_up.get() != nullptr, "second factor should constrain vertical position");
  ExpectTrue(first_hold.get() != nullptr, "builder should add static height hold factors");
  ExpectTrue(summary.late_static_vz_factor_count == 4U,
             "summary should count vz priors");
  ExpectTrue(summary.late_static_up_factor_count == 4U,
             "summary should count up priors");
  ExpectTrue(summary.late_static_height_hold_factor_count == 3U,
             "summary should count height hold factors");
}

void TestInitialDynamicStaticDetectorUsesBaselineImuAndRtkEvidence() {
  auto config = MakeLateStaticConfig();
  config.late_static_window_s = 2.0;
  config.late_static_stride_s = 1.0;
  config.late_static_min_rtkfix_samples = 2;
  const auto gnss = MakeInitialDynamicStaticGnss();
  const auto imu = MakeInitialDynamicStaticImu();

  offline_lc_minimal::InitialDynamicStaticDetectionRequest request;
  request.config = &config;
  request.gnss_samples = &gnss;
  request.imu_samples = &imu;
  request.alignment_start_time_s = 0.0;
  request.alignment_end_time_s = 100.0;
  request.dynamic_start_time_s = 100.0;
  request.processing_end_time_s = 130.0;
  request.should_use_rtkfix_sample =
    [](const offline_lc_minimal::GnssSolutionSample &) { return true; };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  const auto result =
    offline_lc_minimal::InitialDynamicStaticDetector(std::move(request)).Detect();
  ExpectTrue(!result.threshold_diagnostics.empty(),
             "initial dynamic static detector should produce thresholds");
  ExpectTrue(!result.windows.empty(),
             "detector should accept the static tail after dynamic start");
  ExpectTrue(result.windows.front().start_time_s <= 101.0,
             "static window should start near dynamic start");
  ExpectTrue(result.windows.front().end_time_s <= 112.0,
             "static window should stop when RTK/IMU show motion");
  bool saw_dynamic_rejection = false;
  for (const auto &row : result.feature_diagnostics) {
    if (row.window_center_time_s > 112.0 && !row.pass_all) {
      saw_dynamic_rejection = true;
    }
  }
  ExpectTrue(saw_dynamic_rejection,
             "moving candidate windows should be rejected by the combined gates");
}

void TestInitialDynamicStaticDetectorAcceptsShortDynamicPrefix() {
  auto config = MakeLateStaticConfig();
  config.late_static_window_s = 5.0;
  config.late_static_stride_s = 0.5;
  config.initial_dynamic_static_min_duration_s = 8.0;
  config.late_static_min_rtkfix_samples = 5;
  const auto gnss = MakeInitialDynamicStaticGnss();
  const auto imu = MakeInitialDynamicStaticImu();

  offline_lc_minimal::InitialDynamicStaticDetectionRequest request;
  request.config = &config;
  request.gnss_samples = &gnss;
  request.imu_samples = &imu;
  request.alignment_start_time_s = 0.0;
  request.alignment_end_time_s = 100.0;
  request.dynamic_start_time_s = 100.0;
  request.processing_end_time_s = 106.0;
  request.should_use_rtkfix_sample =
    [](const offline_lc_minimal::GnssSolutionSample &) { return true; };
  request.corrected_time_s =
    [](const offline_lc_minimal::GnssSolutionSample &sample) {
      return sample.time_s;
    };

  const auto result =
    offline_lc_minimal::InitialDynamicStaticDetector(std::move(request)).Detect();
  ExpectTrue(!result.windows.empty(),
             "detector should keep an initial static prefix even below normal min duration");
  ExpectTrue(result.windows.front().duration_s < config.initial_dynamic_static_min_duration_s,
             "fixture should exercise the short-prefix path");
  ExpectTrue(result.windows.front().duration_s >= config.late_static_window_s,
             "short prefix should still cover at least one full feature window");
}

void TestInitialDynamicStaticConstraintAddsVerticalStaticPriors() {
  auto config = MakeLateStaticConfig();
  std::vector<double> timestamps{99.0, 100.0, 101.0, 102.0, 103.0, 104.0, 105.0};
  std::vector<offline_lc_minimal::LateStaticWindowRow> windows(1U);
  windows.front().start_time_s = 100.0;
  windows.front().end_time_s = 104.0;
  windows.front().duration_s = 4.0;
  windows.front().valid = true;
  windows.front().rtk_median_up_m = 7.0;
  windows.front().skip_reason = "OK";

  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  offline_lc_minimal::InitialDynamicStaticConstraintBuildRequest request;
  request.config = &config;
  request.state_timestamps = &timestamps;
  request.dynamic_start_index = 1U;
  request.graph = &graph;
  request.run_summary = &summary;
  request.windows = &windows;
  offline_lc_minimal::InitialDynamicStaticConstraintBuilder(std::move(request)).Build();

  ExpectTrue(windows.front().vz_factor_count == 5U,
             "initial dynamic static constraint should add one vz prior per static state");
  ExpectTrue(windows.front().up_factor_count == 5U,
             "initial dynamic static constraint should add one up prior per static state");
  ExpectTrue(windows.front().height_hold_factor_count == 4U,
             "initial dynamic static constraint should add window-internal height holds");
  ExpectTrue(graph.size() == 14U,
             "graph should contain vertical velocity, up, and height-hold priors");
  const auto first_vz =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalVelocityPriorFactor>(graph.at(0));
  const auto first_up =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::VerticalPositionFactor>(graph.at(1));
  const auto first_hold =
    boost::dynamic_pointer_cast<offline_lc_minimal::factor::StaticVerticalPositionHoldFactor>(graph.at(4));
  ExpectTrue(first_vz.get() != nullptr, "first factor should be a vertical velocity prior");
  ExpectTrue(first_up.get() != nullptr, "second factor should be a vertical position prior");
  ExpectTrue(first_hold.get() != nullptr, "builder should add static height hold factors");
  ExpectTrue(summary.initial_dynamic_static_vz_factor_count == 5U,
             "summary should count initial dynamic static vz priors");
  ExpectTrue(summary.initial_dynamic_static_up_factor_count == 5U,
             "summary should count initial dynamic static up priors");
  ExpectTrue(summary.initial_dynamic_static_height_hold_factor_count == 4U,
             "summary should count initial dynamic static height holds");
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
      "TestLateStaticGyroScaleAndAccelStdGate",
      TestLateStaticGyroScaleAndAccelStdGate);
    RunTest(
      "TestLateStaticEstimatorAppliesGyroScale",
      TestLateStaticEstimatorAppliesGyroScale);
    RunTest(
      "TestLateStaticEstimatorUsesInitialStaticAccelStdThreshold",
      TestLateStaticEstimatorUsesInitialStaticAccelStdThreshold);
    RunTest(
      "TestLateStaticReferenceEstimatorUsesWindowRtkHuberMean",
      TestLateStaticReferenceEstimatorUsesWindowRtkHuberMean);
    RunTest(
      "TestLateStaticVerticalBuilderAddsOnlyUpAndVzFactors",
      TestLateStaticVerticalBuilderAddsOnlyUpAndVzFactors);
    RunTest(
      "TestInitialDynamicStaticDetectorUsesBaselineImuAndRtkEvidence",
      TestInitialDynamicStaticDetectorUsesBaselineImuAndRtkEvidence);
    RunTest(
      "TestInitialDynamicStaticDetectorAcceptsShortDynamicPrefix",
      TestInitialDynamicStaticDetectorAcceptsShortDynamicPrefix);
    RunTest(
      "TestInitialDynamicStaticConstraintAddsVerticalStaticPriors",
      TestInitialDynamicStaticConstraintAddsVerticalStaticPriors);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
