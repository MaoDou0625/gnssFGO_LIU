#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "offline_lc_minimal/common/ResultOutputWriters.h"
#include "offline_lc_minimal/core/RtkVerticalDriftGateWeighting.h"
#include "offline_lc_minimal/core/RtkVerticalDriftReferenceEstimator.h"
#include "offline_lc_minimal/core/RtkVerticalLowpassReferenceFilter.h"

namespace {

namespace symbol = gtsam::symbol_shorthand;

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
  const std::vector<offline_lc_minimal::GnssSolutionSample> &samples,
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> *outage_windows = nullptr) {
  offline_lc_minimal::RtkVerticalDriftReferenceEstimateRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.rtk_outage_windows = outage_windows;
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

offline_lc_minimal::RtkOutageWindowRow MakeOutageWindow(
  const double start_time_s,
  const double end_time_s) {
  offline_lc_minimal::RtkOutageWindowRow window;
  window.start_time_s = start_time_s;
  window.end_time_s = end_time_s;
  return window;
}

gtsam::Values MakeZeroPoseValues(const std::size_t count) {
  gtsam::Values values;
  for (std::size_t index = 0; index < count; ++index) {
    values.insert(symbol::X(index), gtsam::Pose3());
  }
  return values;
}

gtsam::Values MakeConstantUpPoseValues(const std::size_t count, const double up_m) {
  gtsam::Values values;
  for (std::size_t index = 0; index < count; ++index) {
    values.insert(
      symbol::X(index),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, up_m)));
  }
  return values;
}

offline_lc_minimal::RtkVerticalDriftReferenceEstimateRequest MakeDynamicRequest(
  const offline_lc_minimal::OfflineRunnerConfig &config,
  const std::vector<offline_lc_minimal::GnssSolutionSample> &samples,
  const gtsam::Values &values,
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> *outage_windows = nullptr) {
  auto request = MakeRequest(config, samples, outage_windows);
  request.alignment_start_time_s = 1.0;
  request.alignment_end_time_s = 0.0;
  request.optimized_values = &values;
  request.find_state_for_time_s = [](const double corrected_time_s) {
    offline_lc_minimal::StateMeasSyncResult sync;
    sync.found_i = true;
    sync.key_index_i = static_cast<std::size_t>(std::llround(corrected_time_s));
    sync.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    return sync;
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

void TestOutageSegmentationBlocksPostOutageBackwardDrift() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_vertical_drift_outage_segmentation = true;
  config.enable_rtk_vertical_drift_gate_weighting = false;
  config.rtk_vertical_drift_sigma_m = 0.050;
  config.rtk_vertical_white_noise_sigma_m = 0.002;
  config.rtk_vertical_drift_huber_sigma_m = 1.0;
  config.rtk_vertical_drift_correlation_time_s = 100.0;
  config.rtk_vertical_drift_max_abs_correction_m = 1.0;

  std::vector<double> up_values(50U, 0.0);
  for (std::size_t index = 30U; index < up_values.size(); ++index) {
    up_values[index] = 0.080;
  }
  const auto samples = MakeStaticSamples(up_values);
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> outage_windows{
    MakeOutageWindow(29.5, 30.5)};

  const auto segmented =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(
      MakeRequest(config, samples, &outage_windows)).Estimate(nullptr);

  std::vector<double> pre_only_values(up_values.begin(), up_values.begin() + 30);
  const auto pre_only_samples = MakeStaticSamples(pre_only_values);
  const auto pre_only =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(
      MakeRequest(config, pre_only_samples)).Estimate(nullptr);

  ExpectTrue(segmented.profile[29U].valid, "pre-outage boundary sample should be valid");
  ExpectTrue(segmented.profile[30U].skip_reason == "inside_rtk_outage",
             "sample inside outage should not be used for drift reference");
  ExpectTrue(segmented.profile[31U].valid, "post-outage sample should be valid");
  ExpectNear(
    segmented.profile[29U].drift_estimate_m,
    pre_only.profile[29U].drift_estimate_m,
    1.0e-12,
    "post-outage drift should not pull the pre-outage smoother");
  ExpectTrue(segmented.profile[29U].drift_segment_index == 0,
             "pre-outage sample should stay in segment 0");
  ExpectTrue(segmented.profile[31U].drift_segment_index == 1,
             "post-outage sample should start a new segment");
  ExpectTrue(segmented.profile[29U].outage_boundary_blocked &&
               segmented.profile[31U].outage_boundary_blocked,
             "boundary-adjacent rows should report blocked outage propagation");

  offline_lc_minimal::OfflineRunnerConfig unsegmented_config = config;
  unsegmented_config.enable_rtk_vertical_drift_outage_segmentation = false;
  const auto unsegmented =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(
      MakeRequest(unsegmented_config, samples, &outage_windows)).Estimate(nullptr);
  ExpectTrue(
    std::abs(unsegmented.profile[29U].drift_estimate_m) >
      std::abs(segmented.profile[29U].drift_estimate_m) + 1.0e-4,
    "unsegmented smoother should show the old backward pull");
}

void TestOutageSegmentationUsesPerSegmentBiasWhenStaticBiasUnavailable() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_vertical_drift_outage_segmentation = true;
  config.enable_rtk_vertical_drift_gate_weighting = false;
  config.rtk_vertical_drift_sigma_m = 0.050;
  config.rtk_vertical_white_noise_sigma_m = 0.002;
  config.rtk_vertical_drift_huber_sigma_m = 1.0;
  config.rtk_vertical_drift_correlation_time_s = 100.0;
  config.rtk_vertical_drift_max_abs_correction_m = 1.0;

  std::vector<double> up_values(50U, 0.0);
  for (std::size_t index = 30U; index < up_values.size(); ++index) {
    up_values[index] = 0.080;
  }
  const auto samples = MakeStaticSamples(up_values);
  const gtsam::Values values = MakeZeroPoseValues(samples.size());
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> outage_windows{
    MakeOutageWindow(29.5, 30.5)};

  const auto segmented =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(
      MakeDynamicRequest(config, samples, values, &outage_windows)).Estimate(nullptr);

  std::vector<double> pre_only_values(up_values.begin(), up_values.begin() + 30);
  const auto pre_only_samples = MakeStaticSamples(pre_only_values);
  const gtsam::Values pre_only_values_graph = MakeZeroPoseValues(pre_only_samples.size());
  const auto pre_only =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(
      MakeDynamicRequest(config, pre_only_samples, pre_only_values_graph)).Estimate(nullptr);

  ExpectNear(
    segmented.profile[29U].constant_bias_m,
    pre_only.profile[29U].constant_bias_m,
    1.0e-12,
    "fallback constant bias should be estimated from the pre-outage segment only");
  ExpectNear(
    segmented.profile[29U].drift_estimate_m,
    pre_only.profile[29U].drift_estimate_m,
    1.0e-12,
    "post-outage samples should not influence pre-outage fallback bias or smoothing");
}

void TestOutageSegmentationCreatesMultipleIndependentSegments() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_vertical_drift_outage_segmentation = true;

  const auto samples = MakeStaticSamples(std::vector<double>(8U, 0.0));
  const std::vector<offline_lc_minimal::RtkOutageWindowRow> outage_windows{
    MakeOutageWindow(2.5, 3.5),
    MakeOutageWindow(5.5, 6.5)};
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(
      MakeRequest(config, samples, &outage_windows)).Estimate(nullptr);

  ExpectTrue(result.profile[0U].drift_segment_role == "PRE_OUTAGE",
             "first segment should be labeled pre-outage");
  ExpectTrue(result.profile[4U].drift_segment_index == 1,
             "middle segment should have index 1");
  ExpectTrue(result.profile[4U].drift_segment_role == "BETWEEN_OUTAGES",
             "middle segment should be labeled between outages");
  ExpectTrue(result.profile[7U].drift_segment_index == 2,
             "last segment should have index 2");
  ExpectTrue(result.profile[7U].drift_segment_role == "POST_OUTAGE",
             "last segment should be labeled post-outage");
  ExpectTrue(result.profile[3U].skip_reason == "inside_rtk_outage" &&
               result.profile[6U].skip_reason == "inside_rtk_outage",
             "samples inside outage windows should be excluded");
}

void TestLowpassReferenceDoesNotCrossOutageSegments() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_vertical_lowpass_reference = true;
  config.rtk_vertical_lowpass_reference_cutoff_hz = 0.10;

  std::vector<offline_lc_minimal::RtkVerticalDriftReferenceDiagnosticRow> profile(40U);
  for (std::size_t index = 0; index < profile.size(); ++index) {
    auto &row = profile[index];
    row.sample_index = index;
    row.time_s = static_cast<double>(index);
    row.corrected_center_up_m = index < 20U ? 0.0 : 1.0;
    row.drift_segment_index = index < 20U ? 0 : 1;
    row.drift_segment_role = index < 20U ? "PRE_OUTAGE" : "POST_OUTAGE";
    row.valid = true;
  }

  const auto summary =
    offline_lc_minimal::ApplyRtkVerticalLowpassReferenceFilter(config, &profile);
  ExpectTrue(summary.valid_count == profile.size(), "all valid rows should be filtered");
  ExpectNear(profile[19U].lowpass_center_up_m, 0.0, 1.0e-12,
             "pre-outage lowpass should not see post-outage level");
  ExpectNear(profile[20U].lowpass_center_up_m, 1.0, 1.0e-12,
             "post-outage lowpass should start from post segment only");
  ExpectTrue(profile[19U].outage_boundary_blocked && profile[20U].outage_boundary_blocked,
             "lowpass split should mark the outage boundary rows");
}

void TestCausalReferenceOverridesPreOutageFullOptimizedReference() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_outage_causal_drift_reference = true;
  config.enable_rtk_vertical_drift_gate_weighting = false;
  config.rtk_vertical_drift_max_abs_correction_m = 10.0;
  config.rtk_vertical_drift_huber_sigma_m = 10.0;

  const auto samples = MakeStaticSamples({1.0, 1.0, 1.0, 1.0, 1.0});
  const gtsam::Values values = MakeConstantUpPoseValues(samples.size(), 10.0);
  std::vector<offline_lc_minimal::RtkOutageCausalNavReferenceRow> causal_rows(samples.size());
  for (std::size_t index = 0; index < causal_rows.size(); ++index) {
    causal_rows[index].sample_index = index;
    causal_rows[index].time_s = static_cast<double>(index);
    causal_rows[index].causal_nav_reference_up_m = 2.0;
    causal_rows[index].outage_boundary_time_s = 2.0;
    causal_rows[index].valid = index <= 2U;
    causal_rows[index].skip_reason = causal_rows[index].valid ? "OK" : "after_causal_boundary";
  }

  auto request = MakeDynamicRequest(config, samples, values);
  request.causal_nav_reference_profile = &causal_rows;
  request.causal_nav_reference_end_time_s = 2.0;
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(std::move(request)).Estimate(nullptr);

  ExpectTrue(result.profile[1U].valid, "pre-outage causal row should be valid");
  ExpectNear(
    result.profile[1U].nav_reference_up_m,
    2.0,
    1.0e-12,
    "pre-outage nav reference should use causal prefix value");
  ExpectTrue(
    result.profile[1U].nav_reference_source == "CAUSAL_PREFIX",
    "pre-outage row should report causal source");
  ExpectNear(
    result.profile[1U].full_reference_up_m,
    10.0,
    1.0e-12,
    "diagnostic should retain full optimized reference");
  ExpectNear(
    result.profile[1U].full_minus_causal_nav_reference_m,
    8.0,
    1.0e-12,
    "diagnostic should report full-minus-causal difference");
  ExpectNear(
    result.profile[3U].nav_reference_up_m,
    10.0,
    1.0e-12,
    "post-outage nav reference should keep full optimized value");
  ExpectTrue(
    result.profile[3U].nav_reference_source == "FULL_OPTIMIZED",
    "post-outage row should report full optimized source");
}

void TestMissingCausalReferenceDoesNotFallBackToFullBeforeBoundary() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_outage_causal_drift_reference = true;

  const auto samples = MakeStaticSamples({1.0, 1.0, 1.0});
  const gtsam::Values values = MakeConstantUpPoseValues(samples.size(), 10.0);
  std::vector<offline_lc_minimal::RtkOutageCausalNavReferenceRow> causal_rows(samples.size());
  causal_rows[0].valid = true;
  causal_rows[0].causal_nav_reference_up_m = 2.0;

  auto request = MakeDynamicRequest(config, samples, values);
  request.causal_nav_reference_profile = &causal_rows;
  request.causal_nav_reference_end_time_s = 2.0;
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(std::move(request)).Estimate(nullptr);

  ExpectTrue(!result.profile[1U].valid, "missing causal pre-outage row should be skipped");
  ExpectTrue(
    result.profile[1U].skip_reason == "missing_causal_nav_reference",
    "pre-outage missing causal reference should not fall back to full optimized state");
}

void TestLateStaticReferenceOverridesFullOptimizedReference() {
  offline_lc_minimal::OfflineRunnerConfig config;
  config.enable_rtk_vertical_drift_gate_weighting = false;
  config.rtk_vertical_drift_max_abs_correction_m = 10.0;
  config.rtk_vertical_drift_huber_sigma_m = 10.0;

  const auto samples = MakeStaticSamples({1.0, 1.0, 1.0, 1.0, 1.0});
  const gtsam::Values values = MakeConstantUpPoseValues(samples.size(), 10.0);
  std::vector<offline_lc_minimal::LateStaticWindowRow> late_static_windows(1U);
  late_static_windows.front().start_time_s = 1.0;
  late_static_windows.front().end_time_s = 3.0;
  late_static_windows.front().valid = true;
  late_static_windows.front().rtk_median_up_m = 1.0;
  late_static_windows.front().skip_reason = "OK";

  auto request = MakeDynamicRequest(config, samples, values);
  request.late_static_windows = &late_static_windows;
  const auto result =
    offline_lc_minimal::RtkVerticalDriftReferenceEstimator(std::move(request)).Estimate(nullptr);

  ExpectTrue(result.profile[2U].valid, "late-static row should be valid");
  ExpectNear(
    result.profile[2U].nav_reference_up_m,
    1.0,
    1.0e-12,
    "late-static row should use RTK median static reference");
  ExpectTrue(
    result.profile[2U].nav_reference_source == "LATE_STATIC_RTK_REFERENCE",
    "late-static row should report late-static reference source");
  ExpectTrue(
    result.profile[2U].static_window_flag,
    "late-static row should be marked static");
  ExpectTrue(
    result.profile[2U].static_window_source == "LATE_STATIC",
    "late-static row should record static source");
  ExpectNear(
    result.profile[4U].nav_reference_up_m,
    10.0,
    1.0e-12,
    "non-static row should still use full optimized reference");
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

void TestOutageSegmentDiagnosticsWriterIncludesColumns() {
  std::vector<offline_lc_minimal::RtkVerticalDriftReferenceDiagnosticRow> rows(1U);
  rows.front().sample_index = 7U;
  rows.front().time_s = 1.0;
  rows.front().drift_segment_index = 1;
  rows.front().drift_segment_role = "POST_OUTAGE";
  rows.front().outage_boundary_blocked = true;

  const auto path =
    std::filesystem::temp_directory_path() / "rtk_vertical_drift_reference_segment_writer_test.csv";
  offline_lc_minimal::WriteRtkVerticalDriftReferenceDiagnosticsCsv(path, rows);
  std::ifstream stream(path);
  std::string header;
  std::getline(stream, header);
  stream.close();
  std::filesystem::remove(path);
  ExpectTrue(
    header.find("drift_segment_index,drift_segment_role,outage_boundary_blocked") !=
      std::string::npos,
    "CSV writer should include outage segment diagnostics");
}

void TestCausalDiagnosticsWriterIncludesColumns() {
  std::vector<offline_lc_minimal::RtkVerticalDriftReferenceDiagnosticRow> rows(1U);
  rows.front().sample_index = 7U;
  rows.front().time_s = 1.0;
  rows.front().nav_reference_source = "CAUSAL_PREFIX";
  rows.front().causal_reference_up_m = 2.0;
  rows.front().full_reference_up_m = 3.0;
  rows.front().full_minus_causal_nav_reference_m = 1.0;
  rows.front().causal_reference_boundary_time_s = 10.0;

  const auto path =
    std::filesystem::temp_directory_path() / "rtk_vertical_drift_reference_causal_writer_test.csv";
  offline_lc_minimal::WriteRtkVerticalDriftReferenceDiagnosticsCsv(path, rows);
  std::ifstream stream(path);
  std::string header;
  std::getline(stream, header);
  stream.close();
  std::filesystem::remove(path);
  ExpectTrue(
    header.find("nav_reference_source,static_window_source,causal_reference_up_m,full_reference_up_m,"
                "full_minus_causal_nav_reference_m,causal_reference_boundary_time_s") !=
      std::string::npos,
    "CSV writer should include causal reference diagnostics");
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
    RunTest(
      "TestOutageSegmentationBlocksPostOutageBackwardDrift",
      TestOutageSegmentationBlocksPostOutageBackwardDrift);
    RunTest(
      "TestOutageSegmentationUsesPerSegmentBiasWhenStaticBiasUnavailable",
      TestOutageSegmentationUsesPerSegmentBiasWhenStaticBiasUnavailable);
    RunTest(
      "TestOutageSegmentationCreatesMultipleIndependentSegments",
      TestOutageSegmentationCreatesMultipleIndependentSegments);
    RunTest(
      "TestLowpassReferenceDoesNotCrossOutageSegments",
      TestLowpassReferenceDoesNotCrossOutageSegments);
    RunTest(
      "TestCausalReferenceOverridesPreOutageFullOptimizedReference",
      TestCausalReferenceOverridesPreOutageFullOptimizedReference);
    RunTest(
      "TestMissingCausalReferenceDoesNotFallBackToFullBeforeBoundary",
      TestMissingCausalReferenceDoesNotFallBackToFullBeforeBoundary);
    RunTest(
      "TestLateStaticReferenceOverridesFullOptimizedReference",
      TestLateStaticReferenceOverridesFullOptimizedReference);
    RunTest("TestGateWeightFormula", TestGateWeightFormula);
    RunTest("TestGateWeightFloor", TestGateWeightFloor);
    RunTest("TestEstimatorWritesGateDiagnostics", TestEstimatorWritesGateDiagnostics);
    RunTest(
      "TestEstimatorGateWeightingReducesLargeDriftEstimate",
      TestEstimatorGateWeightingReducesLargeDriftEstimate);
    RunTest("TestGateDiagnosticsWriterIncludesColumns", TestGateDiagnosticsWriterIncludesColumns);
    RunTest(
      "TestOutageSegmentDiagnosticsWriterIncludesColumns",
      TestOutageSegmentDiagnosticsWriterIncludesColumns);
    RunTest(
      "TestCausalDiagnosticsWriterIncludesColumns",
      TestCausalDiagnosticsWriterIncludesColumns);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
