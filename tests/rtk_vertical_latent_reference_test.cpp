#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/core/RtkVerticalLatentReferenceBuilder.h"
#include "offline_lc_minimal/factor/VerticalRtkFactors.h"

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

offline_lc_minimal::OfflineRunnerConfig MakeConfig() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.enable_rtk_vertical_latent_reference = true;
  config.rtk_vertical_latent_reference_bin_s = 1.0;
  config.rtk_vertical_latent_reference_min_sample_count = 3;
  config.rtk_vertical_latent_reference_measurement_huber_sigma_m = 0.03;
  config.rtk_vertical_latent_reference_smooth_sigma_m = 0.005;
  config.vertical_envelope_gate_sigma_multiple = 2.0;
  config.vertical_envelope_min_half_width_m = 0.03;
  config.early_gnss_relaxation_duration_s = 0.0;
  return config;
}

offline_lc_minimal::RtkVerticalLatentReferenceBuildRequest MakeRequest(
  const offline_lc_minimal::OfflineRunnerConfig &config,
  const std::vector<offline_lc_minimal::GnssSolutionSample> &samples,
  gtsam::NonlinearFactorGraph &graph,
  gtsam::Values &initial_values,
  offline_lc_minimal::RunSummary &summary) {
  offline_lc_minimal::RtkVerticalLatentReferenceBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.first_sample_index = 0;
  request.reference_epoch_s = 0.0;
  request.dynamic_start_time_s = 100.0;
  request.graph = &graph;
  request.initial_values = &initial_values;
  request.run_summary = &summary;
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.01, 0.01, 0.015); };
  return request;
}

void TestBinsUseOneSecondTimeDomainWithoutInterpolation() {
  const auto config = MakeConfig();
  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.05, 1.0),
    MakeSample(0.35, 3.0),
    MakeSample(0.80, 2.0),
    MakeSample(1.10, 10.0),
  };
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  offline_lc_minimal::RunSummary summary;

  const auto result = offline_lc_minimal::RtkVerticalLatentReferenceBuilder(
                        MakeRequest(config, samples, graph, initial_values, summary))
                        .Build();

  ExpectNear(static_cast<double>(result.diagnostics.size()), 2.0, 0.0, "two 1s bins should be created");
  ExpectNear(static_cast<double>(summary.rtk_vertical_latent_reference_measurement_factor_count),
             4.0,
             0.0,
             "one raw-to-latent factor should be added per sample");
  ExpectNear(static_cast<double>(summary.rtk_vertical_latent_reference_smoothness_factor_count),
             1.0,
             0.0,
             "adjacent latent bins should get one smoothness factor");
  ExpectNear(graph.size(), 5.0, 0.0, "graph should contain measurement plus smoothness factors");
  ExpectTrue(initial_values.exists(offline_lc_minimal::RtkVerticalLatentReferenceKey(0)),
             "first latent reference key should be initialized");
  ExpectTrue(initial_values.exists(offline_lc_minimal::RtkVerticalLatentReferenceKey(1)),
             "second latent reference key should be initialized");
  ExpectNear(initial_values.at<double>(offline_lc_minimal::RtkVerticalLatentReferenceKey(0)),
             2.0,
             1e-12,
             "first bin initial reference should use weighted median");
  ExpectTrue(result.sample_references[0].valid && result.sample_references[0].key_index == 0,
             "first sample should map to first bin");
  ExpectTrue(result.sample_references[3].valid && result.sample_references[3].key_index == 1,
             "fourth sample should map to second bin");
  ExpectTrue(!result.diagnostics[0].low_sample_count, "first bin should satisfy min sample count");
  ExpectTrue(result.diagnostics[1].low_sample_count, "second bin should keep low-sample diagnostics");
}

void TestRejectedSamplesDoNotCreateReferences() {
  auto config = MakeConfig();
  config.drop_non_rtkfix = true;
  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeSample(0.05, 1.0),
    MakeSample(0.35, 2.0),
  };
  samples[1].gnssfgo_type_code = 2;
  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial_values;
  offline_lc_minimal::RunSummary summary;

  const auto result = offline_lc_minimal::RtkVerticalLatentReferenceBuilder(
                        MakeRequest(config, samples, graph, initial_values, summary))
                        .Build();

  ExpectNear(static_cast<double>(result.diagnostics.size()), 1.0, 0.0, "only accepted samples should create bins");
  ExpectTrue(result.sample_references[0].valid, "RTK fix sample should be mapped");
  ExpectTrue(!result.sample_references[1].valid, "RTK float sample should be rejected");
  ExpectTrue(result.sample_references[1].skip_reason == "SAMPLE_REJECTED",
             "rejected sample should explain why no latent reference was created");
}

}  // namespace

int main() {
  try {
    RunTest("TestBinsUseOneSecondTimeDomainWithoutInterpolation", TestBinsUseOneSecondTimeDomainWithoutInterpolation);
    RunTest("TestRejectedSamplesDoNotCreateReferences", TestRejectedSamplesDoNotCreateReferences);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
