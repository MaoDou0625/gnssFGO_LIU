#include <iostream>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include "offline_lc_minimal/core/GnssFactorBuilder.h"

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
    throw std::runtime_error(
      message + ": actual=" + std::to_string(actual) + ", expected=" + std::to_string(expected));
  }
}

offline_lc_minimal::GnssSolutionSample MakeGnssSample(const double time_s) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 0.1;
  sample.lon_rad = 0.2;
  sample.h_m = 3.0;
  sample.sigma_lat_m = 0.2;
  sample.sigma_lon_m = 0.1;
  sample.sigma_h_m = 0.3;
  sample.gnssfgo_type_code = 1;
  sample.best_sol_status_code = 1;
  sample.has_enu_position = true;
  sample.enu_position_m = Eigen::Vector3d(4.0, 5.0, 6.0);
  return sample;
}

void TestBuilderAddsRobustHorizontalAndGaussianVerticalFactors() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.gnss_position_noise_model = offline_lc_minimal::GnssNoiseModel::kHuber;
  config.gnss_position_robust_param = 0.5;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::GnssConsistencyRecord> consistency_records;

  offline_lc_minimal::GnssFactorBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.navigation_start_index = 0;
  request.graph = &graph;
  request.trajectory = &trajectory;
  request.run_summary = &summary;
  request.factor_records = &factor_records;
  request.consistency_records = &consistency_records;
  request.collect_consistency_records = true;
  request.dynamic_start_time_s = 1.0;
  request.should_use_sample = [](const auto &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    result.key_index_i = 1;
    result.key_index_j = 1;
    result.timestamp_i_s = 1.0;
    result.timestamp_j_s = 1.0;
    return result;
  };
  request.trajectory_row_index_for_state = [](std::size_t state_index) {
    return static_cast<long long>(state_index);
  };

  offline_lc_minimal::GnssFactorBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 2.0, 0.0, "builder should add horizontal and vertical factors");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 1.0, 0.0, "one GNSS sample should be used");
  ExpectNear(static_cast<double>(summary.gnss_synced_factor_count), 1.0, 0.0, "sample should be synchronized");
  ExpectTrue(trajectory[1].gnss_factor_used, "synchronized trajectory row should be marked");
  ExpectNear(factor_records.front().measurement_enu_m.z(), 6.0, 0.0, "record should retain measured up");
  ExpectTrue(consistency_records.front().vertical_direct_position_factor_used, "vertical factor should be recorded");

  const auto horizontal_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[0]);
  const auto vertical_factor = boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(graph[1]);
  ExpectTrue(static_cast<bool>(horizontal_factor), "horizontal factor should expose a noise model");
  ExpectTrue(static_cast<bool>(vertical_factor), "vertical factor should expose a noise model");
  ExpectTrue(
    static_cast<bool>(boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(horizontal_factor->noiseModel())),
    "horizontal factor should keep robust GNSS noise");
  ExpectTrue(
    !static_cast<bool>(boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(vertical_factor->noiseModel())),
    "vertical Phase 1 factor should use plain Gaussian noise");
}

void TestEnvelopeModeRejectedBeforeGraphMutation() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.vertical_constraint_mode = offline_lc_minimal::VerticalConstraintMode::kEnvelope;
  config.early_gnss_relaxation_duration_s = 0.0;

  std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeGnssSample(0.0),
    MakeGnssSample(1.0),
  };
  gtsam::NonlinearFactorGraph graph;
  std::vector<offline_lc_minimal::TrajectoryRow> trajectory(2);
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::GnssFactorRecord> factor_records;
  std::vector<offline_lc_minimal::GnssConsistencyRecord> consistency_records;

  offline_lc_minimal::GnssFactorBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.navigation_start_index = 0;
  request.graph = &graph;
  request.trajectory = &trajectory;
  request.run_summary = &summary;
  request.factor_records = &factor_records;
  request.consistency_records = &consistency_records;
  request.collect_consistency_records = true;
  request.dynamic_start_time_s = 1.0;
  request.should_use_sample = [](const auto &) { return true; };
  request.is_within_imu_coverage = [](double) { return true; };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.clamped_sigma_m = [](const auto &) { return Eigen::Vector3d(0.1, 0.2, 0.3); };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    result.key_index_i = 1;
    result.key_index_j = 1;
    result.timestamp_i_s = 1.0;
    result.timestamp_j_s = 1.0;
    return result;
  };
  request.trajectory_row_index_for_state = [](std::size_t state_index) {
    return static_cast<long long>(state_index);
  };

  bool threw = false;
  try {
    offline_lc_minimal::GnssFactorBuilder(std::move(request)).Build();
  } catch (const std::runtime_error &exception) {
    threw = std::string(exception.what()).find("vertical_constraint_mode=envelope") != std::string::npos;
  }

  ExpectTrue(threw, "envelope mode should be rejected in Phase 1");
  ExpectNear(static_cast<double>(graph.size()), 0.0, 0.0, "rejected envelope mode should not mutate graph");
  ExpectTrue(factor_records.empty(), "rejected envelope mode should not emit factor records");
  ExpectTrue(consistency_records.empty(), "rejected envelope mode should not emit consistency records");
  ExpectNear(static_cast<double>(summary.gnss_factor_count), 0.0, 0.0, "rejected envelope mode should not update summary");
}

}  // namespace

int main() {
  try {
    RunTest(
      "TestBuilderAddsRobustHorizontalAndGaussianVerticalFactors",
      TestBuilderAddsRobustHorizontalAndGaussianVerticalFactors);
    RunTest(
      "TestEnvelopeModeRejectedBeforeGraphMutation",
      TestEnvelopeModeRejectedBeforeGraphMutation);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
