#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/pointer_cast.hpp>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "offline_lc_minimal/core/RtkVelocityConstraintBuilder.h"
#include "offline_lc_minimal/factor/RtkHorizontalVelocityFactor.h"

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

offline_lc_minimal::GnssSolutionSample MakeRtkFixSample(
  const double time_s,
  const Eigen::Vector3d &enu_position_m) {
  offline_lc_minimal::GnssSolutionSample sample;
  sample.time_s = time_s;
  sample.lat_rad = 0.1;
  sample.lon_rad = 0.2;
  sample.h_m = 3.0;
  sample.sigma_lat_m = 0.01;
  sample.sigma_lon_m = 0.01;
  sample.sigma_h_m = 0.02;
  sample.best_sol_status_code = 1;
  sample.gnssfgo_type_code = 1;
  sample.enu_position_m = enu_position_m;
  sample.has_enu_position = true;
  return sample;
}

void TestHorizontalVelocityFactorResidualAndJacobian() {
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(2, 0.25);
  const offline_lc_minimal::factor::RtkHorizontalVelocityFactor factor(
    gtsam::symbol_shorthand::V(0),
    (gtsam::Vector2() << 1.0, -2.0).finished(),
    noise);

  gtsam::Matrix jacobian;
  const gtsam::Vector residual =
    factor.evaluateError((gtsam::Vector3() << 1.5, -2.25, 9.0).finished(), jacobian);

  ExpectNear(residual.x(), 0.5, 1e-12, "east velocity residual should subtract RTK velocity");
  ExpectNear(residual.y(), -0.25, 1e-12, "north velocity residual should subtract RTK velocity");
  ExpectNear(jacobian(0, 0), 1.0, 1e-12, "jacobian should expose east velocity");
  ExpectNear(jacobian(0, 2), 0.0, 1e-12, "jacobian should ignore vertical velocity in row 0");
  ExpectNear(jacobian(1, 1), 1.0, 1e-12, "jacobian should expose north velocity");
}

void TestBuilderAddsCenteredRtkVelocityFactorAndDiagnostics() {
  auto config = offline_lc_minimal::DefaultConfig();
  config.enable_rtk_velocity_constraint = true;
  config.rtk_velocity_window_s = 2.0;
  config.rtk_velocity_horizontal_sigma_mps = 0.25;

  const std::vector<offline_lc_minimal::GnssSolutionSample> samples{
    MakeRtkFixSample(0.0, Eigen::Vector3d(0.0, 0.0, 0.0)),
    MakeRtkFixSample(1.0, Eigen::Vector3d(1.0, 2.0, 0.0)),
    MakeRtkFixSample(2.0, Eigen::Vector3d(2.0, 4.0, 0.0)),
  };
  const std::vector<double> state_timestamps{1.0};
  gtsam::NonlinearFactorGraph graph;
  offline_lc_minimal::RunSummary summary;
  std::vector<offline_lc_minimal::RtkVelocityDiagnosticRow> diagnostics;

  offline_lc_minimal::RtkVelocityConstraintBuildRequest request;
  request.config = &config;
  request.gnss_samples = &samples;
  request.state_timestamps = &state_timestamps;
  request.graph = &graph;
  request.run_summary = &summary;
  request.diagnostics = &diagnostics;
  request.should_use_sample = [](const auto &sample) {
    return sample.fix_type() == offline_lc_minimal::GnssFixType::kRtkFix;
  };
  request.corrected_time_s = [](const auto &sample) { return sample.time_s; };
  request.find_state_for_time_s = [](double) {
    offline_lc_minimal::StateMeasSyncResult result;
    result.status = offline_lc_minimal::StateMeasSyncStatus::kSynchronizedI;
    result.key_index_i = 0;
    result.key_index_j = 0;
    result.timestamp_i_s = 1.0;
    result.timestamp_j_s = 1.0;
    return result;
  };

  offline_lc_minimal::RtkVelocityConstraintBuilder(std::move(request)).Build();

  ExpectNear(static_cast<double>(graph.size()), 1.0, 0.0, "builder should add one RTK velocity factor");
  ExpectNear(static_cast<double>(summary.rtk_velocity_candidate_count), 1.0, 0.0, "center state RTK sample should be a candidate");
  ExpectNear(static_cast<double>(summary.rtk_velocity_factor_count), 1.0, 0.0, "one centered velocity factor should be used");
  ExpectNear(static_cast<double>(diagnostics.size()), 1.0, 0.0, "diagnostics should keep the center candidate");
  ExpectTrue(diagnostics[0].factor_added, "center sample should add a factor");
  ExpectNear(diagnostics[0].rtk_velocity_mps.x(), 1.0, 1e-12, "RTK east velocity should use centered difference");
  ExpectNear(diagnostics[0].rtk_velocity_mps.y(), 2.0, 1e-12, "RTK north velocity should use centered difference");
  ExpectTrue(
    static_cast<bool>(
      boost::dynamic_pointer_cast<offline_lc_minimal::factor::RtkHorizontalVelocityFactor>(graph[0])),
    "graph factor should be the RTK horizontal velocity factor");

  gtsam::Values values;
  values.insert(gtsam::symbol_shorthand::V(0), (gtsam::Vector3() << 1.1, 1.8, 0.3).finished());
  values.insert(gtsam::symbol_shorthand::X(0), gtsam::Pose3());
  offline_lc_minimal::RtkVelocityConstraintBuilder::PopulateDiagnostics(values, diagnostics, summary);
  ExpectNear(
    diagnostics[0].horizontal_residual_mps,
    std::hypot(0.1, -0.2),
    1e-12,
    "optimized horizontal residual should be populated");
  ExpectNear(
    summary.rtk_velocity_horizontal_residual_rms_mps,
    std::hypot(0.1, -0.2),
    1e-12,
    "summary should report RTK velocity residual RMS");
  ExpectNear(diagnostics[0].rtk_body_y_mps, 2.0, 1e-12, "identity pose should project north velocity to body-y");
}

}  // namespace

int main() {
  try {
    RunTest("TestHorizontalVelocityFactorResidualAndJacobian", TestHorizontalVelocityFactorResidualAndJacobian);
    RunTest(
      "TestBuilderAddsCenteredRtkVelocityFactorAndDiagnostics",
      TestBuilderAddsCenteredRtkVelocityFactorAndDiagnostics);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  return 0;
}
