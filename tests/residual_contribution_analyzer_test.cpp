#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "offline_lc_minimal/core/ResidualContributionAnalyzer.h"
#include "offline_lc_minimal/factor/HorizontalPositionFactor.h"

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
    throw std::runtime_error(message + ": actual=" + std::to_string(actual) +
                             " expected=" + std::to_string(expected));
  }
}

const offline_lc_minimal::ResidualContributionRow *FindModule(
  const std::vector<offline_lc_minimal::ResidualContributionRow> &rows,
  const std::string &module) {
  for (const auto &row : rows) {
    if (row.module == module) {
      return &row;
    }
  }
  return nullptr;
}

void TestAnalyzerGroupsFinalErrorByModule() {
  using gtsam::symbol_shorthand::X;

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values values;
  const auto scalar_noise = gtsam::noiseModel::Isotropic::Sigma(1, 1.0);
  const auto horizontal_noise = gtsam::noiseModel::Isotropic::Sigma(2, 1.0);

  const gtsam::Key scalar_key = gtsam::Symbol('j', 0);
  graph.add(gtsam::PriorFactor<double>(scalar_key, 0.0, scalar_noise));
  values.insert<double>(scalar_key, 2.0);

  graph.add(offline_lc_minimal::factor::HorizontalPositionFactor(
    X(0),
    gtsam::Point2(0.0, 0.0),
    horizontal_noise));
  values.insert(X(0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.0, 4.0, 0.0)));

  const offline_lc_minimal::ResidualContributionReport report =
    offline_lc_minimal::AnalyzeResidualContributions(graph, values, "unit", 7);

  const auto *prior_row = FindModule(report.module_rows, "priors");
  const auto *horizontal_row =
    FindModule(report.module_rows, "gnss_horizontal_position");
  ExpectTrue(prior_row != nullptr, "prior module contribution should exist");
  ExpectTrue(horizontal_row != nullptr, "horizontal GNSS module contribution should exist");

  ExpectNear(prior_row->total_error, 2.0, 1e-12, "prior contribution mismatch");
  ExpectNear(
    horizontal_row->total_error,
    12.5,
    1e-12,
    "horizontal position contribution mismatch");
  ExpectNear(
    prior_row->total_error_fraction + horizontal_row->total_error_fraction,
    1.0,
    1e-12,
    "module contribution fractions should sum to one");
  ExpectTrue(prior_row->stage_name == "unit", "stage name should be preserved");
  ExpectTrue(prior_row->stage_iteration == 7, "stage iteration should be preserved");
}

}  // namespace

int main() {
  try {
    RunTest("TestAnalyzerGroupsFinalErrorByModule", TestAnalyzerGroupsFinalErrorByModule);
  } catch (const std::exception &exception) {
    std::cerr << exception.what() << '\n';
    return 1;
  }
  std::cout << "residual_contribution_analyzer_test passed\n";
  return 0;
}
