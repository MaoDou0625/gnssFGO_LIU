#include "offline_lc_minimal/core/ResidualContributionAnalyzer.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <typeinfo>
#include <utility>

#ifdef __GNUG__
#include <cxxabi.h>
#endif

namespace offline_lc_minimal {
namespace {

struct ResidualAccumulator {
  std::string module;
  std::string factor_type;
  std::size_t factor_count = 0;
  std::size_t evaluated_factor_count = 0;
  std::size_t failed_factor_count = 0;
  double total_error = 0.0;
  double max_error = -std::numeric_limits<double>::infinity();
  long long max_error_factor_index = -1;
};

[[nodiscard]] bool Contains(const std::string &value, const std::string &needle) {
  return value.find(needle) != std::string::npos;
}

[[nodiscard]] std::string DemangleTypeName(const char *name) {
#ifdef __GNUG__
  int status = 0;
  std::unique_ptr<char, decltype(&std::free)> demangled(
    abi::__cxa_demangle(name, nullptr, nullptr, &status),
    &std::free);
  if (status == 0 && demangled != nullptr) {
    return demangled.get();
  }
#endif
  return name != nullptr ? std::string(name) : std::string("UNKNOWN_FACTOR");
}

[[nodiscard]] std::string ClassifyModule(const std::string &factor_type) {
  if (Contains(factor_type, "VerticalMaskedCombinedImuFactor")) {
    return "vertical_jump_imu_mask";
  }
  if (Contains(factor_type, "BazContinuityBreakCombinedImuFactor")) {
    return "rtk_outage_baz_reestimate";
  }
  if (Contains(factor_type, "CombinedImuFactor") ||
      Contains(factor_type, "ImuFactor")) {
    return "imu_preintegration";
  }
  if (Contains(factor_type, "FixedAxisBodyYVelocityEnvelopeFactor")) {
    return "stage1_outage_body_y";
  }
  if (Contains(factor_type, "AttitudeHoldFactor")) {
    return "attitude_hold";
  }
  if (Contains(factor_type, "RollPitchReferenceFactor") ||
      Contains(factor_type, "RelativeYawReferenceFactor") ||
      Contains(factor_type, "AttitudeReferenceFactor")) {
    return "attitude_reference";
  }
  if (Contains(factor_type, "HorizontalPositionHoldFactor") ||
      Contains(factor_type, "HorizontalVelocityHoldFactor")) {
    return "stage2_hold";
  }
  if (Contains(factor_type, "Vehicle")) {
    return "stage2_vehicle_nhc";
  }
  if (Contains(factor_type, "BodyZ")) {
    return "body_z_nhc";
  }
  if (Contains(factor_type, "RtkHorizontalVelocityFactor")) {
    return "rtk_velocity";
  }
  if (Contains(factor_type, "GPSFactor") ||
      Contains(factor_type, "HorizontalPositionFactor")) {
    return "gnss_horizontal_position";
  }
  if (Contains(factor_type, "VerticalPositionFactor") ||
      Contains(factor_type, "VerticalEnvelopeFactor") ||
      Contains(factor_type, "VerticalEnvelopeCenterPullFactor")) {
    return "vertical_position_reference";
  }
  if (Contains(factor_type, "VerticalJump") ||
      Contains(factor_type, "VerticalVelocityRampFactor") ||
      Contains(factor_type, "VerticalVelocityHeightSlopeFactor") ||
      Contains(factor_type, "VerticalVelocityContextMeanContinuityFactor")) {
    return "vertical_jump";
  }
  if (Contains(factor_type, "VerticalVelocityDelta") ||
      Contains(factor_type, "VerticalVelocityPriorFactor") ||
      Contains(factor_type, "VelocityDeltaFactor") ||
      Contains(factor_type, "VerticalPositionRampFactor") ||
      Contains(factor_type, "VerticalPositionVelocityConsistencyFactor") ||
      Contains(factor_type, "VerticalPositionVelocityWindowConsistencyFactor") ||
      Contains(factor_type, "VerticalVelocityMeanFactor")) {
    return "vertical_motion";
  }
  if (Contains(factor_type, "Static")) {
    return "initial_static";
  }
  if (Contains(factor_type, "AngularRateFactor")) {
    return "angular_rate";
  }
  if (Contains(factor_type, "Bias") ||
      Contains(factor_type, "GlobalAccelBiasFactor") ||
      Contains(factor_type, "GlobalPlanarAccelBiasFactor") ||
      Contains(factor_type, "GlobalGyroBiasFactor")) {
    return "bias_constraints";
  }
  if (Contains(factor_type, "PriorFactor")) {
    return "priors";
  }
  if (Contains(factor_type, "BetweenFactor")) {
    return "between_constraints";
  }
  return "other";
}

void Accumulate(
  ResidualAccumulator &accumulator,
  const std::size_t factor_index,
  const double error,
  const bool evaluated) {
  ++accumulator.factor_count;
  if (!evaluated || !std::isfinite(error)) {
    ++accumulator.failed_factor_count;
    return;
  }
  ++accumulator.evaluated_factor_count;
  accumulator.total_error += error;
  if (error > accumulator.max_error) {
    accumulator.max_error = error;
    accumulator.max_error_factor_index = static_cast<long long>(factor_index);
  }
}

[[nodiscard]] ResidualContributionRow MakeRow(
  const ResidualAccumulator &accumulator,
  const double total_error,
  const std::string &stage_name,
  const int stage_iteration) {
  ResidualContributionRow row;
  row.stage_name = stage_name;
  row.stage_iteration = stage_iteration;
  row.module = accumulator.module;
  row.factor_type = accumulator.factor_type;
  row.factor_count = accumulator.factor_count;
  row.evaluated_factor_count = accumulator.evaluated_factor_count;
  row.failed_factor_count = accumulator.failed_factor_count;
  row.total_error = accumulator.total_error;
  if (accumulator.evaluated_factor_count > 0U) {
    row.mean_error =
      accumulator.total_error / static_cast<double>(accumulator.evaluated_factor_count);
    row.max_error = accumulator.max_error;
    row.max_error_factor_index = accumulator.max_error_factor_index;
  }
  if (total_error > 0.0) {
    row.total_error_fraction = accumulator.total_error / total_error;
  }
  return row;
}

void SortRows(std::vector<ResidualContributionRow> &rows) {
  std::sort(
    rows.begin(),
    rows.end(),
    [](const ResidualContributionRow &lhs, const ResidualContributionRow &rhs) {
      if (lhs.total_error != rhs.total_error) {
        return lhs.total_error > rhs.total_error;
      }
      if (lhs.module != rhs.module) {
        return lhs.module < rhs.module;
      }
      return lhs.factor_type < rhs.factor_type;
    });
}

}  // namespace

ResidualContributionReport AnalyzeResidualContributions(
  const gtsam::NonlinearFactorGraph &graph,
  const gtsam::Values &values,
  const std::string &stage_name,
  const int stage_iteration) {
  std::map<std::string, ResidualAccumulator> module_accumulators;
  std::map<std::pair<std::string, std::string>, ResidualAccumulator> factor_accumulators;

  for (std::size_t factor_index = 0; factor_index < graph.size(); ++factor_index) {
    const auto factor = graph.at(factor_index);
    const std::string factor_type =
      factor != nullptr ? DemangleTypeName(typeid(*factor).name()) : "NULL_FACTOR";
    const std::string module = ClassifyModule(factor_type);

    bool evaluated = false;
    double error = std::numeric_limits<double>::quiet_NaN();
    if (factor != nullptr) {
      try {
        error = factor->error(values);
        evaluated = true;
      } catch (const std::exception &) {
        evaluated = false;
      }
    }

    auto &module_accumulator = module_accumulators[module];
    module_accumulator.module = module;
    module_accumulator.factor_type = "ALL";
    Accumulate(module_accumulator, factor_index, error, evaluated);

    auto &factor_accumulator = factor_accumulators[{module, factor_type}];
    factor_accumulator.module = module;
    factor_accumulator.factor_type = factor_type;
    Accumulate(factor_accumulator, factor_index, error, evaluated);
  }

  double total_error = 0.0;
  for (const auto &[_, accumulator] : module_accumulators) {
    total_error += accumulator.total_error;
  }

  ResidualContributionReport report;
  report.module_rows.reserve(module_accumulators.size());
  for (const auto &[_, accumulator] : module_accumulators) {
    report.module_rows.push_back(
      MakeRow(accumulator, total_error, stage_name, stage_iteration));
  }
  report.factor_rows.reserve(factor_accumulators.size());
  for (const auto &[_, accumulator] : factor_accumulators) {
    report.factor_rows.push_back(
      MakeRow(accumulator, total_error, stage_name, stage_iteration));
  }
  SortRows(report.module_rows);
  SortRows(report.factor_rows);
  return report;
}

}  // namespace offline_lc_minimal
