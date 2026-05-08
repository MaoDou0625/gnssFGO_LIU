#include "offline_lc_minimal/core/InitialStaticRtkHeightConstraintBuilder.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

#include <gtsam/linear/NoiseModel.h>

#include "offline_lc_minimal/factor/VerticalRtkFactors.h"

namespace offline_lc_minimal {

namespace {

constexpr double kTimeEpsilonS = 1e-9;

double Median(std::vector<double> values) {
  if (values.empty()) {
    throw std::runtime_error("cannot compute median of an empty vector");
  }
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2U;
  if (values.size() % 2U == 1U) {
    return values[middle];
  }
  return 0.5 * (values[middle - 1U] + values[middle]);
}

bool IsUsableStaticRtkSample(
  const GnssSolutionSample &sample,
  const double corrected_time_s,
  const double alignment_start_time_s,
  const double alignment_end_time_s,
  const OfflineRunnerConfig &config) {
  if (corrected_time_s + kTimeEpsilonS < alignment_start_time_s ||
      corrected_time_s - kTimeEpsilonS > alignment_end_time_s) {
    return false;
  }
  if (!sample.has_valid_position() || !sample.has_enu_position) {
    return false;
  }
  if (sample.fix_type() != GnssFixType::kRtkFix) {
    return false;
  }
  if (config.required_best_sol_status_code > 0 &&
      sample.best_sol_status_code != config.required_best_sol_status_code) {
    return false;
  }
  return std::isfinite(sample.enu_position_m.z());
}

}  // namespace

bool InitialStaticRtkHeightConstraintBuilder::Enabled(const OfflineRunnerConfig &config) {
  return config.enable_initial_static_rtk_height_reference;
}

InitialStaticRtkHeightReference InitialStaticRtkHeightConstraintBuilder::BuildReference(
  const std::vector<GnssSolutionSample> &gnss_samples,
  const double alignment_start_time_s,
  const double alignment_end_time_s,
  const OfflineRunnerConfig &config) {
  InitialStaticRtkHeightReference reference;
  if (!Enabled(config)) {
    return reference;
  }

  std::vector<double> static_rtk_up_values_m;
  static_rtk_up_values_m.reserve(gnss_samples.size());
  for (const auto &sample : gnss_samples) {
    const double corrected_time_s = sample.time_s - config.gnss_time_offset_s;
    if (!IsUsableStaticRtkSample(
          sample,
          corrected_time_s,
          alignment_start_time_s,
          alignment_end_time_s,
          config)) {
      continue;
    }
    static_rtk_up_values_m.push_back(sample.enu_position_m.z());
  }

  reference.sample_count = static_rtk_up_values_m.size();
  const std::size_t min_sample_count =
    static_cast<std::size_t>(std::max(config.initial_static_rtk_height_reference_min_sample_count, 1));
  if (reference.sample_count < min_sample_count) {
    throw std::runtime_error(
      "initial static RTK height reference requires at least " +
      std::to_string(min_sample_count) +
      " valid RTKFIX samples in the static alignment window");
  }

  reference.valid = true;
  reference.reference_up_m = Median(std::move(static_rtk_up_values_m));
  return reference;
}

bool InitialStaticRtkHeightConstraintBuilder::AddVerticalReference(
  const InitialStaticRtkHeightReference &reference,
  const OfflineRunnerConfig &config,
  gtsam::NonlinearFactorGraph &graph,
  const gtsam::Key pose_key) {
  if (!Enabled(config) || !reference.valid) {
    return false;
  }
  graph.add(factor::VerticalPositionFactor(
    pose_key,
    reference.reference_up_m,
    gtsam::noiseModel::Isotropic::Sigma(
      1,
      config.initial_static_rtk_height_reference_sigma_m)));
  return true;
}

}  // namespace offline_lc_minimal
