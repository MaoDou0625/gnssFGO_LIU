#include "offline_lc_minimal/core/GnssPreOutageQualityOverride.h"

#include <algorithm>
#include <cmath>

namespace offline_lc_minimal {
namespace {

constexpr double kTimeEpsilonS = 1.0e-9;

bool HasRequiredStatus(
  const GnssSolutionSample &sample,
  const OfflineRunnerConfig &config) {
  return config.required_best_sol_status_code <= 0 ||
         sample.best_sol_status_code == config.required_best_sol_status_code;
}

bool IsUsableRtkFix(
  const GnssSolutionSample &sample,
  const OfflineRunnerConfig &config) {
  return sample.has_valid_position() &&
         HasRequiredStatus(sample, config) &&
         sample.fix_type() == GnssFixType::kRtkFix;
}

bool IsFiniteSampleTime(const GnssSolutionSample &sample) {
  return std::isfinite(sample.time_s);
}

}  // namespace

GnssPreOutageQualityOverrideSummary ApplyGnssPreOutageQualityOverride(
  const OfflineRunnerConfig &config,
  std::vector<GnssSolutionSample> &gnss_samples) {
  GnssPreOutageQualityOverrideSummary summary;
  if (!config.enable_gnss_preoutage_quality_override ||
      gnss_samples.size() < 2U) {
    return summary;
  }

  std::size_t previous_fix_index = gnss_samples.size();
  for (std::size_t index = 0; index < gnss_samples.size(); ++index) {
    if (!IsFiniteSampleTime(gnss_samples[index]) ||
        !IsUsableRtkFix(gnss_samples[index], config)) {
      continue;
    }

    if (previous_fix_index != gnss_samples.size()) {
      const double gap_s =
        gnss_samples[index].time_s - gnss_samples[previous_fix_index].time_s;
      if (std::isfinite(gap_s) &&
          gap_s >= config.gnss_preoutage_quality_override_min_gap_s) {
        ++summary.planned_outage_count;
        const double outage_first_nonfix_time_s =
          previous_fix_index + 1U < index
            ? gnss_samples[previous_fix_index + 1U].time_s
            : gnss_samples[previous_fix_index].time_s;
        const double preoutage_start_time_s =
          outage_first_nonfix_time_s -
          config.gnss_preoutage_quality_override_duration_s;
        for (std::size_t pre_index = 0; pre_index <= previous_fix_index; ++pre_index) {
          auto &sample = gnss_samples[pre_index];
          if (!IsFiniteSampleTime(sample) ||
              sample.time_s < preoutage_start_time_s ||
              sample.time_s > outage_first_nonfix_time_s + kTimeEpsilonS ||
              !IsUsableRtkFix(sample, config)) {
            continue;
          }
          sample.gnssfgo_type_code = static_cast<int>(GnssFixType::kRtkFloat);
          ++summary.rtkfix_to_float_count;
        }

        if (config.gnss_preoutage_quality_override_mark_outage_nonfix_no_solution) {
          for (std::size_t outage_index = previous_fix_index + 1U;
               outage_index < index;
               ++outage_index) {
            auto &sample = gnss_samples[outage_index];
            if (sample.fix_type() == GnssFixType::kRtkFix) {
              continue;
            }
            const bool was_no_solution =
              sample.fix_type() == GnssFixType::kNoSolution;
            sample.gnssfgo_type_code = static_cast<int>(GnssFixType::kNoSolution);
            if (!was_no_solution) {
              ++summary.nonfix_to_no_solution_count;
            }
          }
        }
      }
    }

    previous_fix_index = index;
  }

  return summary;
}

}  // namespace offline_lc_minimal
