#pragma once

#include <memory>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/RunResultTypes.h"
#include "offline_lc_minimal/core/Stage2VelocityReference.h"

namespace offline_lc_minimal {

[[nodiscard]] std::vector<double> OutageEndHorizontalHandoffTargetTimes(
  const std::vector<double> &state_timestamps,
  const std::vector<GnssFactorRecord> &gnss_factor_records,
  const RtkOutageWindowRow &source_outage,
  const RtkOutageWindowRow &handoff_outage,
  double state_frequency_hz,
  double handoff_guard_duration_s);

[[nodiscard]] std::shared_ptr<Stage2VelocityReference> MutableStage2ReferenceCopy(
  const std::shared_ptr<const Stage2VelocityReference> &reference);

void ApplyOutageEndHorizontalHandoffToStage2Reference(
  Stage2VelocityReference &reference,
  const DataSet &dataset,
  const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &imu_params,
  const OfflineRunResult &post_result,
  double post_first_time_s,
  const std::vector<double> &target_times,
  double state_frequency_hz);

}  // namespace offline_lc_minimal
