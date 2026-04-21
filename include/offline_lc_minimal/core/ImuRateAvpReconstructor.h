#pragma once

#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct OptimizedNodeState {
  double time_s = 0.0;
  gtsam::Pose3 pose;
  gtsam::Vector3 velocity = gtsam::Vector3::Zero();
  gtsam::imuBias::ConstantBias bias;
};

struct ImuRateAvpReconstructionResult {
  std::vector<ImuRateAvpRow> rows;
  std::vector<ImuRateIntervalDiagnostic> diagnostics;
};

class ImuRateAvpReconstructor {
 public:
  static ImuRateAvpReconstructionResult Reconstruct(
    const std::vector<ImuSample> &imu_samples,
    const std::vector<OptimizedNodeState> &node_states,
    const boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> &imu_params,
    bool verbose);
};

}  // namespace offline_lc_minimal
