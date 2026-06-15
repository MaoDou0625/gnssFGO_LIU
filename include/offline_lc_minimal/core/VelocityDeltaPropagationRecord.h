#pragma once

#include <cstddef>

#include <gtsam/base/Vector.h>

namespace offline_lc_minimal {

struct VelocityDeltaPropagationRecord {
  std::size_t state_index_i = 0;
  std::size_t state_index_j = 0;
  double start_time_s = 0.0;
  double end_time_s = 0.0;
  gtsam::Vector3 target_delta_v_mps = gtsam::Vector3::Zero();
};

}  // namespace offline_lc_minimal
