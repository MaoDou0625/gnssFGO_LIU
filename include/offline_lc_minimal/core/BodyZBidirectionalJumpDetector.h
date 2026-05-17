#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "offline_lc_minimal/common/Config.h"
#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

struct BodyZJumpSignalSample {
  double time_s = 0.0;
  double relative_time_s = 0.0;
  double body_z_specific_force_mps2 = std::numeric_limits<double>::quiet_NaN();
  double gravity_projection_z_mps2 = std::numeric_limits<double>::quiet_NaN();
  double body_z_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double body_z_acc_1s_smooth_mps2 = std::numeric_limits<double>::quiet_NaN();
  double integrated_body_z_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double integrated_body_z_velocity_0p2s_smooth_mps = std::numeric_limits<double>::quiet_NaN();
  double integrated_body_z_velocity_1s_smooth_mps = std::numeric_limits<double>::quiet_NaN();
  double signed_step_metric_mps = std::numeric_limits<double>::quiet_NaN();
  double downward_score_mps = std::numeric_limits<double>::quiet_NaN();
  double upward_score_mps = std::numeric_limits<double>::quiet_NaN();
  double body_z_axis_nav_z = std::numeric_limits<double>::quiet_NaN();
};

struct BodyZJumpWindowCandidate {
  std::string direction = "UNKNOWN";
  int selection_level = 0;
  std::size_t start_signal_index = 0;
  std::size_t center_signal_index = 0;
  std::size_t end_signal_index = 0;
  std::size_t start_state_index = 0;
  std::size_t center_state_index = 0;
  std::size_t end_state_index = 0;
  double start_time_s = std::numeric_limits<double>::quiet_NaN();
  double center_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_time_s = std::numeric_limits<double>::quiet_NaN();
  double start_relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double center_relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double end_relative_time_s = std::numeric_limits<double>::quiet_NaN();
  double duration_s = std::numeric_limits<double>::quiet_NaN();
  double pre_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double post_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double signed_delta_velocity_mps = std::numeric_limits<double>::quiet_NaN();
  double direction_score_mps = std::numeric_limits<double>::quiet_NaN();
  double signed_step_metric_mps = std::numeric_limits<double>::quiet_NaN();
  double level_threshold_mps = std::numeric_limits<double>::quiet_NaN();
  double level_max_peak_mps = std::numeric_limits<double>::quiet_NaN();
  double level_noise_floor_mps = std::numeric_limits<double>::quiet_NaN();
  double min_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double max_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double mean_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double body_z_axis_nav_z = std::numeric_limits<double>::quiet_NaN();
  double delta_vz_init_mps = std::numeric_limits<double>::quiet_NaN();
};

struct BodyZJumpDetectionResult {
  std::vector<BodyZJumpSignalSample> signal;
  std::vector<BodyZJumpWindowCandidate> selected_windows;
  std::vector<BodyZJumpWindowCandidate> windows;
};

class BodyZBidirectionalJumpDetector {
 public:
  explicit BodyZBidirectionalJumpDetector(const OfflineRunnerConfig &config);

  [[nodiscard]] BodyZJumpDetectionResult Detect(
    const std::vector<ImuSample> &imu_samples,
    const std::vector<ReferenceNodeState> &seed_reference_states,
    const std::vector<double> &state_timestamps,
    double dynamic_start_time_s,
    double end_time_s) const;

 private:
  const OfflineRunnerConfig &config_;
};

}  // namespace offline_lc_minimal
