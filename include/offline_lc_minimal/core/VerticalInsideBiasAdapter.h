#pragma once

#include <cstddef>
#include <limits>
#include <optional>
#include <vector>

#include "offline_lc_minimal/common/Config.h"

namespace offline_lc_minimal {

struct VerticalInsideBiasUpdate {
  std::size_t anchor_state_index = 0;
  std::size_t current_state_index = 0;
  double current_time_s = std::numeric_limits<double>::quiet_NaN();
  double delta_pitch_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_roll_rad = std::numeric_limits<double>::quiet_NaN();
  double delta_baz_mps2 = std::numeric_limits<double>::quiet_NaN();
  double equivalent_acc_mps2 = std::numeric_limits<double>::quiet_NaN();
  double residual_delta_m = std::numeric_limits<double>::quiet_NaN();
  double window_dt_s = std::numeric_limits<double>::quiet_NaN();
  int observation_count = 0;
};

class VerticalInsideBiasAdapter {
 public:
  explicit VerticalInsideBiasAdapter(const OfflineRunnerConfig &config);

  void RewindFromStateIndex(std::size_t state_index);
  void AcceptUpdate(const VerticalInsideBiasUpdate &update);

  [[nodiscard]] std::optional<VerticalInsideBiasUpdate> ObserveInsideResidual(
    std::size_t state_index,
    double time_s,
    double residual_u_m,
    double sigma_u_m,
    double gate_threshold_m,
    double pitch_rad,
    double roll_rad);

 private:
  struct Observation {
    std::size_t state_index = 0;
    double time_s = 0.0;
    double residual_u_m = 0.0;
    double sigma_u_m = 0.0;
    double gate_threshold_m = 0.0;
    double pitch_rad = 0.0;
    double roll_rad = 0.0;
  };

  void PruneHistory(double latest_time_s);
  [[nodiscard]] std::optional<VerticalInsideBiasUpdate> BuildUpdateCandidate() const;

  const OfflineRunnerConfig &config_;
  std::vector<Observation> observations_;
  double last_update_time_s_ = -std::numeric_limits<double>::infinity();
};

}  // namespace offline_lc_minimal
