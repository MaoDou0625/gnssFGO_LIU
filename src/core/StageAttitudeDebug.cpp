#include "offline_lc_minimal/core/StageAttitudeDebug.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <gtsam/geometry/Rot3.h>

#include "offline_lc_minimal/core/TrajectoryResultBuilder.h"

namespace offline_lc_minimal {
namespace {

double NormalizeAngle(const double angle_rad) {
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

StageAttitudeDebugRow MakeRow(
  const std::string_view source,
  const std::size_t state_index,
  const ReferenceNodeState &state,
  const ReferenceNodeState *previous_state) {
  StageAttitudeDebugRow row;
  row.source = std::string(source);
  row.state_index = state_index;
  row.time_s = state.time_s;
  row.enu_position_m = Eigen::Vector3d(
    state.pose.translation().x(),
    state.pose.translation().y(),
    state.pose.translation().z());
  row.enu_velocity_mps = Eigen::Vector3d(
    state.velocity.x(),
    state.velocity.y(),
    state.velocity.z());
  row.ypr_rad = Rot3ToYpr(state.pose.rotation());
  row.bias_acc = state.bias.accelerometer();
  row.bias_gyro = state.bias.gyroscope();
  row.body_z_axis_nav_z = state.pose.rotation().matrix()(2, 2);

  if (previous_state != nullptr) {
    row.previous_dt_s = state.time_s - previous_state->time_s;
    const gtsam::Vector3 relative_rotvec =
      gtsam::Rot3::Logmap(previous_state->pose.rotation().between(state.pose.rotation()));
    row.relative_rotvec_rad = Eigen::Vector3d(
      relative_rotvec.x(),
      relative_rotvec.y(),
      relative_rotvec.z());
    row.relative_angle_rad = relative_rotvec.norm();
    const Eigen::Vector3d previous_ypr = Rot3ToYpr(previous_state->pose.rotation());
    row.relative_delta_yaw_rad = NormalizeAngle(row.ypr_rad.x() - previous_ypr.x());
  }

  return row;
}

}  // namespace

std::vector<StageAttitudeDebugRow> BuildStageAttitudeDebugRows(
  const std::string_view source,
  const std::vector<ReferenceNodeState> &states) {
  std::vector<StageAttitudeDebugRow> rows;
  rows.reserve(states.size());
  AppendStageAttitudeDebugRows(source, states, rows);
  return rows;
}

void AppendStageAttitudeDebugRows(
  const std::string_view source,
  const std::vector<ReferenceNodeState> &states,
  std::vector<StageAttitudeDebugRow> &rows) {
  rows.reserve(rows.size() + states.size());
  for (std::size_t state_index = 0; state_index < states.size(); ++state_index) {
    const ReferenceNodeState *previous_state =
      state_index > 0U ? &states[state_index - 1U] : nullptr;
    rows.push_back(MakeRow(source, state_index, states[state_index], previous_state));
  }
}

void RecordStageAttitudeDebugRows(
  const std::string_view source,
  const std::vector<ReferenceNodeState> &states,
  std::vector<StageAttitudeDebugRow> &rows) {
  const std::string source_name(source);
  rows.erase(
    std::remove_if(
      rows.begin(),
      rows.end(),
      [&](const StageAttitudeDebugRow &row) {
        return row.source == source_name;
      }),
    rows.end());
  AppendStageAttitudeDebugRows(source, states, rows);
}

}  // namespace offline_lc_minimal
