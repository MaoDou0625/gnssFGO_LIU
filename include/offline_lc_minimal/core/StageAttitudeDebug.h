#pragma once

#include <string_view>
#include <vector>

#include "offline_lc_minimal/common/Types.h"

namespace offline_lc_minimal {

[[nodiscard]] std::vector<StageAttitudeDebugRow> BuildStageAttitudeDebugRows(
  std::string_view source,
  const std::vector<ReferenceNodeState> &states);

void AppendStageAttitudeDebugRows(
  std::string_view source,
  const std::vector<ReferenceNodeState> &states,
  std::vector<StageAttitudeDebugRow> &rows);

void RecordStageAttitudeDebugRows(
  std::string_view source,
  const std::vector<ReferenceNodeState> &states,
  std::vector<StageAttitudeDebugRow> &rows);

}  // namespace offline_lc_minimal
