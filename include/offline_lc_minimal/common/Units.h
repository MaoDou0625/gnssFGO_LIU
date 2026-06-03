#pragma once

namespace offline_lc_minimal {

constexpr double kMicroGToMps2 = 9.80665e-6;

[[nodiscard]] constexpr double MicroGToMps2(const double value_ug) {
  return value_ug * kMicroGToMps2;
}

[[nodiscard]] constexpr double Mps2ToMicroG(const double value_mps2) {
  return value_mps2 / kMicroGToMps2;
}

}  // namespace offline_lc_minimal
