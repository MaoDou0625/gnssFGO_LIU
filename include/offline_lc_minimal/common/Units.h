#pragma once

namespace offline_lc_minimal {

constexpr double kMicroGToMps2 = 9.80665e-6;
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kDegPerHourToRadPerSecond = kPi / (180.0 * 3600.0);

[[nodiscard]] constexpr double MicroGToMps2(const double value_ug) {
  return value_ug * kMicroGToMps2;
}

[[nodiscard]] constexpr double Mps2ToMicroG(const double value_mps2) {
  return value_mps2 / kMicroGToMps2;
}

[[nodiscard]] constexpr double DegPerHourToRadPerSecond(const double value_deg_per_hour) {
  return value_deg_per_hour * kDegPerHourToRadPerSecond;
}

[[nodiscard]] constexpr double RadPerSecondToDegPerHour(const double value_rad_per_second) {
  return value_rad_per_second / kDegPerHourToRadPerSecond;
}

}  // namespace offline_lc_minimal
