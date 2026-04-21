#include "offline_lc_minimal/common/GeoUtils.h"

#include <cmath>

namespace offline_lc_minimal {

namespace {

constexpr double kRadToDeg = 180.0 / M_PI;
constexpr double kDegToRad = M_PI / 180.0;
constexpr double kEarthRotationRateRadps = 7.292115e-5;

}  // namespace

GeoReference::GeoReference(const double origin_lat_rad, const double origin_lon_rad, const double origin_h_m)
    : origin_lat_rad_(origin_lat_rad),
      origin_lon_rad_(origin_lon_rad),
      origin_h_m_(origin_h_m),
      local_cartesian_(origin_lat_rad * kRadToDeg, origin_lon_rad * kRadToDeg, origin_h_m) {}

Eigen::Vector3d GeoReference::Forward(const double lat_rad, const double lon_rad, const double h_m) const {
  double east_m = 0.0;
  double north_m = 0.0;
  double up_m = 0.0;
  local_cartesian_.Forward(lat_rad * kRadToDeg, lon_rad * kRadToDeg, h_m, east_m, north_m, up_m);
  return Eigen::Vector3d(east_m, north_m, up_m);
}

std::array<double, 3> GeoReference::Reverse(const Eigen::Vector3d &enu_position_m) const {
  double lat_deg = 0.0;
  double lon_deg = 0.0;
  double h_m = 0.0;
  local_cartesian_.Reverse(enu_position_m.x(), enu_position_m.y(), enu_position_m.z(), lat_deg, lon_deg, h_m);
  return {lat_deg * kDegToRad, lon_deg * kDegToRad, h_m};
}

Eigen::Vector3d GeoReference::EarthRateEnu() const {
  const double lat = origin_lat_rad_;
  return Eigen::Vector3d(0.0, kEarthRotationRateRadps * std::cos(lat), kEarthRotationRateRadps * std::sin(lat));
}

}  // namespace offline_lc_minimal
