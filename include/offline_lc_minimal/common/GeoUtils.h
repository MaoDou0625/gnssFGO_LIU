#pragma once

#include <array>

#include <Eigen/Core>
#include <GeographicLib/LocalCartesian.hpp>

namespace offline_lc_minimal {

class GeoReference {
 public:
  GeoReference(double origin_lat_rad, double origin_lon_rad, double origin_h_m);

  [[nodiscard]] Eigen::Vector3d Forward(double lat_rad, double lon_rad, double h_m) const;
  [[nodiscard]] std::array<double, 3> Reverse(const Eigen::Vector3d &enu_position_m) const;
  [[nodiscard]] Eigen::Vector3d EarthRateEnu() const;
  [[nodiscard]] double origin_lat_rad() const { return origin_lat_rad_; }
  [[nodiscard]] double origin_lon_rad() const { return origin_lon_rad_; }
  [[nodiscard]] double origin_h_m() const { return origin_h_m_; }

 private:
  double origin_lat_rad_;
  double origin_lon_rad_;
  double origin_h_m_;
  GeographicLib::LocalCartesian local_cartesian_;
};

}  // namespace offline_lc_minimal
