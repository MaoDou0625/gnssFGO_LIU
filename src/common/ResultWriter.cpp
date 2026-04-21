#include "offline_lc_minimal/common/ResultWriter.h"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace offline_lc_minimal {

namespace {

void WriteTextFile(const std::filesystem::path &path, const std::string &content) {
  std::ofstream output_stream(path);
  if (!output_stream.is_open()) {
    throw std::runtime_error("failed to write file: " + path.string());
  }
  output_stream << content;
}

}  // namespace

void ResultWriter::WriteOutputs(
  const std::string &output_dir,
  const OfflineRunnerConfig &config,
  const OfflineRunResult &result,
  const GeoReference &geo_reference) {
  const std::filesystem::path output_path(output_dir);
  std::filesystem::create_directories(output_path);

  WriteTextFile(output_path / "summary.txt",
                result.run_summary.ToMultilineString() + "\n" + result.data_summary.ToMultilineString());
  WriteTextFile(output_path / "data_summary.txt", result.data_summary.ToMultilineString());
  WriteTextFile(output_path / "config_snapshot.cfg", ConfigToString(config));

  {
    std::ofstream trajectory_stream(output_path / "trajectory.csv");
    if (!trajectory_stream.is_open()) {
      throw std::runtime_error("failed to write trajectory.csv");
    }
    trajectory_stream << std::setprecision(17);
    trajectory_stream
      << "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz,"
         "gnss_factor_used,gnss_fix_type,gnss_residual_m,lat_rad,lon_rad,h_m\n";

    for (const auto &row : result.trajectory) {
      const auto llh = geo_reference.Reverse(row.enu_position_m);
      trajectory_stream << row.time_s << ','
                        << row.enu_position_m.x() << ','
                        << row.enu_position_m.y() << ','
                        << row.enu_position_m.z() << ','
                        << row.enu_velocity_mps.x() << ','
                        << row.enu_velocity_mps.y() << ','
                        << row.enu_velocity_mps.z() << ','
                        << row.ypr_rad.x() << ','
                        << row.ypr_rad.y() << ','
                        << row.ypr_rad.z() << ','
                        << row.bias_acc.x() << ','
                        << row.bias_acc.y() << ','
                        << row.bias_acc.z() << ','
                        << row.bias_gyro.x() << ','
                        << row.bias_gyro.y() << ','
                        << row.bias_gyro.z() << ','
                        << (row.gnss_factor_used ? 1 : 0) << ','
                        << ToString(row.gnss_fix_type) << ','
                        << row.gnss_residual_m << ','
                        << llh[0] << ','
                        << llh[1] << ','
                        << llh[2] << '\n';
    }
  }

  if (!result.imu_rate_avp.empty()) {
    std::ofstream imu_rate_stream(output_path / "trajectory_imu_rate.csv");
    if (!imu_rate_stream.is_open()) {
      throw std::runtime_error("failed to write trajectory_imu_rate.csv");
    }
    imu_rate_stream << std::setprecision(17);
    imu_rate_stream
      << "time_s,east_m,north_m,up_m,vx_mps,vy_mps,vz_mps,yaw_rad,pitch_rad,roll_rad,bax,bay,baz,bgx,bgy,bgz,"
         "lat_rad,lon_rad,h_m\n";

    for (const auto &row : result.imu_rate_avp) {
      const auto llh = geo_reference.Reverse(row.enu_position_m);
      imu_rate_stream << row.time_s << ','
                      << row.enu_position_m.x() << ','
                      << row.enu_position_m.y() << ','
                      << row.enu_position_m.z() << ','
                      << row.enu_velocity_mps.x() << ','
                      << row.enu_velocity_mps.y() << ','
                      << row.enu_velocity_mps.z() << ','
                      << row.ypr_rad.x() << ','
                      << row.ypr_rad.y() << ','
                      << row.ypr_rad.z() << ','
                      << row.bias_acc.x() << ','
                      << row.bias_acc.y() << ','
                      << row.bias_acc.z() << ','
                      << row.bias_gyro.x() << ','
                      << row.bias_gyro.y() << ','
                      << row.bias_gyro.z() << ','
                      << llh[0] << ','
                      << llh[1] << ','
                      << llh[2] << '\n';
    }
  }

  if (config.write_debug_csv) {
    std::ofstream residual_stream(output_path / "gnss_residuals.csv");
    if (!residual_stream.is_open()) {
      throw std::runtime_error("failed to write gnss_residuals.csv");
    }
    residual_stream << std::setprecision(17);
    residual_stream << "time_s,gnss_factor_used,gnss_fix_type,gnss_residual_m\n";
    for (const auto &row : result.trajectory) {
      residual_stream << row.time_s << ','
                      << (row.gnss_factor_used ? 1 : 0) << ','
                      << ToString(row.gnss_fix_type) << ','
                      << row.gnss_residual_m << '\n';
    }

    std::ofstream alignment_stream(output_path / "gnss_alignment.csv");
    if (!alignment_stream.is_open()) {
      throw std::runtime_error("failed to write gnss_alignment.csv");
    }
    alignment_stream << std::setprecision(17);
    alignment_stream
      << "sample_index,raw_time_s,corrected_time_s,sync_status,factor_used,fix_type,state_index_i,state_time_i_s,"
         "state_index_j,state_time_j_s,synchronized_state_index,duration_from_state_i_s,meas_east_m,meas_north_m,"
         "meas_up_m,residual_m\n";
    for (const auto &record : result.gnss_factor_records) {
      alignment_stream << record.sample_index << ','
                       << record.raw_time_s << ','
                       << record.corrected_time_s << ','
                       << ToString(record.sync_status) << ','
                       << (record.factor_used ? 1 : 0) << ','
                       << ToString(record.gnss_fix_type) << ','
                       << record.state_index_i << ','
                       << record.state_time_i_s << ','
                       << record.state_index_j << ','
                       << record.state_time_j_s << ','
                       << record.synchronized_state_index << ','
                       << record.duration_from_state_i_s << ','
                       << record.measurement_enu_m.x() << ','
                       << record.measurement_enu_m.y() << ','
                       << record.measurement_enu_m.z() << ','
                       << record.residual_m << '\n';
    }

    if (!result.imu_rate_interval_diagnostics.empty()) {
      std::ofstream imu_rate_diag_stream(output_path / "trajectory_imu_rate_diagnostics.csv");
      if (!imu_rate_diag_stream.is_open()) {
        throw std::runtime_error("failed to write trajectory_imu_rate_diagnostics.csv");
      }
      imu_rate_diag_stream << std::setprecision(17);
      imu_rate_diag_stream
        << "interval_index,start_time_s,end_time_s,imu_sample_count,emitted_sample_count,used_interval,status\n";
      for (const auto &diagnostic : result.imu_rate_interval_diagnostics) {
        imu_rate_diag_stream << diagnostic.interval_index << ','
                             << diagnostic.start_time_s << ','
                             << diagnostic.end_time_s << ','
                             << diagnostic.imu_sample_count << ','
                             << diagnostic.emitted_sample_count << ','
                             << (diagnostic.used_interval ? 1 : 0) << ','
                             << diagnostic.status << '\n';
      }
    }
  }
}

}  // namespace offline_lc_minimal
