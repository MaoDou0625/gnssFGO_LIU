#include "offline_lc_minimal/common/ResultWriter.h"
#include "offline_lc_minimal/common/ResultOutputWriters.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <functional>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>

namespace offline_lc_minimal {

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

  WriteTrajectoryCsv(output_path / "trajectory.csv", result.trajectory, geo_reference);
  const std::vector<TrajectoryRow> dynamic_trajectory_rows =
    FilterDynamicTrajectoryRows(result.trajectory, result.run_summary.dynamic_start_time_s);
  WriteInitialDynamicConsistencyCsv(
    output_path / "initial_dynamic_consistency.csv",
    dynamic_trajectory_rows,
    result.run_summary);
  if (!result.initial_static_trajectory.empty()) {
    WriteTrajectoryCsv(output_path / "initial_static_trajectory.csv", result.initial_static_trajectory, geo_reference);
  }
  if (!result.optimized_static_terminal_forward_trajectory.empty()) {
    WriteTrajectoryCsv(
      output_path / "optimized_static_terminal_forward_trajectory.csv",
      result.optimized_static_terminal_forward_trajectory,
      geo_reference);
  }
  if (!result.reference_node_trajectory.empty()) {
    WriteReferenceNodeCsv(output_path / "reference_node_trajectory.csv", result.reference_node_trajectory, geo_reference);
  }
  if (!result.seed_body_z_acc_diagnostics.empty()) {
    WriteSeedBodyZAccDiagnosticsCsv(
      output_path / "seed_body_z_acc_diagnostics.csv",
      result.seed_body_z_acc_diagnostics);
  }
  if (!result.body_z_seed_jump_windows.empty()) {
    WriteBodyZSeedJumpWindowCsv(
      output_path / "body_z_seed_jump_windows.csv",
      result.body_z_seed_jump_windows);
  }
  if (!result.error_state_trajectory.empty()) {
    WriteErrorStateCsv(output_path / "error_state_trajectory.csv", result.error_state_trajectory);
  }
  if (!result.segment_error_diagnostics.empty()) {
    WriteSegmentErrorCsv(output_path / "segment_error_diagnostics.csv", result.segment_error_diagnostics);
    WriteTextFile(
      output_path / "segment_error_summary.txt",
      BuildSegmentErrorSummaryText(result.segment_error_diagnostics));
  }
  if (!result.gnss_consistency_records.empty()) {
    WriteGnssConsistencyCsv(output_path / "gnss_consistency.csv", result.gnss_consistency_records);
  }
  if (!result.vertical_envelope_diagnostics.empty()) {
    WriteVerticalEnvelopeDiagnosticsCsv(
      output_path / "vertical_envelope_diagnostics.csv",
      result.vertical_envelope_diagnostics);
  }
  if (!result.vertical_velocity_delta_diagnostics.empty()) {
    WriteVerticalVelocityDeltaDiagnosticsCsv(
      output_path / "vertical_velocity_delta_diagnostics.csv",
      result.vertical_velocity_delta_diagnostics);
  }
  if (!result.body_z_nhc_diagnostics.empty()) {
    WriteBodyZNHCDiagnosticsCsv(
      output_path / "body_z_nhc_diagnostics.csv",
      result.body_z_nhc_diagnostics);
  }
  if (!result.vertical_jump_masked_imu_diagnostics.empty()) {
    WriteVerticalJumpMaskedImuDiagnosticsCsv(
      output_path / "vertical_jump_masked_imu_diagnostics.csv",
      result.vertical_jump_masked_imu_diagnostics);
  }
  if (!result.vertical_jump_impulse_diagnostics.empty()) {
    WriteVerticalJumpImpulseDiagnosticsCsv(
      output_path / "vertical_jump_impulse_diagnostics.csv",
      result.vertical_jump_impulse_diagnostics);
  }
  if (!result.vertical_jump_bias_diagnostics.empty()) {
    WriteVerticalJumpBiasDiagnosticsCsv(
      output_path / "vertical_jump_bias_diagnostics.csv",
      result.vertical_jump_bias_diagnostics);
  }
  if (!result.vertical_jump_velocity_ramp_diagnostics.empty()) {
    WriteVerticalJumpVelocityRampDiagnosticsCsv(
      output_path / "vertical_jump_velocity_ramp_diagnostics.csv",
      result.vertical_jump_velocity_ramp_diagnostics);
  }
  if (!result.vertical_jump_continuity_diagnostics.empty()) {
    WriteVerticalJumpContinuityDiagnosticsCsv(
      output_path / "vertical_jump_continuity_diagnostics.csv",
      result.vertical_jump_continuity_diagnostics);
  }
  if (!result.vertical_state_corrections.empty()) {
    WriteVerticalStateCorrectionCsv(
      output_path / "vertical_state_corrections.csv",
      result.vertical_state_corrections);
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
      << "sample_index,raw_time_s,corrected_time_s,sync_status,factor_used,vertical_direct_position_factor_used,"
         "fix_type,trajectory_row_index_i,state_time_i_s,"
         "trajectory_row_index_j,state_time_j_s,synchronized_trajectory_row_index,graph_state_index_i,graph_state_index_j,"
         "graph_synchronized_state_index,duration_from_state_i_s,meas_east_m,meas_north_m,meas_up_m,residual_m\n";
    for (const auto &record : result.gnss_factor_records) {
      const long long graph_state_index_i = std::isfinite(record.state_time_i_s)
                                              ? static_cast<long long>(record.state_index_i)
                                              : -1;
      const long long graph_state_index_j = std::isfinite(record.state_time_j_s)
                                              ? static_cast<long long>(record.state_index_j)
                                              : -1;
      const long long graph_synchronized_state_index =
        record.factor_used &&
            (record.sync_status == StateMeasSyncStatus::kSynchronizedI ||
             record.sync_status == StateMeasSyncStatus::kSynchronizedJ)
          ? static_cast<long long>(record.synchronized_state_index)
          : -1;
      alignment_stream << record.sample_index << ','
                       << record.raw_time_s << ','
                       << record.corrected_time_s << ','
                       << ToString(record.sync_status) << ','
                       << (record.factor_used ? 1 : 0) << ','
                       << (record.vertical_direct_position_factor_used ? 1 : 0) << ','
                       << ToString(record.gnss_fix_type) << ','
                       << record.trajectory_row_index_i << ','
                       << record.state_time_i_s << ','
                       << record.trajectory_row_index_j << ','
                       << record.state_time_j_s << ','
                       << record.synchronized_trajectory_row_index << ','
                       << graph_state_index_i << ','
                       << graph_state_index_j << ','
                       << graph_synchronized_state_index << ','
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
