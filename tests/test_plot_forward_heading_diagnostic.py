import importlib.util
import math
import pathlib
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_forward_heading_diagnostic.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_forward_heading_diagnostic", SCRIPT_PATH)
plot_forward_heading_diagnostic = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_forward_heading_diagnostic)


class PlotForwardHeadingDiagnosticTests(unittest.TestCase):
    def test_estimate_initial_alignment_recovers_identity_pose_without_bias(self) -> None:
        lat_rad = 0.693587
        gravity_mps2 = 9.81
        earth_rate_enu = [
            0.0,
            plot_forward_heading_diagnostic.EARTH_ROTATION_RATE_RADPS * math.cos(lat_rad),
            plot_forward_heading_diagnostic.EARTH_ROTATION_RATE_RADPS * math.sin(lat_rad),
        ]
        acc_bias = [0.0, 0.0, 0.0]
        gyro_bias = [0.0, 0.0, 0.0]

        imu_rows = []
        for index in range(1001):
            time_s = float(index) * 0.1
            imu_rows.append(
                {
                    "time_s": time_s,
                    "gyro_x": earth_rate_enu[0] + gyro_bias[0],
                    "gyro_y": earth_rate_enu[1] + gyro_bias[1],
                    "gyro_z": earth_rate_enu[2] + gyro_bias[2],
                    "acc_x": acc_bias[0],
                    "acc_y": acc_bias[1],
                    "acc_z": gravity_mps2 + acc_bias[2],
                }
            )

        world_from_body, estimated_acc_bias, estimated_gyro_bias, stationary_count = (
            plot_forward_heading_diagnostic.estimate_initial_alignment(
                imu_rows=imu_rows,
                alignment_start_time_s=0.0,
                navigation_start_time_s=100.0,
                origin_lat_rad=lat_rad,
                gravity_mps2=gravity_mps2,
                stationary_gyro_threshold_radps=0.02,
                stationary_acc_tolerance_mps2=0.8,
                min_sample_count=1000,
                min_cross_norm=1e-3,
            )
        )

        self.assertEqual(stationary_count, 1001)
        for estimated, expected in zip(estimated_acc_bias, acc_bias):
            self.assertAlmostEqual(estimated, expected, places=9)
        for estimated, expected in zip(estimated_gyro_bias, gyro_bias):
            self.assertAlmostEqual(estimated, expected, places=12)
        yaw_deg, pitch_deg, roll_deg = plot_forward_heading_diagnostic.rot3_to_ypr_deg(world_from_body)
        self.assertAlmostEqual(yaw_deg, 0.0, places=9)
        self.assertAlmostEqual(pitch_deg, 0.0, places=9)
        self.assertAlmostEqual(roll_deg, 0.0, places=9)

    def test_feedback_alignment_recovers_identity_pose_with_known_bias(self) -> None:
        lat_rad = 0.693587
        gravity_mps2 = 9.81
        earth_rate_enu = [
            0.0,
            plot_forward_heading_diagnostic.EARTH_ROTATION_RATE_RADPS * math.cos(lat_rad),
            plot_forward_heading_diagnostic.EARTH_ROTATION_RATE_RADPS * math.sin(lat_rad),
        ]
        acc_bias = [0.01, -0.02, 0.03]
        gyro_bias = [1.0e-6, -2.0e-6, 3.0e-6]

        imu_rows = []
        for index in range(1001):
            time_s = float(index) * 0.1
            imu_rows.append(
                {
                    "time_s": time_s,
                    "gyro_x": earth_rate_enu[0] + gyro_bias[0],
                    "gyro_y": earth_rate_enu[1] + gyro_bias[1],
                    "gyro_z": earth_rate_enu[2] + gyro_bias[2],
                    "acc_x": acc_bias[0],
                    "acc_y": acc_bias[1],
                    "acc_z": gravity_mps2 + acc_bias[2],
                }
            )

        world_from_body, residual_acc_bias, residual_gyro_bias, stationary_count = (
            plot_forward_heading_diagnostic.estimate_alignment_from_feedback_bias(
                imu_rows=imu_rows,
                alignment_start_time_s=0.0,
                navigation_start_time_s=100.0,
                origin_lat_rad=lat_rad,
                gravity_mps2=gravity_mps2,
                stationary_gyro_threshold_radps=0.02,
                stationary_acc_tolerance_mps2=0.8,
                min_sample_count=1000,
                min_cross_norm=1e-3,
                feedback_acc_bias=acc_bias,
                feedback_gyro_bias=gyro_bias,
            )
        )

        self.assertEqual(stationary_count, 1001)
        for estimated in residual_acc_bias:
            self.assertAlmostEqual(estimated, 0.0, places=9)
        for estimated in residual_gyro_bias:
            self.assertAlmostEqual(estimated, 0.0, places=12)
        yaw_deg, pitch_deg, roll_deg = plot_forward_heading_diagnostic.rot3_to_ypr_deg(world_from_body)
        self.assertAlmostEqual(yaw_deg, 0.0, places=9)
        self.assertAlmostEqual(pitch_deg, 0.0, places=9)
        self.assertAlmostEqual(roll_deg, 0.0, places=9)

    def test_forward_propagate_heading_keeps_constant_yaw_for_zero_corrected_gyro(self) -> None:
        imu_rows = [
            {"time_s": 0.0, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
            {"time_s": 0.5, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
            {"time_s": 1.0, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
        ]

        rows = plot_forward_heading_diagnostic.forward_propagate_heading_rows(
            imu_rows=imu_rows,
            navigation_start_time_s=0.0,
            end_time_s=1.0,
            initial_world_from_body=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            initial_gyro_bias=[0.0, 0.0, 0.0],
        )

        self.assertEqual(len(rows), 3)
        for row in rows:
            self.assertAlmostEqual(row["yaw_deg"], 0.0, places=9)

    def test_forward_propagate_heading_integrates_constant_yaw_rate(self) -> None:
        yaw_rate_radps = math.radians(10.0)
        imu_rows = [
            {"time_s": 0.0, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": yaw_rate_radps, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
            {"time_s": 0.5, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": yaw_rate_radps, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
            {"time_s": 1.0, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": yaw_rate_radps, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
        ]

        rows = plot_forward_heading_diagnostic.forward_propagate_heading_rows(
            imu_rows=imu_rows,
            navigation_start_time_s=0.0,
            end_time_s=1.0,
            initial_world_from_body=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            initial_gyro_bias=[0.0, 0.0, 0.0],
        )

        self.assertAlmostEqual(rows[-1]["yaw_deg"], 10.0, places=6)

    def test_large_rtk_gap_rejects_window(self) -> None:
        rtk_rows = [
            {"time_s": 0.0, "east_m": 0.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "east_m": 1.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 100.0, "east_m": 100.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 101.0, "east_m": 101.0, "north_m": 0.0, "up_m": 0.0},
        ]

        heading_rows = plot_forward_heading_diagnostic.build_rtk_heading_rows(rtk_rows, 1.0)

        self.assertFalse(heading_rows[1]["valid_heading"])
        self.assertEqual(heading_rows[1]["invalid_reason"], "large_gap")


if __name__ == "__main__":
    unittest.main()
