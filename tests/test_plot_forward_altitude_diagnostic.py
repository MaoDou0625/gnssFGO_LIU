import importlib.util
import pathlib
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_forward_altitude_diagnostic.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_forward_altitude_diagnostic", SCRIPT_PATH)
plot_forward_altitude_diagnostic = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_forward_altitude_diagnostic)


class PlotForwardAltitudeDiagnosticTests(unittest.TestCase):
    def test_static_forward_propagation_keeps_altitude_flat(self) -> None:
        imu_rows = [
            {"time_s": 0.0, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
            {"time_s": 0.1, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
            {"time_s": 0.2, "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0, "acc_x": 0.0, "acc_y": 0.0, "acc_z": 9.81},
        ]

        rows = plot_forward_altitude_diagnostic.propagate_forward_altitude_rows(
            imu_rows=imu_rows,
            start_time_s=0.0,
            end_time_s=0.2,
            initial_world_from_body=[[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            initial_acc_bias=[0.0, 0.0, 0.0],
            initial_gyro_bias=[0.0, 0.0, 0.0],
            initial_position_enu_m=[0.0, 0.0, 0.0],
            initial_velocity_enu_mps=[0.0, 0.0, 0.0],
            gravity_mps2=9.81,
        )

        self.assertGreaterEqual(len(rows), 2)
        self.assertAlmostEqual(rows[-1]["up_m"], 0.0, places=9)
        self.assertAlmostEqual(rows[-1]["vz_mps"], 0.0, places=9)

    def test_window_stats_capture_altitude_spread(self) -> None:
        rows = [
            {"time_s": 0.0, "up_m": 1.0, "vz_mps": 0.0},
            {"time_s": 1.0, "up_m": 1.2, "vz_mps": 0.1},
            {"time_s": 2.0, "up_m": 0.9, "vz_mps": -0.2},
        ]

        stats = plot_forward_altitude_diagnostic.compute_window_stats(rows, start_time_s=0.0, duration_s=2.0)

        self.assertAlmostEqual(stats["delta_up_range_m"], 0.3, places=9)
        self.assertAlmostEqual(stats["delta_up_end_m"], -0.1, places=9)
        self.assertGreater(stats["vz_range_mps"], 0.0)

    def test_filter_rows_by_time_window_keeps_bounds(self) -> None:
        rows = [
            {"time_s": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "up_m": 1.0},
            {"time_s": 2.0, "up_m": 2.0},
        ]

        filtered = plot_forward_altitude_diagnostic.filter_rows_by_time_window(
            rows,
            start_time_s=0.5,
            end_time_s=1.5,
        )

        self.assertEqual([row["time_s"] for row in filtered], [1.0])


if __name__ == "__main__":
    unittest.main()
