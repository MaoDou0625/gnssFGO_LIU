import csv
import importlib.util
import pathlib
import tempfile
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_attitude_over_time.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_attitude_over_time", SCRIPT_PATH)
plot_attitude_over_time = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_attitude_over_time)


class PlotAttitudeOverTimeTests(unittest.TestCase):
    def test_read_attitude_rows_unwraps_yaw(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            trajectory_path = pathlib.Path(temp_dir) / "trajectory.csv"
            with trajectory_path.open("w", encoding="utf-8", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [
                        "time_s",
                        "east_m",
                        "north_m",
                        "up_m",
                        "vx_mps",
                        "vy_mps",
                        "vz_mps",
                        "yaw_rad",
                        "pitch_rad",
                        "roll_rad",
                        "bax",
                        "bay",
                        "baz",
                        "bgx",
                        "bgy",
                        "bgz",
                        "lat_rad",
                        "lon_rad",
                        "h_m",
                        "gnss_factor_used",
                        "gnss_fix_type",
                        "gnss_residual_m",
                    ]
                )
                writer.writerow([0.0, 0, 0, 0, 0, 0, 0, 3.12413936107, 0.01745329252, -0.03490658504, 0, 0, 0, 0, 0, 1.0e-6, 0.5, 1.0, 100.0, 1, "RTKFIX", 0.0])
                writer.writerow([1.0, 0, 0, 0, 0, 0, 0, -3.12413936107, 0.02617993878, -0.01745329252, 0, 0, 0, 0, 0, 2.0e-6, 0.5, 1.0, 100.1, 1, "RTKFIX", 0.0])

            rows = plot_attitude_over_time.read_attitude_rows(trajectory_path)

        self.assertEqual(len(rows), 2)
        self.assertAlmostEqual(rows[0]["yaw_deg"], 179.0, places=3)
        self.assertAlmostEqual(rows[1]["yaw_unwrapped_deg"], 181.0, places=3)
        self.assertAlmostEqual(rows[0]["pitch_deg"], 1.0, places=3)
        self.assertAlmostEqual(rows[1]["roll_deg"], -1.0, places=3)
        self.assertAlmostEqual(rows[1]["bgz_radps"], 2.0e-6, places=12)

    def test_make_plot_writes_three_subplot_png(self) -> None:
        rows = [
            {
                "time_s": 0.0,
                "yaw_deg": 0.0,
                "yaw_unwrapped_deg": 0.0,
                "pitch_deg": 1.0,
                "roll_deg": -1.0,
                "bgz_radps": 1.0e-6,
            },
            {
                "time_s": 1.0,
                "yaw_deg": 5.0,
                "yaw_unwrapped_deg": 5.0,
                "pitch_deg": 1.5,
                "roll_deg": -0.5,
                "bgz_radps": 2.0e-6,
            },
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = pathlib.Path(temp_dir) / "attitude_over_time.png"
            plot_attitude_over_time.make_plot(rows, output_path, "Test attitude plot")
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
