import importlib.util
import pathlib
import tempfile
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_nav_vs_rtk.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_nav_vs_rtk", SCRIPT_PATH)
plot_nav_vs_rtk = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_nav_vs_rtk)


class PlotNavVsRtkTests(unittest.TestCase):
    def test_match_pairs_applies_error_deadband(self) -> None:
        trajectory_rows = [
            {
                "time_s": 0.0,
                "east_m": 0.03,
                "north_m": 0.0,
                "up_m": -0.015,
            }
        ]
        rtk_rows = [{"time_s": 0.0, "east_m": 0.0, "north_m": 0.0, "up_m": 0.0}]

        pairs = plot_nav_vs_rtk.match_pairs(
            trajectory_rows,
            rtk_rows,
            tolerance_s=0.1,
            error_deadband_m=0.02,
        )

        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0]["east_error_eval_m"], 0.01)
        self.assertAlmostEqual(pairs[0]["north_error_eval_m"], 0.0)
        self.assertAlmostEqual(pairs[0]["up_error_eval_m"], 0.0)
        self.assertAlmostEqual(pairs[0]["horizontal_error_eval_m"], 0.01)

    def test_stats_text_reports_eval_deadband(self) -> None:
        pairs = [
            {
                "horizontal_error_m": 0.03,
                "position_error_m": 0.03,
                "up_error_m": -0.015,
                "horizontal_error_eval_m": 0.01,
                "position_error_eval_m": 0.01,
                "up_error_eval_m": 0.0,
            }
        ]

        stats_text = plot_nav_vs_rtk.build_stats_text(pairs, error_deadband_m=0.02)

        self.assertIn("eval deadband=0.020 m", stats_text)
        self.assertIn("eval horiz rms=0.010 m", stats_text)
        self.assertIn("eval up rms=0.000 m", stats_text)

    def test_make_plot_writes_png(self) -> None:
        trajectory_rows = [
            {
                "time_s": 0.0,
                "east_m": 0.0,
                "north_m": 0.0,
                "up_m": 0.0,
                "lat_rad": 0.5,
                "lon_rad": 1.0,
                "h_m": 100.0,
                "gnss_fix_type": "RTKFIX",
            },
            {
                "time_s": 1.0,
                "east_m": 1.0,
                "north_m": 0.5,
                "up_m": 0.2,
                "lat_rad": 0.5,
                "lon_rad": 1.0,
                "h_m": 100.2,
                "gnss_fix_type": "RTKFIX",
            },
        ]
        rtk_rows = [
            {"time_s": 0.0, "east_m": 0.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "east_m": 0.8, "north_m": 0.4, "up_m": 0.1},
        ]
        pairs = [
            {
                "time_s": 0.0,
                "east_error_m": 0.0,
                "north_error_m": 0.0,
                "up_error_m": 0.0,
                "horizontal_error_m": 0.0,
                "position_error_m": 0.0,
            },
            {
                "time_s": 1.0,
                "east_error_m": 0.2,
                "north_error_m": 0.1,
                "up_error_m": 0.1,
                "horizontal_error_m": 0.22360679775,
                "position_error_m": 0.24494897428,
            },
        ]

        with tempfile.TemporaryDirectory() as temp_dir:
            output_path = pathlib.Path(temp_dir) / "nav_vs_rtk.png"
            plot_nav_vs_rtk.make_plot(
                trajectory_rows,
                rtk_rows,
                pairs,
                output_path,
                "Test nav plot",
                error_deadband_m=0.02,
            )
            self.assertTrue(output_path.exists())
            self.assertGreater(output_path.stat().st_size, 0)


if __name__ == "__main__":
    unittest.main()
