import importlib.util
import pathlib
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_heading_vs_rtk.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_heading_vs_rtk", SCRIPT_PATH)
plot_heading_vs_rtk = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_heading_vs_rtk)


class PlotHeadingVsRtkTests(unittest.TestCase):
    def test_directional_window_keeps_sparse_valid_heading(self) -> None:
        rtk_rows = [
            {"time_s": 0.0, "east_m": 0.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "east_m": 1.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 2.0, "east_m": 2.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 3.0, "east_m": 3.0, "north_m": 0.0, "up_m": 0.0},
        ]

        heading_rows = plot_heading_vs_rtk.build_rtk_heading_rows(rtk_rows, 1.0)

        self.assertFalse(heading_rows[0]["valid_heading"])
        self.assertTrue(heading_rows[1]["valid_heading"])
        self.assertAlmostEqual(heading_rows[1]["heading_deg"], 0.0, places=6)
        self.assertTrue(heading_rows[2]["valid_heading"])
        self.assertAlmostEqual(heading_rows[2]["heading_deg"], 0.0, places=6)
        self.assertFalse(heading_rows[3]["valid_heading"])

    def test_low_displacement_window_is_rejected(self) -> None:
        rtk_rows = [
            {"time_s": 0.0, "east_m": 0.00, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 0.5, "east_m": 0.05, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "east_m": 0.10, "north_m": 0.0, "up_m": 0.0},
        ]

        heading_rows = plot_heading_vs_rtk.build_rtk_heading_rows(rtk_rows, 1.0)

        self.assertFalse(heading_rows[1]["valid_heading"])
        self.assertEqual(heading_rows[1]["invalid_reason"], "low_displacement")

    def test_large_rtk_gap_rejects_window(self) -> None:
        rtk_rows = [
            {"time_s": 0.0, "east_m": 0.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "east_m": 1.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 100.0, "east_m": 100.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 101.0, "east_m": 101.0, "north_m": 0.0, "up_m": 0.0},
        ]

        heading_rows = plot_heading_vs_rtk.build_rtk_heading_rows(rtk_rows, 1.0)

        self.assertFalse(heading_rows[1]["valid_heading"])
        self.assertEqual(heading_rows[1]["invalid_reason"], "large_gap")

    def test_match_heading_pairs_wraps_error_into_signed_range(self) -> None:
        nav_rows = [
            {"time_s": 10.0, "yaw_deg": -179.0, "yaw_unwrapped_deg": -179.0},
        ]
        rtk_heading_rows = [
            {
                "time_s": 10.0,
                "heading_deg": 179.0,
                "heading_unwrapped_deg": 179.0,
                "window_displacement_m": 1.0,
                "valid_heading": True,
                "invalid_reason": "valid",
            }
        ]

        pairs = plot_heading_vs_rtk.match_heading_pairs(nav_rows, rtk_heading_rows, tolerance_s=0.12)

        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0]["heading_error_deg"], 2.0, places=6)

    def test_match_heading_pairs_raises_when_no_time_match_exists(self) -> None:
        nav_rows = [
            {"time_s": 0.0, "yaw_deg": 0.0, "yaw_unwrapped_deg": 0.0},
        ]
        rtk_heading_rows = [
            {
                "time_s": 1.0,
                "heading_deg": 0.0,
                "heading_unwrapped_deg": 0.0,
                "window_displacement_m": 1.0,
                "valid_heading": True,
                "invalid_reason": "valid",
            }
        ]

        with self.assertRaises(ValueError):
            plot_heading_vs_rtk.match_heading_pairs(nav_rows, rtk_heading_rows, tolerance_s=0.12)


if __name__ == "__main__":
    unittest.main()
