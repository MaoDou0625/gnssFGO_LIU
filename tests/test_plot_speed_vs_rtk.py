import importlib.util
import pathlib
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_speed_vs_rtk.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_speed_vs_rtk", SCRIPT_PATH)
plot_speed_vs_rtk = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_speed_vs_rtk)


class PlotSpeedVsRtkTests(unittest.TestCase):
    def test_centered_difference_recovers_constant_velocity(self) -> None:
        rtk_rows = [
            {"time_s": 0.0, "east_m": 0.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "east_m": 1.0, "north_m": 2.0, "up_m": -1.0},
            {"time_s": 2.0, "east_m": 2.0, "north_m": 4.0, "up_m": -2.0},
        ]

        speed_rows = plot_speed_vs_rtk.build_rtk_speed_rows(rtk_rows, 1.0)

        self.assertFalse(speed_rows[0]["valid_speed"])
        self.assertTrue(speed_rows[1]["valid_speed"])
        self.assertAlmostEqual(speed_rows[1]["vx_mps"], 1.0, places=6)
        self.assertAlmostEqual(speed_rows[1]["vy_mps"], 2.0, places=6)
        self.assertAlmostEqual(speed_rows[1]["vz_mps"], -1.0, places=6)
        self.assertAlmostEqual(speed_rows[1]["speed_mps"], (1.0**2 + 2.0**2 + 1.0**2) ** 0.5, places=6)
        self.assertFalse(speed_rows[2]["valid_speed"])

    def test_large_gap_marks_speed_invalid(self) -> None:
        rtk_rows = [
            {"time_s": 0.0, "east_m": 0.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 1.0, "east_m": 1.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 100.0, "east_m": 100.0, "north_m": 0.0, "up_m": 0.0},
            {"time_s": 101.0, "east_m": 101.0, "north_m": 0.0, "up_m": 0.0},
        ]

        speed_rows = plot_speed_vs_rtk.build_rtk_speed_rows(rtk_rows, 1.0)

        self.assertFalse(speed_rows[1]["valid_speed"])
        self.assertEqual(speed_rows[1]["invalid_reason"], "large_gap")

    def test_match_speed_pairs_computes_component_errors(self) -> None:
        nav_rows = [
            {
                "time_s": 10.0,
                "vx_mps": 1.5,
                "vy_mps": -0.5,
                "vz_mps": 0.25,
                "speed_mps": (1.5**2 + (-0.5) ** 2 + 0.25**2) ** 0.5,
            }
        ]
        rtk_speed_rows = [
            {
                "time_s": 10.0,
                "vx_mps": 1.0,
                "vy_mps": -1.0,
                "vz_mps": 0.0,
                "speed_mps": (1.0**2 + (-1.0) ** 2) ** 0.5,
                "window_dt_s": 1.0,
                "valid_speed": True,
                "invalid_reason": "valid",
            }
        ]

        pairs = plot_speed_vs_rtk.match_speed_pairs(nav_rows, rtk_speed_rows, tolerance_s=0.12)

        self.assertEqual(len(pairs), 1)
        self.assertAlmostEqual(pairs[0]["vx_error_mps"], 0.5, places=6)
        self.assertAlmostEqual(pairs[0]["vy_error_mps"], 0.5, places=6)
        self.assertAlmostEqual(pairs[0]["vz_error_mps"], 0.25, places=6)


if __name__ == "__main__":
    unittest.main()
