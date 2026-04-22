import importlib.util
import pathlib
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_optimized_vertical_profile.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_optimized_vertical_profile", SCRIPT_PATH)
plot_optimized_vertical_profile = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_optimized_vertical_profile)


class PlotOptimizedVerticalProfileTests(unittest.TestCase):
    def test_compute_delta_up_uses_first_row_as_origin(self) -> None:
        rows = [
            {"time_s": 0.0, "up_m": 5.0, "vz_mps": 0.0},
            {"time_s": 1.0, "up_m": 5.2, "vz_mps": 0.1},
            {"time_s": 2.0, "up_m": 4.9, "vz_mps": -0.2},
        ]

        delta_up = plot_optimized_vertical_profile.compute_delta_up(rows)
        self.assertAlmostEqual(delta_up[0], 0.0, places=9)
        self.assertAlmostEqual(delta_up[1], 0.2, places=9)
        self.assertAlmostEqual(delta_up[2], -0.1, places=9)

    def test_compute_vertical_stats_captures_spread(self) -> None:
        rows = [
            {"time_s": 0.0, "up_m": 1.0, "vz_mps": 0.0},
            {"time_s": 1.0, "up_m": 1.2, "vz_mps": 0.1},
            {"time_s": 2.0, "up_m": 0.9, "vz_mps": -0.2},
        ]

        stats = plot_optimized_vertical_profile.compute_vertical_stats(rows)
        self.assertAlmostEqual(stats["delta_up_range_m"], 0.3, places=9)
        self.assertGreater(stats["vz_std_mps"], 0.0)

    def test_filter_valid_rtk_speed_rows_drops_invalid_rows(self) -> None:
        rows = [
            {"time_s": 0.0, "valid_speed": False, "vz_mps": 0.0, "invalid_reason": "boundary"},
            {"time_s": 1.0, "valid_speed": True, "vz_mps": -0.5, "invalid_reason": "valid"},
        ]

        filtered = plot_optimized_vertical_profile.filter_valid_rtk_speed_rows(rows)
        self.assertEqual(filtered, [{"time_s": 1.0, "vz_mps": -0.5}])


if __name__ == "__main__":
    unittest.main()
