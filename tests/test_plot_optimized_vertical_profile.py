import importlib.util
import pathlib
import tempfile
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

    def test_compute_delta_up_can_anchor_to_navigation_start_time(self) -> None:
        rows = [
            {"time_s": -100.0, "up_m": 10.0, "vz_mps": 0.0},
            {"time_s": -50.0, "up_m": 10.4, "vz_mps": 0.0},
            {"time_s": 0.0, "up_m": 10.2, "vz_mps": 0.0},
            {"time_s": 1.0, "up_m": 10.5, "vz_mps": 0.1},
        ]

        delta_up = plot_optimized_vertical_profile.compute_delta_up(
            rows,
            reference_time_s=0.0,
        )
        self.assertAlmostEqual(delta_up[0], -0.2, places=9)
        self.assertAlmostEqual(delta_up[1], 0.2, places=9)
        self.assertAlmostEqual(delta_up[2], 0.0, places=9)
        self.assertAlmostEqual(delta_up[3], 0.3, places=9)

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

    def test_filter_rows_from_time_discards_pre_navigation_samples(self) -> None:
        rows = [
            {"time_s": -1.0, "up_m": 10.0, "vz_mps": 0.0},
            {"time_s": 0.0, "up_m": 10.1, "vz_mps": 0.0},
            {"time_s": 1.0, "up_m": 10.2, "vz_mps": 0.1},
        ]

        filtered = plot_optimized_vertical_profile.filter_rows_from_time(rows, 0.0)
        self.assertEqual(
            filtered,
            [
                {"time_s": 0.0, "up_m": 10.1, "vz_mps": 0.0},
                {"time_s": 1.0, "up_m": 10.2, "vz_mps": 0.1},
            ],
        )

    def test_resolve_dynamic_start_time_reads_summary_field(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            trajectory_path = temp_path / "trajectory.csv"
            summary_path = temp_path / "summary.txt"
            trajectory_path.write_text(
                "time_s,up_m,vz_mps\n5701.4,10.0,0.0\n5801.6,10.1,0.0\n",
                encoding="utf-8",
            )
            summary_path.write_text(
                "navigation_start_time_s=5701.4\n"
                "dynamic_start_time_s=5801.6\n",
                encoding="utf-8",
            )

            dynamic_start_time_s = plot_optimized_vertical_profile.resolve_dynamic_start_time(trajectory_path)
            self.assertAlmostEqual(dynamic_start_time_s, 5801.6, places=9)


if __name__ == "__main__":
    unittest.main()
