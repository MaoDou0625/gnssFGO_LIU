import importlib.util
import pathlib
import tempfile
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_alignment_navigation_continuity.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_alignment_navigation_continuity", SCRIPT_PATH)
plot_alignment_navigation_continuity = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_alignment_navigation_continuity)


class PlotAlignmentNavigationContinuityTests(unittest.TestCase):
    def test_resolve_plot_context_uses_alignment_start(self) -> None:
        rows = [
            {"time_s": 5701.3996, "up_m": 0.0, "vz_mps": 0.0, "pitch_rad": 0.0, "roll_rad": 0.0, "baz": -0.1, "baz_ug": -10197.162129779282},
            {"time_s": 5801.5970, "up_m": 0.1, "vz_mps": 0.0, "pitch_rad": 0.0, "roll_rad": 0.0, "baz": -0.1, "baz_ug": -10197.162129779282},
        ]
        summary = {
            "alignment_start_time_s": 5701.4,
            "dynamic_start_time_s": 5801.6,
        }

        context = plot_alignment_navigation_continuity.resolve_plot_context(summary, rows)

        self.assertAlmostEqual(context.reference_time_s, 5701.3996, places=9)
        self.assertAlmostEqual(context.dynamic_start_time_s, 5801.6, places=9)

    def test_read_envelope_rows_keeps_single_run_gate_values(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = pathlib.Path(temp_dir) / "vertical_envelope_diagnostics.csv"
            path.write_text(
                "corrected_time_s,rtk_up_m,half_width_m,factor_used\n"
                "1.0,2.0,0.4,1\n"
                "2.0,2.1,0.8,0\n"
                "3.0,2.2,0.4,1\n",
                encoding="utf-8",
            )

            rows = plot_alignment_navigation_continuity.read_envelope_rows(path)

            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(rows[0]["half_width_m"], 0.4, places=9)
            self.assertAlmostEqual(rows[1]["time_s"], 3.0, places=9)

    def test_read_raw_rtk_rows_uses_config_snapshot_and_static_samples(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_path = pathlib.Path(temp_dir)
            gnss_path = temp_path / "gnss_solution.txt"
            gnss_path.write_text(
                "10.0 0.0 0.0 101.0 0.01 0.01 0.02 0 0 10 1 7 1 0 0 0 1 4\n"
                "11.0 0.0 0.0 91.0 NaN NaN NaN 0 0 10 NaN NaN 4 0 0 0 1 4\n"
                "12.0 0.0 0.0 102.0 0.01 0.01 0.02 0 0 10 2 7 1 0 0 0 1 4\n"
                "13.0 0.0 0.0 103.0 0.01 0.01 0.02 0 0 10 1 7 2 0 0 0 1 4\n",
                encoding="utf-8",
            )
            config_path = temp_path / "config_snapshot.cfg"
            config_path.write_text(
                f"gnss_path={gnss_path}\n"
                "required_best_sol_status_code=1\n"
                "gnss_vertical_sigma_mode=fixed\n"
                "gnss_vertical_fixed_sigma_m=0.20\n"
                "gnss_sigma_scale_up=1.0\n"
                "rtkfix_scale=1.0\n"
                "vertical_envelope_gate_sigma_multiple=2.0\n",
                encoding="utf-8",
            )

            result = plot_alignment_navigation_continuity.read_raw_rtk_rows(
                config_path,
                {"origin_h_m": 100.0},
            )

            self.assertEqual(result.source, "raw_gnss")
            self.assertEqual(len(result.rows), 1)
            self.assertAlmostEqual(result.rows[0]["time_s"], 10.0, places=9)
            self.assertAlmostEqual(result.rows[0]["rtk_up_m"], 1.0, places=9)
            self.assertAlmostEqual(result.rows[0]["half_width_m"], 0.4, places=9)

    def test_read_nhc_windows_drops_skipped_factors(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = pathlib.Path(temp_dir) / "body_z_nhc_diagnostics.csv"
            path.write_text(
                "start_time_s,end_time_s,factor_added\n"
                "10.0,11.0,1\n"
                "12.0,13.0,0\n",
                encoding="utf-8",
            )

            windows = plot_alignment_navigation_continuity.read_nhc_windows(path)

            self.assertEqual(windows, [(10.0, 11.0)])

    def test_padded_axis_limits_uses_height_values_without_gate_width(self) -> None:
        limits = plot_alignment_navigation_continuity.padded_axis_limits([0.0, 0.02, -0.01, 0.01])

        self.assertIsNotNone(limits)
        self.assertGreater(limits[0], -0.05)
        self.assertLess(limits[1], 0.05)

    def test_padded_axis_limits_expands_tiny_ranges(self) -> None:
        limits = plot_alignment_navigation_continuity.padded_axis_limits([0.001, 0.001])

        self.assertIsNotNone(limits)
        self.assertAlmostEqual(limits[1] - limits[0], 0.02, places=9)


if __name__ == "__main__":
    unittest.main()
