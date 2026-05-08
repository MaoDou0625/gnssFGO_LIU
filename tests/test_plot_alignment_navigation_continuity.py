import importlib.util
import pathlib
import tempfile
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_alignment_navigation_continuity.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_alignment_navigation_continuity", SCRIPT_PATH)
plot_alignment_navigation_continuity = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_alignment_navigation_continuity)


class PlotAlignmentNavigationContinuityTests(unittest.TestCase):
    def test_resolve_plot_context_uses_alignment_start_and_static_baz_reference(self) -> None:
        rows = [
            {"time_s": 5701.3996, "up_m": 0.0, "vz_mps": 0.0, "pitch_rad": 0.0, "roll_rad": 0.0, "baz": -0.1},
            {"time_s": 5801.5970, "up_m": 0.1, "vz_mps": 0.0, "pitch_rad": 0.0, "roll_rad": 0.0, "baz": -0.1},
        ]
        summary = {
            "alignment_start_time_s": 5701.4,
            "dynamic_start_time_s": 5801.6,
            "static_vertical_bias_ref_mps2": -0.005,
        }

        context = plot_alignment_navigation_continuity.resolve_plot_context(summary, rows)

        self.assertAlmostEqual(context.reference_time_s, 5701.3996, places=9)
        self.assertAlmostEqual(context.dynamic_start_time_s, 5801.6, places=9)
        self.assertAlmostEqual(context.static_baz_ref_mps2, -0.005, places=9)

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


if __name__ == "__main__":
    unittest.main()
