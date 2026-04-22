import importlib.util
import pathlib
import unittest


SCRIPT_PATH = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "plot_segment_error_diagnostics.py"
SCRIPT_SPEC = importlib.util.spec_from_file_location("plot_segment_error_diagnostics", SCRIPT_PATH)
plot_segment_error_diagnostics = importlib.util.module_from_spec(SCRIPT_SPEC)
SCRIPT_SPEC.loader.exec_module(plot_segment_error_diagnostics)


class PlotSegmentErrorDiagnosticsTests(unittest.TestCase):
    def test_total_variation_accumulates_absolute_differences(self) -> None:
        self.assertAlmostEqual(plot_segment_error_diagnostics.total_variation([0.0, 1.0, -1.0]), 3.0, places=9)

    def test_summarize_reports_component_ranges(self) -> None:
        rows = [
            {
                "mid_time_s": 0.0,
                "dtheta_x_rad": 0.0,
                "dtheta_y_rad": 0.1,
                "dtheta_z_rad": -0.1,
                "dv_x_mps": 1.0,
                "dv_y_mps": 0.0,
                "dv_z_mps": -1.0,
                "dp_x_m": 0.01,
                "dp_y_m": 0.02,
                "dp_z_m": 0.03,
                "dbg_x_radps": 0.0,
                "dbg_y_radps": 0.0,
                "dbg_z_radps": 0.0,
                "dba_x_mps2": 0.1,
                "dba_y_mps2": 0.2,
                "dba_z_mps2": 0.3,
                "mean_prefit_nis": 1.0,
                "mean_postfit_nis": 1.0,
                "mean_covariance_scale": 1.0,
            },
            {
                "mid_time_s": 1.0,
                "dtheta_x_rad": 0.2,
                "dtheta_y_rad": -0.2,
                "dtheta_z_rad": 0.3,
                "dv_x_mps": -2.0,
                "dv_y_mps": 2.0,
                "dv_z_mps": 0.5,
                "dp_x_m": -0.02,
                "dp_y_m": 0.01,
                "dp_z_m": 0.00,
                "dbg_x_radps": 0.01,
                "dbg_y_radps": -0.02,
                "dbg_z_radps": 0.03,
                "dba_x_mps2": -0.4,
                "dba_y_mps2": 0.5,
                "dba_z_mps2": -0.6,
                "mean_prefit_nis": 1.0,
                "mean_postfit_nis": 1.0,
                "mean_covariance_scale": 1.0,
            },
        ]

        stats = plot_segment_error_diagnostics.summarize(rows)
        target = next(row for row in stats if row["group"] == "dtheta" and row["component"] == "x")
        self.assertAlmostEqual(float(target["range"]), 0.2, places=9)
        self.assertAlmostEqual(float(target["end_minus_start"]), 0.2, places=9)


if __name__ == "__main__":
    unittest.main()
