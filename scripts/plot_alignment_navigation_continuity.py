#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import NamedTuple

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "matplotlib is required to generate plots. "
        "Use a Python environment with matplotlib installed."
    ) from exc


TIME_EPSILON_S = 1e-9
MICRO_G_TO_MPS2 = 9.80665e-6
RTK_FIX_CODE = 1
RTK_FLOAT_CODE = 2
SINGLE_CODE = 3
NO_SOLUTION_CODE = 4


class PlotContext(NamedTuple):
    reference_time_s: float
    dynamic_start_time_s: float | None


class RtkPlotRows(NamedTuple):
    rows: list[dict[str, float]]
    source: str


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot one run from static alignment start through dynamic navigation."
    )
    parser.add_argument("--run-dir", required=True, help="Offline run output directory")
    parser.add_argument("--output", required=True, help="Output PNG path")
    parser.add_argument(
        "--title",
        default="Alignment-to-navigation continuity",
        help="Figure title",
    )
    return parser.parse_args()


def safe_float(value: str | None) -> float:
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def mps2_to_ug(value: float) -> float:
    return value / MICRO_G_TO_MPS2


def read_summary(path: Path) -> dict[str, float]:
    result: dict[str, float] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        parsed = safe_float(value.strip())
        if math.isfinite(parsed):
            result[key.strip()] = parsed
    return result


def read_config_values(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    if not path.exists():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        stripped = line.strip()
        if not stripped or stripped.startswith("#") or "=" not in stripped:
            continue
        key, value = stripped.split("=", 1)
        result[key.strip()] = value.strip()
    return result


def config_float(config: dict[str, str], key: str, default: float) -> float:
    value = safe_float(config.get(key))
    return value if math.isfinite(value) else default


def config_int(config: dict[str, str], key: str, default: int) -> int:
    value = safe_float(config.get(key))
    return int(round(value)) if math.isfinite(value) else default


def rounded_int(value: str, default: int) -> int:
    parsed = safe_float(value)
    return int(round(parsed)) if math.isfinite(parsed) else default


def resolve_data_path(path_value: str | None, base_dir: Path) -> Path | None:
    if not path_value:
        return None
    normalized = path_value.strip()
    direct_path = Path(normalized)
    if direct_path.exists():
        return direct_path
    if normalized.startswith("/mnt/") and len(normalized) > 6 and normalized[5].isalpha():
        drive = normalized[5].upper()
        return Path(f"{drive}:{normalized[6:]}")
    if direct_path.is_absolute():
        return direct_path
    return base_dir / direct_path


def read_trajectory(path: Path) -> list[dict[str, float]]:
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            rows.append(
                {
                    "time_s": safe_float(raw.get("time_s")),
                    "up_m": safe_float(raw.get("up_m")),
                    "vz_mps": safe_float(raw.get("vz_mps")),
                    "pitch_rad": safe_float(raw.get("pitch_rad")),
                    "roll_rad": safe_float(raw.get("roll_rad")),
                    "baz": safe_float(raw.get("baz")),
                    "baz_ug": mps2_to_ug(safe_float(raw.get("baz"))),
                }
            )
    rows = [row for row in rows if math.isfinite(row["time_s"])]
    rows.sort(key=lambda row: row["time_s"])
    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")
    return rows


def read_raw_rtk_rows(config_path: Path, summary: dict[str, float]) -> RtkPlotRows:
    config = read_config_values(config_path)
    gnss_path = resolve_data_path(config.get("gnss_path"), config_path.parent)
    origin_h_m = summary.get("origin_h_m")
    if gnss_path is None or not gnss_path.exists() or origin_h_m is None or not math.isfinite(origin_h_m):
        return RtkPlotRows([], "none")

    required_status_code = config_int(config, "required_best_sol_status_code", 0)
    fixed_sigma_m = config_float(config, "gnss_vertical_fixed_sigma_m", 0.20)
    sigma_scale_up = config_float(config, "gnss_sigma_scale_up", 1.0)
    rtkfix_scale = config_float(config, "rtkfix_scale", 1.0)
    rtkfloat_scale = config_float(config, "rtkfloat_scale", 2.0)
    single_scale = config_float(config, "single_scale", 5.0)
    drop_non_rtkfix = config.get("drop_non_rtkfix", "false").strip().lower() == "true"
    drop_no_solution = config.get("drop_no_solution", "true").strip().lower() == "true"
    position_sigma_floor_up_m = config_float(config, "position_sigma_floor_up_m", 0.0)
    position_sigma_ceiling_m = config_float(config, "position_sigma_ceiling_m", 50.0)
    gate_multiple = config_float(config, "vertical_envelope_gate_sigma_multiple", 2.0)
    min_half_width_m = config_float(config, "vertical_envelope_min_half_width_m", 0.0)
    use_fixed_sigma = config.get("gnss_vertical_sigma_mode", "from_file").strip().lower() == "fixed"

    def fix_scale(fix_code: int) -> float:
        if fix_code == RTK_FIX_CODE:
            return rtkfix_scale
        if fix_code == RTK_FLOAT_CODE:
            return rtkfloat_scale
        if fix_code == SINGLE_CODE:
            return single_scale
        return max(single_scale * 10.0, single_scale)

    rows: list[dict[str, float]] = []
    with gnss_path.open("r", encoding="utf-8") as file:
        for line in file:
            tokens = line.split()
            if len(tokens) < 18:
                continue
            time_s = safe_float(tokens[0])
            h_m = safe_float(tokens[3])
            sigma_h_m = safe_float(tokens[6])
            status_code = rounded_int(tokens[10], 0)
            fix_code = rounded_int(tokens[12], 4)
            if drop_non_rtkfix and fix_code != RTK_FIX_CODE:
                continue
            if drop_no_solution and fix_code == NO_SOLUTION_CODE:
                continue
            if required_status_code > 0 and status_code != required_status_code:
                continue
            if not (math.isfinite(time_s) and math.isfinite(h_m)):
                continue
            sigma_u_m = fixed_sigma_m if use_fixed_sigma else sigma_h_m
            if not math.isfinite(sigma_u_m):
                continue
            sigma_u_m *= sigma_scale_up
            sigma_u_m = min(
                max(sigma_u_m, max(position_sigma_floor_up_m, 1e-9)),
                position_sigma_ceiling_m,
            )
            sigma_u_m *= fix_scale(fix_code)
            rows.append(
                {
                    "time_s": time_s,
                    # Local ENU Up is effectively the ellipsoidal height delta for this
                    # short plot span; the solver's exact LocalCartesian conversion is
                    # not needed for visual gate inspection.
                    "rtk_up_m": h_m - origin_h_m,
                    "half_width_m": max(min_half_width_m, gate_multiple * sigma_u_m),
                }
            )
    rows.sort(key=lambda row: row["time_s"])
    return RtkPlotRows(rows, "raw_gnss")


def read_envelope_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            if raw.get("factor_used", "1") == "0":
                continue
            time_s = safe_float(raw.get("corrected_time_s"))
            rtk_up_m = safe_float(raw.get("rtk_up_m"))
            half_width_m = safe_float(raw.get("half_width_m"))
            if not (math.isfinite(time_s) and math.isfinite(rtk_up_m) and math.isfinite(half_width_m)):
                continue
            rows.append(
                {
                    "time_s": time_s,
                    "rtk_up_m": rtk_up_m,
                    "half_width_m": half_width_m,
                }
            )
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_rtk_lowpass_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            if raw.get("lowpass_valid", "0") != "1":
                continue
            time_s = safe_float(raw.get("time_s"))
            raw_up_m = safe_float(raw.get("raw_up_m"))
            lowpass_up_m = safe_float(raw.get("lowpass_up_m"))
            raw_minus_lowpass_m = safe_float(raw.get("raw_minus_lowpass_m"))
            if not (math.isfinite(time_s) and math.isfinite(raw_up_m) and math.isfinite(lowpass_up_m)):
                continue
            rows.append(
                {
                    "time_s": time_s,
                    "raw_up_m": raw_up_m,
                    "lowpass_up_m": lowpass_up_m,
                    "raw_minus_lowpass_m": raw_minus_lowpass_m,
                }
            )
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_rtk_latent_reference_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            time_s = safe_float(raw.get("bin_center_time_s"))
            up_m = safe_float(raw.get("optimized_reference_up_m"))
            if not math.isfinite(up_m):
                up_m = safe_float(raw.get("initial_reference_up_m"))
            sample_count = safe_float(raw.get("sample_count"))
            if math.isfinite(time_s) and math.isfinite(up_m):
                rows.append(
                    {
                        "time_s": time_s,
                        "latent_reference_up_m": up_m,
                        "sample_count": sample_count,
                    }
                )
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_rtk_latent_sample_comparison_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    rows: list[dict[str, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            time_s = safe_float(raw.get("corrected_time_s"))
            raw_up_m = safe_float(raw.get("raw_rtk_up_m"))
            latent_up_m = safe_float(raw.get("optimized_latent_reference_up_m"))
            raw_minus_latent_m = safe_float(raw.get("raw_minus_optimized_latent_m"))
            if not (
                math.isfinite(time_s)
                and math.isfinite(raw_up_m)
                and math.isfinite(latent_up_m)
                and math.isfinite(raw_minus_latent_m)
            ):
                continue
            rows.append(
                {
                    "time_s": time_s,
                    "raw_rtk_up_m": raw_up_m,
                    "latent_reference_up_m": latent_up_m,
                    "raw_minus_latent_m": raw_minus_latent_m,
                }
            )
    rows.sort(key=lambda row: row["time_s"])
    return rows


def read_nhc_windows(path: Path) -> list[tuple[float, float]]:
    if not path.exists():
        return []
    windows: list[tuple[float, float]] = []
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            if raw.get("factor_added", "1") == "0":
                continue
            start_time_s = safe_float(raw.get("start_time_s"))
            end_time_s = safe_float(raw.get("end_time_s"))
            if math.isfinite(start_time_s) and math.isfinite(end_time_s) and end_time_s > start_time_s:
                windows.append((start_time_s, end_time_s))
    windows.sort(key=lambda window: window[0])
    return windows


def read_body_z_nhc_state_rows(path: Path) -> list[dict[str, float]]:
    if not path.exists():
        return []
    buckets: dict[float, list[tuple[float, float]]] = {}
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        for raw in reader:
            time_s = safe_float(raw.get("time_s"))
            raw_body_z = safe_float(raw.get("raw_v_body_z_mps", raw.get("fixed_body_z_velocity_mps")))
            corrected_body_z = safe_float(raw.get("corrected_v_body_z_mps"))
            if not math.isfinite(corrected_body_z):
                corrected_body_z = raw_body_z
            if not (math.isfinite(time_s) and math.isfinite(raw_body_z) and math.isfinite(corrected_body_z)):
                continue
            buckets.setdefault(time_s, []).append((raw_body_z, corrected_body_z))
    rows: list[dict[str, float]] = []
    for time_s, values in buckets.items():
        raw_mean = sum(value[0] for value in values) / len(values)
        corrected_mean = sum(value[1] for value in values) / len(values)
        rows.append(
            {
                "time_s": time_s,
                "raw_v_body_z_mps": raw_mean,
                "corrected_v_body_z_mps": corrected_mean,
            }
        )
    rows.sort(key=lambda row: row["time_s"])
    return rows


def nearest_time(rows: list[dict[str, float]], target_time_s: float) -> float:
    return min(rows, key=lambda row: abs(row["time_s"] - target_time_s))["time_s"]


def resolve_plot_context(
    summary: dict[str, float],
    trajectory_rows: list[dict[str, float]],
) -> PlotContext:
    reference_raw = summary.get("alignment_start_time_s", trajectory_rows[0]["time_s"])
    dynamic_start_time_s = summary.get("dynamic_start_time_s")
    return PlotContext(
        reference_time_s=nearest_time(trajectory_rows, reference_raw),
        dynamic_start_time_s=dynamic_start_time_s,
    )


def rows_from_time(rows: list[dict[str, float]], start_time_s: float) -> list[dict[str, float]]:
    return [row for row in rows if row["time_s"] + TIME_EPSILON_S >= start_time_s]


def relative_time(rows: list[dict[str, float]], reference_time_s: float) -> list[float]:
    return [row["time_s"] - reference_time_s for row in rows]


def relative_values(rows: list[dict[str, float]], key: str) -> list[float]:
    if not rows:
        return []
    reference = rows[0][key]
    return [row[key] - reference for row in rows]


def values_minus_reference(rows: list[dict[str, float]], key: str, reference: float) -> list[float]:
    return [row[key] - reference for row in rows]


def scalar_stats(values: list[float]) -> dict[str, float]:
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return {"count": 0.0, "mean": math.nan, "std": math.nan, "max_abs": math.nan}
    mean = sum(finite_values) / len(finite_values)
    variance = sum((value - mean) ** 2 for value in finite_values) / len(finite_values)
    return {
        "count": float(len(finite_values)),
        "mean": mean,
        "std": math.sqrt(variance),
        "max_abs": max(abs(value) for value in finite_values),
    }


def padded_axis_limits(
    values: list[float],
    min_span: float = 0.02,
    padding_ratio: float = 0.08,
) -> tuple[float, float] | None:
    finite_values = [value for value in values if math.isfinite(value)]
    if not finite_values:
        return None
    lower = min(finite_values)
    upper = max(finite_values)
    span = upper - lower
    if span < min_span:
        center = 0.5 * (lower + upper)
        half_span = 0.5 * min_span
        return center - half_span, center + half_span
    padding = span * padding_ratio
    return lower - padding, upper + padding


def shade_static_and_dynamic(axis, context: PlotContext) -> None:
    if (
        context.dynamic_start_time_s is None or
        context.dynamic_start_time_s <= context.reference_time_s + TIME_EPSILON_S
    ):
        return
    dynamic_rel_time_s = context.dynamic_start_time_s - context.reference_time_s
    axis.axvspan(
        0.0,
        dynamic_rel_time_s,
        color="#17becf",
        alpha=0.08,
        linewidth=0.0,
        label="static alignment",
    )
    axis.axvline(
        dynamic_rel_time_s,
        color="#222222",
        linestyle="--",
        linewidth=1.0,
        alpha=0.75,
        label="dynamic start",
    )


def shade_nhc_windows(axis, windows: list[tuple[float, float]], reference_time_s: float) -> None:
    label_added = False
    for start_time_s, end_time_s in windows:
        if end_time_s <= reference_time_s:
            continue
        axis.axvspan(
            max(start_time_s - reference_time_s, 0.0),
            end_time_s - reference_time_s,
            color="#d62728",
            alpha=0.10,
            linewidth=0.0,
            label="Body-Z NHC window" if not label_added else None,
        )
        label_added = True


def plot_alignment_continuity(
    trajectory_rows: list[dict[str, float]],
    envelope_rows: list[dict[str, float]],
    lowpass_rtk_rows: list[dict[str, float]],
    latent_rtk_rows: list[dict[str, float]],
    nhc_windows: list[tuple[float, float]],
    body_z_state_rows: list[dict[str, float]],
    context: PlotContext,
    output_path: Path,
    title: str,
) -> None:
    plot_rows = rows_from_time(trajectory_rows, context.reference_time_s)
    envelope_rows = rows_from_time(envelope_rows, context.reference_time_s)
    lowpass_rtk_rows = rows_from_time(lowpass_rtk_rows, context.reference_time_s)
    latent_rtk_rows = rows_from_time(latent_rtk_rows, context.reference_time_s)
    x = relative_time(plot_rows, context.reference_time_s)
    up_reference_m = plot_rows[0]["up_m"]

    body_z_state_rows = rows_from_time(body_z_state_rows, context.reference_time_s)

    fig, axes = plt.subplots(5, 1, figsize=(15, 13), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=14)

    for axis in axes:
        shade_static_and_dynamic(axis, context)
        shade_nhc_windows(axis, nhc_windows, context.reference_time_s)
        axis.grid(True, alpha=0.3)

    optimized_up_delta = values_minus_reference(plot_rows, "up_m", up_reference_m)
    height_axis_values = list(optimized_up_delta)
    axes[0].plot(
        x,
        optimized_up_delta,
        color="#1f77b4",
        linewidth=1.1,
        label="optimized up",
    )
    if envelope_rows:
        rtk_x = relative_time(envelope_rows, context.reference_time_s)
        rtk_delta = values_minus_reference(envelope_rows, "rtk_up_m", up_reference_m)
        lower = [center - row["half_width_m"] for center, row in zip(rtk_delta, envelope_rows)]
        upper = [center + row["half_width_m"] for center, row in zip(rtk_delta, envelope_rows)]
        height_axis_values.extend(rtk_delta)
        axes[0].plot(rtk_x, rtk_delta, color="#9467bd", linewidth=0.9, alpha=0.85, label="RTK up center")
        axes[0].fill_between(rtk_x, lower, upper, color="#9467bd", alpha=0.10, linewidth=0.0, label="RTK gate")
    if lowpass_rtk_rows:
        lowpass_x = relative_time(lowpass_rtk_rows, context.reference_time_s)
        lowpass_delta = values_minus_reference(lowpass_rtk_rows, "lowpass_up_m", up_reference_m)
        height_axis_values.extend(lowpass_delta)
        axes[0].plot(
            lowpass_x,
            lowpass_delta,
            color="#2ca02c",
            linewidth=1.2,
            alpha=0.9,
            label="low-pass RTK center",
        )
    if latent_rtk_rows:
        latent_x = relative_time(latent_rtk_rows, context.reference_time_s)
        latent_delta = values_minus_reference(latent_rtk_rows, "latent_reference_up_m", up_reference_m)
        height_axis_values.extend(latent_delta)
        axes[0].plot(
            latent_x,
            latent_delta,
            color="#17becf",
            linewidth=1.3,
            alpha=0.95,
            label="latent RTK reference",
        )
    height_limits = padded_axis_limits(height_axis_values)
    if height_limits is not None:
        axes[0].set_ylim(*height_limits)
    axes[0].set_ylabel("Delta up [m]")
    axes[0].set_title("Up continuity from static alignment start")
    axes[0].legend(loc="upper left", ncol=3, fontsize=8)

    axes[1].plot(x, [row["vz_mps"] for row in plot_rows], color="#1f77b4", linewidth=1.0, label="optimized vz")
    axes[1].axhline(0.0, color="#222222", linewidth=0.7, alpha=0.5)
    axes[1].set_ylabel("Vz [m/s]")
    axes[1].set_title("Vertical velocity")

    axes[2].plot(x, [row["baz_ug"] for row in plot_rows], color="#2ca02c", linewidth=1.0, label="ba_z")
    axes[2].set_ylabel("ba_z [ug]")
    axes[2].set_title("Vertical accelerometer bias")

    if body_z_state_rows:
        body_z_x = relative_time(body_z_state_rows, context.reference_time_s)
        axes[3].plot(
            body_z_x,
            [row["raw_v_body_z_mps"] for row in body_z_state_rows],
            color="#7f7f7f",
            linewidth=0.8,
            alpha=0.8,
            label="raw body-z velocity",
        )
        axes[3].plot(
            body_z_x,
            [row["corrected_v_body_z_mps"] for row in body_z_state_rows],
            color="#d62728",
            linewidth=0.9,
            label="corrected body-z velocity",
        )
    axes[3].axhline(0.0, color="#222222", linewidth=0.7, alpha=0.5)
    axes[3].set_ylabel("Body-z v [m/s]")
    axes[3].set_title("Raw vs leakage-corrected Body-Z velocity")
    axes[3].legend(loc="upper left", fontsize=8)

    pitch_deg = [math.degrees(row["pitch_rad"]) for row in plot_rows]
    roll_deg = [math.degrees(row["roll_rad"]) for row in plot_rows]
    axes[4].plot(x, pitch_deg, color="#ff7f0e", linewidth=0.9, label="pitch")
    axes[4].plot(x, roll_deg, color="#d62728", linewidth=0.9, linestyle="--", label="roll")
    axes[4].set_ylabel("Attitude [deg]")
    axes[4].set_title("Pitch and roll")
    axes[4].set_xlabel("Time since static alignment start [s]")
    axes[4].legend(loc="upper left", fontsize=8)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    run_dir = Path(args.run_dir)
    trajectory_rows = read_trajectory(run_dir / "trajectory.csv")
    summary = read_summary(run_dir / "summary.txt")
    raw_rtk_rows = read_raw_rtk_rows(run_dir / "config_snapshot.cfg", summary)
    envelope_rows = raw_rtk_rows.rows or read_envelope_rows(run_dir / "vertical_envelope_diagnostics.csv")
    rtk_source = raw_rtk_rows.source if raw_rtk_rows.rows else "vertical_envelope_diagnostics"
    lowpass_rtk_rows = read_rtk_lowpass_rows(run_dir / "rtk_vertical_lowpass_reference_diagnostics.csv")
    latent_rtk_rows = read_rtk_latent_reference_rows(run_dir / "rtk_vertical_latent_reference_diagnostics.csv")
    latent_sample_rows = read_rtk_latent_sample_comparison_rows(
        run_dir / "rtk_vertical_latent_reference_sample_comparison.csv"
    )
    nhc_windows = read_nhc_windows(run_dir / "body_z_nhc_diagnostics.csv")
    body_z_state_rows = read_body_z_nhc_state_rows(run_dir / "body_z_nhc_state_diagnostics.csv")
    context = resolve_plot_context(summary, trajectory_rows)
    output_path = Path(args.output)

    plot_alignment_continuity(
        trajectory_rows,
        envelope_rows,
        lowpass_rtk_rows,
        latent_rtk_rows,
        nhc_windows,
        body_z_state_rows,
        context,
        output_path,
        args.title,
    )

    print(f"plot_saved={output_path}")
    print(f"plot_start_time_s={context.reference_time_s}")
    if context.dynamic_start_time_s is not None:
        print(f"dynamic_start_time_s={context.dynamic_start_time_s}")
    if envelope_rows:
        print(f"first_rtk_gate_half_width_m={envelope_rows[0]['half_width_m']}")
        print(f"rtk_plot_source={rtk_source}")
    if lowpass_rtk_rows:
        print(f"lowpass_rtk_count={len(lowpass_rtk_rows)}")
    if latent_rtk_rows:
        print(f"latent_rtk_reference_count={len(latent_rtk_rows)}")
    if latent_sample_rows:
        stats = scalar_stats([row["raw_minus_latent_m"] for row in latent_sample_rows])
        print(f"latent_sample_comparison_count={int(stats['count'])}")
        print(f"raw_minus_latent_mean_m={stats['mean']:.9g}")
        print(f"raw_minus_latent_std_m={stats['std']:.9g}")
        print(f"raw_minus_latent_max_abs_m={stats['max_abs']:.9g}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
