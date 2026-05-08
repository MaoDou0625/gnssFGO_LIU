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


class PlotContext(NamedTuple):
    reference_time_s: float
    dynamic_start_time_s: float | None


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
    nhc_windows: list[tuple[float, float]],
    context: PlotContext,
    output_path: Path,
    title: str,
) -> None:
    plot_rows = rows_from_time(trajectory_rows, context.reference_time_s)
    envelope_rows = rows_from_time(envelope_rows, context.reference_time_s)
    x = relative_time(plot_rows, context.reference_time_s)

    fig, axes = plt.subplots(4, 1, figsize=(15, 11), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=14)

    for axis in axes:
        shade_static_and_dynamic(axis, context)
        shade_nhc_windows(axis, nhc_windows, context.reference_time_s)
        axis.grid(True, alpha=0.3)

    axes[0].plot(x, relative_values(plot_rows, "up_m"), color="#1f77b4", linewidth=1.1, label="optimized up")
    if envelope_rows:
        rtk_x = relative_time(envelope_rows, context.reference_time_s)
        rtk_delta = relative_values(envelope_rows, "rtk_up_m")
        lower = [center - row["half_width_m"] for center, row in zip(rtk_delta, envelope_rows)]
        upper = [center + row["half_width_m"] for center, row in zip(rtk_delta, envelope_rows)]
        axes[0].plot(rtk_x, rtk_delta, color="#9467bd", linewidth=0.9, alpha=0.85, label="RTK up center")
        axes[0].fill_between(rtk_x, lower, upper, color="#9467bd", alpha=0.10, linewidth=0.0, label="RTK gate")
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

    pitch_deg = [math.degrees(row["pitch_rad"]) for row in plot_rows]
    roll_deg = [math.degrees(row["roll_rad"]) for row in plot_rows]
    axes[3].plot(x, pitch_deg, color="#ff7f0e", linewidth=0.9, label="pitch")
    axes[3].plot(x, roll_deg, color="#d62728", linewidth=0.9, linestyle="--", label="roll")
    axes[3].set_ylabel("Attitude [deg]")
    axes[3].set_title("Pitch and roll")
    axes[3].set_xlabel("Time since static alignment start [s]")
    axes[3].legend(loc="upper left", fontsize=8)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def main() -> int:
    args = parse_args()
    run_dir = Path(args.run_dir)
    trajectory_rows = read_trajectory(run_dir / "trajectory.csv")
    summary = read_summary(run_dir / "summary.txt")
    envelope_rows = read_envelope_rows(run_dir / "vertical_envelope_diagnostics.csv")
    nhc_windows = read_nhc_windows(run_dir / "body_z_nhc_diagnostics.csv")
    context = resolve_plot_context(summary, trajectory_rows)
    output_path = Path(args.output)

    plot_alignment_continuity(
        trajectory_rows,
        envelope_rows,
        nhc_windows,
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
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
