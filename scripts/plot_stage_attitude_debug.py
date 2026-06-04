#!/usr/bin/env python3
"""Plot stage attitude debug trajectories exported by offline_lc_runner."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DEFAULT_SOURCES = (
    "base_graph_initial_values",
    "base_graph_optimized",
    "body_z_seed",
    "stage2_anchor_imu_delta",
)


def parse_float(value: str) -> float:
    try:
        return float(value)
    except ValueError:
        return math.nan


def read_debug_rows(path: Path) -> dict[str, list[dict[str, float]]]:
    rows_by_source_time: dict[str, dict[float, dict[str, float]]] = defaultdict(dict)
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            source = row["source"]
            time_s = parse_float(row["time_s"])
            rows_by_source_time[source][time_s] = {
                "time_s": time_s,
                "yaw_rad": parse_float(row["yaw_rad"]),
                "pitch_rad": parse_float(row["pitch_rad"]),
                "roll_rad": parse_float(row["roll_rad"]),
                "relative_angle_rad": parse_float(row["relative_angle_rad"]),
                "relative_rotvec_z_rad": parse_float(row["relative_rotvec_z_rad"]),
            }
    rows_by_source: dict[str, list[dict[str, float]]] = {}
    for source, rows_by_time in rows_by_source_time.items():
        rows_by_source[source] = list(rows_by_time.values())
    for rows in rows_by_source.values():
        rows.sort(key=lambda item: item["time_s"])
    return rows_by_source


def read_outage_windows(path: Path) -> list[tuple[float, float]]:
    if not path.exists():
        return []
    windows: list[tuple[float, float]] = []
    with path.open("r", newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        for row in reader:
            start_s = parse_float(row.get("start_time_s", "nan"))
            end_s = parse_float(row.get("end_time_s", "nan"))
            if math.isfinite(start_s) and math.isfinite(end_s) and end_s > start_s:
                windows.append((start_s, end_s))
    return windows


def source_series(rows: list[dict[str, float]], key: str) -> tuple[np.ndarray, np.ndarray]:
    times = np.array([row["time_s"] for row in rows], dtype=float)
    values = np.array([row[key] for row in rows], dtype=float)
    return times, values


def plot_debug(
    rows_by_source: dict[str, list[dict[str, float]]],
    outage_windows: list[tuple[float, float]],
    sources: list[str],
    output_path: Path,
    time_start_s: float | None,
    time_end_s: float | None,
) -> None:
    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True)
    plot_specs = (
        ("yaw_rad", "yaw rad", True),
        ("pitch_rad", "pitch rad", False),
        ("roll_rad", "roll rad", False),
        ("relative_angle_rad", "adjacent |rotvec| rad", False),
    )

    for axis, (key, label, unwrap) in zip(axes, plot_specs):
        for source in sources:
            rows = rows_by_source.get(source, [])
            if not rows:
                continue
            times, values = source_series(rows, key)
            finite = np.isfinite(times) & np.isfinite(values)
            if time_start_s is not None:
                finite &= times >= time_start_s
            if time_end_s is not None:
                finite &= times <= time_end_s
            times = times[finite]
            values = values[finite]
            if times.size == 0:
                continue
            if unwrap:
                values = np.unwrap(values)
            axis.plot(times, values, linewidth=1.1, label=source)

        for start_s, end_s in outage_windows:
            axis.axvspan(start_s, end_s, color="0.9", alpha=0.7, linewidth=0)
        axis.set_ylabel(label)
        axis.grid(True, alpha=0.25)

    axes[-1].set_xlabel("time s")
    if time_start_s is not None or time_end_s is not None:
        axes[-1].set_xlim(left=time_start_s, right=time_end_s)
    axes[0].legend(loc="best", fontsize=8)
    fig.suptitle("Stage attitude debug trajectories")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=160)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument(
        "--debug-csv",
        type=Path,
        help="Path to stage_attitude_debug_trajectory.csv; defaults to run-dir file.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Output PNG path; defaults to run-dir/stage_attitude_debug_comparison.png.",
    )
    parser.add_argument(
        "--sources",
        nargs="+",
        default=list(DEFAULT_SOURCES),
        help="Source names to plot in order.",
    )
    parser.add_argument("--time-start", type=float, default=None)
    parser.add_argument("--time-end", type=float, default=None)
    args = parser.parse_args()

    debug_csv = args.debug_csv or args.run_dir / "stage_attitude_debug_trajectory.csv"
    output_path = args.output or args.run_dir / "stage_attitude_debug_comparison.png"
    rows_by_source = read_debug_rows(debug_csv)
    outage_windows = read_outage_windows(args.run_dir / "rtk_outage_windows.csv")
    plot_debug(
        rows_by_source,
        outage_windows,
        list(args.sources),
        output_path,
        args.time_start,
        args.time_end,
    )
    print(output_path)


if __name__ == "__main__":
    main()
