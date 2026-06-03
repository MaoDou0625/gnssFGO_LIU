#!/usr/bin/env python

from __future__ import annotations

import argparse
import csv
import math
import statistics
from pathlib import Path

try:
    import matplotlib.pyplot as plt
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "matplotlib is required to generate plots. "
        "Use a Python environment with matplotlib installed."
    ) from exc


GROUPS = [
    ("dtheta", "Attitude Error [rad]", ["dtheta_x_rad", "dtheta_y_rad", "dtheta_z_rad"]),
    ("dv", "Velocity Error [m/s]", ["dv_x_mps", "dv_y_mps", "dv_z_mps"]),
    ("dp", "Position Error [m]", ["dp_x_m", "dp_y_m", "dp_z_m"]),
    ("dbg", "Gyro Bias Error [rad/s]", ["dbg_x_radps", "dbg_y_radps", "dbg_z_radps"]),
    ("dba", "Accel Bias Error [m/s^2]", ["dba_x_mps2", "dba_y_mps2", "dba_z_mps2"]),
]

COMPONENT_LABELS = {
    "x": "x",
    "y": "y",
    "z": "z",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot low-rate error-state trajectories and print summary stats.")
    parser.add_argument("--error-state", required=True, help="Path to error_state_trajectory.csv")
    parser.add_argument("--output", required=True, help="Output image path")
    parser.add_argument("--csv-output", default="", help="Optional summary CSV output path")
    parser.add_argument("--title", default="Estimated error-state trajectories", help="Figure title")
    return parser.parse_args()


def read_rows(path: Path) -> list[dict[str, float]]:
    with path.open("r", encoding="utf-8", newline="") as file:
        reader = csv.DictReader(file)
        rows = []
        for raw in reader:
            row = {"time_s": float(raw["time_s"])}
            for _, _, columns in GROUPS:
                for column in columns:
                    row[column] = float(raw[column])
            rows.append(row)
    if not rows:
        raise ValueError(f"No rows found in {path}")
    rows.sort(key=lambda row: row["time_s"])
    return rows


def total_variation(values: list[float]) -> float:
    return sum(abs(values[index + 1] - values[index]) for index in range(len(values) - 1))


def summarize(rows: list[dict[str, float]]) -> list[dict[str, float | str]]:
    stats: list[dict[str, float | str]] = []
    for group_name, _, columns in GROUPS:
        for column in columns:
            values = [row[column] for row in rows]
            component = column.split("_")[1]
            stats.append(
                {
                    "group": group_name,
                    "component": component,
                    "min": min(values),
                    "max": max(values),
                    "range": max(values) - min(values),
                    "mean": statistics.mean(values),
                    "std": statistics.pstdev(values),
                    "end_minus_start": values[-1] - values[0],
                    "total_variation": total_variation(values),
                }
            )
    return stats


def write_summary_csv(path: Path, stats: list[dict[str, float | str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as file:
        writer = csv.DictWriter(
            file,
            fieldnames=[
                "group",
                "component",
                "min",
                "max",
                "range",
                "mean",
                "std",
                "end_minus_start",
                "total_variation",
            ],
        )
        writer.writeheader()
        for row in stats:
            writer.writerow(row)


def make_plot(rows: list[dict[str, float]], output_path: Path, title: str) -> None:
    time0 = rows[0]["time_s"]
    times = [row["time_s"] - time0 for row in rows]

    fig, axes = plt.subplots(len(GROUPS), 1, figsize=(16, 13), sharex=True, constrained_layout=True)
    fig.suptitle(title, fontsize=16)

    colors = {"x": "#1f77b4", "y": "#ff7f0e", "z": "#2ca02c"}

    for axis, (group_name, ylabel, columns) in zip(axes, GROUPS):
        for column in columns:
            component = column.split("_")[1]
            axis.plot(times, [row[column] for row in rows], label=COMPONENT_LABELS[component], color=colors[component], linewidth=1.15)
        axis.set_ylabel(ylabel)
        axis.set_title(group_name)
        axis.grid(True, alpha=0.3)
        axis.legend(loc="upper right")

    axes[-1].set_xlabel("Time since first error node [s]")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=180)
    plt.close(fig)


def print_stats(stats: list[dict[str, float | str]]) -> None:
    for row in stats:
        print(
            f"{row['group']}_{row['component']}: "
            f"range={float(row['range']):.6e}, "
            f"std={float(row['std']):.6e}, "
            f"end_minus_start={float(row['end_minus_start']):.6e}, "
            f"total_variation={float(row['total_variation']):.6e}"
        )


def main() -> int:
    args = parse_args()
    error_state_path = Path(args.error_state)
    output_path = Path(args.output)
    csv_output_path = (
        Path(args.csv_output)
        if args.csv_output
        else output_path.with_name(output_path.stem + "_summary.csv")
    )

    rows = read_rows(error_state_path)
    stats = summarize(rows)
    make_plot(rows, output_path, args.title)
    write_summary_csv(csv_output_path, stats)

    print(f"plot_saved={output_path}")
    print(f"csv_saved={csv_output_path}")
    print_stats(stats)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
